#include <AccelStepper.h>
#include "MotorPresetsBridge.h"

// ===================== CONFIG GÉNÉRALE =====================
#define NBMOTEURS    10
#define NBDATA       34
#define NBPASPARTOUR 3200
#define STEP_DRIVER  AccelStepper::DRIVER

// Indexs dans ABC:
//  - ABC[32] : commande d'arrêt d'urgence (0 = clear, 1 = start ramp, 2 = forced instant)
//  - ABC[33] : profil réactivité 0=SOFT,1=MEDIUM,2=NERVOUS,3=VERY_NERVOUS
//  - ABC[31] : dynamic command (0=none,1=force global ON,2=mask per-motor,3=auto per-motor, <0=force OFF)
//  - ABC[30] : explicit toggle for AUTO dynamic mode (non-zero -> force AUTO)
const uint8_t PROFILE_DATA_INDEX = NBDATA - 1;  // = 33
const uint8_t EMERGENCY_CMD_INDEX = 32;
const uint8_t DYNAMIC_CMD_INDEX = 31;
const uint8_t AUTO_DYNAMIC_CMD_INDEX = 30;
const long DYN_MODE_AUTO = 3;

// ===================== PINS MOTEURS =====================//COUCOULB
// Adapter à ton câblage réel.
const uint8_t PINDIRECTION[NBMOTEURS] = {6, 9, 12, 26, 29, 32, 34, 37, 39, 41};
const uint8_t PINSPEED[NBMOTEURS]    = {5, 8, 11, 25, 28, 31, 33, 36, 38, 40};

// Enable par moteur (ou -1 si pas utilisé)
const int8_t ENABLEPIN[NBMOTEURS] = {4, 7, 10, 24, 27, 30, 35, 42, 43, 44};

AccelStepper stepper[NBMOTEURS] = {
  AccelStepper(STEP_DRIVER, PINSPEED[0], PINDIRECTION[0]),
  AccelStepper(STEP_DRIVER, PINSPEED[1], PINDIRECTION[1]),
  AccelStepper(STEP_DRIVER, PINSPEED[2], PINDIRECTION[2]),
  AccelStepper(STEP_DRIVER, PINSPEED[3], PINDIRECTION[3]),
  AccelStepper(STEP_DRIVER, PINSPEED[4], PINDIRECTION[4]),
  AccelStepper(STEP_DRIVER, PINSPEED[5], PINDIRECTION[5]),
  AccelStepper(STEP_DRIVER, PINSPEED[6], PINDIRECTION[6]),
  AccelStepper(STEP_DRIVER, PINSPEED[7], PINDIRECTION[7]),
  AccelStepper(STEP_DRIVER, PINSPEED[8], PINDIRECTION[8]),
  AccelStepper(STEP_DRIVER, PINSPEED[9], PINDIRECTION[9])
};

// ===================== SIGNE PAR MOTEUR =====================
// posMax = DIR_SIGN[i] * currentPosition()
int8_t DIR_SIGN[NBMOTEURS] = {-1, -1, +1, -1, -1, -1, +1, +1, -1, -1};

// ===================== PROFIL VITESSE / ACCEL =====================
const float VMAX_HARD       = 16000.0f;
const float ACC_HARD        = 1200.0f;

const float VMIN_SOFT       = 4000.0f;
const float ACC_MIN_SOFT    = 400.0f;

const float VMIN_USEFUL     = 2000.0f;
const float ACC_MIN_USEFUL  = 400.0f;

const float NORM_V_UP_PER_S   = 6000.0f;
const float NORM_V_DOWN_PER_S = 6000.0f;
const float NORM_A_UP_PER_S   = 8000.0f;
const float NORM_A_DOWN_PER_S = 8000.0f;

const unsigned long FLIP_TOTAL_MS = 400;
const unsigned long FLIP_EXP_MS   = 200;
const unsigned long FLIP_LIN_MS   = 200;
const float FLIP_EXP_STRENGTH     = 0.25f;
const float FLIP_LIN_STRENGTH     = 0.6f;

// --- Paramètres arrêt d'urgence ---
const long EMERGENCY_STEP_THRESHOLD = 3200;    // seuil déclenchement (steps entre deux boucles)
const float EMERGENCY_V_TARGET      = 320.0f;  // vitesse cible (steps/s)
const float EMERGENCY_A_SCALE       = 1.0f;    // EM_A = EMERGENCY_V_TARGET * scale
const unsigned long EMERGENCY_RAMP_MS = 3000;  // durée rampe (ms)

// ✨ Paramètres heuristiques DIST→V/A (changés via presets)
float HEUR_D_MIN;   // distance min
float HEUR_D_MAX;   // distance max

float HEUR_V_MIN;   // vitesse min
float HEUR_V_MAX;   // vitesse max

float HEUR_A_MIN;   // accel min
float HEUR_A_MAX;   // accel max

// --- Per-motor presets (initialized from profile defaults, can be tuned per motor)
float MOTOR_HEUR_D_MIN[NBMOTEURS];
float MOTOR_HEUR_D_MAX[NBMOTEURS];
float MOTOR_HEUR_V_MIN[NBMOTEURS];
float MOTOR_HEUR_V_MAX[NBMOTEURS];
float MOTOR_HEUR_A_MIN[NBMOTEURS];
float MOTOR_HEUR_A_MAX[NBMOTEURS];

// ✨ Index symboliques de profils
const uint8_t PROFILE_SOFT_IDX    = 0;
const uint8_t PROFILE_MEDIUM_IDX  = 1;
const uint8_t PROFILE_NERVOUS_IDX = 2;
const uint8_t PROFILE_VERY_NERVOUS_IDX = 3; // nouveau profil

struct MotionProfileParams {
  float dMin;
  float dMax;
  float vMin;
  float vMax;
  float aMin;
  float aMax;
};

const MotionProfileParams PROFILE_SOFT = {10.0f, 1500.0f, 400.0f, 4000.0f, 200.0f, 600.0f};
const MotionProfileParams PROFILE_MEDIUM = {1.0f, 5000.0f, 1600.0f, 12000.0f, 200.0f, 800.0f}; // géré par le logiciel par acc2
const MotionProfileParams PROFILE_NERVOUS = {5.0f, 6000.0f, 1200.0f, 12000.0f, 200.0f, 1200.0f};
const MotionProfileParams PROFILE_VERY_NERVOUS = {1.0f, 6000.0f, 1600.0f, 16000.0f, 200.0f, 1000.0f}; // nouveau

uint8_t currentProfile = PROFILE_MEDIUM_IDX;
bool changementDeDYNAMIQUE = false;
// per-motor override flags for dynamic mode (false = use global flag only)
bool changementDeDYNAMIQUE_perMotor[NBMOTEURS] = { false };

// Automatic dynamic detection support
const uint8_t DYN_HISTORY_WINDOW = 8;
long distHistory[NBMOTEURS][DYN_HISTORY_WINDOW];
uint8_t distHistIdx[NBMOTEURS];
bool dynAutoEnabled[NBMOTEURS]; // last computed auto decision

// apply per-motor arrays from HEUR_* defaults
void applyPerMotorPresetsFromProfile() {
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    MOTOR_HEUR_D_MIN[i] = HEUR_D_MIN;
    MOTOR_HEUR_D_MAX[i] = HEUR_D_MAX;
    MOTOR_HEUR_V_MIN[i] = HEUR_V_MIN;
    MOTOR_HEUR_V_MAX[i] = HEUR_V_MAX;
    MOTOR_HEUR_A_MIN[i] = HEUR_A_MIN;
    MOTOR_HEUR_A_MAX[i] = HEUR_A_MAX;
  }
}

void applyMotionProfile(uint8_t profileIndex) {
  const MotionProfileParams* p;
  switch (profileIndex) {
    case PROFILE_SOFT_IDX: p = &PROFILE_SOFT; break;
    case PROFILE_MEDIUM_IDX: p = &PROFILE_MEDIUM; break;
    case PROFILE_NERVOUS_IDX: p = &PROFILE_NERVOUS; break;
    case PROFILE_VERY_NERVOUS_IDX: p = &PROFILE_VERY_NERVOUS; break;
    default: p = &PROFILE_MEDIUM; break;
  }
  HEUR_D_MIN = p->dMin;
  HEUR_D_MAX = p->dMax;
  HEUR_V_MIN = p->vMin;
  HEUR_V_MAX = p->vMax;
  HEUR_A_MIN = p->aMin;
  HEUR_A_MAX = p->aMax;

  // update per-motor presets
  applyPerMotorPresetsFromProfile();
}

// ===================== ETAT MOTEURS / HEURISTIQUES =====================
float vUsed[NBMOTEURS];
float aUsed[NBMOTEURS];
int   lastDir[NBMOTEURS];
bool  inFlip[NBMOTEURS];
unsigned long flipStartMs[NBMOTEURS];

long  targetPos[NBMOTEURS];
long  lastStreamTarget[NBMOTEURS];
float lastStreamV[NBMOTEURS];

// --- Variables arrêt d'urgence ---
bool emergencyActive = false;
unsigned long emergencyStartMs = 0;
float emergencyVStart[NBMOTEURS];
float emergencyAStart[NBMOTEURS];
long  lastLoopPosition[NBMOTEURS];

// ===================== DONNÉES REÇUES DE MAX =====================
long ABC[NBDATA] = {0};

// Pour savoir si les IN ont changé (pour n’envoyer que si modif)
long lastABC[NBDATA];
bool lastInValid = false;

// ===================== RÉCEPTION TRAME <...> SUR Serial (USB) =====================
const byte numChars = 200;
char receivedChars[numChars];
char tempChars[numChars];
bool newData = false;

// ===================== TIMERS AFFICHAGE =====================
unsigned long lastInMs   = 0;
unsigned long lastOutMs  = 0;
unsigned long lastDistMs = 0;
const unsigned long PRINT_INTERVAL_MS = 250;

// ===================== RÉCEPTION RAW AVEC <...> =====================
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  static unsigned long frameStartMs = 0;
  const char startMarker = '<';
  const char endMarker   = '>';
  const unsigned long FRAME_TIMEOUT_MS = 25;

  while (Serial.available() > 0 && newData == false) {
    char rc = (char)Serial.read();
    if (rc == '\r' || rc == '\n') continue;
    if (!recvInProgress) {
      if (rc == startMarker) {
        recvInProgress = true;
        ndx = 0;
        frameStartMs = millis();
      }
    } else {
      if (millis() - frameStartMs > FRAME_TIMEOUT_MS) {
        recvInProgress = false;
        ndx = 0;
        continue;
      }
      if (rc == endMarker) {
        if (ndx >= numChars) ndx = numChars - 1;
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
        break;
      } else {
        if (ndx < numChars - 1) {
          receivedChars[ndx++] = rc;
        } else {
          recvInProgress = false;
          ndx = 0;
        }
      }
    }
  }
}

// ===================== ENVOIS SERIE =====================
void maybeSendINSerial() {
  bool changed = false;
  if (!lastInValid) changed = true;
  else {
    for (uint8_t i = 0; i < NBDATA; i++) if (ABC[i] != lastABC[i]) { changed = true; break; }
  }
  if (!changed) return;
  for (uint8_t i = 0; i < NBDATA; i++) lastABC[i] = ABC[i];
  lastInValid = true;
  Serial.print('0');
  for (uint8_t i = 0; i < NBDATA; i++) { Serial.print(' '); Serial.print(ABC[i]); }
  Serial.println();
}

bool allMotorsAtTarget() {
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long d = stepper[i].distanceToGo(); if (d < 0) d = -d;
    if (d >= 1) return false;
  }
  return true;
}

void maybeSendDISTSerial() {
  if (allMotorsAtTarget()) return;
  Serial.print('2');
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long distInt  = stepper[i].distanceToGo();
    long distReal = DIR_SIGN[i] * distInt;
    Serial.print(' '); Serial.print(distReal);
  }
  Serial.println();
}

void maybeSendOUTSerial() {
  if (allMotorsAtTarget()) return;
  Serial.print('1');
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long curInternal = stepper[i].currentPosition();
    long curReal     = DIR_SIGN[i] * curInternal;
    long vNow        = (long)vUsed[i];
    long aNow        = (long)aUsed[i];
    Serial.print(' ');
    Serial.print(curReal);
    Serial.print(' ');
    Serial.print(vNow);
    Serial.print(' ');
    Serial.print(aNow);
  }
  Serial.println();
}

// ===================== PARSING TRAME & APPLICATION =====================
void parseData() {
    strncpy(tempChars, receivedChars, numChars);
  tempChars[numChars - 1] = '\0';
  char *ptr = tempChars;
  char *endptr;
  uint8_t tokenIndex = 0;
  bool parseError = false;
  while (tokenIndex < NBDATA && *ptr != '\0') {
    long val = strtol(ptr, &endptr, 10);
    if (endptr == ptr) { parseError = true; break; }
    ABC[tokenIndex++] = val;
    if (*endptr == ',') ptr = endptr + 1; else break;
  }
  if (parseError) { newData = false; return; }
  for (uint8_t i = tokenIndex; i < NBDATA; ++i) ABC[i] = 0;

  // PROFILE: ABC[33] modulates reactivity: 0=SOFT,1=MEDIUM,2=NERVOUS,3=VERY_NERVOUS
  long profRaw = ABC[PROFILE_DATA_INDEX];
  uint8_t requestedProfile;
  if (profRaw <= 0) requestedProfile = PROFILE_SOFT_IDX;
  else if (profRaw == 1) requestedProfile = PROFILE_MEDIUM_IDX;
  else if (profRaw == 2) requestedProfile = PROFILE_NERVOUS_IDX;
  else if (profRaw == 3) requestedProfile = PROFILE_VERY_NERVOUS_IDX;
  else requestedProfile = currentProfile;

  // activer/désactiver mode changementDeDYNAMIQUE (séparé des profiles)
  long dynRaw = ABC[DYNAMIC_CMD_INDEX];

  // permettre à l'entrée ABC[30] d'activer le mode AUTO dynamiquement (contrôle explicite)
  if (AUTO_DYNAMIC_CMD_INDEX < NBDATA && ABC[AUTO_DYNAMIC_CMD_INDEX] != 0) {
    dynRaw = DYN_MODE_AUTO;
  }

  // dynRaw semantics:
  //  0 = none (do not toggle dynamic here)
  //  1 = global ON
  //  2 = per-motor mask in ABC[10..19] (non-zero -> enable per motor)
  //  3 = AUTO per-motor (heuristic based on recent requested distances)
  // <0 or other = global OFF
  if (dynRaw == 1) {
    changementDeDYNAMIQUE = true;
    for (uint8_t m = 0; m < NBMOTEURS; ++m) {
      changementDeDYNAMIQUE_perMotor[m] = false;
      dynAutoEnabled[m] = false;
    }
  } else if (dynRaw == 2) {
    changementDeDYNAMIQUE = false;
    for (uint8_t m = 0; m < NBMOTEURS; ++m) {
      uint8_t idx = 10 + m;
      if (idx < NBDATA) changementDeDYNAMIQUE_perMotor[m] = (ABC[idx] != 0);
      else changementDeDYNAMIQUE_perMotor[m] = false;
      dynAutoEnabled[m] = false;
    }
  } else if (dynRaw == DYN_MODE_AUTO) {
    changementDeDYNAMIQUE = false; // global off, per-motor auto will decide
    // auto decision will be computed below when we have per-motor deltas
  } else {
    // default: no dynamic change requested
    changementDeDYNAMIQUE = false;
    for (uint8_t m = 0; m < NBMOTEURS; ++m) {
      changementDeDYNAMIQUE_perMotor[m] = false;
      dynAutoEnabled[m] = false;
    }
  }

  if (requestedProfile != currentProfile) {
    currentProfile = requestedProfile;
    applyMotionProfile(currentProfile);
    for (uint8_t i = 0; i < NBMOTEURS; i++) {
      if (vUsed[i] < MOTOR_HEUR_V_MIN[i]) vUsed[i] = MOTOR_HEUR_V_MIN[i];
      if (aUsed[i] < MOTOR_HEUR_A_MIN[i]) aUsed[i] = MOTOR_HEUR_A_MIN[i];
    }
  }

  // update target positions (ABC[0..9]) and maintain history for auto dynamic detection
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long rawPos    = ABC[i];
    long signedPos = DIR_SIGN[i] * rawPos;
    long d = signedPos - lastStreamTarget[i];
    // lastStreamTarget used also to compute lastStreamV
    lastStreamTarget[i] = signedPos;
    lastStreamV[i] = (float)abs(d) / 0.025f;
    targetPos[i] = signedPos;
    stepper[i].moveTo(signedPos);

    // update history of requested deltas (magnitude of new request)
    long mag = (d >= 0) ? d : -d;
    distHistory[i][distHistIdx[i]] = mag;
    distHistIdx[i] = (distHistIdx[i] + 1) % DYN_HISTORY_WINDOW;
  }

  // If AUTO mode requested (dynRaw == DYN_MODE_AUTO), compute per-motor heuristic decisions
  if (dynRaw == DYN_MODE_AUTO) {
    // Heuristic:
    //  - compute mean and stddev on last DYN_HISTORY_WINDOW requested deltas
    //  - score = 0.6 * mean + 0.4 * stddev
    //  - enable dynamic for motor if score > AUTO_ENABLE_SCORE (tunable)
    const float AUTO_ENABLE_SCORE = 800.0f; // tunable threshold
    for (uint8_t i = 0; i < NBMOTEURS; ++i) {
      // compute mean and stddev
      long sum = 0;
      long maxv = 0;
      for (uint8_t k = 0; k < DYN_HISTORY_WINDOW; ++k) {
        long v = distHistory[i][k];
        sum += v;
        if (v > maxv) maxv = v;
      }
      float mean = (float)sum / (float)DYN_HISTORY_WINDOW;
      // stddev
      float sqsum = 0.0f;
      for (uint8_t k = 0; k < DYN_HISTORY_WINDOW; ++k) {
        float diff = (float)distHistory[i][k] - mean;
        sqsum += diff * diff;
      }
      float stddev = sqrt(sqsum / (float)DYN_HISTORY_WINDOW);
      float score = 0.6f * mean + 0.4f * stddev;

      // additional rules:
      //  - if recent requests are very small (mean < 200) -> still enable dynamic (fine control)
      //  - if recent requests are extremely large (mean > 3000) -> enable dynamic
      bool enable = false;
      if (mean < 200.0f) enable = true;
      else if (mean > 3000.0f) enable = true;
      else if (score > AUTO_ENABLE_SCORE) enable = true;
      else enable = false;

      dynAutoEnabled[i] = enable;
      // reset explicit perMask flag when in AUTO mode
      changementDeDYNAMIQUE_perMotor[i] = false;
    }
  }

  newData = false;
}

// ===================== HELPERS PROFIL =====================
float approachTimed(float currentVal, float targetVal, float upRatePerS, float downRatePerS, float dtMs) {
  float upStep   = upRatePerS   * (dtMs / 1000.0f);
  float downStep = downRatePerS * (dtMs / 1000.0f);
  float diff     = targetVal - currentVal;
  if (diff > 0) { if (diff > upStep) diff = upStep; }
  else if (diff < 0) { if (diff < -downStep) diff = -downStep; }
  return currentVal + diff;
}

void getFlipRates(unsigned long flipAgeMs, float &vUp, float &vDown, float &aUp, float &aDown) {
  vUp   = NORM_V_UP_PER_S; vDown = NORM_V_DOWN_PER_S; aUp = NORM_A_UP_PER_S; aDown = NORM_A_DOWN_PER_S;
  if (flipAgeMs >= FLIP_TOTAL_MS) return;
  if (flipAgeMs < FLIP_EXP_MS) {
    float t = (float)flipAgeMs / (float)FLIP_EXP_MS;
    float k = 1.0f - t;
    float factor = FLIP_EXP_STRENGTH + (1.0f - FLIP_EXP_STRENGTH) * (k * k);
    vUp *= factor; vDown *= factor; aUp *= factor; aDown *= factor;
  } else {
    unsigned long linAge = flipAgeMs - FLIP_EXP_MS;
    float t = (float)linAge / (float)FLIP_LIN_MS; if (t > 1.0f) t = 1.0f;
    float factor = FLIP_LIN_STRENGTH + (1.0f - FLIP_LIN_STRENGTH) * t;
    vUp *= factor; vDown *= factor; aUp *= factor; aDown *= factor;
  }
}

// nouveau : calculer des taux dynamiques en fonction de la distance (par-moteur)
void computeDynamicRates(uint8_t motorIdx, long distAbs, float &vUp, float &vDown, float &aUp, float &aDown) {
  float baseVUp   = NORM_V_UP_PER_S;
  float baseVDown = NORM_V_DOWN_PER_S;
  float baseAUp   = NORM_A_UP_PER_S;
  float baseADown = NORM_A_DOWN_PER_S;

  float dmin = MOTOR_HEUR_D_MIN[motorIdx];
  float dmax = MOTOR_HEUR_D_MAX[motorIdx];
  float nd = 0.0f;
  float span = (dmax - dmin);
  if (span > 0.001f) nd = ((float)distAbs - dmin) / span;
  if (nd < 0.0f) nd = 0.0f;
  if (nd > 1.0f) nd = 1.0f;

  float factor = 0.25f + 0.75f * nd;
  float curve = factor * factor * (3.0f - 2.0f * factor);
  float finalFactor = 0.4f + 0.6f * curve;

  vUp   = baseVUp   * finalFactor;
  vDown = baseVDown * finalFactor;
  aUp   = baseAUp   * finalFactor;
  aDown = baseADown * finalFactor;

  if (vUp < 1000.0f) vUp = 1000.0f;
  if (vDown < 1000.0f) vDown = 1000.0f;
  if (aUp < 200.0f) aUp = 200.0f;
  if (aDown < 200.0f) aDown = 200.0f;
}

float computeSpeedFromDistance(uint8_t motorIdx, long distAbs) {
  float dmin = MOTOR_HEUR_D_MIN[motorIdx];
  float dmax = MOTOR_HEUR_D_MAX[motorIdx];
  float vmin = MOTOR_HEUR_V_MIN[motorIdx];
  float vmax = MOTOR_HEUR_V_MAX[motorIdx];

  float d = (float)distAbs;
  if (d < dmin) d = dmin;
  if (d > dmax) d = dmax;
  float x = (d - dmin) / (dmax - dmin);
  float f = x * x * (3.0f - 2.0f * x);
  float v = vmin + f * (vmax - vmin);
  if (v > VMAX_HARD) v = VMAX_HARD;
  if (v < vmin) v = vmin;
  return v;
}

float computeAccelFromDistance(uint8_t motorIdx, long distAbs) {
  float dmin = MOTOR_HEUR_D_MIN[motorIdx];
  float dmax = MOTOR_HEUR_D_MAX[motorIdx];
  float amin = MOTOR_HEUR_A_MIN[motorIdx];
  float amax = MOTOR_HEUR_A_MAX[motorIdx];

  float d = (float)distAbs;
  if (d < dmin) d = dmin;
  if (d > dmax) d = dmax;
  float x = (d - dmin) / (dmax - dmin);
  float f = x * x * (3.0f - 2.0f * x);
  float a = amin + f * (amax - amin);
  if (a > ACC_HARD) a = ACC_HARD;
  if (a < amin) a = amin;
  return a;
}

// ===================== SETUP =====================
unsigned long lastLoopMs = 0;

void setup() {
  Serial.begin(115200);
  applyMotionProfile(currentProfile);

  // charger presets par moteur depuis MotorPresetsBridge.h
  loadPerMotorPresetsFromLibrary();

  // init history arrays
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    for (uint8_t k = 0; k < DYN_HISTORY_WINDOW; ++k) distHistory[i][k] = 0;
    distHistIdx[i] = 0;
    dynAutoEnabled[i] = false;
  }

  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    stepper[i].setMinPulseWidth(5);
    if (ENABLEPIN[i] >= 0) { pinMode(ENABLEPIN[i], OUTPUT); digitalWrite(ENABLEPIN[i], LOW); }
    pinMode(PINDIRECTION[i], OUTPUT);
    pinMode(PINSPEED[i], OUTPUT);
    stepper[i].setMaxSpeed(MOTOR_HEUR_V_MIN[i]);
    stepper[i].setAcceleration(MOTOR_HEUR_A_MIN[i]);
    stepper[i].moveTo(1600);
    stepper[i].run();
    vUsed[i] = MOTOR_HEUR_V_MIN[i];
    aUsed[i] = MOTOR_HEUR_A_MIN[i];
    lastDir[i] = 0;
    inFlip[i]  = false;
    flipStartMs[i] = 0;
    targetPos[i]   = 0;
    lastStreamTarget[i] = 0;
    lastStreamV[i]      = 0;
    lastLoopPosition[i] = stepper[i].currentPosition();
    emergencyVStart[i] = vUsed[i];
    emergencyAStart[i] = aUsed[i];
    changementDeDYNAMIQUE_perMotor[i] = false;
  }
  lastLoopMs = millis();
  unsigned long now = millis();
  lastInMs   = now;
  lastOutMs  = now;
  lastDistMs = now;
  lastInValid = false;

  // visible startup message on serial
  Serial.println("<READY>");
}

// démarrer l'arrêt d'urgence (appelé une fois lors du déclenchement)
void startEmergencyStop(unsigned long nowMs) {
  if (emergencyActive) return;
  emergencyActive = true;
  emergencyStartMs = nowMs;
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    emergencyVStart[i] = vUsed[i];
    emergencyAStart[i] = aUsed[i];
  }
  // signal visible sur port série
  Serial.print("<EMERGENCY START ");
  Serial.print(nowMs);
  Serial.println(">");
}

// clear / annuler l'arrêt d'urgence (commande via ABC[32] == 0)
void clearEmergencyStop(unsigned long nowMs) {
  if (!emergencyActive) return;
  emergencyActive = false;
  Serial.print("<EMERGENCY CLEARED ");
  Serial.print(nowMs);
  Serial.println(">");
}

// Lire commande d'urgence venant de ABC[32]
// valeurs conventionnelles:
//  0 = annuler l'arrêt d'urgence (clear)
//  1 = démarrer la rampe d'arrêt d'urgence (start ramp)
//  2 = forcer l'arrêt d'urgence (rampe instantanée -> target atteint immédiatement)
void checkEmergencyCommandFromABC(unsigned long nowMs) {
  long cmd = ABC[EMERGENCY_CMD_INDEX];
  if (cmd == 1) {
    startEmergencyStop(nowMs);
  } else if (cmd == 2) {
    startEmergencyStop(nowMs);
    // forcer la rampe complète d'urgence
    emergencyStartMs = nowMs - EMERGENCY_RAMP_MS;
    Serial.print("<EMERGENCY FORCED ");
    Serial.print(nowMs);
    Serial.println(">");
  } else if (cmd == 0) {
    if (emergencyActive) clearEmergencyStop(nowMs);
  }
}

// ===================== LOOP =====================
void loop() {
  // 1) Réception depuis Max
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
    unsigned long nowAfterParse = millis();
    // contrôle de l'arrêt d'urgence via ABC[32]
    checkEmergencyCommandFromABC(nowAfterParse);
  }

  // 2) dt
  unsigned long nowMs = millis();
  float dtMs = (float)(nowMs - lastLoopMs);
  if (dtMs < 0) dtMs = 0;
  if (dtMs > 50) dtMs = 50;
  lastLoopMs = nowMs;

  // --- Détection arrêt d'urgence : si déplacement suspect entre deux boucles ---
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    long curPos = stepper[i].currentPosition();
    long d = curPos - lastLoopPosition[i];
    if (d < 0) d = -d;
    if (!emergencyActive && d > EMERGENCY_STEP_THRESHOLD) {
      startEmergencyStop(nowMs);
      break;
    }
  }

  // 3) Mise à jour profils / vitesses / accels
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long curPos  = stepper[i].currentPosition();
    long tgtPos  = targetPos[i];
    long dist    = tgtPos - curPos;
    long distAbs = (dist >= 0) ? dist : -dist;
    int dirNow = (dist > 0) ? +1 : (dist < 0) ? -1 : 0;

    if (emergencyActive) {
      unsigned long elapsed = nowMs - emergencyStartMs;
      float t = (elapsed >= EMERGENCY_RAMP_MS) ? 1.0f : (float)elapsed / (float)EMERGENCY_RAMP_MS;
      float emergencyA_target = EMERGENCY_V_TARGET * EMERGENCY_A_SCALE;
      if (emergencyA_target < 50.0f) emergencyA_target = 50.0f;
      if (emergencyA_target > ACC_HARD) emergencyA_target = ACC_HARD;
      vUsed[i] = emergencyVStart[i] + (EMERGENCY_V_TARGET - emergencyVStart[i]) * t;
      aUsed[i] = emergencyAStart[i] + (emergencyA_target - emergencyAStart[i]) * t;
      if (vUsed[i] > VMAX_HARD) vUsed[i] = VMAX_HARD;
      if (vUsed[i] < EMERGENCY_V_TARGET) vUsed[i] = EMERGENCY_V_TARGET;
      if (aUsed[i] > ACC_HARD) aUsed[i] = ACC_HARD;
      if (aUsed[i] < 10.0f) aUsed[i] = 10.0f;
      stepper[i].setMaxSpeed(vUsed[i]);
      stepper[i].setAcceleration(aUsed[i]);
      stepper[i].moveTo(tgtPos);
      continue;
    }

    // normal behaviour
    float vTarget = computeSpeedFromDistance(i, distAbs);
    float aTarget = computeAccelFromDistance(i, distAbs);
    float vStream = lastStreamV[i];
    if (vStream > vTarget) {
      vTarget = vStream;
      if (aTarget < ACC_MIN_SOFT) aTarget = ACC_MIN_SOFT;
    }

    if (dirNow != 0 && lastDir[i] != 0 && dirNow != lastDir[i]) {
      inFlip[i] = true;
      flipStartMs[i] = nowMs;
    }
    lastDir[i] = dirNow;

    float vUpRate = NORM_V_UP_PER_S, vDownRate = NORM_V_DOWN_PER_S, aUpRate = NORM_A_UP_PER_S, aDownRate = NORM_A_DOWN_PER_S;

    // déterminer si on utilise le changement dynamique pour ce moteur:
    // - global flag OR explicit per-motor mask OR automatic decision when DYN_MODE_AUTO chosen
    bool useDynamicThisMotor = changementDeDYNAMIQUE || changementDeDYNAMIQUE_perMotor[i] || dynAutoEnabled[i];

    if (useDynamicThisMotor) {
      computeDynamicRates(i, distAbs, vUpRate, vDownRate, aUpRate, aDownRate);
    }

    if (inFlip[i]) {
      unsigned long flipAge = nowMs - flipStartMs[i];
      if (flipAge >= FLIP_TOTAL_MS) inFlip[i] = false;
      else {
        float f_vUp, f_vDown, f_aUp, f_aDown;
        getFlipRates(flipAge, f_vUp, f_vDown, f_aUp, f_aDown);
        if (useDynamicThisMotor) {
          vUpRate   = vUpRate   * (f_vUp   / NORM_V_UP_PER_S);
          vDownRate = vDownRate * (f_vDown / NORM_V_DOWN_PER_S);
          aUpRate   = aUpRate   * (f_aUp   / NORM_A_UP_PER_S);
          aDownRate = aDownRate * (f_aDown / NORM_A_DOWN_PER_S);
        } else {
          vUpRate = f_vUp; vDownRate = f_vDown; aUpRate = f_aUp; aDownRate = f_aDown;
        }
        if (vTarget < VMIN_USEFUL) vTarget = VMIN_USEFUL;
        if (aTarget < ACC_MIN_USEFUL) aTarget = ACC_MIN_USEFUL;
      }
    }

    vUsed[i] = approachTimed(vUsed[i], vTarget, vUpRate, vDownRate, dtMs);
    aUsed[i] = approachTimed(aUsed[i], aTarget, aUpRate, aDownRate, dtMs);
    if (vUsed[i] > VMAX_HARD) vUsed[i] = VMAX_HARD;
    if (vUsed[i] < MOTOR_HEUR_V_MIN[i]) vUsed[i] = MOTOR_HEUR_V_MIN[i];
    if (aUsed[i] > ACC_HARD) aUsed[i] = ACC_HARD;
    if (aUsed[i] < MOTOR_HEUR_A_MIN[i]) aUsed[i] = MOTOR_HEUR_A_MIN[i];

    stepper[i].setMaxSpeed(vUsed[i]);
    stepper[i].setAcceleration(aUsed[i]);
    stepper[i].moveTo(tgtPos);
  }

  // 4) Intégration
  for (uint8_t i = 0; i < NBMOTEURS; i++) stepper[i].run();

  // 5) Envois périodiques
  unsigned long nowMs2 = millis();
  if (nowMs2 - lastInMs >= PRINT_INTERVAL_MS) { lastInMs = nowMs2;
    // maybeSendINSerial(); 
    }
  if (nowMs2 - lastOutMs >= PRINT_INTERVAL_MS) { lastOutMs = nowMs2;
   //  maybeSendOUTSerial(); }
  if (nowMs2 - lastDistMs >= PRINT_INTERVAL_MS) { lastDistMs = nowMs2;
    // maybeSendDISTSerial();
   }

  // mise à jour positions de référence pour la prochaine détection
  for (uint8_t i = 0; i < NBMOTEURS; ++i) lastLoopPosition[i] = stepper[i].currentPosition();
}