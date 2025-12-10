#include <AccelStepper.h>

// ...existing code...

// ===================== CONFIG GÉNÉRALE =====================
#define NBMOTEURS    10
#define NBDATA       34      //
#define NBPASPARTOUR 3200
#define STEP_DRIVER  AccelStepper::DRIVER

// ✨ Index de la donnée de profil dans ABC
const uint8_t PROFILE_DATA_INDEX = NBDATA - 1;  // = 33

// ===================== PINS MOTEURS =====================
// Adapter à ton câblage réel.
const uint8_t PINDIRECTION[NBMOTEURS] = {6, 9, 12, 26, 29, 32, 34, 37, 39, 41};
const uint8_t PINSPEED[NBMOTEURS]    = {5, 8, 11, 25, 28, 31, 33, 36, 38, 40};

// Enable par moteur (ou -1 si pas utilisé)
const int8_t ENABLEPIN[NBMOTEURS] = {
  4, 7, 10, 24, 27, 30, 35, 38, 39, 42
};

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
// Actual way of motors, depanding of cablage -1,-1,+1,-1,-1,-1,+1,+1,-1,-1
int8_t DIR_SIGN[NBMOTEURS] = {
 // +1, +1, +1, +1, +1, +1, +1, +1, +1, +1
  -1, -1, +1, -1, -1, -1, +1, +1, -1, -1
};

// ===================== PROFIL VITESSE / ACCEL =====================

const float VMAX_HARD       = 12800.0f;
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
const long EMERGENCY_STEP_THRESHOLD = 3200;    // seuil déclenchement (steps)
const float EMERGENCY_V_TARGET      = 320.0f;  // vitesse cible (steps/s)
const float EMERGENCY_A_SCALE       = 1.0f;    // EM_A = EMERGENCY_V_TARGET * EMERGENCY_A_SCALE
const unsigned long EMERGENCY_RAMP_MS = 3000;  // durée rampe (ms)

// ✨ Paramètres heuristiques DIST→V/A (changés via presets)
float HEUR_D_MIN;   // distance min
float HEUR_D_MAX;   // distance max

float HEUR_V_MIN;   // vitesse min
float HEUR_V_MAX;   // vitesse max

float HEUR_A_MIN;   // accel min
float HEUR_A_MAX;   // accel max

// ✨ Index symboliques de profils
const uint8_t PROFILE_SOFT_IDX    = 0;
const uint8_t PROFILE_MEDIUM_IDX  = 1;
const uint8_t PROFILE_NERVOUS_IDX = 2;

// ✨ Structure pour les presets
struct MotionProfileParams {
  float dMin;
  float dMax;
  float vMin;
  float vMax;
  float aMin;
  float aMax;
};

// ✨ 3 PRESETS : SOFT / MEDIUM / NERVOUS
const MotionProfileParams PROFILE_SOFT = {
  10.0f,   // dMin
  1500.0f, // dMax
  400.0f,  // vMin (très doux)
  4000.0f, // vMax (pas trop rapide)
  200.0f,  // aMin
  600.0f   // aMax (accél. modérées)
};

const MotionProfileParams PROFILE_MEDIUM = {
  10.0f,   // dMin
  2000.0f, // dMax
  800.0f,  // vMin
  8000.0f, // vMax
  400.0f,  // aMin
  900.0f   // aMax
};

const MotionProfileParams PROFILE_NERVOUS = {
  5.0f,    // dMin (réagit même pour petites erreurs)
  2500.0f, // dMax
  1200.0f, // vMin (jamais très lent)
  12000.0f,// vMax (très rapide, clampé par VMAX_HARD)
  600.0f,  // aMin
  1200.0f  // aMax (maxi, clamp = ACC_HARD)
};

// ✨ profil courant (modifiable par Max en temps réel)
uint8_t currentProfile = PROFILE_MEDIUM_IDX;

// ✨ Applique un preset aux variables heuristiques
void applyMotionProfile(uint8_t profileIndex) {
  const MotionProfileParams* p;

  switch (profileIndex) {
    case PROFILE_SOFT_IDX:
      p = &PROFILE_SOFT;
      break;
    case PROFILE_NERVOUS_IDX:
      p = &PROFILE_NERVOUS;
      break;
    case PROFILE_MEDIUM_IDX:
    default:
      p = &PROFILE_MEDIUM;
      break;
  }

  HEUR_D_MIN = p->dMin;
  HEUR_D_MAX = p->dMax;
  HEUR_V_MIN = p->vMin;
  HEUR_V_MAX = p->vMax;
  HEUR_A_MIN = p->aMin;
  HEUR_A_MAX = p->aMax;
}

float vUsed[NBMOTEURS];
float aUsed[NBMOTEURS];
int   lastDir[NBMOTEURS];
bool  inFlip[NBMOTEURS];
unsigned long flipStartMs[NBMOTEURS];

long  targetPos[NBMOTEURS];        // repère interne
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
const unsigned long PRINT_INTERVAL_MS = 25;

// ===================== RÉCEPTION RAW AVEC <...> =====================
// Réception renforcée : timeout de trame, protection débordement buffer,
// null-termination sûre.
void recvWithStartEndMarkers() {
  static bool recvInProgress = false;
  static byte ndx = 0;
  static unsigned long frameStartMs = 0;
  const char startMarker = '<';
  const char endMarker   = '>';
  const unsigned long FRAME_TIMEOUT_MS = 50; // abort incomplete frames after this

  while (Serial.available() > 0 && newData == false) {
    char rc = (char)Serial.read();
    if (rc == '\r' || rc == '\n') continue;  // on ignore CR/LF

    if (!recvInProgress) {
      if (rc == startMarker) {
        recvInProgress = true;
        ndx = 0;
        frameStartMs = millis();
      }
    } else {
      // abort if frame too old
      if (millis() - frameStartMs > FRAME_TIMEOUT_MS) {
        recvInProgress = false;
        ndx = 0;
        continue;
      }
      if (rc == endMarker) {
        // finish frame safely
        if (ndx >= numChars) ndx = numChars - 1;
        receivedChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true; // trame complète prête
        break;
      } else {
        if (ndx < numChars - 1) {
          receivedChars[ndx++] = rc;
        } else {
          // buffer full -> abort this frame
          recvInProgress = false;
          ndx = 0;
        }
      }
    }
  }
}

// ===================== ENVOI TYPE 0 : IN (ABC) =====================
// <0, ABC0, ABC1, ..., ABC32>
void maybeSendINSerial() {
  bool changed = false;

  if (!lastInValid) {
    changed = true;
  } else {
    for (uint8_t i = 0; i < NBDATA; i++) {
      if (ABC[i] != lastABC[i]) {
        changed = true;
        break;
      }
    }
  }

  if (!changed) return;

  for (uint8_t i = 0; i < NBDATA; i++) {
    lastABC[i] = ABC[i];
  }
  lastInValid = true;

  Serial.print('0'); // type 0 = IN
  for (uint8_t i = 0; i < NBDATA; i++) {
    Serial.print(' ');
    Serial.print(ABC[i]);
  }
  Serial.println();
}

// ===================== ALL AT TARGET ? =====================
bool allMotorsAtTarget() {
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long d = stepper[i].distanceToGo();
    if (d < 0) d = -d;
    if (d >= 1) return false;
  }
  return true;
}

// ===================== ENVOI TYPE 2 : DIST =====================
// <2, dist0, dist1, ..., dist9>
void maybeSendDISTSerial() {
  if (allMotorsAtTarget()) return;

  Serial.print('2'); // type 2 = DIST
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long distInt  = stepper[i].distanceToGo();      // repère interne
    long distReal = DIR_SIGN[i] * distInt;          // repère Max
    Serial.print(' ');
    Serial.print(distReal);
  }
  Serial.println();
}

// ===================== ENVOI TYPE 1 : OUT =====================
// <1, pos0,v0,a0, pos1,v1,a1, ..., pos9,v9,a9>
void maybeSendOUTSerial() {
  if (allMotorsAtTarget()) return;

  Serial.print('1'); // type 1 = OUT
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long curInternal = stepper[i].currentPosition(); // repère interne
    long curReal     = DIR_SIGN[i] * curInternal;    // repère Max
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

// ===================== PARSE & APPLY (données venant de Max) =====================
// Parsing renforcé : strtol avec vérification, complétion si moins de tokens,
// application de profil et mise à jour des cibles.
void parseData() {
  // copie sûre
  strncpy(tempChars, receivedChars, numChars);
  tempChars[numChars - 1] = '\0';

  char *ptr = tempChars;
  char *endptr;
  uint8_t tokenIndex = 0;
  bool parseError = false;

  // parse tokens séparés par ','
  while (tokenIndex < NBDATA && *ptr != '\0') {
    long val = strtol(ptr, &endptr, 10);
    if (endptr == ptr) {
      // pas de conversion valide
      parseError = true;
      break;
    }
    ABC[tokenIndex++] = val;

    if (*endptr == ',') {
      ptr = endptr + 1;
    } else {
      // fin de la chaîne
      break;
    }
  }

  if (parseError) {
    // on rejette la trame malformée
    newData = false;
    return;
  }

  // Si moins de tokens fournis, remplir le reste à 0
  for (uint8_t i = tokenIndex; i < NBDATA; ++i) ABC[i] = 0;

  // ✨ 1) Profil de mouvement : ABC[PROFILE_DATA_INDEX] = 0 / 1 / 2
  long profRaw = ABC[PROFILE_DATA_INDEX];
  uint8_t requestedProfile;

  if (profRaw <= 0) {
    requestedProfile = PROFILE_SOFT_IDX;
  } else if (profRaw == 1) {
    requestedProfile = PROFILE_MEDIUM_IDX;
  } else {
    requestedProfile = PROFILE_NERVOUS_IDX;
  }

  if (requestedProfile != currentProfile) {
    currentProfile = requestedProfile;
    applyMotionProfile(currentProfile);   // ✨ changement de caractère global

    // (optionnel) on remet v/a utilisées dans la nouvelle plage min
    for (uint8_t i = 0; i < NBMOTEURS; i++) {
      if (vUsed[i] < HEUR_V_MIN) vUsed[i] = HEUR_V_MIN;
      if (aUsed[i] < HEUR_A_MIN) aUsed[i] = HEUR_A_MIN;
    }
  }

  // 2) Mettre à jour les cibles des 10 moteurs (repère interne)
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long rawPos    = ABC[i];               // repère Max
    long signedPos = DIR_SIGN[i] * rawPos; // repère interne AccelStepper

    long d = signedPos - lastStreamTarget[i];
    lastStreamTarget[i] = signedPos;

    // estimation vitesse consigne depuis Max
    lastStreamV[i] = (float)abs(d) / 0.025f;

    targetPos[i] = signedPos;
    stepper[i].moveTo(signedPos);
  }

  newData = false;
}

// ===================== HELPERS PROFIL =====================

float approachTimed(float currentVal, float targetVal,
                    float upRatePerS, float downRatePerS,
                    float dtMs)
{
  float upStep   = upRatePerS   * (dtMs / 1000.0f);
  float downStep = downRatePerS * (dtMs / 1000.0f);
  float diff     = targetVal - currentVal;

  if (diff > 0) {
    if (diff > upStep) diff = upStep;
  } else if (diff < 0) {
    if (diff < -downStep) diff = -downStep;
  }
  return currentVal + diff;
}

void getFlipRates(unsigned long flipAgeMs,
                  float &vUp, float &vDown,
                  float &aUp, float &aDown)
{
  vUp   = NORM_V_UP_PER_S;
  vDown = NORM_V_DOWN_PER_S;
  aUp   = NORM_A_UP_PER_S;
  aDown = NORM_A_DOWN_PER_S;

  if (flipAgeMs >= FLIP_TOTAL_MS) return;

  if (flipAgeMs < FLIP_EXP_MS) {
    float t = (float)flipAgeMs / (float)FLIP_EXP_MS;
    float k = 1.0f - t;
    float factor = FLIP_EXP_STRENGTH + (1.0f - FLIP_EXP_STRENGTH) * (k * k);
    vUp   *= factor;
    vDown *= factor;
    aUp   *= factor;
    aDown *= factor;
  } else {
    unsigned long linAge = flipAgeMs - FLIP_EXP_MS;
    float t = (float)linAge / (float)FLIP_LIN_MS;
    if (t > 1.0f) t = 1.0f;
    float factor = FLIP_LIN_STRENGTH + (1.0f - FLIP_LIN_STRENGTH) * t;
    vUp   *= factor;
    vDown *= factor;
    aUp   *= factor;
    aDown *= factor;
  }
}

// DIST → V vitesse (smoothstep + presets)
float computeSpeedFromDistance(long distAbs) {
  float d = (float)distAbs;

  if (d < HEUR_D_MIN) d = HEUR_D_MIN;
  if (d > HEUR_D_MAX) d = HEUR_D_MAX;

  float x = (d - HEUR_D_MIN) / (HEUR_D_MAX - HEUR_D_MIN);

  float f = x * x * (3.0f - 2.0f * x);  // smoothstep

  float v = HEUR_V_MIN + f * (HEUR_V_MAX - HEUR_V_MIN);

  if (v > VMAX_HARD) v = VMAX_HARD;
  if (v < HEUR_V_MIN) v = HEUR_V_MIN;

  return v;
}

// DIST → A accel (smoothstep + presets)
float computeAccelFromDistance(long distAbs) {
  float d = (float)distAbs;

  if (d < HEUR_D_MIN) d = HEUR_D_MIN;
  if (d > HEUR_D_MAX) d = HEUR_D_MAX;

  float x = (d - HEUR_D_MIN) / (HEUR_D_MAX - HEUR_D_MIN);

  float f = x * x * (3.0f - 2.0f * x);  // smoothstep

  float a = HEUR_A_MIN + f * (HEUR_A_MAX - HEUR_A_MIN);

  if (a > ACC_HARD) a = ACC_HARD;
  if (a < HEUR_A_MIN) a = HEUR_A_MIN;

  return a;
}

// ===================== SETUP =====================

unsigned long lastLoopMs = 0;

void setup() {
  Serial.begin(115200);   // lien avec Max

  // ✨ Appliquer le profil par défaut au démarrage (MEDIUM)
  applyMotionProfile(currentProfile);

  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    stepper[i].setMinPulseWidth(5);

    if (ENABLEPIN[i] >= 0) {
      pinMode(ENABLEPIN[i], OUTPUT);
      digitalWrite(ENABLEPIN[i], LOW);
    }

    pinMode(PINDIRECTION[i], OUTPUT);
    pinMode(PINSPEED[i], OUTPUT);

    stepper[i].setMaxSpeed(HEUR_V_MIN);
    stepper[i].setAcceleration(HEUR_A_MIN);
   // stepper[i].setCurrentPosition(0);
    stepper[i].moveTo(1600);
    stepper[i].run();

    vUsed[i] = HEUR_V_MIN;
    aUsed[i] = HEUR_A_MIN;
    lastDir[i] = 0;
    inFlip[i]  = false;
    flipStartMs[i] = 0;
    targetPos[i]   = 0;
    lastStreamTarget[i] = 0;
    lastStreamV[i]      = 0;

    // initialisation positions pour détection
    lastLoopPosition[i] = stepper[i].currentPosition();

    // init emergency arrays
    emergencyVStart[i] = vUsed[i];
    emergencyAStart[i] = aUsed[i];
  }

  lastLoopMs = millis();
  unsigned long now = millis();
  lastInMs   = now;
  lastOutMs  = now;
  lastDistMs = now;

  lastInValid = false;
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
}

// ===================== LOOP =====================

void loop() {
  // 1) Réception depuis Max
  recvWithStartEndMarkers();
  if (newData) {
    parseData();
  }

  // 2) dt
  unsigned long nowMs = millis();
  float dtMs = (float)(nowMs - lastLoopMs);
  if (dtMs < 0)  dtMs = 0;
  if (dtMs > 50) dtMs = 50;
  lastLoopMs = nowMs;

    // --- Détection arrêt d'urgence : si déplacement suspect entre deux boucles ---
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    long curPos = stepper[i].currentPosition();
    long d = curPos - lastLoopPosition[i];
    if (d < 0) d = -d;
    if (!emergencyActive && d > EMERGENCY_STEP_THRESHOLD) {
      startEmergencyStop(nowMs);
      break; // suffit qu'un seul déclencheur
    }
  }

  // 3) Mise à jour profils
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    long curPos  = stepper[i].currentPosition();
    long tgtPos  = targetPos[i];
    long dist    = tgtPos - curPos;
    long distAbs = (dist >= 0) ? dist : -dist;

    int dirNow = 0;
    if (dist > 0)      dirNow = +1;
    else if (dist < 0) dirNow = -1;

     // si on est en arrêt d'urgence : on écrase les cibles v/a pour les ramper vers la cible d'urgence
    if (emergencyActive) {
      unsigned long elapsed = nowMs - emergencyStartMs;
      float t = (elapsed >= EMERGENCY_RAMP_MS) ? 1.0f : (float)elapsed / (float)EMERGENCY_RAMP_MS;

      // vitesse cible et accel cible d'urgence
      float emergencyA_target = EMERGENCY_V_TARGET * EMERGENCY_A_SCALE;
      if (emergencyA_target < 50.0f) emergencyA_target = 50.0f;
      if (emergencyA_target > ACC_HARD) emergencyA_target = ACC_HARD;

      // interpolation linéaire depuis les valeurs de départ
      vUsed[i] = emergencyVStart[i] + (EMERGENCY_V_TARGET - emergencyVStart[i]) * t;
      aUsed[i] = emergencyAStart[i] + (emergencyA_target - emergencyAStart[i]) * t;

      // clamps
      if (vUsed[i] > VMAX_HARD) vUsed[i] = VMAX_HARD;
      if (vUsed[i] < EMERGENCY_V_TARGET) vUsed[i] = EMERGENCY_V_TARGET;
      if (aUsed[i] > ACC_HARD) aUsed[i] = ACC_HARD;
      if (aUsed[i] < 10.0f) aUsed[i] = 10.0f;

      // forcer la consigne de mouvement (on garde la target position)
      stepper[i].setMaxSpeed(vUsed[i]);
      stepper[i].setAcceleration(aUsed[i]);
      stepper[i].moveTo(tgtPos);

      // continuer les autres moteurs (on ne fait pas 'continue' ici pour garder logique uniforme)
      continue;
    }

    // comportement normal (préexistant)

    float vTarget = computeSpeedFromDistance(distAbs);
    float aTarget = computeAccelFromDistance(distAbs);

    // feedforward depuis lastStreamV : plafond
    float vStream = lastStreamV[i];
    if (vStream > vTarget) {
      vTarget = vStream;
      if (aTarget < ACC_MIN_SOFT) aTarget = ACC_MIN_SOFT;
    }

    // gestion flip de direction
    if (dirNow != 0 && lastDir[i] != 0 && dirNow != lastDir[i]) {
      inFlip[i] = true;
      flipStartMs[i] = nowMs;
    }
    lastDir[i] = dirNow;

    float vUpRate   = NORM_V_UP_PER_S;
    float vDownRate = NORM_V_DOWN_PER_S;
    float aUpRate   = NORM_A_UP_PER_S;
    float aDownRate = NORM_A_DOWN_PER_S;

    if (inFlip[i]) {
      unsigned long flipAge = nowMs - flipStartMs[i];
      if (flipAge >= FLIP_TOTAL_MS) {
        inFlip[i] = false;
      } else {
        getFlipRates(flipAge, vUpRate, vDownRate, aUpRate, aDownRate);
        if (vTarget < VMIN_USEFUL)    vTarget = VMIN_USEFUL;
        if (aTarget < ACC_MIN_USEFUL) aTarget = ACC_MIN_USEFUL;
      }
    }

    vUsed[i] = approachTimed(vUsed[i], vTarget,
                             vUpRate, vDownRate, dtMs);
    aUsed[i] = approachTimed(aUsed[i], aTarget,
                             aUpRate, aDownRate, dtMs);

    if (vUsed[i] > VMAX_HARD) vUsed[i] = VMAX_HARD;
    if (vUsed[i] < HEUR_V_MIN) vUsed[i] = HEUR_V_MIN;

    if (aUsed[i] > ACC_HARD)  aUsed[i] = ACC_HARD;
    if (aUsed[i] < HEUR_A_MIN) aUsed[i] = HEUR_A_MIN;

    stepper[i].setMaxSpeed(vUsed[i]);
    stepper[i].setAcceleration(aUsed[i]);
    stepper[i].moveTo(tgtPos);
  }

  // 4) Intégration
  for (uint8_t i = 0; i < NBMOTEURS; i++) {
    stepper[i].run();
  }

  // 5) Envois périodiques
  unsigned long nowMs2 = millis();
  if (nowMs2 - lastInMs >= PRINT_INTERVAL_MS) {
    lastInMs = nowMs2;
    maybeSendINSerial();
  }

  if (nowMs2 - lastOutMs >= PRINT_INTERVAL_MS) {
    lastOutMs = nowMs2;
    maybeSendOUTSerial();
  }

  if (nowMs2 - lastDistMs >= PRINT_INTERVAL_MS) {
    lastDistMs = nowMs2;
    maybeSendDISTSerial();
  }

    // mise à jour positions de référence pour la prochaine détection
  for (uint8_t i = 0; i < NBMOTEURS; ++i) {
    lastLoopPosition[i] = stepper[i].currentPosition();
  }
}