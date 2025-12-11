#ifndef MOTOR_PRESETS_BRIDGE_H
#define MOTOR_PRESETS_BRIDGE_H

#include <Arduino.h>

// déclarations externes des tableaux présents dans le .ino
extern float MOTOR_HEUR_D_MIN[];
extern float MOTOR_HEUR_D_MAX[];
extern float MOTOR_HEUR_V_MIN[];
extern float MOTOR_HEUR_V_MAX[];
extern float MOTOR_HEUR_A_MIN[];
extern float MOTOR_HEUR_A_MAX[];

// structure pour presets par moteur
struct MotorHeurPreset {
  float dMin, dMax;
  float vMin, vMax;
  float aMin, aMax;
};

// valeurs par défaut (10 moteurs) — modifie ici si besoin
static const MotorHeurPreset PER_MOTOR_PRESETS[10] = {
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f},
  {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f}
};

// charge les presets dans les tableaux du sketch
inline void loadPerMotorPresetsFromLibrary() {
  for (uint8_t i = 0; i < 10; ++i) {
    MOTOR_HEUR_D_MIN[i] = PER_MOTOR_PRESETS[i].dMin;
    MOTOR_HEUR_D_MAX[i] = PER_MOTOR_PRESETS[i].dMax;
    MOTOR_HEUR_V_MIN[i] = PER_MOTOR_PRESETS[i].vMin;
    MOTOR_HEUR_V_MAX[i] = PER_MOTOR_PRESETS[i].vMax;
    MOTOR_HEUR_A_MIN[i] = PER_MOTOR_PRESETS[i].aMin;
    MOTOR_HEUR_A_MAX[i] = PER_MOTOR_PRESETS[i].aMax;
  }
}

#endif // MOTOR_PRESETS_BRIDGE_H