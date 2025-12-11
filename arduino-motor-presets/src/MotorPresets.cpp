#include "MotorPresets.h"

MotorPreset motorPresets[] = {
    {1000, 2000, 500, 1000},  // Preset for Motor 1
    {1500, 2500, 600, 1200},  // Preset for Motor 2
    {2000, 3000, 700, 1400},  // Preset for Motor 3
    {2500, 3500, 800, 1600},  // Preset for Motor 4
    {3000, 4000, 900, 1800},  // Preset for Motor 5
    {3500, 4500, 1000, 2000}, // Preset for Motor 6
    {4000, 5000, 1100, 2200}, // Preset for Motor 7
    {4500, 5500, 1200, 2400}, // Preset for Motor 8
    {5000, 6000, 1300, 2600}, // Preset for Motor 9
    {5500, 6500, 1400, 2800}  // Preset for Motor 10
};

void loadMotorPresets() {
    // Load presets from storage or initialize defaults
}

void applyMotorPreset(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= sizeof(motorPresets) / sizeof(motorPresets[0])) {
        return; // Invalid motor index
    }
    // Apply the preset configuration to the specified motor
}

void printMotorPresets() {
    for (int i = 0; i < sizeof(motorPresets) / sizeof(motorPresets[0]); i++) {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.print(": VMax=");
        Serial.print(motorPresets[i].vMax);
        Serial.print(", VMin=");
        Serial.print(motorPresets[i].vMin);
        Serial.print(", AMax=");
        Serial.print(motorPresets[i].aMax);
        Serial.print(", AMin=");
        Serial.println(motorPresets[i].aMin);
    }
}