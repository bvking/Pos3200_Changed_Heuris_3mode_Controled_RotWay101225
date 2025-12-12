#ifndef MOTOR_PRESETS_BRIDGE_H
#define MOTOR_PRESETS_BRIDGE_H

#include <Arduino.h>

// Define motor preset structures
struct MotorPreset {
    float dMin;
    float dMax;
    float vMin;
    float vMax;
    float aMin;
    float aMax;
};

// Define the number of motors
const uint8_t NBMOTEURS = 10;

// Motor presets array
MotorPreset motorPresets[NBMOTEURS];

// Function to load motor presets
void loadPerMotorPresetsFromLibrary() {
    // Example presets for each motor
    motorPresets[0] = {10.0f, 1500.0f, 400.0f, 4000.0f, 200.0f, 600.0f};
    motorPresets[1] = {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f};
    motorPresets[2] = {5.0f, 2500.0f, 1200.0f, 12000.0f, 600.0f, 1200.0f};
    motorPresets[3] = {10.0f, 1500.0f, 400.0f, 4000.0f, 200.0f, 600.0f};
    motorPresets[4] = {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f};
    motorPresets[5] = {5.0f, 2500.0f, 1200.0f, 12000.0f, 600.0f, 1200.0f};
    motorPresets[6] = {10.0f, 1500.0f, 400.0f, 4000.0f, 200.0f, 600.0f};
    motorPresets[7] = {10.0f, 2000.0f, 800.0f, 8000.0f, 400.0f, 900.0f};
    motorPresets[8] = {5.0f, 2500.0f, 1200.0f, 12000.0f, 600.0f, 1200.0f};
    motorPresets[9] = {10.0f, 1500.0f, 400.0f, 4000.0f, 200.0f, 600.0f};
}

#endif // MOTOR_PRESETS_BRIDGE_H