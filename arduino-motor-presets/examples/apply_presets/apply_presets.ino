#include <Arduino.h>
#include "MotorPresets.h"

// Example motor presets
MotorPreset motorPresets[] = {
    {1, 200, 1000, 500},  // Motor 1: ID, Max Speed, Acceleration, Steps per Revolution
    {2, 300, 1500, 500},  // Motor 2: ID, Max Speed, Acceleration, Steps per Revolution
    {3, 400, 2000, 500},  // Motor 3: ID, Max Speed, Acceleration, Steps per Revolution
    {4, 500, 2500, 500},  // Motor 4: ID, Max Speed, Acceleration, Steps per Revolution
};

void setup() {
    Serial.begin(115200);
    // Apply presets to each motor
    for (int i = 0; i < sizeof(motorPresets) / sizeof(motorPresets[0]); i++) {
        applyMotorPreset(motorPresets[i]);
    }
}

void loop() {
    // Main loop can be used for motor control logic
}