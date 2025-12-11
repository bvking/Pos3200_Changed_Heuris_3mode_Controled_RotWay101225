#include <AccelStepper.h>
#include "MotorPresets.h"

// Number of motors
#define NBMOTEURS 10

// Motor instances
AccelStepper steppers[NBMOTEURS];

// Motor presets
MotorPreset motorPresets[NBMOTEURS];

// Function to initialize motor presets
void initializeMotorPresets() {
    for (int i = 0; i < NBMOTEURS; i++) {
        motorPresets[i].maxSpeed = 1000.0f; // Example value
        motorPresets[i].acceleration = 500.0f; // Example value
        motorPresets[i].stepPin = 3 + i; // Example pin assignment
        motorPresets[i].dirPin = 2 + i; // Example pin assignment
        steppers[i] = AccelStepper(AccelStepper::DRIVER, motorPresets[i].stepPin, motorPresets[i].dirPin);
        steppers[i].setMaxSpeed(motorPresets[i].maxSpeed);
        steppers[i].setAcceleration(motorPresets[i].acceleration);
    }
}

void setup() {
    Serial.begin(115200);
    initializeMotorPresets();
}

void loop() {
    for (int i = 0; i < NBMOTEURS; i++) {
        steppers[i].run();
    }
}