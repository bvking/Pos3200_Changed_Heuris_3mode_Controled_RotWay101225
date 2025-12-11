#ifndef MOTOR_PRESETS_H
#define MOTOR_PRESETS_H

#include <Arduino.h>

struct MotorPreset {
    float maxSpeed;      // Maximum speed for the motor
    float acceleration;  // Acceleration for the motor
    float deceleration;  // Deceleration for the motor
    int stepPin;        // Pin connected to the step signal
    int dirPin;         // Pin connected to the direction signal
    int enablePin;      // Pin to enable the motor driver
};

class MotorPresets {
public:
    MotorPresets();
    void applyPreset(int motorIndex);
    void loadPresets();
    void setPreset(int motorIndex, const MotorPreset& preset);
    MotorPreset getPreset(int motorIndex);

private:
    MotorPreset presets[10]; // Array to hold presets for 10 motors
};

#endif // MOTOR_PRESETS_H