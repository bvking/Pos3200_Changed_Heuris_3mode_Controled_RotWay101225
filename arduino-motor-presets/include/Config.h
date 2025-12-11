#ifndef CONFIG_H
#define CONFIG_H

// Motor pin assignments
const int MOTOR_COUNT = 10;
const int MOTOR_PINS[MOTOR_COUNT][2] = {
    {2, 3},   // Motor 1: {step pin, direction pin}
    {4, 5},   // Motor 2
    {6, 7},   // Motor 3
    {8, 9},   // Motor 4
    {10, 11}, // Motor 5
    {12, 13}, // Motor 6
    {A0, A1}, // Motor 7
    {A2, A3}, // Motor 8
    {A4, A5}, // Motor 9
    {A6, A7}  // Motor 10
};

// Motor parameters
const float DEFAULT_MAX_SPEED = 1000.0; // Default maximum speed for motors
const float DEFAULT_ACCELERATION = 500.0; // Default acceleration for motors

#endif // CONFIG_H