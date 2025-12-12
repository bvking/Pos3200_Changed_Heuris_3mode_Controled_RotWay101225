# Pos3200_Changed_Heuris_3mode_Controled_RotWay101225

## Project Overview

This project is designed to control multiple stepper motors using the Arduino platform. It utilizes the AccelStepper library for precise motor control and implements dynamic motion profiles based on user-defined presets. The system includes safety features such as an emergency stop mechanism and serial communication for real-time monitoring and control.

## Key Components

- **Main Program**: 
  - `Pos3200_Changed_Heuris_3mode_Controled_RotWay101225.ino`: The main Arduino sketch that initializes the motor control system, sets up parameters, and handles communication.

- **Motor Presets**:
  - `MotorPresetsBridge.h`: Contains definitions and functions for loading and managing motor presets, allowing for flexible motor configurations.

- **Motor Control Library**:
  - `lib/AccelStepper/AccelStepper.h`: The header file for the AccelStepper library, providing an interface for controlling stepper motors with methods for speed, acceleration, and movement management.

- **Configuration**:
  - `platformio.ini`: Configuration file for PlatformIO, specifying the project environment, libraries, and build settings.

## Setup Instructions

1. **Install PlatformIO**: Ensure you have PlatformIO installed in your development environment.

2. **Clone the Repository**: Clone this project repository to your local machine.

3. **Open the Project**: Open the project folder in PlatformIO.

4. **Install Dependencies**: PlatformIO will automatically install the required libraries specified in `platformio.ini`.

5. **Upload the Program**: Connect your Arduino board and upload the program using the PlatformIO interface.

## Usage Guidelines

- **Serial Communication**: The system communicates via serial input. Ensure your serial monitor is set to the correct baud rate (115200) to interact with the program.

- **Emergency Stop**: The program includes an emergency stop feature that can be triggered via serial commands. Refer to the code comments for specific command values.

- **Motor Configuration**: Modify the motor presets in `MotorPresetsBridge.h` to customize motor behavior according to your application needs.

## Additional Information

This project is designed for flexibility and scalability, allowing for the control of multiple motors with varying configurations. The dynamic motion profiles enable smooth operation tailored to specific tasks, while the emergency stop feature ensures safety during operation. 

For further assistance or contributions, please refer to the project's issue tracker or contact the project maintainer.