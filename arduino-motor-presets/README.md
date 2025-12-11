# Arduino Motor Presets

This project provides a framework for managing and applying preset configurations for stepper motors using the Arduino platform. It includes a main sketch, header and source files for motor presets, and an example application demonstrating how to use the presets.

## Project Structure

- **src/**: Contains the main Arduino sketch and motor preset files.
  - **PresetMotorConfig.ino**: The main sketch that initializes motor configurations and applies presets.
  - **MotorPresets.h**: Header file defining the `MotorPreset` struct and constants for motor presets.
  - **MotorPresets.cpp**: Implementation of functions to load, apply, and manage motor presets.

- **include/**: Contains configuration constants and settings.
  - **Config.h**: Configuration constants such as pin assignments and motor parameters.

- **examples/**: Contains example sketches demonstrating the usage of the project.
  - **apply_presets/**: Example sketch for applying motor presets.
    - **apply_presets.ino**: Demonstrates how to use the motor presets.

- **lib/**: Contains libraries for enhanced motor control.
  - **AccelStepperWrapper/**: Wrapper for the AccelStepper library.
    - **AccelStepperWrapper.h**: Header file for the wrapper.
    - **AccelStepperWrapper.cpp**: Implementation of the wrapper methods.

- **platformio.ini**: Configuration file for PlatformIO, specifying environment settings and libraries.

- **.vscode/**: Contains settings for the Visual Studio Code environment.
  - **settings.json**: Settings for formatting and linting preferences.

- **README.md**: Documentation for the project, including setup instructions and usage examples.

- **LICENSE**: Licensing information for the project.

## Setup Instructions

1. Clone the repository to your local machine.
2. Open the project in your preferred IDE (e.g., PlatformIO, Arduino IDE).
3. Install the required libraries if prompted.
4. Upload the `PresetMotorConfig.ino` sketch to your Arduino board.
5. Use the example sketch in `examples/apply_presets/apply_presets.ino` to see how to apply motor presets.

## Usage

The project allows you to define various motor configurations as presets. You can easily switch between these presets in your application, enabling flexible motor control for different scenarios.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.