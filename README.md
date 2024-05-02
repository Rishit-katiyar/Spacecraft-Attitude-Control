# 🚀 Spacecraft Attitude Control System 🛰️

Welcome to the Spacecraft Attitude Control System repository! This project provides an Arduino-based implementation of an attitude control system for small-scale spacecraft, featuring stabilization, pointing, and maneuvering capabilities using reaction wheels, thrusters, or magnetorquers.

## Overview

This repository contains code and documentation for building and implementing an attitude control system for small-scale spacecraft. The system utilizes Arduino microcontrollers, sensors (gyroscope, accelerometer, magnetometer), and actuators (reaction wheels, thrusters, or magnetorquers) to control the spacecraft's orientation in space.

## Features

- Stabilization: Keep the spacecraft oriented in a desired orientation.
- Pointing: Point the spacecraft towards a specific direction or target.
- Maneuvering: Perform controlled maneuvers for orbital adjustments or attitude changes.

## Getting Started

### Prerequisites

Before you begin, make sure you have the following installed:

- Arduino IDE: [Download and Install Arduino IDE](https://www.arduino.cc/en/software)
- Necessary Libraries: 
  - Adafruit BNO055: `Sketch -> Include Library -> Manage Libraries -> Search for "Adafruit BNO055" -> Install`
  - (Add any other libraries here if needed)

### Installation

1. Clone the repository to your local machine:

    ```bash
    git clone https://github.com/your-username/Spacecraft-Attitude-Control.git
    ```

2. Open the Arduino IDE and navigate to `File -> Open` and select the `attitude_control.ino` file from the cloned repository.

3. Upload the sketch to your Arduino board by selecting `Sketch -> Upload`.

### Hardware Setup

1. Connect the components as shown in the following circuit diagram:

    ```
        +---------------------------------------------+
        |                   Arduino                   |
        +---------------------------------------------+
        |                +------------------------+   |
        |                |       BNO055           |   |
        |                |      Sensor Module     |   |
        |                +-----------+------------+   |
        |                            |                |
        |                      SDA, SCL (I2C)        |
        |                            |                |
        |                            |                |
        |                            |                |
        |                +-----------v------------+   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                |                        |   |
        |                +------------------------+   |
        |                                             |
        |                Reaction Wheel               |
        |                Control Circuit             |
        |                                             |
        |                  +----------+               |
        |                  |   PWM    |               |
        |                  |  Output  |               |
        |                  +----+-----+               |
        |                       |                     |
        |                       | Motor Power        |
        |                       v                     |
        |                  +----+-----+               |
        |                  |    DC    |               |
        |                  |  Motor   |               |
        |                  +----------+               |
        |                                             |
        |                                             |
        |                Thruster                     |
        |                Control Circuit             |
        |                                             |
        |                  +----------+               |
        |                  |   PWM    |               |
        |                  |  Output  |               |
        |                  +----+-----+               |
        |                       |                     |
        |                       | Motor Power        |
        |                       v                     |
        |                  +----+-----+               |
        |                  |    DC    |               |
        |                  |  Motor   |               |
        |                  +----------+               |
        |                                             |
        |                Magnetorquer                 |
        |                Control Circuit             |
        |                                             |
        |                  +----------+               |
        |                  |   PWM    |               |
        |                  |  Output  |               |
        |                  +----+-----+               |
        |                       |                     |
        |                       | Current            |
        |                       v                     |
        |                  +----+-----+               |
        |                  |   Coil   |               |
        |                  |         |               |
        |                  +----------+               |
        |                                             |
        +---------------------------------------------+
    ```

2. Ensure proper power supply and ground connections for all components.

## Usage

1. Upload the `attitude_control.ino` sketch to your Arduino board.

2. Power on the spacecraft and monitor the serial output for sensor data and control outputs.

3. Send commands from your ground control system to adjust the spacecraft's orientation and perform maneuvers as needed.

## Contributing

Contributions are welcome! Feel free to open issues or pull requests for bug fixes, improvements, or new features.

## License

This project is licensed under the GNU General Public License v3.0. See the [LICENSE.txt](LICENSE.txt) file for details.
