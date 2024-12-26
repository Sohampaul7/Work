# Distance-Based Stepper Motor Control with ToF Sensor
This repository contains minimal examples for testing and experimenting with various components, such as Time-of-Flight (ToF) sensors, stepper motors, and push buttons. The final goal is to control a stepper motor to maintain a reference distance from the ground using ToF sensor readings, while using push buttons to send emergeny stop function. The reference value and any emergency request will use micro-ros communication, for simpler debugging we use serial communication.

## Overview
The project demonstrates how to interface and use a ToF sensor to measure the distance from a mounting point, and how to control a stepper motor to adjust its position so that it maintains a constant reference distance from the ground. These examples are modular, allowing you to test and integrate each component individually and together.

### Features
* __ToF Sensor Integration__: Interface with a ToF sensor to measure distance from the ground.
* __Stepper Motor Control__: Use a stepper motor to adjust the position of the object.
* __Push Button Integration__: Implement push buttons for emergency stop.
* __Serial Communication__: Examples for monitoring real-time data (e.g., distance readings, motor status) and for debugging. It is also necessary for integrating the system with external tools or frameworks like ROS 2.

## Project Directory Structure
```plaintext
.
├── examples/
│   ├── motor_control/
│   │   ├── accelstepper_minimal_example
│   │   ├── accelstepper_motor_sensor
│   │   ├── limit_switch_accelstepper
│   │   ├── mk2_sensor_motor_minimal_example
│   │   ├── sensor_motor_minimal_example
│   │   └── stepper_minimal_example
│   ├── button/
│   │   ├── button_interrupt
│   │   ├── button_interrupt_check
│   │   ├── button_stop
│   │   └── limit_switch
│   ├── sensor/
│   │   ├── TOF_minimal_example
│   └── communication/
│       ├── ros2_server
│       ├── send_serial_data
├── wiring_diagrams/
│   └── diagrams (placeholder for now)
├── README.md
```
## Installation
 1. Clone the repository:
```
git clone https://github.com/yourusername/stepper-motor-tof-sensor.git
cd stepper-motor-tof-sensor
```
2. Install the required library:
    * AccelStepper (for stepper motor control)


3. Open the desired example from the examples folder. Upload it to your microcontroller.

## Usage
### Suggested Workflow
1. Begin with the `TOF_minimal_example` to ensure the Time-of-Flight (ToF) sensor is reading distances correctly.
2. Test motor movement using `accelstepper_minimal_example`.
3. Combine ToF sensor feedback with stepper motor control with `mk2_sensor_motor_minimal_example`.
4. Implement emergency stop or toggle features using `button_interrupt` or `button_stop`.
5. Test integration of limit switch control using `limit_switch_accelstepper`.
6. Use `send_serial_data` to transmit sensor and motor data via serial communication.
7. Experiment with `ros2_server` to integrate the setup with ROS 2 for advanced communication and control.

## Examples Overview
| Example  | Purpose | Dependencies |
| ------------- |:-------------:| :-------------:|
| `TOF_minimal_example` | Reads distance data from the ToF sensor. | TFMini-S Sensor
| `accelstepper_minimal_example`| Tests basic stepper motor functionality.|AccelStepper Library
| `mk2_sensor_motor_minimal_example`| Combines ToF sensor readings with stepper motor control in a minimal example. |AccelStepper, TFMini-S
| `button_interrupt`| Integrates limit switch functionality with stepper motor and ToF sensor.|Push Buttons
| `limit_switch_accelstepper`| Implements an emergency stop function using button interrupts.|AccelStepper, TFMini-S,Push Buttons
| `send_serial_data`| Sends real-time sensor and motor data over serial communication.|Serial Communication Tools
| `ros2_server` | Demonstrates ROS 2 integration for real-time communication and control of sensors and motors.|ROS 2 Environment


## Usage
__Controlling Stepper Motor with ToF Sensor:__
1. The ToF sensor will measure the distance from the ground.
2. The stepper motor will adjust its position to maintain a reference distance.
3. You can modify the reference distance by changing the threshold in the code or via a serial monitor.

__Emergency stop with Push Button:__
You can use push buttons to halt the stepper motor's movements.

## Wiring diagram
-placeholder-

## Contributing
Feel free to contribute to the project by submitting pull requests. If you find any bugs or have suggestions for improvements, open an issue, and I’ll be happy to review it.

## Authors and acknowledgment
### Authors
- **Soham Paul**  
  Primary author of the code. Responsible for designing, implementing, and testing the core functionality.

### Acknowledgments
- **Eric Schoneberg**  
  Provided valuable guidance and insights during the development process.
- **Riyan Cyriac Jose**  
  Suggested optimizations for better performance and provided guidance throughout the project.

