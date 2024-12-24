# Distance-Based Stepper Motor Control with ToF Sensor
This repository contains minimal examples for testing and experimenting with various components, such as Time-of-Flight (ToF) sensors, stepper motors, and push buttons. The ultimate goal of this project is to control a stepper motor to maintain a reference distance from the ground based on ToF sensor readings.

## Overview
The project demonstrates how to interface and use a ToF sensor to measure the distance from a mounting point, and how to control a stepper motor to adjust its position so that it maintains a constant reference distance from the ground. These examples are modular, allowing you to test and integrate each component individually and together.

## Key Features:
* __ToF Sensor Integration__: Interface with a ToF sensor to measure distance from the ground.
* __Stepper Motor Control__: Use a stepper motor to adjust the position of the object.
* __Push Button Integration__: Implement push buttons for emergency stop.
* __Modular Examples__: Examples for individual testing of components like the ToF sensor, stepper motor, and buttons.

## Installation
 1. Clone the repository:
```
git clone https://github.com/yourusername/stepper-motor-tof-sensor.git
cd stepper-motor-tof-sensor
```
2. Install the required libraries:
    1. AccelStepper for stepper motor control


3. Open the desired example from the examples folder. Upload it to your microcontroller.

## Sketches
The repository contains minimal examples for each component. Below is an overview of each example:

1. `accelstepper_minimal_example`: Moves a stepper motor to a target position and reverses direction using AccelStepper.
2. `accelstepper_motor_sensor`: Controls motor position dynamically based on distance readings from a TFMini-S sensor.
3. `button_interrupt`: Adds a limit switch and emergency stop functionality to motor control.
4. `button_interrupt_check`: Uses a button interrupt to stop and restart the motor manually.
5. `button_stop`: Toggles motor operation with a button while responding to distance thresholds.
6. `limit_switch`: Implements motor control with ToF data and a button to toggle ON/OFF states.
7. `limit_switch_accelstepper`: Implements a stepper motor control system using the AccelStepper library, integrating a TFMini-S ToF sensor for distance-based motor positioning with emergency stop and limit switch functionality.
8. `mk2_sensor_motor_minimal_example`: Provides basic stepper motor control with TFMini-S ToF sensor distance readings, performing single steps based on threshold comparisons.
9. `ros2_server`: Introduces ROS 2 integration for microcontrollers, enabling real-time communication and action handling for motor and sensor systems.
10. `send_serial_data`: Combines stepper motor control, sensor feedback, and serial communication to achieve coordinated robotic actuation.
11. `sensor_motor_minimal_example`: Enables continuous bidirectional rotation of a stepper motor, with dynamic delay adjustments for speed control in each direction.
12. `stepper_minimal_example`: Extends stepper motor functionality with fine-tuned control over angular movement and speed through programmable microsecond delays.
13. `TOF_minimal_example`: Reads distance data from a TFMini-S ToF sensor via UART, validating with checksums and converting raw data into centimeter measurements.


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

## License
For open source projects, say how it is licensed.


# Markdown syntax guide

## Headers

# This is a Heading h1
## This is a Heading h2
###### This is a Heading h6

## Emphasis

*This text will be italic*  
_This will also be italic_

**This text will be bold**  
__This will also be bold__

_You **can** combine them_

## Lists

### Unordered

* Item 1
* Item 2
* Item 2a
* Item 2b
    * Item 3a
    * Item 3b

### Ordered

1. Item 1
2. Item 2
3. Item 3
    1. Item 3a
    2. Item 3b

## Images

![This is an alt text.](/image/sample.webp "This is a sample image.")

## Links

You may be using [Markdown Live Preview](https://markdownlivepreview.com/).

## Blockquotes

> Markdown is a lightweight markup language with plain-text-formatting syntax, created in 2004 by John Gruber with Aaron Swartz.
>
>> Markdown is often used to format readme files, for writing messages in online discussion forums, and to create rich text using a plain text editor.

## Tables

| Left columns  | Right columns |
| ------------- |:-------------:|
| left foo      | right foo     |
| left bar      | right bar     |
| left baz      | right baz     |

## Blocks of code

```
let message = 'Hello world';
alert(message);
```

## Inline code

This web site is using `markedjs/marked`.
