# Distance-Based Stepper Motor Control with ToF Sensor
This repository contains minimal examples for testing and experimenting with various components, such as Time-of-Flight (ToF) sensors, stepper motors, and push buttons. The ultimate goal of this project is to control a stepper motor to maintain a reference distance from the ground based on ToF sensor readings.

## Overview
The project demonstrates how to interface and use a ToF sensor to measure the distance from a mounting point, and how to control a stepper motor to adjust its position so that it maintains a constant reference distance from the ground. These examples are modular, allowing you to test and integrate each component individually and together.

## Key Features:
* ToF Sensor Integration: Interface with a ToF sensor to measure distance from the ground.
* Stepper Motor Control: Use a stepper motor to adjust the position of the object.
* Push Button Integration: Implement push buttons for emergency stops.
* Modular Examples: Examples for individual testing of components like the ToF sensor, stepper motor, and buttons.

## Installation
### 1. Clone the repository:
```
git clone https://github.com/yourusername/stepper-motor-tof-sensor.git
cd stepper-motor-tof-sensor
```
### 2. Install the required libraries:
1. AccelStepper for stepper motor control


### 3. Upload the code:

Open the desired example from the examples folder. Upload it to your microcontroller.

## Examples
The repository contains minimal examples for each component:

* `ToF_Sensor_Test` : A basic example to read and print distance data from the ToF sensor.
* `Stepper_Motor_Control`: Controls the stepper motor based on distance measurements.
* `Push_Button_Control`: Integrates a push button to trigger specific actions.


## Usage
### Controlling Stepper Motor with ToF Sensor:

1. The ToF sensor will measure the distance from the ground.
2. The stepper motor will adjust its position to maintain a reference distance.
3. You can modify the reference distance by changing the threshold in the code or via external input like a button.

### Emergency stop with Push Button:
You can use push buttons to halt the stepper motor's movements.

## Wiring diagram
-placeholder-

## Contributing
Feel free to contribute to the project by submitting pull requests. If you find any bugs or have suggestions for improvements, open an issue, and I’ll be happy to review it.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.

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
