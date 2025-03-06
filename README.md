
> Open this page at [https://edusharks.github.io/kiddo_blocks/](https://edusharks.github.io/kiddo_blocks/)

![](https://raw.githubusercontent.com/Edusharks/kiddo_blocks/3849ef750b6cd30a110a30ad01959e1b838ea8ac/icon.png)

## Use as Extension

This repository can be added as an **extension** in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **New Project**
* click on **Extensions** under the gearwheel menu
* search for **https://github.com/edusharks/kiddo_blocks** and import

## Edit this project

To edit this repository in MakeCode.

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **Import** then click on **Import URL**
* paste **https://github.com/edusharks/kiddo_blocks** and click import

#### Metadata (used for search, rendering)

* for PXT/microbit
<script src="https://makecode.com/gh-pages-embed.js"></script><script>makeCodeRender("{{ site.makecode.home_url }}", "{{ site.github.owner_name }}/{{ site.github.repository_name }}");</script>


///////////////////////////////////////////////////////////////////////////////////

Kiddo Extension for MakeCode
The Kiddo Extension is a custom MakeCode extension designed to simplify programming for the Kiddo robot. It provides blocks for controlling sensors, actuators, and other components commonly used in robotics projects. This extension is ideal for beginners and advanced users alike, offering an intuitive way to interact with hardware.

Table of Contents
Installation

Blocks Overview

Pins

Sensors

Actuators

Usage Examples

Contributing

License

Installation
Open MakeCode for micro:bit .

Click on Extensions in the toolbar.

Search for the GitHub link of this extension (e.g., https://github.com/Edusharks/kiddo_blocks).

Click on the extension to add it to your project.

Blocks Overview
Pins
These blocks allow you to read from and write to digital and analog pins.

Digital Pins
Read Digital Pin: Reads the value (0 or 1) from a specified digital pin.

blocks
Copy
kiddo.digitalRead(kiddoDigitalPin.P3)
Write Digital Pin: Writes a value (0 or 1) to a specified digital pin.

blocks
Copy
kiddo.digitalWrite(kiddoDigitalPin.P3, 1)
Analog Pins
Read Analog Pin: Reads an analog value (0–1023) from a specified analog pin.

blocks
Copy
kiddo.analogRead(kiddoAnalogPin.P3)
Write Analog Pin: Writes an analog value (0–1023) to a specified analog pin.

blocks
Copy
kiddo.analogWrite(AnalogPin.P3, 512)
Sensors
These blocks allow you to interact with various sensors.

Line Follower
Initialize Line Follower: Initializes the line follower sensors (default pins: P1 and P2).

blocks
Copy
kiddo.initializeLineFollower()
Read Right Sensor: Reads the value from the right line follower sensor (1 for black, 0 for white).

blocks
Copy
kiddo.readRightLineFollowerSensor()
Read Left Sensor: Reads the value from the left line follower sensor (1 for black, 0 for white).

blocks
Copy
kiddo.readLeftLineFollowerSensor()
Ultrasonic Sensor
Initialize Ultrasonic Sensor: Initializes the ultrasonic sensor (default pins: P8 for TRIG, P9 for ECHO).

blocks
Copy
kiddo.initializeUltrasonicSensor()
Detect Obstacle Distance: Measures the distance to an obstacle in centimeters or inches.

blocks
Copy
kiddo.detectObstacleDistance(Unit.Centimeters)
IR Receiver
Initialize IR Receiver: Initializes the IR receiver (default pin: P0).

blocks
Copy
kiddo.initializeIrReceiver()
On IR Button: Triggers an event when a specific IR button is pressed or released.

blocks
Copy
kiddo.onIrButton(IrButton.CH, IrButtonAction.Pressed, () => {})
IR Button Values: Returns the code of the last pressed IR button.

blocks
Copy
kiddo.irButton()
IR Datagram: Returns the IR datagram as a 32-bit hexadecimal string.

blocks
Copy
kiddo.irDatagram()
Actuators
These blocks allow you to control motors, LEDs, and servos.

DC Motors
Control Motor: Controls a specific motor (left or right) in a direction (forward or backward) at a given speed (0–100).

blocks
Copy
kiddo.controlMotor(Motor.R, Direction.Forward, 50)
Stop Motor: Stops a specific motor.

blocks
Copy
kiddo.stopMotor(Motor.R)
Stop All Motors: Stops both motors.

blocks
Copy
kiddo.stopAllMotors()
Move: Moves both motors forward or backward at the same speed.

blocks
Copy
kiddo.move(Direction.Forward, 50)
RGB LED
Initialize LED Strip: Initializes the NeoPixel LED strip (default pin: P12).

blocks
Copy
kiddo.initializeLedStrip()
Set LED Color: Sets the color of the entire LED strip.

blocks
Copy
kiddo.setLedColor(0xFF0000) // Red
Clear LED: Turns off all LEDs.

blocks
Copy
kiddo.clearLed()
Servo Motors
Move Positional Servo: Moves a positional servo to a specified angle (0–180 degrees).

blocks
Copy
kiddo.movePositionalServo(DigitalPin.P0, 90)
Move Continuous Servo: Controls the speed and direction of a continuous rotation servo.

blocks
Copy
kiddo.moveContinuousServo(DigitalPin.P0, 50)
Stop Continuous Servo: Stops a continuous rotation servo.

blocks
Copy
kiddo.stopContinuousServo(DigitalPin.P0)
Usage Examples
Line Following Robot
blocks
Copy
kiddo.initializeLineFollower()
basic.forever(() => {
    if (kiddo.readRightLineFollowerSensor() === 1) {
        kiddo.controlMotor(Motor.R, Direction.Forward, 50)
    } else {
        kiddo.controlMotor(Motor.R, Direction.Backward, 50)
    }
    if (kiddo.readLeftLineFollowerSensor() === 1) {
        kiddo.controlMotor(Motor.L, Direction.Forward, 50)
    } else {
        kiddo.controlMotor(Motor.L, Direction.Backward, 50)
    }
})
Obstacle Avoidance
blocks
Copy
kiddo.initializeUltrasonicSensor()
basic.forever(() => {
    let distance = kiddo.detectObstacleDistance(Unit.Centimeters)
    if (distance < 10) {
        kiddo.move(Direction.Backward, 50)
        basic.pause(500)
        kiddo.move(Direction.Left, 50)
        basic.pause(500)
    } else {
        kiddo.move(Direction.Forward, 50)
    }
})
IR Remote Control
blocks
Copy
kiddo.initializeIrReceiver()
kiddo.onIrButton(IrButton.CH, IrButtonAction.Pressed, () => {
    kiddo.move(Direction.Forward, 50)
})
kiddo.onIrButton(IrButton.CH, IrButtonAction.Released, () => {
    kiddo.stopAllMotors()
})
Contributing
Contributions are welcome! If you find a bug or have a feature request, please open an issue on the GitHub repository.

License
This project is licensed under the MIT License. See the LICENSE file for details.

Let me know if you need further assistance! 😊