
# Kiddo Robot Extension

![Kiddo Robot](https://raw.githubusercontent.com/Edusharks/kiddo_blocks/3849ef750b6cd30a110a30ad01959e1b838ea8ac/icon.png)

This extension provides a comprehensive set of blocks for controlling the Kiddo Robot, including sensors, actuators, and other utilities. It is designed to work with the MakeCode editor and supports various functionalities such as motor control, sensor reading, IR remote control, and RGB LED control.

**Created by Edusharks** 

[Website](https://www.edusharks.com/)

---

## 🚀 Features
- **Motor Control:** Manage motors individually or simultaneously.
- **Sensor Integration:** Line follower and ultrasonic distance sensors.
- **IR Remote Control:** Receive and handle IR signals.
- **RGB LED Support:** Control NeoPixel LED strips with color options.
- **Servo Motors:** Control positional and continuous rotation servos.

---

## 🛠️ Installation

* open [https://makecode.microbit.org/](https://makecode.microbit.org/)
* click on **New Project**
* click on ** ➕Extensions** under the Block menu
* 🔍 search for **https://github.com/edusharks/kiddo_blocks** and import


## 📘 Usage

### Pins
- **Digital Read:** Get a digital value (0/1) from a pin.
  ```javascript
  Kiddo.digitalRead(Kiddo.kiddoDigitalPin.P3)
  ```
- **Digital Write:** Set a digital value (0/1) to a pin.
  ```javascript
  Kiddo.digitalWrite(Kiddo.kiddoDigitalPin.P3, 1)
  ```
- **Analog Read:** Get an analog value (0-1023) from a pin.
  ```javascript
  Kiddo.analogRead(Kiddo.kiddoAnalogPin.P3)
  ```
- **Analog Write:** Set an analog value (0-1023) to a pin.
  ```javascript
  Kiddo.analogWrite(AnalogPin.P3, 512)
  ```

---

## 🔍 Sensors

### Line Follower
- **Initialize Line Follower:**
  ```javascript
  Kiddo.initializeLineFollower()
  ```
- **Read Right Sensor:**
  ```javascript
  Kiddo.readRightLineFollowerSensor()
  ```
- **Read Left Sensor:**
  ```javascript
  Kiddo.readLeftLineFollowerSensor()
  ```

### Ultrasonic Sensor
- **Initialize Ultrasonic Sensor:**
  ```javascript
  Kiddo.initializeUltrasonicSensor()
  ```
- **Detect Obstacle Distance:**
  ```javascript
  Kiddo.detectObstacleDistance(Kiddo.Unit.Centimeters)
  ```

---

## 🔧 Actuators

### Motors
- **Control Motor:**
  ```javascript
  Kiddo.controlMotor(Kiddo.Motor.R, Kiddo.Direction.Forward, 50)
  ```
- **Stop Motor:**
  ```javascript
  Kiddo.stopMotor(Kiddo.Motor.R)
  ```
- **Move Both Motors:**
  ```javascript
  Kiddo.move(Kiddo.Direction.Forward, 50)
  ```

### RGB LED
- **Initialize LED Strip:**
  ```javascript
  Kiddo.initializeLedStrip()
  ```
- **Set LED Color:**
  ```javascript
  Kiddo.setLedColor(0xFF0000)
  ```
- **Clear LED:**
  ```javascript
  Kiddo.clearLed()
  ```

### Servo Motors
- **Move Positional Servo:**
  ```javascript
  Kiddo.movePositionalServo(DigitalPin.P0, 90)
  ```
- **Move Continuous Servo:**
  ```javascript
  Kiddo.moveContinuousServo(DigitalPin.P0, 50)
  ```
- **Stop Continuous Servo:**
  ```javascript
  Kiddo.stopContinuousServo(DigitalPin.P0)
  ```

---

## 🚀 Examples

### Line Following Robot
```javascript
Kiddo.initializeLineFollower()
basic.forever(function () {
    if (Kiddo.readLeftLineFollowerSensor() === 1 && Kiddo.readRightLineFollowerSensor() === 1) {
        Kiddo.move(Kiddo.Direction.Forward, 50)
    } else if (Kiddo.readLeftLineFollowerSensor() === 1) {
        Kiddo.controlMotor(Kiddo.Motor.R, Kiddo.Direction.Forward, 50)
        Kiddo.stopMotor(Kiddo.Motor.L)
    } else if (Kiddo.readRightLineFollowerSensor() === 1) {
        Kiddo.controlMotor(Kiddo.Motor.L, Kiddo.Direction.Forward, 50)
        Kiddo.stopMotor(Kiddo.Motor.R)
    } else {
        Kiddo.stopAllMotors()
    }
})
```

### IR Remote Controlled Robot
```javascript
Kiddo.initializeIrReceiver()
Kiddo.onIrButton(Kiddo.IrButton.CH, Kiddo.IrButtonAction.Pressed, function () {
    Kiddo.move(Kiddo.Direction.Forward, 50)
})
Kiddo.onIrButton(Kiddo.IrButton.CH, Kiddo.IrButtonAction.Released, function () {
    Kiddo.stopAllMotors()
})
```

---

## 📝 License
This project is licensed under the **MIT License**. See the `LICENSE` file for details.

---

## 🙌 Contributing
Contributions are welcome! If you have improvements or new features to add, feel free to submit a pull request or open an issue.

Happy coding with **Kiddo Robot**! 🚀🤖

---

Got any adjustments in mind or new features you want to highlight? Let us know! ✨

