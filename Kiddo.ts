/**
 * Custom blocks for Kiddo
 */
const enum IrButton {
    //% block="any"
    Any = -1,
}

//% weight=100 color=#993366 icon="\uf2db" block="Kiddo"
namespace KiddoRobot {

    // Define custom enums for digital pins
    export enum kiddoDigitalPin {
        //% block="P3"
        P3 = DigitalPin.P3,
        //% block="P4"
        P4 = DigitalPin.P4,
        //% block="P5"
        P5 = DigitalPin.P5,
        //% block="P6"
        P6 = DigitalPin.P6,
        //% block="P7"
        P7 = DigitalPin.P7,
        //% block="P10"
        P10 = DigitalPin.P10,
        //% block="P11"
        P11 = DigitalPin.P11,
        //% block="P19"
        P19 = DigitalPin.P19,
        //% block="P20"
        P20 = DigitalPin.P20
    }

    // Define custom enums for analog pins
    export enum kiddoAnalogPin {
        //% block="P3"
        P3 = AnalogPin.P3,
        //% block="P4"
        P4 = AnalogPin.P4,
        //% block="P10"
        P10 = AnalogPin.P10
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////
    //  PINS  //
    ////////////////////

    // Function for reading a digital value from a pin
    //% subcategory="Pins"
    //% group="Digital Pins"
    //% weight=150 blockGap=8
    //% blockId="digital_read"
    //% block="read digital pin %pin"
    export function digitalRead(pin: kiddoDigitalPin): number {
        return pins.digitalReadPin(pin);
    }

    // Function for writing a digital value to a pin
    //% subcategory="Pins"
    //% group="Digital Pins"
    //% blockId="digital_write"
    //% weight=140 blockGap=50
    //% block="write digital pin %pin |to %value"
    export function digitalWrite(pin: kiddoDigitalPin, value: number): void {
        pins.digitalWritePin(pin, value);
    }

    // Function for reading an analog value from a pin
    //% subcategory="Pins"
    //% group="Analog Pins"
    //% blockId="analog_read"
    //% weight=130 blockGap=8
    //% block="read analog pin %pin"
    export function analogRead(pin: kiddoAnalogPin): number {
        return pins.analogReadPin(pin);
    }

    // Function for writing an analog value to a pin
    //% subcategory="Pins"
    //% group="Analog Pins"
    //% weight=120 blockGap=8
    //% blockId="analog_write"
    //% block="write analog pin %pin |to %value"
    //% value.min=0 value.max=1023
    export function analogWrite(pin: kiddoAnalogPin, value: number): void {
        pins.analogWritePin(pin, value);
    }

    /////////////////////////////////////////////////////////////////////////////////

    ////////////////////
    //  Sensors //
    ////////////////////


    // IR Sensor 

    // Hardcoded sensor pins
    const RIGHT_SENSOR_PIN: AnalogPin = AnalogPin.P1; // Right sensor on P1
    const LEFT_SENSOR_PIN: AnalogPin = AnalogPin.P2;  // Left sensor on P2

    // Fixed thresholds for black and white surfaces
    const RIGHT_SENSOR_THRESHOLD: number = 5.2; // Fixed threshold for right sensor
    const LEFT_SENSOR_THRESHOLD: number = 5.2;  // Fixed threshold for left sensor

    /**
     * Initialize the line follower sensors with default pins (P1 and P2).
     */
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="initialize line follower"
    //% weight=100
    export function initializeLineFollower(): void {
        // No initialization logic needed since thresholds are fixed
    }

    /**
     * Read the sensor value from the right sensor.
     */
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="read right sensor"
    //% weight=90
    export function readRightLineFollowerSensor(): number {
        let rawValue = pins.analogReadPin(RIGHT_SENSOR_PIN);
        return (rawValue < RIGHT_SENSOR_THRESHOLD) ? 1 : 0; // 1 for black, 0 for white
    }

    /**
     * Read the sensor value from the left sensor.
     */
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="read left sensor"
    //% weight=80
    export function readLeftLineFollowerSensor(): number {
        let rawValue = pins.analogReadPin(LEFT_SENSOR_PIN);
        return (rawValue < LEFT_SENSOR_THRESHOLD) ? 1 : 0; // 1 for black, 0 for white
    }

    ///////////////////////////////////////////////////////////////////////////

    //Ultrasonic Sensor

    // Ultrasonic sensor pins
    const ULTRASONIC_TRIG_PIN = DigitalPin.P8; // Default TRIG pin
    const ULTRASONIC_ECHO_PIN = DigitalPin.P9; // Default ECHO pin

    // Enum for Distance Units
    export enum Unit {
        //% block="cm"
        Centimeters,
        //% block="inches"
        Inches
    }

    /**
     * Initialize the ultrasonic sensor.
     */
    //% subcategory="Sensors"
    //% group="Obstacle Detection"
    //% block="initialize ultrasonic sensor"
    //% weight=110
    export function initializeUltrasonicSensor(): void {
        // Configure TRIG pin as output
        pins.setPull(ULTRASONIC_TRIG_PIN, PinPullMode.PullNone);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0); // Set TRIG pin low initially

        // Configure ECHO pin as input
        pins.setPull(ULTRASONIC_ECHO_PIN, PinPullMode.PullNone);
    }

    /**
     * Detect obstacle distance using ultrasonic sensor
     * @param unit Desired distance unit (cm or inches)
     * @param maxCmDistance Maximum measurable distance in centimeters (default is 500)
     */
    //% subcategory="Sensors"
    //% group="Obstacle Detection"
    //% block="detect obstacle distance in %unit"
    //% weight=100
    //% unit.defl=Unit.Centimeters
    export function detectObstacleDistance(unit: Unit, maxCmDistance = 500): number {
        // Send pulse to trigger ultrasonic sensor
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0);
        control.waitMicros(2);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 1);
        control.waitMicros(10);
        pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0);

        // Read pulse duration from echo pin
        const d = pins.pulseIn(ULTRASONIC_ECHO_PIN, PulseValue.High, maxCmDistance * 58);

        // Handle case where no echo is received
        if (d === 0) {
            return -1; // Return -1 for no response
        }

        // Convert pulse duration to distance
        switch (unit) {
            case Unit.Centimeters:
                return Math.idiv(d, 58); // Convert to cm
            case Unit.Inches:
                return Math.idiv(d, 148); // Convert to inches
            default:
                return -1;
        }
    }

    /////////////////////////////////////////////////////////////////////////////

    //IR Receiver

    let irState: IrState;

    const IR_REPEAT = 256;
    const IR_INCOMPLETE = 257;
    const IR_DATAGRAM = 258;

    const REPEAT_TIMEOUT_MS = 120;

    interface IrState {
        hasNewDatagram: boolean;
        bitsReceived: uint8;
        addressSectionBits: uint16;
        commandSectionBits: uint16;
        hiword: uint16;
        loword: uint16;
        activeCommand: number;
        repeatTimeout: number;
        onIrButtonPressed: IrButtonHandler[];
        onIrButtonReleased: IrButtonHandler[];
        onIrDatagram: () => void;
    }

    class IrButtonHandler {
        irButton: IrButton;
        onEvent: () => void;

        constructor(irButton: IrButton, onEvent: () => void) {
            this.irButton = irButton;
            this.onEvent = onEvent;
        }
    }

    function appendBitToDatagram(bit: number): number {
        irState.bitsReceived += 1;

        if (irState.bitsReceived <= 8) {
            irState.hiword = (irState.hiword << 1) + bit;
        } else if (irState.bitsReceived <= 16) {
            irState.hiword = (irState.hiword << 1) + bit;
        } else if (irState.bitsReceived <= 32) {
            irState.loword = (irState.loword << 1) + bit;
        }

        if (irState.bitsReceived === 32) {
            irState.addressSectionBits = irState.hiword & 0xffff;
            irState.commandSectionBits = irState.loword & 0xffff;
            return IR_DATAGRAM;
        } else {
            return IR_INCOMPLETE;
        }
    }

    function decode(markAndSpace: number): number {
        if (markAndSpace < 1600) {
            // low bit
            return appendBitToDatagram(0);
        } else if (markAndSpace < 2700) {
            // high bit
            return appendBitToDatagram(1);
        }

        irState.bitsReceived = 0;

        if (markAndSpace < 12500) {
            // Repeat detected
            return IR_REPEAT;
        } else if (markAndSpace < 14500) {
            // Start detected
            return IR_INCOMPLETE;
        } else {
            return IR_INCOMPLETE;
        }
    }

    function enableIrMarkSpaceDetection(pin: DigitalPin) {
        pins.setPull(pin, PinPullMode.PullNone);

        let mark = 0;
        let space = 0;

        pins.onPulsed(pin, PulseValue.Low, () => {
            mark = pins.pulseDuration();
        });

        pins.onPulsed(pin, PulseValue.High, () => {
            space = pins.pulseDuration();
            const status = decode(mark + space);

            if (status !== IR_INCOMPLETE) {
                handleIrEvent(status);
            }
        });
    }

    function handleIrEvent(irEvent: number) {
        if (irEvent === IR_DATAGRAM || irEvent === IR_REPEAT) {
            irState.repeatTimeout = input.runningTime() + REPEAT_TIMEOUT_MS;
        }

        if (irEvent === IR_DATAGRAM) {
            irState.hasNewDatagram = true;

            if (irState.onIrDatagram) {
                control.inBackground(() => {
                    irState.onIrDatagram();
                });
            }

            const newCommand = irState.commandSectionBits >> 8;

            if (newCommand !== irState.activeCommand) {
                if (irState.activeCommand >= 0) {
                    const releasedHandler = irState.onIrButtonReleased.find(h => h.irButton === irState.activeCommand || IrButton.Any === h.irButton);
                    if (releasedHandler) {
                        control.inBackground(() => {
                            releasedHandler.onEvent();
                        });
                    }
                }

                const pressedHandler = irState.onIrButtonPressed.find(h => h.irButton === newCommand || IrButton.Any === h.irButton);
                if (pressedHandler) {
                    control.inBackground(() => {
                        pressedHandler.onEvent();
                    });
                }

                irState.activeCommand = newCommand;
            }
        }
    }

    function initIrState() {
        if (irState) {
            return;
        }

        irState = {
            hasNewDatagram: false,
            bitsReceived: 0,
            addressSectionBits: 0,
            commandSectionBits: 0,
            hiword: 0,
            loword: 0,
            activeCommand: -1,
            repeatTimeout: 0,
            onIrButtonPressed: [],
            onIrButtonReleased: [],
            onIrDatagram: undefined,
        };
    }

    /**
     * Initializes the IR receiver at pin P0.
     */
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% blockId=kiddo_infrared_initialize_receiver
    //% block="Initialize IR Receiver"
    //% weight=90
    export function initializeIrReceiver(): void {
        initIrState();
        enableIrMarkSpaceDetection(DigitalPin.P0); // Fixed to P0
        control.inBackground(() => {
            while (true) {
                notifyIrEvents();
                basic.pause(REPEAT_TIMEOUT_MS);
            }
        });
    }

    function notifyIrEvents() {
        if (irState.activeCommand === -1) {
            return;
        }

        const now = input.runningTime();
        if (now > irState.repeatTimeout) {
            const handler = irState.onIrButtonReleased.find(h => h.irButton === irState.activeCommand || IrButton.Any === h.irButton);
            if (handler) {
                control.inBackground(() => {
                    handler.onEvent();
                });
            }

            irState.bitsReceived = 0;
            irState.activeCommand = -1;
        }
    }

    /**
     * Returns the code of the IR button that was pressed last. Returns -1 (IrButton.Any) if no button has been pressed yet.
     */
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% blockId=kiddo_infrared_ir_button_pressed
    //% block="IR button Values"
    //% weight=70
    export function irButton(): number {
        basic.pause(0); // Yield to support background processing
        if (!irState) {
            return IrButton.Any;
        }
        return irState.commandSectionBits >> 8;
    }

    /**
     * Do something when an IR datagram is received.
     * @param handler body code to run when the event is raised
     */
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% blockId=kiddo_infrared_on_ir_datagram
    //% block="on IR datagram received"
    //% weight=40
    export function onIrDatagram(handler: () => void) {
        initIrState();
        irState.onIrDatagram = handler;
    }

    /**
     * Returns the IR datagram as a 32-bit hexadecimal string.
     */
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% blockId=kiddo_infrared_ir_datagram
    //% block="IR datagram"
    //% weight=30
    export function irDatagram(): string {
        basic.pause(0); // Yield to support background processing
        initIrState();
        return "0x" + ir_rec_to16BitHex(irState.addressSectionBits) + ir_rec_to16BitHex(irState.commandSectionBits);
    }

    function ir_rec_to16BitHex(value: number): string {
        let hex = "";
        for (let pos = 0; pos < 4; pos++) {
            let remainder = value % 16;
            if (remainder < 10) {
                hex = remainder.toString() + hex;
            } else {
                hex = String.fromCharCode(55 + remainder) + hex;
            }
            value = Math.idiv(value, 16);
        }
        return hex;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////


    ////////////////////
    //  Actuators //
    ////////////////////

    // DC Motor

    // Motor control pins
    const R_PIN1 = DigitalPin.P14;
    const R_PIN2 = DigitalPin.P13;
    const L_PIN1 = DigitalPin.P15;
    const L_PIN2 = DigitalPin.P16;

    // Define motor enum
    export enum Motor {
        //% block="R"
        R,
        //% block="L"
        L
    }

    // Define direction enum
    export enum Direction {
        //% block="forward"
        Forward,
        //% block="backward"
        Backward
    }

    // Define turn enum
    export enum Turn {
        //% block="left"
        Left,
        //% block="right"
        Right
    }

    // Control a specific motor in a direction at a given speed
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="motor %motor direction %direction at speed %speed"
    //% speed.min=0 speed.max=100
    //% weight=100

    export function controlMotor(motor: Motor, direction: Direction, speed: number): void {
        let pwmSpeed = Math.map(speed, 0, 100, 0, 1023);

        if (motor === Motor.R) {
            if (direction === Direction.Forward) {
                pins.analogWritePin(R_PIN1, pwmSpeed);
                pins.digitalWritePin(R_PIN2, 0);
            } else if (direction === Direction.Backward) {
                pins.digitalWritePin(R_PIN1, 0);
                pins.analogWritePin(R_PIN2, pwmSpeed);
            }
        } else if (motor === Motor.L) {
            if (direction === Direction.Forward) {
                pins.analogWritePin(L_PIN1, pwmSpeed);
                pins.digitalWritePin(L_PIN2, 0);
            } else if (direction === Direction.Backward) {
                pins.digitalWritePin(L_PIN1, 0);
                pins.analogWritePin(L_PIN2, pwmSpeed);
            }
        }
    }

    // Stop a specific motor
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="stop motor %motor"
    //% weight=90

    export function stopMotor(motor: Motor): void {
        if (motor === Motor.R) {
            pins.digitalWritePin(R_PIN1, 0);
            pins.digitalWritePin(R_PIN2, 0);
        } else if (motor === Motor.L) {
            pins.digitalWritePin(L_PIN1, 0);
            pins.digitalWritePin(L_PIN2, 0);
        }
    }

    // Stop all motors
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="stop all motors"
    //% weight=80

    export function stopAllMotors(): void {
        pins.digitalWritePin(R_PIN1, 0);
        pins.digitalWritePin(R_PIN2, 0);
        pins.digitalWritePin(L_PIN1, 0);
        pins.digitalWritePin(L_PIN2, 0);
    }

    // Move both motors forward or backward at the same speed
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="move %direction at speed %speed"
    //% speed.min=0 speed.max=100
    //% weight=60

    export function move(direction: Direction, speed: number): void {
        let pwmSpeed = Math.map(speed, 0, 100, 0, 1023);

        if (direction === Direction.Forward) {
            // Move forward: M1 and M2 forward
            pins.analogWritePin(R_PIN1, pwmSpeed);
            pins.digitalWritePin(R_PIN2, 0);
            pins.analogWritePin(L_PIN1, pwmSpeed);
            pins.digitalWritePin(L_PIN2, 0);
        } else if (direction === Direction.Backward) {
            // Move backward: M1 and M2 backward
            pins.digitalWritePin(R_PIN1, 0);
            pins.analogWritePin(R_PIN2, pwmSpeed);
            pins.digitalWritePin(L_PIN1, 0);
            pins.analogWritePin(L_PIN2, pwmSpeed);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////

    //RGB LED

    // NeoPixel LED strip pin (hardcoded to P12)
    const LED_PIN: DigitalPin = DigitalPin.P12; // LED pin is fixed to P12
    const NUM_LEDS = 8; // Number of LEDs in the strip

    // LED buffer
    let ledBuffer: Buffer = control.createBuffer(NUM_LEDS * 3); // Initialize buffer once


    /**
     * Initialize the NeoPixel LED strip and clear it by default.
     */
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="Kiddo initialize LED"
    //% weight=100

    export function initializeLedStrip(): void {
        // Clear the LED buffer (turn off all LEDs)
        ledBuffer.fill(0);

        // Reset the data line
        pins.digitalWritePin(LED_PIN, 0);
        control.waitMicros(50); // Wait for the strip to reset

        // Update the LED strip to apply the cleared state
        updateLedStrip();
    }

    /**
     * Set the color of the entire LED strip.
     * @param color The color to set the LED strip to (in RGB format).
     */
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="Kiddo set LED color %color"
    //% weight=90

    //% color.shadow="brightColorNumberPicker"
    export function setLedColor(color: number): void {
        // Extract RGB components from the color value
        let red = (color >> 16) & 0xFF;
        let green = (color >> 8) & 0xFF;
        let blue = color & 0xFF;

        // Set the color for each LED
        for (let i = 0; i < NUM_LEDS; i++) {
            let offset = i * 3;
            ledBuffer[offset] = green; // GRB format
            ledBuffer[offset + 1] = red;
            ledBuffer[offset + 2] = blue;
        }

        // Update the LED strip
        updateLedStrip();
    }

    /**
     * Clear the LED strip (turn off all LEDs).
     */
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="Kiddo clear LED"
    //% weight=80

    export function clearLed(): void {
        // Fill the buffer with zeros (turn off all LEDs)
        for (let i = 0; i < NUM_LEDS * 3; i++) {
            ledBuffer[i] = 0;
        }

        // Update the LED strip
        updateLedStrip();
    }

    // Helper function to update the LED strip with the current buffer data
    function updateLedStrip(): void {
        // Send the buffer data to the NeoPixel strip
        light.sendWS2812Buffer(ledBuffer, LED_PIN);
    }

    //% subcategory="Actuators"
    //% group="RGB LED"
    //% blockId=brightColorNumberPicker
    //% block="%value"
    //% shim=TD_ID
    //% colorSecondary="#FFFFFF"
    //% value.fieldEditor="colornumber"
    //% value.fieldOptions.decompileLiterals=true
    //% value.fieldOptions.colours='["#FF0000","#00FF00","#0000FF","#FFFF00","#00FFFF","#FF00FF","#FFFFFF","#000000"]'
    //% value.fieldOptions.columns=8
    //% value.fieldOptions.className='rgbColorPicker'
    export function __colorNumberPicker(value: number): number {
        return value;
    }


    ///////////////////////////////////////////////////////////////////////////////

    // Servo Motor
    
    function getDigitalPin(pin: kiddoDigitalPin): kiddoDigitalPin {
        // Return the pin directly (assuming it's a valid digital pin)
        return pin;
    }


    // Define servo positions enumeration
    export enum ServoPosition {
        //% block="0 degrees"
        Zero = 0,
        //% block="45 degrees"
        FortyFive = 45,
        //% block="90 degrees"
        Ninety = 90,
        //% block="135 degrees"
        OneThirtyFive = 135,
        //% block="180 degrees"
        OneEighty = 180
    }


    /**
    * Moves a servo to a specified position.
    * @param pin which pin to control
    * @param position the position to move to
    */
    //% subcategory="Actuators"
    //% weight=150 blockGap=8
    //% group="Positional Servo"
    //% blockId="move_positional_servo"
    //% block="move servo on pin %pin|to position %position"
    //% position.min=0 position.max=180
    export function movePositionalServo(pin: kiddoDigitalPin, position: number): void {
        const angle = Math.clamp(0, 180, position);  // Ensure angle is within 0-180 range
        pins.servoWritePin(getDigitalPin(pin), angle);
    }

    /**
     * Moves a servo to a specified position.
     * @param pin which pin to control
     * @param position the position to move to
     */
    //% subcategory="Actuators"
    //% weight=120 blockGap=8
    //% group="Positional Servo"
    //% blockId="move_positional_servo_to_fixed_point"
    //% block="move servo on pin %pin|to position %position"
    export function movePositionalServofixed(pin: kiddoDigitalPin, position: ServoPosition): void {
        // Using the ServoPosition enum, which already contains the angle values
        pins.servoWritePin(getDigitalPin(pin), position);  // position is directly the angle in degrees
    }

    //% subcategory="Actuators"
    //% weight=10 blockGap=8
    //% group="Positional Servo" 
    //% blockId="move_servo_from_to"
    //% block="move servo on pin %pin|from angle %from|to angle %to|over %duration seconds"
    //% from.min=0 from.max=180
    //% to.min=0 to.max=180
    //% duration.min=1 duration.max=10
    export function moveServoFromTo1(pin: kiddoDigitalPin, from: number, to: number, duration: number): void {
        const startAngle = Math.clamp(0, 180, from);
        const endAngle = Math.clamp(0, 180, to);
        const steps = Math.abs(endAngle - startAngle);
        const stepDuration = duration * 1000 / steps;

        // Loop through each step and move the servo
        for (let i = 0; i <= steps; i++) {
            // Calculate the current angle for the servo based on the direction of movement
            const currentAngle = startAngle + (endAngle > startAngle ? i : -i);

            // Ensure that we don't exceed the target angle, especially when rounding
            if ((endAngle > startAngle && currentAngle >= endAngle) || (endAngle < startAngle && currentAngle <= endAngle)) {
                // Set the final angle to the target and break the loop
                pins.servoWritePin(getDigitalPin(pin), endAngle);
                break; // exit the loop once we reach the target angle
            } else {
                // Otherwise, move the servo to the current calculated angle
                pins.servoWritePin(getDigitalPin(pin), currentAngle);
            }

            // Pause for the time before moving to the next step
            basic.pause(stepDuration);
        }
    }

    /**
     * Controls the speed and direction of a continuous rotation servo.
     * @param pin which pin to control
     * @param speed the speed of the servo motor (-100 to 100)
     */
    //% subcategory="Actuators"
    //% weight=40 blockGap=8
    //% group="Continuous Servo"
    //% blockId="move_continuous_servo"
    //% block="set continuous servo on pin %pin|to speed %speed"
    //% speed.min=-100 speed.max=100
    export function moveContinuousServo(pin: kiddoDigitalPin, speed: number): void {
        const speedValue = Math.clamp(-100, 100, speed);  // Ensure speed is within -100 to 100 range

        // Map speed to the correct PWM values (using -100 for full reverse and 100 for full forward)
        const pwmValue = Math.map(speedValue, -100, 100, 0, 180);  // 40-120 range typically works for continuous servos

        // Write the PWM value to the pin
        pins.servoWritePin(getDigitalPin(pin), pwmValue);
    }

    /**
     * Stops the continuous rotation servo.
     * @param pin which pin to control
     */
    //% subcategory="Actuators"
    //% weight=30 blockGap=8
    //% group="Continuous Servo"
    //% blockId="stop_continuous_servo"
    //% block="stop continuous servo on pin %pin"
    export function stopContinuousServo(pin: kiddoDigitalPin): void {
        // Set speed to 0 to stop the motor
        pins.servoWritePin(getDigitalPin(pin), 90);  // 90 typically stops a continuous servo
    }
}