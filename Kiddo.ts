/**
 * Custom blocks for Kiddo Robot
 * Version: 2.6 - Fixed music dependency errors by using basic tones.
 */

//% weight=100 color=#993366 icon="\uf2db" block="Kiddo"
namespace KiddoRobot {

    //==========================================================================
    //                              STATE VARIABLES
    //==========================================================================
    let irState: IrState;
    const NUM_LEDS = 4, LED_PIN = DigitalPin.P12;
    let ledBuffer: Buffer = null, ledBrightness = 255;
    let rightSensorThreshold = 512, leftSensorThreshold = 512;

    //==========================================================================
    //                                  ENUMS
    //==========================================================================
    export enum kiddoDigitalPin { P3 = DigitalPin.P3, P4 = DigitalPin.P4, P5 = DigitalPin.P5, P6 = DigitalPin.P6, P7 = DigitalPin.P7, P10 = DigitalPin.P10, P11 = DigitalPin.P11, P19 = DigitalPin.P19, P20 = DigitalPin.P20 }
    export enum kiddoAnalogPin { P3 = AnalogPin.P3, P4 = AnalogPin.P4, P10 = AnalogPin.P10 }
    export enum Motor { Right, Left }
    export enum Direction { Forward, Backward }
    export enum Unit { Centimeters, Inches }
    export enum ServoPosition { Zero = 0, FortyFive = 45, Ninety = 90, OneThirtyFive = 135, OneEighty = 180 }
    export enum LedGroup { All, Front, Bottom }
    export enum LedID { BottomLeft = 1, FrontLeft = 2, FrontRight = 3, BottomRight = 4 }
    const enum IrButton { any = -1 }
    export enum LedColor { Red = 0xFF0000, Orange = 0xFF7F00, Yellow = 0xFFFF00, Green = 0x00FF00, Blue = 0x0000FF, Indigo = 0x4B0082, Violet = 0x8A2BE2, Purple = 0xFF00FF, White = 0xFFFFFF, Black = 0x000000 }

    //==========================================================================
    //                              PINS SUB-CATEGORY
    //==========================================================================
    //% subcategory="Pins"
    //% group="Digital Pins"
    //% weight=150 blockId="kiddo_digital_read" block="read digital pin %pin"
    export function digitalRead(pin: kiddoDigitalPin): number { return pins.digitalReadPin(pin); }
    //% subcategory="Pins"
    //% group="Digital Pins"
    //% weight=140 blockId="kiddo_digital_write" block="write digital pin %pin to %value"
    export function digitalWrite(pin: kiddoDigitalPin, value: number): void { pins.digitalWritePin(pin, value); }
    //% subcategory="Pins"
    //% group="Analog Pins"
    //% weight=130 blockId="kiddo_analog_read" block="read analog pin %pin"
    export function analogRead(pin: kiddoAnalogPin): number { return pins.analogReadPin(pin); }
    //% subcategory="Pins"
    //% group="Analog Pins"
    //% weight=120 blockId="kiddo_analog_write" block="write analog pin %pin to %value"
    //% value.min=0 value.max=1023
    export function analogWrite(pin: kiddoAnalogPin, value: number): void { pins.analogWritePin(pin, value); }

    //==========================================================================
    //                            SENSORS SUB-CATEGORY
    //==========================================================================
    //-------------------------- Line Follower --------------------------
    const RIGHT_SENSOR_PIN: AnalogPin = AnalogPin.P1, LEFT_SENSOR_PIN: AnalogPin = AnalogPin.P2;
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="set line follower threshold to %threshold"
    //% threshold.min=0 threshold.max=1023 threshold.defl=512 weight=100
    export function setLineFollowerThreshold(threshold: number): void { rightSensorThreshold = threshold; leftSensorThreshold = threshold; }
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="read right sensor (1=black, 0=white)" weight=90
    export function readRightLineFollowerSensor(): number { return (pins.analogReadPin(RIGHT_SENSOR_PIN) < rightSensorThreshold) ? 1 : 0; }
    //% subcategory="Sensors"
    //% group="Line Follower"
    //% block="read left sensor (1=black, 0=white)" weight=80
    export function readLeftLineFollowerSensor(): number { return (pins.analogReadPin(LEFT_SENSOR_PIN) < leftSensorThreshold) ? 1 : 0; }

    //-------------------------- Obstacle Detection (Ultrasonic) --------------------------
    const ULTRASONIC_TRIG_PIN = DigitalPin.P8, ULTRASONIC_ECHO_PIN = DigitalPin.P9;
    //% subcategory="Sensors"
    //% group="Obstacle Detection"
    //% block="ultrasonic distance in %unit" weight=100
    export function detectObstacleDistance(unit: Unit, maxCmDistance = 500): number { pins.setPull(ULTRASONIC_TRIG_PIN, PinPullMode.PullNone); pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0); control.waitMicros(2); pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 1); control.waitMicros(10); pins.digitalWritePin(ULTRASONIC_TRIG_PIN, 0); const d = pins.pulseIn(ULTRASONIC_ECHO_PIN, PulseValue.High, maxCmDistance * 58); if (d === 0) return -1; switch (unit) { case Unit.Centimeters: return Math.idiv(d, 58); case Unit.Inches: return Math.idiv(d, 148); default: return -1; } }

    //-------------------------- IR Receiver --------------------------
    const IR_REPEAT = 256, IR_INCOMPLETE = 257, IR_DATAGRAM = 258, REPEAT_TIMEOUT_MS = 120;
    interface IrState { hasNewDatagram: boolean; bitsReceived: uint8; addressSectionBits: uint16; commandSectionBits: uint16; hiword: uint16; loword: uint16; activeCommand: number; repeatTimeout: number; onIrDatagram: () => void; }
    function initIrState() { if (irState) return; irState = { hasNewDatagram: false, bitsReceived: 0, addressSectionBits: 0, commandSectionBits: 0, hiword: 0, loword: 0, activeCommand: -1, repeatTimeout: 0, onIrDatagram: undefined }; }
    function enableIrMarkSpaceDetection(pin: DigitalPin) { let mark = 0; pins.setPull(pin, PinPullMode.PullNone); pins.onPulsed(pin, PulseValue.Low, () => { mark = pins.pulseDuration(); }); pins.onPulsed(pin, PulseValue.High, () => { const status = decode(mark + pins.pulseDuration()); if (status !== IR_INCOMPLETE) handleIrEvent(status); }); }
    function decode(markAndSpace: number): number { if (markAndSpace < 1600) return appendBitToDatagram(0); else if (markAndSpace < 2700) return appendBitToDatagram(1); irState.bitsReceived = 0; if (markAndSpace < 12500) return IR_REPEAT; else if (markAndSpace < 14500) return IR_INCOMPLETE; else return IR_INCOMPLETE; }
    function appendBitToDatagram(bit: number): number { irState.bitsReceived += 1; if (irState.bitsReceived <= 16) irState.hiword = (irState.hiword << 1) + bit; else if (irState.bitsReceived <= 32) irState.loword = (irState.loword << 1) + bit; if (irState.bitsReceived === 32) { irState.addressSectionBits = irState.hiword & 0xffff; irState.commandSectionBits = irState.loword & 0xffff; return IR_DATAGRAM; } else return IR_INCOMPLETE; }
    function handleIrEvent(irEvent: number) { if (irEvent === IR_DATAGRAM || irEvent === IR_REPEAT) irState.repeatTimeout = input.runningTime() + REPEAT_TIMEOUT_MS; if (irEvent === IR_DATAGRAM) { irState.hasNewDatagram = true; if (irState.onIrDatagram) control.inBackground(() => irState.onIrDatagram()); const newCommand = irState.commandSectionBits >> 8; if (newCommand !== irState.activeCommand) irState.activeCommand = newCommand; } }
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% block="connect IR receiver to pin P0" weight=90
    export function connectIrReceiver(): void { initIrState(); enableIrMarkSpaceDetection(DigitalPin.P0); control.inBackground(() => { while (true) { if (irState.activeCommand !== -1 && input.runningTime() > irState.repeatTimeout) { irState.bitsReceived = 0; irState.activeCommand = -1; } basic.pause(REPEAT_TIMEOUT_MS); } }); }
    //% subcategory="Sensors"
    //% group="IR Receiver"
    //% block="IR button value" weight=70
    export function irButton(): number { basic.pause(0); if (!irState) return IrButton.any; return irState.commandSectionBits >> 8; }

    //==========================================================================
    //                           ACTUATORS SUB-CATEGORY
    //==========================================================================
    //-------------------------- DC Motors --------------------------
    const R_PIN1 = DigitalPin.P14, R_PIN2 = DigitalPin.P13, L_PIN1 = DigitalPin.P16, L_PIN2 = DigitalPin.P15;
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="run motor %motor %direction at speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50 weight=100
    export function controlMotor(motor: Motor, direction: Direction, speed: number): void { let pwmSpeed = Math.map(speed, 0, 100, 0, 1023); if (motor === Motor.Right) { pins.analogWritePin(direction === Direction.Forward ? R_PIN1 : R_PIN2, pwmSpeed); pins.digitalWritePin(direction === Direction.Forward ? R_PIN2 : R_PIN1, 0); } else if (motor === Motor.Left) { pins.analogWritePin(direction === Direction.Forward ? L_PIN1 : L_PIN2, pwmSpeed); pins.digitalWritePin(direction === Direction.Forward ? L_PIN2 : L_PIN1, 0); } }
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="stop motor %motor" weight=90
    export function stopMotor(motor: Motor): void { controlMotor(motor, Direction.Forward, 0); }
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="move %direction at speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50 weight=80
    export function move(direction: Direction, speed: number): void { controlMotor(Motor.Left, direction, speed); controlMotor(Motor.Right, direction, speed); }
    //% subcategory="Actuators"
    //% group="Motors"
    //% block="stop all motors" weight=70
    export function stopAllMotors(): void { stopMotor(Motor.Left); stopMotor(Motor.Right); }

    //-------------------------- RGB LEDs --------------------------
    function _initLEDs() { if (!ledBuffer) { ledBuffer = pins.createBuffer(NUM_LEDS * 3); ledBuffer.fill(0); _updateLEDs(); } }
    function _updateLEDs() { if (ledBuffer) light.sendWS2812Buffer(ledBuffer, LED_PIN); }
    function _setPixelColor(pixel: number, color: number) { _initLEDs(); let r = (color >> 16) & 0xFF, g = (color >> 8) & 0xFF, b = (color) & 0xFF; r = (r * ledBrightness) / 255; g = (g * ledBrightness) / 255; b = (b * ledBrightness) / 255; const offset = pixel * 3; ledBuffer[offset] = g; ledBuffer[offset + 1] = r; ledBuffer[offset + 2] = b; }
    function ledIdToIndex(id: LedID): number { return id - 1; }
    function hslToRgb(h: number, s: number, l: number): number { h %= 360; s /= 100; l /= 100; let c = (1 - Math.abs(2 * l - 1)) * s, x = c * (1 - Math.abs((h / 60) % 2 - 1)), m = l - c / 2, r = 0, g = 0, b = 0; if (h < 60) { r = c; g = x; } else if (h < 120) { r = x; g = c; } else if (h < 180) { g = c; b = x; } else if (h < 240) { g = x; b = c; } else if (h < 300) { r = x; b = c; } else { r = c; b = x; } r = Math.round((r + m) * 255); g = Math.round((g + m) * 255); b = Math.round((b + m) * 255); return (r << 16) | (g << 8) | b; }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="set %group LEDs to %color=kiddo_color_picker" weight=100
    export function setGroupLedColor(group: LedGroup, color: number): void { _initLEDs(); switch (group) { case LedGroup.All: for (let i = 0; i < NUM_LEDS; i++) _setPixelColor(i, color); break; case LedGroup.Front: _setPixelColor(1, color); _setPixelColor(2, color); break; case LedGroup.Bottom: _setPixelColor(0, color); _setPixelColor(3, color); break; } _updateLEDs(); }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% blockId="kiddo_set_group_led_simple" block="set %group LEDs to %color" weight=99
    export function setGroupLedColorSimple(group: LedGroup, color: LedColor): void { setGroupLedColor(group, color); }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="set LED %id to %color=kiddo_color_picker" weight=95
    export function setSingleLedColor(id: LedID, color: number): void { _setPixelColor(ledIdToIndex(id), color); _updateLEDs(); }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="turn off %group LEDs" weight=90
    export function turnOffLeds(group: LedGroup): void { setGroupLedColor(group, 0x000000); }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% block="set LED brightness to %brightness"
    //% brightness.min=0 brightness.max=255 brightness.defl=255 weight=85
    export function setLedBrightness(brightness: number): void { ledBrightness = Math.clamp(0, 255, brightness); }
    //% subcategory="Actuators"
    //% group="RGB LED"
    //% blockId="kiddo_color_picker" block="%value"
    //% value.shadow="colorNumberPicker" weight=80 blockGap=40
    export function colorPicker(value: number): number { return value; }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show rainbow animation" weight=70
    export function showRainbowAnimation(): void { _initLEDs(); for (let i = 0; i < NUM_LEDS; i++) { _setPixelColor(i, hslToRgb((i * 360) / NUM_LEDS, 100, 50)); } _updateLEDs(); }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show comet animation with color %color=kiddo_color_picker" weight=69
    export function showCometAnimation(color: number): void { _initLEDs(); for (let i = 0; i < NUM_LEDS * 2; i++) { ledBuffer.fill(0); _setPixelColor(i % NUM_LEDS, color); _updateLEDs(); basic.pause(80); } }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show breathing effect with color %color=kiddo_color_picker" weight=68
    export function showBreathingAnimation(color: number): void { const originalBrightness = ledBrightness; for (let i = 0; i < 255; i++) { ledBrightness = i; setGroupLedColor(LedGroup.All, color); control.waitMicros(2000); } for (let i = 255; i > 0; i--) { ledBrightness = i; setGroupLedColor(LedGroup.All, color); control.waitMicros(2000); } ledBrightness = originalBrightness; }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show theater chase with color %color=kiddo_color_picker" weight=67
    export function showTheaterChaseAnimation(color: number): void { _initLEDs(); for (let j = 0; j < 10; j++) { for (let q = 0; q < 3; q++) { for (let i = 0; i < NUM_LEDS; i += 3) _setPixelColor(i + q, color); _updateLEDs(); basic.pause(100); for (let i = 0; i < NUM_LEDS; i += 3) _setPixelColor(i + q, 0); } } }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show Cylon scanner with color %color=kiddo_color_picker" weight=66
    export function showCylonScannerAnimation(color: number): void { _initLEDs(); for (let i = 0; i < NUM_LEDS - 1; i++) { _setPixelColor(i, color); _updateLEDs(); basic.pause(100); ledBuffer.fill(0); } for (let i = NUM_LEDS - 1; i > 0; i--) { _setPixelColor(i, color); _updateLEDs(); basic.pause(100); ledBuffer.fill(0); } }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show sparkle with color %color=kiddo_color_picker" weight=65
    export function showSparkleAnimation(color: number): void { _initLEDs(); for (let i = 0; i < 20; i++) { const pixel = Math.randomRange(0, NUM_LEDS - 1); _setPixelColor(pixel, color); _updateLEDs(); basic.pause(50); _setPixelColor(pixel, 0); } _updateLEDs(); }
    //% subcategory="Actuators"
    //% group="RGB LED Animations"
    //% block="show strobe with color %color=kiddo_color_picker for %flashes flashes"
    //% flashes.defl=10 weight=64
    export function showStrobeAnimation(color: number, flashes: number): void { _initLEDs(); for (let i = 0; i < flashes; i++) { setGroupLedColor(LedGroup.All, color); basic.pause(50); turnOffLeds(LedGroup.All); basic.pause(50); } }

    //-------------------------- Servo Motors --------------------------
    //% subcategory="Actuators"
    //% group="Servo"
    //% block="move servo on pin %pin to %position °"
    //% position.min=0 position.max=180 position.defl=90 weight=100
    export function movePositionalServo(pin: kiddoDigitalPin, position: number): void { pins.servoWritePin(pin as number, Math.clamp(0, 180, position)); }
    //% subcategory="Actuators"
    //% group="Servo"
    //% block="move servo on pin %pin to %position" weight=90
    export function movePositionalServofixed(pin: kiddoDigitalPin, position: ServoPosition): void { pins.servoWritePin(pin as number, position); }
    //% subcategory="Actuators"
    //% group="Servo"
    //% block="run continuous servo on pin %pin at speed %speed"
    //% speed.min=-100 speed.max=100 weight=70
    export function moveContinuousServo(pin: kiddoDigitalPin, speed: number): void { const pwmValue = Math.map(Math.clamp(-100, 100, speed), -100, 100, 0, 180); pins.servoWritePin(pin as number, pwmValue); }
    //% subcategory="Actuators"
    //% group="Servo"
    //% block="stop continuous servo on pin %pin" weight=60
    export function stopContinuousServo(pin: kiddoDigitalPin): void { pins.servoWritePin(pin as number, 90); }

    //==========================================================================
    //                        INTERACTIONS SUB-CATEGORY
    //==========================================================================

    //% subcategory="Interactions"
    //% block="act happy" weight=100
    export function actHappy(): void {
        basic.showIcon(IconNames.Happy);

        // --- Launch sound in the background ---
        control.inBackground(function () {
            music.playTone(Note.C4, 100);
            music.playTone(Note.E4, 100);
            music.playTone(Note.G4, 100);
            music.playTone(Note.C5, 200);
        })

        // --- Perform lights and movement at the same time ---
        showRainbowAnimation();
        for (let i = 0; i < 2; i++) {
            controlMotor(Motor.Left, Direction.Forward, 80);
            controlMotor(Motor.Right, Direction.Backward, 80);
            basic.pause(150);
            controlMotor(Motor.Left, Direction.Backward, 80);
            controlMotor(Motor.Right, Direction.Forward, 80);
            basic.pause(150);
        }
        stopAllMotors();
        turnOffLeds(LedGroup.All);
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="act sad" weight=99
    export function actSad(): void {
        const originalBrightness = ledBrightness;
        basic.showIcon(IconNames.Sad);
        setLedBrightness(50);
        setGroupLedColor(LedGroup.All, LedColor.Blue);

        // --- Launch sound in the background ---
        control.inBackground(function () {
            music.playTone(Note.A3, 300);
            music.playTone(Note.G3, 300);
            music.playTone(Note.F3, 400);
        })

        // --- Perform movement at the same time ---
        move(Direction.Backward, 30);
        basic.pause(1000); // The duration of the sad retreat

        // --- Cleanup ---
        stopAllMotors();
        setLedBrightness(originalBrightness);
        turnOffLeds(LedGroup.All);
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="act angry" weight=98
    export function actAngry(): void {
        basic.showIcon(IconNames.Angry);

        // --- Launch sound in the background ---
        control.inBackground(function () {
            for (let i = 0; i < 5; i++) {
                music.playTone(Note.C3, 100);
                music.rest(50);
            }
        })

        // --- Interleave strobe and shake for a chaotic effect ---
        for (let i = 0; i < 4; i++) {
            setGroupLedColor(LedGroup.All, LedColor.Red);
            move(Direction.Forward, 100);
            basic.pause(80);
            turnOffLeds(LedGroup.All);
            move(Direction.Backward, 100);
            basic.pause(80);
        }

        // --- Cleanup ---
        stopAllMotors();
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="act surprised" weight=97
    export function actSurprised(): void {
        // This emotion is a fast sequence, so it works better sequentially.
        basic.showIcon(IconNames.Surprised);
        music.playTone(Note.C5, 150);
        setGroupLedColor(LedGroup.All, LedColor.White);
        move(Direction.Backward, 100);
        basic.pause(250);
        stopAllMotors();
        basic.pause(500); // Pause to "process" the surprise
        turnOffLeds(LedGroup.All);
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="act curious" weight=96
    export function actCurious(): void {
        basic.showIcon(IconNames.Asleep); // Looks like squinting eyes

        // --- Launch sound in the background ---
        control.inBackground(function () {
            music.playTone(Note.C4, 100);
            basic.pause(300);
            music.playTone(Note.G4, 150);
        })

        // --- Perform "scanning" light and movement together ---
        controlMotor(Motor.Left, Direction.Backward, 40);
        controlMotor(Motor.Right, Direction.Forward, 40);
        showCylonScannerAnimation(LedColor.Blue); // This takes time

        controlMotor(Motor.Left, Direction.Forward, 40);
        controlMotor(Motor.Right, Direction.Backward, 40);
        showCylonScannerAnimation(LedColor.Blue); // This takes time

        // --- Cleanup ---
        stopAllMotors();
        turnOffLeds(LedGroup.All);
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="act affectionate" weight=95
    export function actAffectionate(): void {
        basic.showIcon(IconNames.Heart);

        // --- Launch sound in the background ---
        control.inBackground(function () {
            music.playTone(Note.E4, 150);
            music.rest(50);
            music.playTone(Note.G4, 200);
            music.rest(50);
            music.playTone(Note.E4, 150);
            music.rest(50);
            music.playTone(Note.G4, 200);
        })

        // --- Perform breathing lights and nudge together ---
        showBreathingAnimation(LedColor.Purple);
        move(Direction.Forward, 30);
        basic.pause(1000);

        // --- Cleanup ---
        stopAllMotors();
        turnOffLeds(LedGroup.All);
        basic.clearScreen();
    }

    //% subcategory="Interactions"
    //% block="go to sleep" weight=94
    export function goToSleep(): void {
        // This is a sequence, so it should be blocking.
        basic.showIcon(IconNames.Asleep);
        music.playTone(Note.G3, 300);
        music.rest(music.beat(BeatFraction.Eighth));
        music.playTone(Note.F3, 500);
        turnOffLeds(LedGroup.All);
        stopAllMotors();
    }

}
