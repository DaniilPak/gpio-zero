# Changelog

## 0.1.0 — 2026-11-10

First release.

- Pin layer: the `Pin` and `PinFactory` traits; the `cdev` backend (default), the `rppal` backend and
  `MockFactory` with a virtual clock.
- Output devices: `OutputDevice`, `DigitalOutputDevice`, `PwmOutputDevice`, `Led`, `PwmLed`, `RgbLed`, `Buzzer`,
  `TonalBuzzer`, `Motor`, `PhaseEnableMotor`, `Servo`, `AngularServo`.
- Input devices: `InputDevice`, `DigitalInputDevice`, `Button`, `MotionSensor`, `LineSensor`, `LightSensor`,
  `DistanceSensor`, `RotaryEncoder`.
- SPI: MCP3001/3002/3004/3008/3201/3202/3204/3208, over hardware or software SPI.
- Composite devices: `LedBoard`, `LedBarGraph`, `TrafficLights`, `ButtonBoard`, `Robot`.
- The source/values pattern and `tools`.
