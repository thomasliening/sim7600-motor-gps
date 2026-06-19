# sim7600-motor-gps

Arduino Uno + DFRobot SIM7600G-H 4G shield controlling a NEMA17 stepper motor via SMS over Hologram IoT cellular, with GPS location reporting.

## Hardware

| Part | Notes |
|------|-------|
| Arduino Uno R3 | |
| DFRobot SIM7600G-H 4G Shield | PWRKEY on D12 |
| Hologram IoT SIM | APN: `hologram` |
| CNC Shield V3 | STEP=D2, DIR=D5, ENABLE=D8 |
| TMC2208 driver | 1/16 microstepping → 400 steps = 90° |
| NEMA17 stepper | |
| GPS antenna | Plugged into GNSS ANT port on shield |

## Pin Map

| Arduino Pin | Function |
|-------------|----------|
| D2 | STEP |
| D5 | DIR |
| D7 | SoftwareSerial RX (SIM7600 TX) |
| D8 | SoftwareSerial TX (SIM7600 RX) — shared with CNC ENABLE |
| D12 | SIM7600 PWRKEY |
| D13 | Status LED |

> **D8 note:** SoftwareSerial TX and CNC ENABLE share D8. The sketch drives D8 HIGH before every AT command (UART idle) and LOW only inside `moveStepper()`.

## SMS Commands

Send any of these as an SMS to the SIM card's phone number:

| Message | Action |
|---------|--------|
| `f` | Rotate motor forward 90° (unlock) |
| `b` | Rotate motor backward 90° (lock) |
| `gps` | Reply with decimal lat,lon coordinates |

The `gps` reply (e.g. `40.714056,-89.428230`) can be pasted directly into Google Maps.

## Setup

1. Insert Hologram SIM, connect cellular and GPS antennas
2. Flash `sms_motor/sms_motor.ino` to the Arduino
3. Open Serial Monitor at 115200 baud to watch startup
4. Find the SIM phone number at [console.hologram.io](https://console.hologram.io) → Devices → SIM details
5. Text `f`, `b`, or `gps` from your phone

GPS cold-start fix takes 60–120 seconds outdoors. Subsequent boots are faster (warm start).

## Notes

- Firmware: `LE20B03SIM7600M22` — HTTPS/SSL and MQTT not supported; SMS used instead
- GPS starts automatically on boot via `AT+CGPS=1`
- Motor steps: 400 steps per 90° at 1/16 microstepping