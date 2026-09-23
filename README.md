# HalteausdauerTest

A small measuring device based on the **M5StickC PLUS** and the **M5StickC ToF HAT** (VL53L0X time-of-flight sensor) that measures how long and how high something is lifted and held.

After zeroing the starting position, a measurement starts automatically once the height exceeds a threshold and ends automatically when it is lowered again. The device then shows the hold duration, the maximum height and the area under the height-time curve (AUC).

## Features

- **Automatic start/stop**: a measurement starts at 10 mm of lift and ends once the height has been back below 10 mm for at least 400 ms.
- **Results**:
  - Hold duration in seconds
  - Maximum height in mm
  - AUC (height × time) in mm·s, a single value combining height and duration
- **Zeroing** from 20 samples as the reference for the starting position
- **Median filter** (5 samples) to suppress ToF sensor outliers
- **Audio feedback** via the built-in buzzer:
  - Double beep when zeroing is complete
  - One beep per second while measuring
- **Battery indicator** in percent (estimated from the battery voltage)
- **Auto power-off** after 90 s of inactivity (not during zeroing or a running measurement)
- **Power saving**: reduced CPU clock (80 MHz), ToF sensor only active when needed, adaptive sensor and display update rates
- **Flicker-free display** using an off-screen frame buffer

## Hardware

| Component | Description |
|---|---|
| M5StickC PLUS | ESP32 board with display, battery, buzzer and buttons |
| M5StickC ToF HAT | Plug-on ToF distance sensor with a VL53L0X |

### Connection

The ToF HAT simply plugs onto the HAT port on top of the M5StickC PLUS; no extra wiring is needed. It communicates over I²C:

| Signal | GPIO |
|---|---|
| SDA | 0 |
| SCL | 26 |

Valid sensor readings range from 20 mm to 2000 mm. The "height" is the difference between the current distance and the distance measured during zeroing, so the setup must be arranged so that lifting **increases** the distance to the sensor.

## Installation

1. Install the [Arduino IDE](https://www.arduino.cc/en/software).
2. Install ESP32/M5Stack board support via the Boards Manager and select **M5StickC-Plus** as the board.
3. Install the following libraries via the Library Manager:
   - `M5StickCPlus`
   - `VL53L0X` (by Pololu)
4. Open `HalteausdauerTest.ino`, compile it and upload it to the M5StickC PLUS.

## Usage

The on-screen texts are in German; the state names shown on the display are given in bold below.

1. Switch the device on. The start screen prompts you to zero the sensor.
2. Bring the object into its starting position and press **button A** (the large button on the front).
3. Keep still while zeroing (**NULLUNG**) until the double beep sounds.
4. The device now waits for the lift (**WAIT LIFT**).
5. Lift and hold. The measurement (**RUNNING**) starts automatically, with one beep per second.
6. Lower again. After 400 ms below the threshold the measurement ends and the result (**DONE**) is shown.
7. Press **button A** to zero again and start a new measurement.

### Flow

```
IDLE ──[Btn A]──> NULLUNG ──[20 samples]──> WAIT LIFT ──[height ≥ 10 mm]──> RUNNING ──[height ≤ 10 mm for 400 ms]──> DONE
                     ^                                                                                                  │
                     └────────────────────────────────────────[Btn A]──────────────────────────────────────────────────┘
```

Button A starts a new zeroing from any state.

## Configuration

All important parameters are constants at the top of [HalteausdauerTest.ino](HalteausdauerTest.ino) and can be adjusted there:

| Parameter | Default | Meaning |
|---|---|---|
| `START_THRESHOLD_MM` | 10 mm | Height at which a measurement starts |
| `STOP_THRESHOLD_MM` | 10 mm | Height below which a measurement ends |
| `STOP_HOLD_MS` | 400 ms | How long the height must stay below the stop threshold |
| `CALIBRATION_SAMPLES` | 20 | Number of samples used for zeroing |
| `SMOOTHING_WINDOW` | 5 | Median filter window size |
| `SENSOR_UPDATE_ACTIVE_MS` | 40 ms | Sensor sampling interval during zeroing and measuring (25 Hz) |
| `SENSOR_UPDATE_WAIT_MS` | 80 ms | Sensor sampling interval while waiting for the lift |
| `AUTO_SHUTOFF_MS` | 90 s | Time until automatic power-off |
| `LCD_BRIGHTNESS` | 15 | Display brightness |
| `CPU_FREQ_MHZ` | 80 MHz | ESP32 CPU clock |
| `BUZZER_ENABLED` | `true` | Enable/disable the buzzer |
| `BEEP_INTERVAL_MS` | 1000 ms | Interval between beeps while measuring |
