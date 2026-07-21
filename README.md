# Handheld Cadence Tracker

A responsive handheld walking and running cadence tracker built with an
ESP32-C5, an MPU9250-series inertial sensor, and a 128×64 SSD1306 OLED.

The tracker measures rhythmic handheld motion, filters out gravity and
orientation changes, validates step timing, and displays both the current and
average cadence in steps per minute (SPM). An LED provides immediate feedback
whenever a step is detected.

## Features

- 125 Hz accelerometer and gyroscope sampling
- Orientation-independent handheld motion detection
- Automatic stillness calibration at startup
- Adaptive accelerometer and gyroscope thresholds
- Hysteretic peak detection and false-motion rejection
- Rhythm locking for more reliable step validation
- Recovery from an occasional missed step impact
- Session step counter
- Average active cadence display
- Responsive current cadence display
- Immediate, non-blocking LED step indication
- OLED and sensor access coordinated to protect sample timing
- No button required

## Display

During tracking, the OLED shows:

- **AVG SPM:** Average cadence across accepted active step intervals
- **NOW:** Smoothed current cadence
- **STEPS:** Total confirmed and recovered steps
- **MOT:** Live motion-strength indicator
- **AVG CAD / AVG SEEK:** Whether a stable walking or running rhythm has been
  established

Pauses and rejected hand movements are not included in the average cadence.
The average remains available if rhythm lock is temporarily lost.

## Hardware

- ESP32-C5 development board
- MPU9250, MPU9255, or MPU6500 motion sensor
- 128×64 SSD1306 I²C OLED at address `0x3C` or `0x3D`
- Indicator LED
- 220–330 Ω LED resistor
- Jumper wires or a custom PCB

## Wiring

The firmware defaults to the original split-bus wiring below.

| Device | Signal | ESP32-C5 pin |
|---|---|---:|
| OLED | SDA | GPIO23 |
| OLED | SCL | GPIO24 |
| MPU | SDA | GPIO25 |
| MPU | SCL | GPIO26 |
| LED | Anode through 220–330 Ω resistor | GPIO2 |
| LED | Cathode | GND |

Connect the ESP32-C5, OLED, and MPU grounds together. Power each module using
the voltage supported by the specific breakout board being used.

The ESP32-C5 has one high-performance I²C controller that can be routed to the
arbitrary GPIO pins above. The firmware therefore locks the controller during
an OLED transfer, selects the OLED pins, transmits one frame, restores the MPU
pins, and releases the sensor task. The display refresh is limited to 2 FPS so
these interruptions remain short and infrequent.

## Firmware requirements

- Arduino IDE 2.x
- Espressif ESP32 Arduino core with ESP32-C5 support
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)

The MPU is configured directly through its registers. A separate MPU9250
Arduino library is not required.

## Installation

1. Install the Espressif ESP32 board package in Arduino IDE.
2. Install **Adafruit GFX Library** and **Adafruit SSD1306** using Library
   Manager.
3. Open [`cadence_tracker_screen_average.ino`](cadence_tracker_screen_average.ino).
4. Select **ESP32C5 Dev Module** as the board.
5. Select the correct serial port.
6. Compile and upload the sketch.

The firmware has been compile-tested with the following board identifier:

```text
esp32:esp32:esp32c5
```

## Using the tracker

1. Turn on or reset the tracker.
2. Confirm that the LED flashes three times during its startup test.
3. Hold the tracker completely still while the calibration screen reaches
   100%.
4. Hold the tracker naturally and begin walking or running.
5. The first valid motion flashes the LED immediately.
6. Two plausibly timed detections establish rhythm lock and begin the cadence
   display.

If the device moves repeatedly during calibration, the stable calibration
window restarts automatically.

## How cadence is calculated

The accelerometer magnitude is used so detection does not depend on the
orientation of the device. A slow baseline removes gravity and gradual hand
orientation changes. Filtered gyroscope motion then corroborates the
acceleration signal.

Candidate peaks must satisfy motion-strength, timing, and refractory-period
checks. After rhythm lock:

- Recent accepted intervals produce the responsive **NOW** cadence.
- All accepted active intervals produce the session **AVG SPM** cadence.
- A gap near two expected periods can recover one weak or missed impact.
- Pauses and rejected peaks do not lower the active-cadence average.

## Configuration and tuning

The default values are intended as a balanced starting point for handheld
walking and running. Test the tracker before changing them.

Useful constants near the top of the sketch include:

| Constant | Purpose |
|---|---|
| `SENSOR_PERIOD_MS` | Sensor sampling interval |
| `DISPLAY_PERIOD_MS` | OLED refresh interval |
| `MIN_STEP_INTERVAL_MS` | Maximum allowed cadence |
| `MAX_STEP_INTERVAL_MS` | Minimum cadence used while seeking rhythm |
| `PEAK_TRIGGER_SCORE` | Motion strength required to open a candidate peak |
| `PEAK_RELEASE_SCORE` | Hysteresis level used to finish a peak |

For missed real steps, reduce the lower acceleration-threshold clamp slightly.
For false steps caused by ordinary hand movement, increase
`PEAK_TRIGGER_SCORE` in small increments. Change only one value at a time and
test at multiple walking and running speeds.

## KiCad hardware files

KiCad design files for the custom cadence-tracker hardware will be added to
this repository. The planned hardware release includes:

- KiCad project file (`.kicad_pro`)
- Schematic (`.kicad_sch`)
- PCB layout (`.kicad_pcb`)
- Component and connector details
- Fabrication outputs and a bill of materials when available

A recommended repository location is:

```text
hardware/
└── kicad/
    ├── handheld_cadence_tracker.kicad_pro
    ├── handheld_cadence_tracker.kicad_sch
    └── handheld_cadence_tracker.kicad_pcb
```

Until those files are uploaded, the wiring table above is the reference for
building the prototype.

## Suggested repository structure

```text
.
├── cadence_tracker_screen_average.ino
├── README.md
├── hardware/
│   └── kicad/                 # KiCad files will be added here
└── docs/                      # Optional photos and build notes
```

## Troubleshooting

### OLED remains blank

- Confirm SDA is connected to GPIO23 and SCL to GPIO24.
- Confirm the OLED is an I²C SSD1306 display.
- Check whether its address is `0x3C` or `0x3D`; the firmware tries both.
- Verify common ground and the module's supply voltage.
- Open Serial Monitor at 115200 baud and check for `OLED not found`.

### Tracker stays on the calibration screen

Place it on a stable surface or hold it completely still. Sustained movement
intentionally restarts calibration.

### Cadence remains blank

The tracker needs two plausibly timed motion peaks before rhythm lock. Confirm
that the LED responds while walking naturally.

### MPU initialization fails

- Confirm SDA is connected to GPIO25 and SCL to GPIO26.
- Check power and ground.
- The firmware checks MPU addresses `0x68` and `0x69` automatically.



No license has been selected yet. Add a `LICENSE` file before distributing or
accepting contributions if you want to define reuse terms explicitly.
