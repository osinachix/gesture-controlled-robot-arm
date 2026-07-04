# Hand-Gesture Controlled Robot Arm (RGB)

Real-time control of a 4-DOF robot arm using hand gestures captured from an
ordinary RGB webcam. Hand landmarks are detected with
[MediaPipe](https://developers.google.com/mediapipe), mapped to servo targets
on the host PC, and streamed over serial to an Arduino driving the arm through
a PCA9685 servo controller. A suction pump (via a relay) acts as the
end-effector, toggled by a thumb gesture.

Project:
**"Comparison of Depth and RGB-Only Algorithms of Hand Gesture Recognition for
Robot Arm Control."**

![Pipeline](docs/architecture.svg)

## Demo

The system running live: MediaPipe tracks the right hand, the on-screen sliders
show each fingertip's mapped position, and the run console (bottom left) prints
the exact `X..Y..Z..J..K..L..` frames being streamed to the Arduino over serial.

![Open palm tracked, with the fingertip-to-slider overlay and serial output in the console](docs/screenshots/hand-tracking-open-palm.png)

As the hand moves, the red markers slide along their bands and the serial values
change accordingly — this is the position-to-command mapping happening in real
time.

![Hand in a different position, showing the markers and serial values responding to movement](docs/screenshots/hand-tracking-moved.png)

*(Captured during development, June 2023. These frames show the RGB pipeline
driving the controller; they are a working demonstration, not the thesis's
quantitative RGB-vs-depth evaluation.)*

## Scope of this repository

This repo contains the **RGB (colour-camera) pipeline** — the working,
hardware-tested system that controls the physical arm from a standard webcam.

The thesis also evaluates and compares an RGB-only approach against a
depth-based (RGB-D) approach. That comparison is analysed in the written
thesis; the depth-camera capture/analysis is **not** part of the code here.
If you're looking for the quantitative RGB-vs-depth results, see the thesis
document, not this repository. I'd rather be upfront about that than have you
go hunting for depth code that isn't here.

## How it works

```
webcam (RGB) → MediaPipe landmarks → map fingertip positions to
6 control values (X Y Z J K L) → serial string → Arduino/PCA9685 → servos + pump
```

The right hand drives the arm. Fingertip positions map to joints:

| Control | Source landmark | Joint |
|---------|-----------------|-------|
| `J` | wrist (0), x-position | base rotation |
| `X` | index tip (8), y-position | shoulder |
| `Y` | middle tip (12), y-position | elbow |
| `Z` | ring tip (16), y-position | wrist |
| `K` / `L` | thumb tip (4) / thumb joint (3), x | suction pump on/off |

Each value is clamped to a safe band before being sent, so the firmware never
receives a command that would drive a servo past its end stop. On the Arduino
side, values act as *incremental position hints*: below a low trigger the servo
steps one way, above a high trigger it steps the other, otherwise it holds —
which keeps motion smooth rather than jumpy.

## Repository layout

```
gesture-controlled-robot-arm/
├── python/
│   ├── hand_tracking.py            # Reusable MediaPipe hand-detector class
│   └── gesture_arm_controller.py   # Main controller: vision → serial
├── firmware/
│   ├── roboarm_firmware.ino        # Canonical, cleaned Arduino firmware
│   └── archive/                    # Original sketch iterations (provenance)
├── docs/
│   ├── architecture.svg
│   └── screenshots/                # Demo captures of the system running
├── requirements.txt
├── LICENSE
└── README.md
```

## Getting started

### 1. Host (Python)

```bash
git clone https://github.com/<your-username>/gesture-controlled-robot-arm.git
cd gesture-controlled-robot-arm
python -m venv .venv && source .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Test the vision pipeline **without any hardware**:

```bash
cd python
python hand_tracking.py             # webcam demo: draws the hand skeleton
python gesture_arm_controller.py --no-serial   # full pipeline, prints commands
```

Run with the arm connected (set your port):

```bash
python gesture_arm_controller.py --port /dev/ttyACM0   # Linux
python gesture_arm_controller.py --port COM5           # Windows
python gesture_arm_controller.py --port /dev/cu.usbmodem14101  # macOS
```

Press `q` in the video window to quit.

### 2. Arduino (firmware)

1. Install the **Adafruit PWM Servo Driver** library via the Arduino Library
   Manager.
2. Open `firmware/roboarm_firmware.ino`, select your board and port, upload.
3. Tune `US_MIN` / `US_MAX` in the sketch to match your servos' range.

## Hardware

- Arduino Uno / Nano
- Adafruit 16-channel 12-bit PWM/servo driver (PCA9685)
- 4 × servos (base, shoulder, elbow, wrist)
- Suction pump + relay module (end-effector)
- RGB webcam

Wiring (PCA9685 → Arduino): `VCC→5V, GND→GND, SDA→A4, SCL→A5`. Servo power via
the driver's V+ terminal.

## Honest notes on the code

A few things a reviewer should know, kept here rather than hidden:

- The serial parser in the firmware uses nested `Serial.read()`/`parseInt()`
  calls. It works with this host's fixed message format but is timing-sensitive.
  A more robust rewrite would read a full line with
  `Serial.readStringUntil('\n')` and tokenise it. It's left in its
  tested-on-hardware form, with the limitation documented in the sketch.
- The Python was refactored from the original coursework scripts for
  readability: the MediaPipe confidence arguments (which were passed
  positionally and swapped in the original) are now passed by keyword, bounds
  checks were added, and the serial port is configurable. The control logic
  itself is unchanged from what ran on the physical arm.
- Original firmware iterations are preserved in `firmware/archive/` with a note
  explaining each.

## License

MIT — see [LICENSE](LICENSE).
