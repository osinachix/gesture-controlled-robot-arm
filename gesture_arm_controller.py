"""
gesture_arm_controller.py
=========================
Real-time controller that maps right-hand finger positions to a 4-DOF robot
arm and streams the resulting servo targets to an Arduino over serial.

Pipeline
--------
    webcam (RGB) --> MediaPipe landmarks --> map fingertip positions to
    six control values (X, Y, Z, J, K, L) --> send "X..Y..Z..J..K..L.."
    string over serial --> Arduino/PCA9685 --> servos + suction relay

Control mapping (right hand)
----------------------------
    J  <- wrist landmark (0)  x-position      -> base rotation
    X  <- index landmark (8)  y-position      -> shoulder
    Y  <- middle landmark (12) y-position     -> elbow
    Z  <- ring landmark (16)  y-position      -> wrist
    K  <- thumb tip (4)  x                    } together decide the
    L  <- thumb IP joint (3) x                } suction pump on/off

Each raw pixel coordinate is clamped to a safe band before being sent, so the
firmware never receives a value that would drive a servo past its end stop.

Usage
-----
    python gesture_arm_controller.py                 # with Arduino attached
    python gesture_arm_controller.py --no-serial     # vision only, no hardware
    python gesture_arm_controller.py --port COM5     # override serial port
"""

import argparse
import sys

import cv2

from hand_tracking import HandDetector

try:
    import serial  # pyserial
except ImportError:  # allow --no-serial to work without pyserial installed
    serial = None


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
# Default serial port. On macOS this looked like /dev/cu.usbmodemXXXX; on
# Windows it will be e.g. "COM5"; on Linux typically "/dev/ttyUSB0" or
# "/dev/ttyACM0". Override at the command line with --port.
DEFAULT_PORT = "/dev/cu.usbmodem14101"
BAUD_RATE = 9600

CAM_WIDTH, CAM_HEIGHT = 1240, 720

# Clamp bands for each control value: (raw_min, raw_max).
# These match the trigger windows the firmware expects.
CLAMP = {
    "J": (51, 499),   # wrist x  -> base
    "X": (21, 179),   # index y  -> shoulder
    "Y": (21, 179),   # middle y -> elbow
    "Z": (21, 179),   # ring y   -> wrist
}


def clamp(value, lo, hi):
    """Constrain value to the inclusive range [lo, hi]."""
    return max(lo, min(hi, value))


def build_command(x, y, z, j, k, l):
    """Assemble the fixed-format serial string the firmware parses."""
    return f"X{x}Y{y}Z{z}J{j}K{k}L{l}"


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def run(port=DEFAULT_PORT, use_serial=True):
    arduino = None
    if use_serial:
        if serial is None:
            sys.exit("pyserial is not installed. Run 'pip install pyserial' "
                     "or pass --no-serial.")
        try:
            arduino = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except serial.SerialException as exc:
            sys.exit(f"Could not open serial port {port!r}: {exc}\n"
                     f"Tip: run with --no-serial to test the vision pipeline "
                     f"without hardware.")

    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    detector = HandDetector(max_hands=2)

    print("Controller running. Press 'q' in the video window to quit.")
    while True:
        ok, raw = cam.read()
        if not ok:
            break
        frame = cv2.flip(raw, 1)  # mirror so movement feels natural

        frame = detector.find_hands(frame, draw=False)

        for hand_index in range(2):
            landmarks, _ = detector.find_position(
                frame, hand_index=hand_index, draw=False
            )
            if not landmarks:
                continue
            if detector.get_handedness(hand_index) != "Right":
                continue  # only the right hand drives the arm

            # --- extract the control landmarks -------------------------- #
            wrist_x = landmarks[0][1]
            index_y = landmarks[8][2]
            middle_y = landmarks[12][2]
            ring_y = landmarks[16][2]
            thumb_tip_x = landmarks[4][1]
            thumb_ip_x = landmarks[3][1]

            j = clamp(wrist_x, *CLAMP["J"])
            x = clamp(index_y, *CLAMP["X"])
            y = clamp(middle_y, *CLAMP["Y"])
            z = clamp(ring_y, *CLAMP["Z"])
            k = thumb_tip_x
            l = thumb_ip_x

            # Visual feedback: colour the thumb marker by pump state.
            pump_on = k < l
            thumb_color = (0, 0, 255) if not pump_on else (46, 98, 84)
            cv2.circle(frame, (landmarks[4][1], landmarks[4][2]),
                       10, thumb_color, cv2.FILLED)

            command = build_command(x, y, z, j, k, l)
            if arduino is not None:
                arduino.write(command.encode("utf-8"))
            print(command, "| pump", "ON" if pump_on else "off")

        cv2.imshow("Gesture Robot Arm", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.release()
    cv2.destroyAllWindows()
    if arduino is not None:
        arduino.close()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT,
                   help=f"Serial port (default: {DEFAULT_PORT})")
    p.add_argument("--no-serial", action="store_true",
                   help="Run vision only, without opening a serial port.")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run(port=args.port, use_serial=not args.no_serial)
