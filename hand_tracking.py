"""
hand_tracking.py
================
A thin, reusable wrapper around Google MediaPipe Hands for RGB hand-landmark
detection.

This module exposes a single class, ``HandDetector``, that:
  * runs MediaPipe on a BGR frame (the format OpenCV gives you),
  * returns the 21 hand landmarks as pixel coordinates,
  * offers small helpers (distance between two landmarks, which fingers are up).

It is deliberately camera-agnostic and knows nothing about robots or serial
ports. The robot-control logic lives in ``gesture_arm_controller.py`` so this
file can be reused for any other hand-tracking project.

Landmark indices (MediaPipe convention):
    0  = wrist
    4  = thumb tip
    8  = index finger tip
    12 = middle finger tip
    16 = ring finger tip
    20 = pinky tip

Author: Osinachi Mbakamma
Original coursework/thesis project: "Comparison of Depth and RGB-Only
Algorithms of Hand Gesture Recognition for Robot Arm Control"
(M.Eng, Wroclaw University of Science and Technology).
"""

import math

import cv2
import mediapipe as mp


class HandDetector:
    """Detect hands in an RGB/BGR frame and expose landmark coordinates.

    Parameters
    ----------
    static_mode : bool
        If True, MediaPipe treats every frame as an independent image
        (slower, more robust). If False, it tracks across frames (faster).
    max_hands : int
        Maximum number of hands to detect.
    model_complexity : int
        0 or 1. Higher is more accurate but slower.
    detection_confidence : float
        Minimum confidence for the initial hand detection.
    tracking_confidence : float
        Minimum confidence for landmark tracking between frames.
    """

    #: Landmark indices of the five finger tips (thumb -> pinky).
    TIP_IDS = [4, 8, 12, 16, 20]

    def __init__(
        self,
        static_mode=False,
        max_hands=2,
        model_complexity=1,
        detection_confidence=0.5,
        tracking_confidence=0.5,
    ):
        self.static_mode = static_mode
        self.max_hands = max_hands
        self.model_complexity = model_complexity
        self.detection_confidence = detection_confidence
        self.tracking_confidence = tracking_confidence

        self.mp_hands = mp.solutions.hands
        # NOTE: In the original coursework code these arguments were passed
        # positionally and the detection/tracking confidences were swapped.
        # Here they are passed by keyword so the mapping is unambiguous.
        self.hands = self.mp_hands.Hands(
            static_image_mode=self.static_mode,
            max_num_hands=self.max_hands,
            model_complexity=self.model_complexity,
            min_detection_confidence=self.detection_confidence,
            min_tracking_confidence=self.tracking_confidence,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Populated by find_hands() on every frame.
        self.results = None
        self.landmark_list = []

    # ------------------------------------------------------------------ #
    # Core detection
    # ------------------------------------------------------------------ #
    def find_hands(self, frame, draw=True):
        """Run MediaPipe on a BGR frame and optionally draw the skeleton.

        Returns the (possibly annotated) frame.
        """
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(frame_rgb)

        if self.results.multi_hand_landmarks and draw:
            for hand_landmarks in self.results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
        return frame

    def find_position(self, frame, hand_index=0, draw=True):
        """Return landmark pixel coordinates for one detected hand.

        Returns
        -------
        landmark_list : list[[id, x, y]]
            21 landmarks in pixel coordinates. Empty if no hand is found.
        bbox : tuple(xmin, ymin, xmax, ymax) or ()
            Bounding box around the hand.
        """
        x_list, y_list = [], []
        bbox = ()
        self.landmark_list = []

        if not self.results or not self.results.multi_hand_landmarks:
            return self.landmark_list, bbox

        # Guard against requesting a hand index that wasn't detected.
        if hand_index >= len(self.results.multi_hand_landmarks):
            return self.landmark_list, bbox

        hand = self.results.multi_hand_landmarks[hand_index]
        h, w, _ = frame.shape
        for lm_id, lm in enumerate(hand.landmark):
            cx, cy = int(lm.x * w), int(lm.y * h)
            x_list.append(cx)
            y_list.append(cy)
            self.landmark_list.append([lm_id, cx, cy])
            if draw:
                cv2.circle(frame, (cx, cy), 6, (255, 0, 255), cv2.FILLED)

        xmin, xmax = min(x_list), max(x_list)
        ymin, ymax = min(y_list), max(y_list)
        bbox = (xmin, ymin, xmax, ymax)
        if draw:
            cv2.rectangle(
                frame, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20),
                (0, 255, 0), 2,
            )
        return self.landmark_list, bbox

    def get_handedness(self, hand_index=0):
        """Return 'Left' / 'Right' for a detected hand, or None."""
        if not self.results or not self.results.multi_handedness:
            return None
        if hand_index >= len(self.results.multi_handedness):
            return None
        return self.results.multi_handedness[hand_index].classification[0].label

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def find_distance(self, id_a, id_b, frame, draw=True):
        """Euclidean pixel distance between two landmarks.

        Returns (length, frame, [x1, y1, x2, y2, cx, cy]).
        """
        x1, y1 = self.landmark_list[id_a][1], self.landmark_list[id_a][2]
        x2, y2 = self.landmark_list[id_b][1], self.landmark_list[id_b][2]
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

        if draw:
            cv2.circle(frame, (x1, y1), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (x2, y2), 5, (255, 255, 255), cv2.FILLED)
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), cv2.FILLED)
            cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 3)

        length = int(math.hypot(x2 - x1, y2 - y1))
        return length, frame, [x1, y1, x2, y2, cx, cy]

    def fingers_up(self, handedness="Right"):
        """Return a list [thumb, index, middle, ring, pinky] of 0/1.

        The thumb test depends on which hand is shown (its tip moves left/right
        rather than up/down), so ``handedness`` must be supplied. In the
        original code this was hard-coded to a right hand.
        """
        if not self.landmark_list:
            return [0, 0, 0, 0, 0]

        fingers = []

        # Thumb: compare tip x against the joint below it, direction depends
        # on which hand it is.
        tip_x = self.landmark_list[self.TIP_IDS[0]][1]
        joint_x = self.landmark_list[self.TIP_IDS[0] - 1][1]
        if handedness == "Right":
            fingers.append(1 if tip_x < joint_x else 0)
        else:  # Left hand is mirrored
            fingers.append(1 if tip_x > joint_x else 0)

        # Four fingers: tip is above (smaller y) the PIP joint => finger is up.
        for i in range(1, 5):
            tip_y = self.landmark_list[self.TIP_IDS[i]][2]
            pip_y = self.landmark_list[self.TIP_IDS[i] - 2][2]
            fingers.append(1 if tip_y < pip_y else 0)

        return fingers


def main():
    """Standalone demo: open the webcam and print the thumb-tip coordinate."""
    import time

    cap = cv2.VideoCapture(0)
    detector = HandDetector()
    prev_time = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = detector.find_hands(frame)
        landmarks, _ = detector.find_position(frame)
        if landmarks:
            print("Thumb tip:", landmarks[4])

        now = time.time()
        fps = 1 / (now - prev_time) if prev_time else 0
        prev_time = now
        cv2.putText(
            frame, f"FPS: {int(fps)}", (10, 70),
            cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3,
        )

        cv2.imshow("Hand Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
