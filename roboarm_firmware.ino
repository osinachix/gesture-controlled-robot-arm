/*
 * roboarm_firmware.ino
 * ====================
 * Firmware for a 4-DOF robot arm driven by hand-gesture data streamed from a
 * host PC over serial (see ../python/gesture_arm_controller.py).
 *
 * Hardware
 * --------
 *   - Arduino Uno / Nano
 *   - Adafruit 16-channel 12-bit PWM/Servo driver (PCA9685), I2C addr 0x40
 *   - 4 servos (base, shoulder, elbow, wrist) on PCA9685 channels 0-3
 *   - Suction-pump relay on digital pin 3
 *
 * Wiring (PCA9685 -> Arduino):
 *   VCC->5V   GND->GND   SDA->A4   SCL->A5
 *   Servo power is supplied through the driver's V+ screw terminal.
 *
 * Serial protocol
 * ---------------
 * The host sends one fixed-format string per update:
 *
 *     X<val>Y<val>Z<val>J<val>K<val>L<val>
 *
 * Each letter is a channel of control data:
 *   X -> shoulder   Y -> elbow   Z -> wrist   J -> base
 *   K, L -> thumb tip / thumb joint x-positions, compared to toggle the pump.
 *
 * The values are POSITION HINTS, not absolute angles: if a value sits below
 * the low trigger the servo steps one increment one way; above the high
 * trigger it steps the other way; in between it holds. This gives smooth,
 * incremental motion instead of jumpy absolute jumps.
 *
 * Known limitation (kept honest, documented rather than silently "fixed"):
 * the parser below reads bytes with nested Serial.read()/parseInt() calls.
 * It works with the current host format but is sensitive to timing and
 * partial reads. A more robust version would read a full line with
 * Serial.readStringUntil('\n') and tokenise it. Left as-is here because this
 * is the version that was actually tested on the hardware.
 *
 * Author: Osinachi Mbakamma
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// PCA9685 at the default I2C address 0x40.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pulse-width limits in microseconds. Tune to your servos so they reach their
// full range without hitting the mechanical end stops.
#define US_MIN      1000
#define US_MAX      2000
#define SERVO_FREQ  50      // analog servos update at ~50 Hz
#define STEP        20      // microseconds moved per update

const int RELAY_PIN = 3;    // suction-pump relay

// Current servo pulse positions (initialised to mid-travel).
uint16_t posBase     = 1500;   // channel 0  <- J
uint16_t posShoulder = 1500;   // channel 1  <- X
uint16_t posElbow    = 1500;   // channel 2  <- Y
uint16_t posWrist    = 1500;   // channel 3  <- Z

// Incoming control values.
int X, Y, Z, J, K, L;

void setup() {
  Serial.begin(9600);
  Serial.println("Ready");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);   // pump off at start

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Move all four servos to their neutral start position.
  pwm.writeMicroseconds(0, posBase);
  pwm.writeMicroseconds(1, posShoulder);
  pwm.writeMicroseconds(2, posElbow);
  pwm.writeMicroseconds(3, posWrist);
  delay(10);
}

/*
 * stepServo: move one servo one STEP toward the direction implied by the
 * incoming value, staying within [US_MIN, US_MAX]. Returns the new position.
 *
 *   value < lowTrigger  -> step by -STEP (or +STEP if invert)
 *   value > highTrigger -> step by +STEP (or -STEP if invert)
 *   otherwise           -> hold
 */
uint16_t stepServo(uint8_t channel, uint16_t pos, int value,
                   int lowTrigger, int highTrigger, bool invert) {
  int lowDir  = invert ? +STEP : -STEP;
  int highDir = invert ? -STEP : +STEP;

  if (value < lowTrigger && pos > US_MIN && pos < US_MAX) {
    pos += lowDir;
    pwm.writeMicroseconds(channel, pos);
    if (pos <= US_MIN) pos = US_MIN + STEP;   // avoid sticking at the limit
    if (pos >= US_MAX) pos = US_MAX - STEP;
  } else if (value > highTrigger && pos > US_MIN && pos < US_MAX) {
    pos += highDir;
    pwm.writeMicroseconds(channel, pos);
    if (pos <= US_MIN) pos = US_MIN + STEP;
    if (pos >= US_MAX) pos = US_MAX - STEP;
  }
  return pos;
}

// Apply the latest control values to the servos and the pump relay.
void applyControls() {
  posShoulder = stepServo(1, posShoulder, X,  90, 150, false);
  posElbow    = stepServo(2, posElbow,    Y,  70, 130, false);
  posWrist    = stepServo(3, posWrist,    Z,  70, 130, true);   // ring finger inverted
  posBase     = stepServo(0, posBase,     J, 220, 370, false);

  // Suction pump: thumb tip (K) left of the joint (L) turns the pump ON.
  if (K < L) {
    digitalWrite(RELAY_PIN, HIGH);   // pump on
  } else {
    digitalWrite(RELAY_PIN, LOW);    // pump off
  }
}

void loop() {
  if (Serial.available() > 0) {
    // Parse the fixed X..Y..Z..J..K..L.. frame.
    if (Serial.read() == 'X') {
      X = Serial.parseInt();
      if (Serial.read() == 'Y') {
        Y = Serial.parseInt();
        if (Serial.read() == 'Z') {
          Z = Serial.parseInt();
          if (Serial.read() == 'J') {
            J = Serial.parseInt();
            if (Serial.read() == 'K') {
              K = Serial.parseInt();
              if (Serial.read() == 'L') {
                L = Serial.parseInt();
                applyControls();
              }
            }
          }
        }
      }
    }
    // Flush anything left so the next frame starts clean.
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}
