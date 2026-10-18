/*
 * arm.ino — full arm control. Homes to a resting pose, holds each joint within
 * its safe limits, slew-limits all motion, and takes commands over USB serial.
 *
 * Calibration below is from your jog session. Channels:
 *   0 base     1 shoulder     2 elbow     3 wrist     4 gripper(SG90)
 *
 * SERIAL PROTOCOL (from controller.py), one line at 115200 baud:
 *   lateral,reach,wrist,grab\n
 *     lateral 0-180  -> base
 *     reach   0-180  -> shoulder
 *     wrist   0-180  -> wrist
 *     grab    0 or 1 -> gripper (1 = closed, 0 = open)
 *   The elbow has no control signal, so it stays parked (ELBOW_PARK).
 *
 * TEST IT ALONE FIRST (no computer sending): just upload with 12V on. The arm
 * should ease to its resting pose and hold there quietly. That proves the
 * calibration and homing before we connect live control.
 *
 * NOTE: close the Arduino Serial Monitor before running controller.py — only
 * one program can use the port at a time.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_FREQ 50
#define OSC_FREQ   27000000      // jitter calibration; tune 25e6..27e6 if needed

// ---- Channel map ----
#define BASE_CH     0
#define SHOULDER_CH 1
#define ELBOW_CH    2
#define WRIST_CH    3
#define GRIP_CH     4
const int USED[] = {BASE_CH, SHOULDER_CH, ELBOW_CH, WRIST_CH, GRIP_CH};
const int NUSED = 5;

// ---- Elbow (no control signal) parks here. Change or gang later if you want.
#define ELBOW_PARK 0     // elbow parks here (your calibrated rest of 20)

// ---- Gripper open/closed angles (within its 75-180 range).
//      If clench opens instead of closes, swap these two numbers.
#define GRIP_OPEN   165   // open end (backed off from 180 to avoid strain)
#define GRIP_CLOSED 65  // you confirmed 75 = closed

// ---- Motion smoothing ----
#define SLEW_DEG   2             // max degrees per update step
#define UPDATE_MS  15            // update interval

// ---- Base "move then relax" ----
// The base rotates on a vertical axis, so gravity can't back-drive it: once
// positioned it holds by friction. Relaxing the servo then kills jitter and
// saves current. (Never do this on shoulder/elbow -- they'd droop.)
#define BASE_RELAX_ENABLED true
#define BASE_REENGAGE_DEG  4     // wake the base if commanded this far from where it sits
#define BASE_RELAX_MS      600   // relax this long after reaching the target

// ---- Per-channel calibration ----
int servoMin[16], servoMax[16];  // pulse for 0 deg / 180 deg
int angleMin[16], angleMax[16];  // safe angle limits
int   target[16];
float current[16];

String line = "";
unsigned long lastUpdate = 0;

bool baseRelaxed = false;            // is the base currently limp?
unsigned long baseSettledSince = 0;  // when the base first reached its target

int angleToPulse(int ch, int angle) {
  angle = constrain(angle, angleMin[ch], angleMax[ch]);
  return map(angle, 0, 180, servoMin[ch], servoMax[ch]);
}

void writeServo(int ch, int angle) {
  pwm.setPWM(ch, 0, angleToPulse(ch, angle));
}

void setTarget(int ch, int angle) {
  target[ch] = constrain(angle, angleMin[ch], angleMax[ch]);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  // defaults
  for (int i = 0; i < 16; i++) {
    servoMin[i] = 205; servoMax[i] = 410;
    angleMin[i] = 0;   angleMax[i] = 180;
  }

  // --- your calibration ---
  servoMin[BASE_CH]=205;     servoMax[BASE_CH]=410;
  servoMin[SHOULDER_CH]=205; servoMax[SHOULDER_CH]=410;
  servoMin[ELBOW_CH]=100;    servoMax[ELBOW_CH]=500;
  servoMin[WRIST_CH]=100;    servoMax[WRIST_CH]=560;
  servoMin[GRIP_CH]=110;     servoMax[GRIP_CH]=500;

  angleMin[GRIP_CH]=75;      angleMax[GRIP_CH]=180;   // gripper limited range

  // --- resting pose (start position) ---
  int rest[16];
  rest[BASE_CH]=90; rest[SHOULDER_CH]=15; rest[ELBOW_CH]=ELBOW_PARK;
  rest[WRIST_CH]=75; rest[GRIP_CH]=GRIP_OPEN;

  for (int k = 0; k < NUSED; k++) {
    int ch = USED[k];
    target[ch]  = rest[ch];
    current[ch] = rest[ch];
    writeServo(ch, rest[ch]);     // home immediately
  }

  Serial.println("ARM ready (resting). Send: lateral,reach,wrist,grab");
}

void parseLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  int v[4]; int idx = 0; int start = 0;
  for (int i = 0; i <= s.length() && idx < 4; i++) {
    if (i == s.length() || s[i] == ',') {
      v[idx++] = s.substring(start, i).toInt();
      start = i + 1;
    }
  }
  if (idx < 4) return;            // malformed line -> ignore
  setTarget(BASE_CH, v[0]);       // lateral
  setTarget(SHOULDER_CH, v[1]);   // reach
  setTarget(WRIST_CH, v[2]);      // wrist
  target[GRIP_CH] = (v[3] != 0) ? GRIP_CLOSED : GRIP_OPEN;
  // elbow stays parked
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { parseLine(line); line = ""; }
    else if (c != '\r') line += c;
  }

  if (millis() - lastUpdate >= UPDATE_MS) {
    lastUpdate = millis();
    for (int k = 0; k < NUSED; k++) {
      int ch = USED[k];

      // --- Base: move to target, then relax (go limp) ---
      if (ch == BASE_CH && BASE_RELAX_ENABLED) {
        // A big enough new command wakes the base back up.
        if (fabs(target[ch] - current[ch]) > BASE_REENGAGE_DEG) {
          baseRelaxed = false;
          baseSettledSince = 0;
        }
        if (baseRelaxed) continue;   // stay limp, hold by friction

        // slew toward target
        if (current[ch] < target[ch]) current[ch] = min(current[ch] + SLEW_DEG, (float)target[ch]);
        else if (current[ch] > target[ch]) current[ch] = max(current[ch] - SLEW_DEG, (float)target[ch]);
        writeServo(ch, (int)current[ch]);

        // once it's sitting on the target, start the relax countdown
        if (fabs(target[ch] - current[ch]) < 1.0) {
          if (baseSettledSince == 0) baseSettledSince = millis();
          else if (millis() - baseSettledSince > BASE_RELAX_MS) {
            pwm.setPWM(ch, 0, 0);    // relax: no pulse -> servo goes limp
            baseRelaxed = true;
          }
        } else {
          baseSettledSince = 0;
        }
        continue;
      }

      // --- All other joints: normal slew + hold ---
      if (current[ch] < target[ch]) current[ch] = min(current[ch] + SLEW_DEG, (float)target[ch]);
      else if (current[ch] > target[ch]) current[ch] = max(current[ch] - SLEW_DEG, (float)target[ch]);
      writeServo(ch, (int)current[ch]);
    }
  }
}
