/*
 * jog.ino — SAFE jog v3. Per-channel pulse range + live range tuning.
 *
 * FIXES:
 *   - Bug fix: commanding a channel to 90 as its FIRST move now actually
 *     sends a pulse (before, a no-op move sent nothing and the servo stayed
 *     limp -- that's why channel 4 "did nothing").
 *   - Per-channel pulse limits, so wrist/gripper can have more travel than
 *     the base/shoulder/elbow without over-driving those.
 *   - Live tuning: set a channel's range from the Serial Monitor, no re-upload.
 *
 * UPLOAD WITH 12V UNPLUGGED, then plug 12V in after it's running.
 *
 * Serial Monitor (115200, Newline):
 *   number 0-180  -> current channel eases to that angle
 *   c1 / c2 ...   -> switch channel
 *   x             -> release (limp) current channel
 *   min205        -> set current channel's MIN pulse (short end)
 *   max410        -> set current channel's MAX pulse (long end)
 *
 * PULSE SCALE (counts): ~110 = 0.5ms (far short) ... ~500 = 2.4ms (far long).
 *   Default is 205-410 (~1.0-2.0ms, safe). To get MORE range, lower min toward
 *   ~120 and/or raise max toward ~500 -- but do it in small steps and hit x
 *   the instant a servo buzzes (that means you've pushed past its stop).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int CHANNEL = 0;
#define SERVO_FREQ 50
// Oscillator calibration to reduce jitter/humming. Tune between
// 25000000 and 27000000 until a held servo sits as still as possible.
#define OSC_FREQ 27000000
#define STEP_DEG 1
#define STEP_DELAY_MS 20     // higher = slower

int servoMin[16];
int servoMax[16];
int currentAngle[16];        // -1 = off/limp
String buffer = "";

int angleToPulse(int channel, int angle) {
  return map(angle, 0, 180, servoMin[channel], servoMax[channel]);
}

void releaseCh(int channel) {
  pwm.setPWM(channel, 0, 0);
  currentAngle[channel] = -1;
  Serial.print("channel "); Serial.print(channel); Serial.println(" released (limp)");
}

void goTo(int channel, int target) {
  target = constrain(target, 0, 180);
  int a = (currentAngle[channel] < 0) ? 90 : currentAngle[channel];
  int step = (target >= a) ? STEP_DEG : -STEP_DEG;
  while (a != target) {
    if (abs(target - a) < STEP_DEG) a = target;
    else a += step;
    pwm.setPWM(channel, 0, angleToPulse(channel, a));
    delay(STEP_DELAY_MS);
  }
  pwm.setPWM(channel, 0, angleToPulse(channel, target));  // always send, even if no stepping
  currentAngle[channel] = target;
  Serial.print("channel "); Serial.print(channel); Serial.print(" -> ");
  Serial.print(target); Serial.print(" deg (pulse ");
  Serial.print(angleToPulse(channel, target)); Serial.println(")");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(OSC_FREQ);   // reduce jitter
  pwm.setPWMFreq(SERVO_FREQ);
  delay(300);

  for (int i = 0; i < 16; i++) {
    servoMin[i] = 205;       // ~1.0 ms  (safe default)
    servoMax[i] = 410;       // ~2.0 ms
    pwm.setPWM(i, 0, 0);     // all limp on boot
    currentAngle[i] = -1;
  }
  // Wider travel for the joints that needed it:
  servoMin[2] = 100; servoMax[2] = 500;   // elbow (MG996R) - your calibration
  servoMin[3] = 100; servoMax[3] = 560;   // wrist (MG996R) - your calibration
  // gripper: SAFE default for the new servo. Widen in small steps with
  // min###/max### only if it needs more travel and stays quiet.
  servoMin[4] = 205; servoMax[4] = 410;

  Serial.println("SAFE jog v3 ready. All servos limp until commanded.");
  Serial.println("number=angle | c1/c2..=channel | x=release | min### / max### = pulse range");
  Serial.print("channel "); Serial.println(CHANNEL);
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      buffer.trim();
      if (buffer.length() > 0) {
        String b = buffer; b.toLowerCase();
        if (b.startsWith("x")) {
          releaseCh(CHANNEL);
        } else if (b.startsWith("min")) {
          servoMin[CHANNEL] = b.substring(3).toInt();
          Serial.print("ch "); Serial.print(CHANNEL);
          Serial.print(" min pulse = "); Serial.println(servoMin[CHANNEL]);
          if (currentAngle[CHANNEL] >= 0) goTo(CHANNEL, currentAngle[CHANNEL]);
        } else if (b.startsWith("max")) {
          servoMax[CHANNEL] = b.substring(3).toInt();
          Serial.print("ch "); Serial.print(CHANNEL);
          Serial.print(" max pulse = "); Serial.println(servoMax[CHANNEL]);
          if (currentAngle[CHANNEL] >= 0) goTo(CHANNEL, currentAngle[CHANNEL]);
        } else if (b.startsWith("c")) {
          int newCh = b.substring(1).toInt();
          if (newCh >= 0 && newCh <= 15) {
            CHANNEL = newCh;
            Serial.print("switched to channel "); Serial.println(CHANNEL);
          }
        } else {
          goTo(CHANNEL, buffer.toInt());
        }
      }
      buffer = "";
    } else {
      buffer += ch;
    }
  }
}
