/*
 * Motor PWM ramp test — Teensy 4.1 + motor driver PCB
 *
 * Sequence (per direction):
 *   0% -> 25% -> 50% -> 75% -> 100%, held 3s at each step
 *   Then an extra 3s hold at 100%
 *   Then stopped for 3s
 * Runs forward, then repeats the same sequence in reverse.
 *
 * BEFORE RUNNING: update MOTOR_PWM_PIN, MOTOR_DIR_PIN, and
 * DIR_FORWARD/DIR_REVERSE to match your driver's wiring.
 * If your driver has a separate ENABLE pin, add it — see note
 * in setup().
 */

const uint8_t MOTOR_PWM_PIN = 2;   // PWM signal into motor driver
const uint8_t MOTOR_DIR_PIN = 3;   // Direction pin on motor driver

const bool DIR_FORWARD = LOW;      // flip these if rotation is reversed
const bool DIR_REVERSE = HIGH;

const uint16_t STEP_DELAY_MS = 3000;
const uint8_t  PWM_MAX = 255;      // 8-bit PWM resolution (Teensy default)

const uint8_t RAMP_STEPS[] = {0, 25, 50, 75, 100};   // percent of full speed
const uint8_t NUM_RAMP_STEPS = sizeof(RAMP_STEPS) / sizeof(RAMP_STEPS[0]);

void setup() {
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);

  // If your driver needs an ENABLE pin held HIGH to run, add:
  // pinMode(MOTOR_EN_PIN, OUTPUT);
  // digitalWrite(MOTOR_EN_PIN, HIGH);

  setMotorSpeed(0, DIR_FORWARD);   // start stopped
}

void loop() {
  runDirectionSequence(DIR_FORWARD);
  runDirectionSequence(DIR_REVERSE);
}

// Ramps 0 -> 100% (3s per step), holds an extra 3s at 100%,
// then stops for 3s — all in the given direction.
void runDirectionSequence(bool direction) {
  for (uint8_t i = 0; i < NUM_RAMP_STEPS; i++) {
    setMotorSpeed(RAMP_STEPS[i], direction);
    delay(STEP_DELAY_MS);
  }

  setMotorSpeed(100, direction);   // extra hold at full speed
  delay(STEP_DELAY_MS);

  setMotorSpeed(0, direction);     // stop
  delay(STEP_DELAY_MS);
}

// Sets motor direction and speed (0-100%).
void setMotorSpeed(uint8_t percent, bool direction) {
  percent = constrain(percent, 0, 100);
  digitalWrite(MOTOR_DIR_PIN, direction);
  analogWrite(MOTOR_PWM_PIN, map(percent, 0, 100, 0, PWM_MAX));
}
