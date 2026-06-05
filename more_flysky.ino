const int ch1Pin = 3;   // INT0
const int ch2Pin = 2;   // INT1
const int L_RPWM = 5;
const int L_LPWM = 6;
const int R_RPWM = 9;
const int R_LPWM = 10;

volatile uint16_t ch1Raw = 1500;
volatile uint16_t ch2Raw = 1500;
volatile uint32_t ch1Start = 0;
volatile uint32_t ch2Start = 0;

void ch1ISR() {
  if (digitalRead(ch1Pin) == HIGH)
    ch1Start = micros();
  else {
    uint16_t w = micros() - ch1Start;
    if (w > 900 && w < 2100) ch1Raw = w;
  }
}

void ch2ISR() {
  if (digitalRead(ch2Pin) == HIGH)
    ch2Start = micros();
  else {
    uint16_t w = micros() - ch2Start;
    if (w > 900 && w < 2100) ch2Raw = w;
  }
}

// ===============================
// Smoothing (exponential moving avg)
// ===============================
float smoothedLeft  = 0;
float smoothedRight = 0;
const float ALPHA   = 0.25;

// ===============================
// Deadband on motor output
// ===============================
const int DEADBAND = 15;
int applyDeadband(int val) {
  if (abs(val) < DEADBAND) return 0;
  return val;
}

// ===============================
// Center deadzone on raw RC pulse
// If within ±CENTER_ZONE of 1500, snap to 1500
// ===============================
const int CENTER_ZONE = 30;   // ±30µs around 1500 → tune this if needed
uint16_t applyCenterZone(uint16_t pulse) {
  if (pulse > (1500 - CENTER_ZONE) && pulse < (1500 + CENTER_ZONE))
    return 1500;
  return pulse;
}

void setup() {
  Serial.begin(115200);
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ch1Pin), ch1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), ch2ISR, CHANGE);

  Serial.println("Ready");
}

void loop() {
  noInterrupts();
  uint16_t ch1 = ch1Raw;
  uint16_t ch2 = ch2Raw;
  interrupts();

  // FAILSAFE
  if (ch1 < 900 || ch1 > 2100 || ch2 < 900 || ch2 > 2100) {
    driveMotor(L_RPWM, L_LPWM, 0);
    driveMotor(R_RPWM, R_LPWM, 0);
    smoothedLeft  = 0;
    smoothedRight = 0;
    Serial.println("FAILSAFE — CH1: " + String(ch1) + "  CH2: " + String(ch2));
    delay(20);
    return;
  }

  // Snap jittery center values to exactly 1500
  ch1 = applyCenterZone(ch1);
  ch2 = applyCenterZone(ch2);

  int throttle = map(ch1, 900, 2100, -255, 255);
  int steering  = map(ch2, 900, 2100, -255, 255);

  throttle = constrain(throttle, -255, 255);
  steering  = constrain(steering,  -255, 255);

  throttle = applyDeadband(throttle);
  steering  = applyDeadband(steering);

  int leftTarget  = constrain(throttle + steering, -255, 255);
  int rightTarget = constrain(throttle - steering, -255, 255);

  smoothedLeft  += ALPHA * (leftTarget  - smoothedLeft);
  smoothedRight += ALPHA * (rightTarget - smoothedRight);

  int leftSpeed  = (int)smoothedLeft;
  int rightSpeed = (int)smoothedRight;

  driveMotor(L_RPWM, L_LPWM, leftSpeed);
  driveMotor(R_RPWM, R_LPWM, rightSpeed);

  // ── Serial print ──────────────────────────────────────────
  Serial.print("CH1: ");      Serial.print(ch1);
  Serial.print("  CH2: ");    Serial.print(ch2);
  Serial.print("  THR: ");    Serial.print(throttle);
  Serial.print("  STR: ");    Serial.print(steering);
  Serial.print("  L_spd: ");  Serial.print(leftSpeed);
  Serial.print("  R_spd: ");  Serial.println(rightSpeed);
  // ──────────────────────────────────────────────────────────

  delay(20);
}

void driveMotor(int rpwm, int lpwm, int speedVal) {
  if (speedVal > 0) {
    analogWrite(rpwm, speedVal);
    analogWrite(lpwm, 0);
  } else if (speedVal < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speedVal);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}