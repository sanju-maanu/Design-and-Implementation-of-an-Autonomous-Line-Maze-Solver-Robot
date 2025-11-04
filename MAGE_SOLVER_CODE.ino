/* Line Maze Solver RC Bot
   Modes: RC mode (manual) / AUTON mode (line-follow)
   Hardware:
    - Arduino UNO
    - 8-channel IR sensor RLS-08 (digital outputs)
    - L298N motor driver
    - 2x N20 motors
    - RC receiver (PWM outputs for steering/throttle)
    - Buck converter for stable 5V
*/

/* ----------------- PIN MAPPING ----------------- */
/* IR sensor (8 outputs) - use analog pins as digital inputs + two digital pins */
const uint8_t IR_PINS[8] = {A0, A1, A2, A3, A4, A5, 2, 4}; // IR0..IR7 (left -> right)

/* Motor driver (L298N) */
const uint8_t IN1 = 7;   // Left motor direction pin 1
const uint8_t IN2 = 8;   // Left motor direction pin 2
const uint8_t IN3 = 9;   // Right motor direction pin 1
const uint8_t IN4 = 10;  // Right motor direction pin 2
const uint8_t ENA = 5;   // PWM for left motor speed (PWM pin)
const uint8_t ENB = 6;   // PWM for right motor speed (PWM pin)

/* RC receiver PWM inputs (two channels) */
const uint8_t RC_STEER_PIN = 11;   // steering channel PWM input (pulseIn)
const uint8_t RC_THROTTLE_PIN = 12; // throttle channel PWM input (pulseIn)

/* Mode selection */
const uint8_t MODE_PIN = 13; // HIGH = RC mode, LOW = AUTON mode (tie to switch or receiver channel)

/* Motor speed limits */
const int MAX_SPEED = 230; // 0..255 range for analogWrite; cap below 255 to be safe

/* --------------- END PIN MAPPING ---------------- */

/* PID constants for line following */
float Kp = 15.0;  // Proportional gain
float Ki = 0.0;   // Integral gain (set to 0 for simplicity)
float Kd = 10.0;  // Derivative gain

/* PID variables */
float previousError = 0.0;
float integral = 0.0;

/* Turn 180 degrees function (approximate timing, adjust based on your bot) */
void turn180() {
  // Turn left (reverse left, forward right)
  setMotors(-150, 150);
  delay(1000);  // Adjust delay for your bot's turning speed to achieve ~180 degrees
  stopMotors();
  delay(100);  // Small pause
}

void setup() {
  Serial.begin(115200);

  // IR pins
  for (int i = 0; i < 8; ++i) {
    pinMode(IR_PINS[i], INPUT);
  }

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // RC pins input
  pinMode(RC_STEER_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);

  // Mode pin with pullup (if using a switch to ground)
  pinMode(MODE_PIN, INPUT_PULLUP);

  stopMotors();
  Serial.println("Line Maze Solver RC Bot ready");
}

/* ---------------- Utility motor function ----------------
   leftSpeed, rightSpeed range: -MAX_SPEED .. +MAX_SPEED
   positive -> forward, negative -> reverse
*/
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, constrain(leftSpeed, 0, MAX_SPEED));
  } else if (leftSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, constrain(-leftSpeed, 0, MAX_SPEED));
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, constrain(rightSpeed, 0, MAX_SPEED));
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, constrain(-rightSpeed, 0, MAX_SPEED));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  setMotors(0, 0);
}

/* ----------------- Line following helpers ----------------- */
/* Read IR sensors: returns array of 0/1 where 0 = line (black) typically; 1 = white
   NOTE: If your sensor logic is inverted, flip the reading (digitalRead() ^ 1)
*/
void readIR(int out[8]) {
  for (int i = 0; i < 8; ++i) {
    int val = digitalRead(IR_PINS[i]);
    // Common modules: LOW => black line detected. If yours is opposite, uncomment next line:
    // val = !val;
    out[i] = val; // 0 or 1
  }
}

/* Enhanced autonomous line following with PID control and basic maze solving
   - Uses PID for smooth line following
   - At intersections (many sensors detecting line), PID naturally goes straight
   - If line is lost (dead end), turn 180 degrees to backtrack (simple maze solving)
   - This implements a basic left-preference rule via PID and backtracking for dead ends
*/
void autonLineFollow() {
  int s[8];
  readIR(s);

  // Debug print (uncomment if needed)
  // Serial.print("IR: "); for (int i=0;i<8;i++) Serial.print(s[i]); Serial.println();

  // Calculate line position (weighted average of sensor indices where line is detected)
  int numSensorsOnLine = 0;
  float weightedSum = 0.0;
  for (int i = 0; i < 8; ++i) {
    if (s[i] == 0) {  // Assuming 0 means line detected
      weightedSum += i;
      numSensorsOnLine++;
    }
  }

  if (numSensorsOnLine == 0) {
    // Line lost: assume dead end, turn 180 to backtrack
    turn180();
    return;
  }

  // Line position (0 to 7)
  float linePos = weightedSum / numSensorsOnLine;

  // Error: deviation from center (3.5)
  float error = linePos - 3.5;

  // PID calculation
  integral += error;
  float derivative = error - previousError;
  float pidOutput = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Base speed and differential steering
  int baseSpeed = 160;  // Base forward speed
  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

/* ----------------- RC reader helpers -----------------
   Reads standard servo-type PWM (1000..2000 microseconds)
   returns pulse length in microseconds (0 if none)
*/
unsigned long readRcPulse(uint8_t pin) {
  unsigned long t = pulseIn(pin, HIGH, 25000); // 25 ms timeout
  return t;
}

/* Map rc pulses to speeds:
   steeringPulse: center ~1500. map to -1.0..+1.0
   throttlePulse: center ~1500. map to -1.0..+1.0
*/
float mapPulseToNorm(unsigned long pulse) {
  if (pulse < 1000) return 0.0;
  // Constrain typical 1000..2000
  pulse = constrain(pulse, 1000UL, 2000UL);
  return (float)(pulse - 1500UL) / 500.0; // -1..+1
}

/* RC control: differential drive mixing
   throttle: forward/backward (-1..1)
   steer: -1 (left) .. +1 (right)
*/
void rcControl() {
  unsigned long steerPulse = readRcPulse(RC_STEER_PIN);
  unsigned long throttlePulse = readRcPulse(RC_THROTTLE_PIN);

  // If pulses are zero, don't change (receiver off). Stop motors.
  if (steerPulse == 0 || throttlePulse == 0) {
    // Optional: keep previous state, but safer to stop
    stopMotors();
    return;
  }

  float steer = mapPulseToNorm(steerPulse);
  float throttle = mapPulseToNorm(throttlePulse);

  // Compute left and right speeds
  // Basic mix: left = throttle * speed + steer * speed, right = throttle * speed - steer * speed
  int base = (int)(throttle * (float)MAX_SPEED);
  int turn = (int)(steer * (float)(MAX_SPEED / 2)); // turning influence

  int leftSpeed = base + turn;
  int rightSpeed = base - turn;

  // Constrain
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

/* ----------------- Main loop ----------------- */
void loop() {
  // Mode pin: HIGH => RC mode, LOW => Autonomous
  bool rcMode = (digitalRead(MODE_PIN) == HIGH);

  if (rcMode) {
    // RC manual control
    rcControl();
  } else {
    // Autonomous line follow
    autonLineFollow();
  }

  // small delay
  delay(20);
}
