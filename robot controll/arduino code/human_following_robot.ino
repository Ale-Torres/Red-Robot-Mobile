// Pin definitions for Motor 1 (Left motor)
const int en1A = 2; // Yellow wire for Motor 1 encoder channel A
const int en1B = 4; // Gray wire for Motor 1 encoder channel B
const int m1 = 12;  // Direction control pin for Motor 1
const int pwm1 = 3; // PWM control pin for Motor 1

// Pin definitions for Motor 2 (Right motor)
const int en2A = 5; // Yellow wire for Motor 2 encoder channel A
const int en2B = 6; // Gray wire for Motor 2 encoder channel B
const int m2 = 13;  // Direction control pin for Motor 2
const int pwm2 = 11;// PWM control pin for Motor 2

// Speed target in rotations per second
const float spd_target = 2; // Base speed (will scale down for slow mode)

char ch = ' '; // Character input from Serial

// Variables to hold current speeds
float spd_current1 = 0.0;
float spd_current2 = 0.0;

// Encoder pulse counters
volatile int count1 = 0;
volatile int count2 = 0;

// Timing and encoder configuration
float interval = 200;    // Interval in milliseconds for speed calculation
float ppr = 4096;        // Pulses per revolution for encoder

// PID constants
#define kp 16
#define ki 25
#define kd 0.1

// Time tracking for PID
float T_curr;
float T_prev1 = 0; // For Motor 1 PID
float T_prev2 = 0; // For Motor 2 PID

float T1_curr, T1_prev = 0;   // Timing for Motor 1
float theta1;                 // Position of Motor 1
float theta1_prev = 0;          
double t1 = 0;
double t1_prev = 0;

float T2_curr, T2_prev = 0;   // Timing for Motor 2
float theta2;                 // Position of Motor 2
float theta2_prev = 0;
double t2 = 0;
double t2_prev = 0;

// PID error components for each motor
float E1_int = 0, E1_der, E1_prev = 0;
float E2_int = 0, E2_der, E2_prev = 0;

// PID controller for Motor 1
float PIDCalc1(float given, float target) {
  T_curr = millis();
  float dt = (float)(T_curr - T_prev1) / 1000.0;
  T_prev1 = T_curr;

  float E = target - given;
  E1_int += E * dt;
  E1_der = (E - E1_prev) / dt;
  E1_prev = E;

  float effort = kp * E + ki * E1_int + kd * E1_der;
  return effort;
}

// PID controller for Motor 2
float PIDCalc2(float given, float target) {
  T_curr = millis();
  float dt = (float)(T_curr - T_prev2) / 1000.0;
  T_prev2 = T_curr;

  float E = target - given;
  E2_int += E * dt;
  E2_der = (E - E2_prev) / dt;
  E2_prev = E;

  float effort = kp * E + ki * E2_int + kd * E2_der;
  return effort;
}

// Function to actuate both motors simultaneously
void actuate2(float effort1, float effort2) {
  int dir1 = (effort1 >= 0) ? HIGH : LOW;
  int dir2 = (effort2 >= 0) ? LOW : HIGH;  // reverse logic if motor is wired opposite

  int power1 = abs(effort1);
  int power2 = abs(effort2);

  if (power1 > 255) power1 = 255;
  if (power2 > 255) power2 = 255;

  analogWrite(pwm1, power1);
  analogWrite(pwm2, power2);

  digitalWrite(m1, dir1);
  digitalWrite(m2, dir2);
}
void resetPID() {
  E1_int = 0;
  E1_prev = 0;
  E2_int = 0;
  E2_prev = 0;
}


// Stop both motors
void stop() {
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
  digitalWrite(m1, 0);
  digitalWrite(m2, 0);
}

// PID-controlled slow forward
void forwardSlow() {
  // --- MOTOR 1 SPEED MEASUREMENT ---
  T1_curr = millis();

  if ((T1_curr - t1) > interval)
  {
    t1 = millis(); // Update current time
    theta1 = (float)(count1 / ppr); // Convert pulses to rotations
    float dt1 = (float)(t1 - t1_prev); // Time difference in ms

    // Compute speed (rotations/sec)
    spd_current1 = (theta1 - theta1_prev) / (dt1 / 1000.0);

    // Reset for next cycle
    count1 = 0;
    theta1_prev = theta1;
    t1_prev = t1;
  }

  // Compute PID effort for Motor 1
  float effort1 = PIDCalc1(spd_current1, spd_target);
 

  // --- MOTOR 2 SPEED MEASUREMENT ---
  T2_curr = millis();

  if ((T2_curr - t2) > interval)
  {
    t2 = millis();
    theta2 = (float)(count2 / ppr);
    float dt2 = (float)(t2 - t2_prev);

    spd_current2 = (theta2 - theta2_prev) / (dt2 / 1000.0);

    count2 = 0;
    theta2_prev = theta2;
    t2_prev = t2;
  }

  // Compute PID effort for Motor 2
  float effort2 = PIDCalc2(spd_current2, spd_target);
 

  // Send efforts to both motors
  actuate2(effort1, effort2);
}

// Setup function
void setup() {
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  pinMode(pwm1, OUTPUT);
  pinMode(m1, OUTPUT);

  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  pinMode(pwm2, OUTPUT);
  pinMode(m2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(en1A), pulse_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(en2A), pulse_count2, RISING);

  Serial.begin(115200);
  Serial.println("start");
}
void backwardSlow() {
  // --- MOTOR 1 SPEED MEASUREMENT ---
  T1_curr = millis();

  if ((T1_curr - t1) > interval)
  {
    t1 = millis(); // Update current time
    theta1 = (float)(count1 / ppr); // Convert pulses to rotations
    float dt1 = (float)(t1 - t1_prev); // Time difference in ms

    // Compute speed (rotations/sec)
    spd_current1 = (theta1 - theta1_prev) / (dt1 / 1000.0);

    // Reset for next cycle
    count1 = 0;
    theta1_prev = theta1;
    t1_prev = t1;
  }

  // Compute PID effort for Motor 1 (negate target speed)
  float effort1 = PIDCalc1(spd_current1, -spd_target);  // negative target


  // --- MOTOR 2 SPEED MEASUREMENT ---
  T2_curr = millis();

  if ((T2_curr - t2) > interval)
  {
    t2 = millis();
    theta2 = (float)(count2 / ppr);
    float dt2 = (float)(t2 - t2_prev);

    spd_current2 = (theta2 - theta2_prev) / (dt2 / 1000.0);

    count2 = 0;
    theta2_prev = theta2;
    t2_prev = t2;
  }

  // Compute PID effort for Motor 2 (negate target speed)
  float effort2 = PIDCalc2(spd_current2, -spd_target);  // negative target


  // Send efforts to both motors
  actuate2(effort1, effort2);
}
void turnRight() {
  // Set motor directions for turning right:
  // Left motor forward
  digitalWrite(m1, HIGH);  
  // Right motor backward (since your actuate2 reverses logic, HIGH should work)
  digitalWrite(m2, HIGH);  

  // Set motor speeds (adjust as needed)
  analogWrite(pwm1, 60);
  analogWrite(pwm2, 60);

}
void turnLeft() {
  // Set motor directions for turning right:
  // Left motor forward
  digitalWrite(m1, LOW);  
  // Right motor backward (since your actuate2 reverses logic, HIGH should work)
  digitalWrite(m2, LOW);  

  // Set motor speeds (adjust as needed)
  analogWrite(pwm1, 60);
  analogWrite(pwm2, 60);

}

// Main loop
void loop() {
  // Only update command if there is new serial data
  if (Serial.available() > 0) {
    char newCh = Serial.read();

    if (newCh == 'w' || newCh == 'a' || newCh == 's' || newCh == 'd' || newCh == ' ') {
      if (newCh != ch) {
        ch = newCh;
        
        resetPID();  // optional: reset PID when switching commands
      }
    }
  }


  // Run the appropriate function continuously
  Serial.print("Command: ");
  Serial.println(ch);
  
  if (ch == 'w') {
    forwardSlow();
  }
  else if (ch == 's') {
    backwardSlow();
  }
  else if (ch == 'a') {
    turnLeft();
  }
  else if (ch == 'd') {
    turnRight();
  }
  else if (ch == ' ') {
    stop();
  }
}


// ISR for Motor 1 encoder
void pulse_count1() {
  count1++;
}

// ISR for Motor 2 encoder
void pulse_count2() {
  count2++;
}
