const int en1A = 2;
const int en1B = 4;
const int m1 = 12;
const int pwm1 = 3;
const int br1 = 9;

const int en2A = 5;
const int en2B = 6;
const int m2 = 13;
const int pwm2 = 11;
const int br2 = 8;

char ch = ' ';

const float spd_target = 3.0;
float spd_current = 0.0;
float spd_current1 = 0.0;
float spd_current2 = 0.0;

volatile int count1 = 0;
volatile int count2 = 0;
float interval = 200, t0 = 0, ppr = 4096;

int counter = 0;

//pid variables
#define kp 10
#define ki 18
#define kd 1.0

float T_curr, T_prev = 0;
float T1_curr, T1_prev = 0;
float theta1;
float theta1_prev=0;
double t1 = 0;
double t1_prev = 0;

float T2_curr, T2_prev = 0;
float theta2;
float theta2_prev=0;
double t2 = 0;
double t2_prev = 0;

float E1_int = 0, E1_der, E1_prev = 0;
float E2_int = 0, E2_der, E2_prev = 0;

float st_time = millis();

void run_straight(float power1, float power2)
{
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  analogWrite(pwm1, power1);
  analogWrite(pwm2, power2);
}

void reverse(float power1, float power2)
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  analogWrite(pwm1, power1);
  analogWrite(pwm2, power2);
}

void left(float power1, float power2)
{
  //takes left till given angle then goes straight at that angle
  float ang = (45.0 * PI)/180.0;
  float t = ang/spd_target;
  float st_time = millis();
  
  while((millis()-st_time)<=(t*1000))
  {
    analogWrite(pwm1, power1);
    analogWrite(pwm2, power2);
    
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
  }

  ch = 'w';
  
//  digitalWrite(m1, HIGH);
//  digitalWrite(m2, LOW);
}

void right(float power1, float power2)
{
  float ang = (45.0 * PI)/180.0;
  float t = ang/spd_target;
  float st_time = millis();

  while((millis()-st_time)<=(t*1000))
  {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    
    analogWrite(pwm1, power1);
    analogWrite(pwm2, power2);
  }

  ch = 'w';
  
//  digitalWrite(m1, HIGH);
//  digitalWrite(m2, LOW);
}

void u_turn(float power1, float power2)
{
  float t = PI/spd_target;
  float st_time = millis();

  while((millis()-st_time)<=(t*1000))
  {
    //performs cloclwise u-turn
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    
    analogWrite(pwm1, power1);
    analogWrite(pwm2, power2);
  }

  ch = 'w';
  
//  digitalWrite(m1, HIGH);
//  digitalWrite(m2, LOW);
}

void brake()
{
  digitalWrite(br1, HIGH);
  digitalWrite(br2, HIGH);
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

float PIDCalc1(float given, float target)
{
  //time calc
  T_curr = millis();
  float dt = (float)(T_curr - T_prev)/1000.0;
  T_prev = T_curr;

  //error
  float E = target - given;
  
  //integral
  E1_int = E1_int + E*dt;
  
  //derivative
  E1_der = (E - E1_prev)/dt;
  E1_prev = E;

  float effort = kp*E + ki*E1_int + kd*E1_der;
  return effort;
}

float PIDCalc2(float given, float target)
{
  //time calc
  T_curr = millis();
  float dt = (float)(T_curr - T_prev)/1000.0;
  T_prev = T_curr;

  //error
  float E = target - given;
  
  //integral
  E2_int = E2_int + E*dt;

  //derivative
  E2_der = (E - E2_prev)/dt;
  E2_prev = E;

  float effort = kp*E + ki*E2_int + kd*E2_der;
  return effort;
}

void actuate2(float effort1, float effort2)
{
  //effort signal process
//  int dir;
  int power1 = fabs(effort1);
  int power2 = fabs(effort2);
  
  if(power1>255){
    power1 = 255;
  }
  if(power2>255){
    power2 = 255;
  }

  //actuating
//
  if (Serial.available()) 
  {
//    char ch = Serial.read();

    char temp = Serial.read();
    if (temp == 'w' || temp == 'a' || temp == 's' || temp == 'd' || temp == 'u' || temp == ' ')
    {
      ch = temp;
    }
  }
      Serial.println(ch);

      if (ch == 'w')
      {
        run_straight(power1, power2);
      }        
      else if (ch == 's')
      {
        reverse(power1, power2);
      }
      else if (ch == 'a')
      {
        left(power1, power2);
      } 
      else if (ch == 'd')
      {
        right(power1, power2);
      }  
      else if (ch == 'u')
      {
        u_turn(power1, power2);
      }
      else if (ch == ' ')
      {
        brake();  
      }
    
    else ;  
}

void setup()
{
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
}

void loop()
{
//Left - Motor 1
//Speed calculation
  
  T1_curr = millis();

  if((T1_curr - t1)> interval)
  {   
    
    t1 = millis();
    theta1 = (float) (count1/4096.0);    
    float dt1 = ((float) (t1-t1_prev)); 

    spd_current1 = (float)(theta1 - theta1_prev)/(dt1/1000.0);
    count1 = 0;
    t1_prev=t1;
  }

  float effort1 = PIDCalc1(spd_current1, spd_target);
  Serial.print("Speed current 1 = ");
  Serial.println(spd_current1);
//Serial.print("Effort 1 = ");
//  Serial.println(effort1);
  
//  //Right - Motor 2

  T2_curr = millis();

  if((T2_curr - t2)> interval)
  {   
    
    t2 = millis();
    theta2 = (float) (count2/4096.0);    
    float dt2 = ((float) (t2-t2_prev)); 

    spd_current2 = (float)(theta2 - theta2_prev)/(dt2/1000.0);
    count2 = 0;
    t2_prev=t2;
  }

  float effort2 = PIDCalc2(spd_current2, spd_target);
  Serial.print("Speed current 2 = ");
  Serial.println(spd_current2);
//Serial.print("Effort 2 = ");
//  Serial.println(effort2);

  actuate2(effort1, effort2);
}

void pulse_count1()
{
 count1++;
}

void pulse_count2()
{
 count2++;
}
