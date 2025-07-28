const int en1A = 2;
const int en1B = 4;
const int m1 = 12;
const int pwm1 = 3;

const int en2A = 5;
const int en2B = 6;
const int m2 = 13;
const int pwm2 = 11;

const float spd_target = 3.0;
float spd_current = 0.0;
volatile int count = 0;
float interval = 200, t0 = 0, ppr = 4096;

//pid variables
#define kp 10
#define ki 18
#define kd 1.0
float T_curr, T_prev = 0;
float theta;
float theta_prev=0;
double t = 0;
double t_prev = 0;
float E_int = 0, E_der, E_prev = 0;

float PIDCalc(float given, float target)
{
  //time calc
  T_curr = millis();
  float dt = (float)(T_curr - T_prev)/1000.0;
  T_prev = T_curr;

  //error
  float E = target - given;
//    Serial.print("E = ");
//    Serial.println(E);
  //integral
  E_int = E_int + E*dt;
//    Serial.print("E_int = ");
//    Serial.println(E_int);
  //derivative
  E_der = (E - E_prev)/dt;
  E_prev = E;

  float effort = kp*E + ki*E_int + kd*E_der;
//    Serial.print("effort = ");
//    Serial.println(effort);
  return effort;
}

void actuate(float effort)
{
  //effort signal process
  int dir;
  int power = fabs(effort);
  if(power>255){
    power = 255;
  }
//  if(power<150){
//    power = 255;
//  }

  if(effort>0){
    dir=1;
  }
  if(effort<0){
    dir=-1;
  }

  //actuating
  analogWrite(pwm2, power);
  if(dir==1){
    digitalWrite(m2,HIGH);
  }
  if(dir==-1){
    digitalWrite(m2,LOW);
  }
}

void setup()
{
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  pinMode(pwm2, OUTPUT);
  pinMode(m2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(en2A), pulse_count, RISING);
  Serial.begin(115200);
}

void loop()
{
  T_curr = millis();

  if((T_curr - t)> interval)
  {
    
    t = millis();
    theta = (float) (count/4096.0);    
    float dt = ((float) (t-t_prev)); 

    spd_current = (float)(theta - theta_prev)/(dt/1000.0);
    count = 0;
    t_prev=t;
  }
  actuate(PIDCalc(spd_current, spd_target));
  Serial.print("Speed current ");
  Serial.println(String(spd_current));
}

void pulse_count()
{
 count++;
// Serial.print("I'm called!!!!");
// Serial.println(count);
}
