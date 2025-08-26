//Including necessary libraries

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/Twist.h>

//Creating a ROS node
ros::NodeHandle_<ArduinoHardware, 1, 1, 552, 552> nh;;

//Creating subscriber and publisher nodes
std_msgs::Float32 msg;
//std_msgs::Int16 count1;
//std_msgs::Int16 count2;
//geometry_msgs::Twist msg;

//Encoder variables
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

int count1 = 0;
int count2 = 0;

//Choice character
char ch = 'w';

//Speed variables
float spd_target = 1.0;
float spd_current1 = 0.0;
float spd_current2 = 0.0;

//Timing variables
float interval = 200, t0 = 0, ppr = 4096;

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

//pid variables
#define kp 10
#define ki 18
#define kd 1.0

float E1_int = 0, E1_der, E1_prev = 0;
float E2_int = 0, E2_der, E2_prev = 0;

//ros::Publisher pulse_count1("left_ticks", &count1);
// 
//ros::Publisher pulse_count2("right_ticks", &count2);

float st_time = millis();

float move1;
float move2;

//Callback function for subscriber to receive target velocity in rad/s
void motorVel( const std_msgs::Float32& msg)
{
  spd_target = msg.data;
}

//Callback function for subscriber to receive target velocity in rad/s
//void motorCtrl( const geometry_msgs::Twist& cmd_vel)
//{
//  move1 = cmd_vel.linear.x;
//  move2 = cmd_vel.angular.z;
//  
//  if (move1 > 0 && move2 == 0)
//  {
//    ch = 'w';
//  }
//  else if (move1 > 0 && move2 > 0)
//  {
//    ch = 'a';
//  }
//  else if (move1 > 0 && move2 < 0)
//  {
//    ch = 'd';
//  }
//  else if (move1 < 0)
//  {
//    ch = 's';
//  }
//}

//void motorCtrl( const std_msgs::Char& choice)
//{
//  ch = choice.data;
//}

//ros::Subscriber<std_msgs::data_type> subscriber_name("topic_name", parameter_name)

//ROS subsriber receiving target velocity of Float32 type
ros::Subscriber<std_msgs::Float32> sub1("runmotor", &motorVel);

//ROS subsriber receiving target velocity of Char type
//ros::Subscribertd_msgs::Char> sub2("controlmotor", &motorCtrl);

//Function to make the bot run straight
void run_straight(float power1, float power2)
{
  digitalWrite(m1, HIGH);
  digitalWrite(m2, LOW);
  analogWrite(pwm1, power1);
  analogWrite(pwm2, power2);
}

//Function to make the bot run in reverse
void reverse(float power1, float power2)
{
  digitalWrite(m1, LOW);
  digitalWrite(m2, HIGH);
  analogWrite(pwm1, power1);
  analogWrite(pwm2, power2);
}

//Function to make the bot take a left
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

//Function to make the bot take a right
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

//Function to make the bot take a u-turn
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

//Function to make the bot stop
void brake()
{
  digitalWrite(br1, HIGH);
  digitalWrite(br2, HIGH);
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

//PID calculation for left motor
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

//PID calculation for right motor
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

//Actuation function
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
//  if (Serial.available()) 
//  {
////    char ch = Serial.read();
//
//    char temp = Serial.read();
//    if (temp == 'w' || temp == 'a' || temp == 's' || temp == 'd' || temp == 'u' || temp == ' ')
//    {
//      ch = temp;
//    }
//  }
//      Serial.println(ch);

      ch = 'w';

//    Bot motion based on choice
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
  
  //Pin setup 
  pinMode(en1A, INPUT_PULLUP);
  pinMode(en1B, INPUT_PULLUP);
  pinMode(pwm1, OUTPUT);
  pinMode(m1, OUTPUT);
  
  pinMode(en2A, INPUT_PULLUP);
  pinMode(en2B, INPUT_PULLUP);
  pinMode(pwm2, OUTPUT);
  pinMode(m2, OUTPUT);
  
  //Interrupt for tracking encoder pulse change
  attachInterrupt(digitalPinToInterrupt(en1A), calc_pulse_count1, RISING);
  attachInterrupt(digitalPinToInterrupt(en2A), calc_pulse_count2, RISING);
  
  //ROS node setup
//  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub1);
//  nh.subscribe(sub2);
//  nh.advertise(pulse_count1);
//  nh.advertise(pulse_count2);
  
  Serial.begin(57600);
}

void loop()
{
  nh.spinOnce();
  Serial.println(spd_target);
//  Serial.println(msg.data);

  //Left - Motor 1
  //Speed calculation
  
  T1_curr = millis();

  if((T1_curr - t1)> interval)
  {   
//    pulse_count1.publish(&count1);
    t1 = millis();
    theta1 = (float) (count1/4096.0);    
    float dt1 = ((float) (t1-t1_prev)); 

    spd_current1 = (float)(theta1 - theta1_prev)/(dt1/1000.0);
//    count1.data = 0;
    count1 = 0;
    t1_prev=t1;
  }

  float effort1 = PIDCalc1(spd_current1, spd_target);
  Serial.print("Speed current 1 = ");
  Serial.println(spd_current1);
  
  //Right - Motor 2
  //Speed calculation

  T2_curr = millis();

  if((T2_curr - t2)> interval)
  {   
//    pulse_count2.publish(&count2);
    t2 = millis();
    theta2 = (float) (count2/4096.0);    
    float dt2 = ((float) (t2-t2_prev));
     
    spd_current2 = (float)(theta2 - theta2_prev)/(dt2/1000.0);
//    count2.data = 0;
    count2 = 0;
    t2_prev=t2;
  }

  float effort2 = PIDCalc2(spd_current2,spd_target);
  
  Serial.print("Speed current 2 = ");
  Serial.println(spd_current2);

  actuate2(effort1, effort2);

  //delay(1);
}

//Function to increment pulse count for left motor
void calc_pulse_count1()
{
// count1.data++;
  count1++;
}

//Function to increment pulse count for right motor
void calc_pulse_count2()
{
// count2.data++;
  count2++;
}
