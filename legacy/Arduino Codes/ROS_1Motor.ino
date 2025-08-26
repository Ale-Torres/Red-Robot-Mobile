/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/Empty.h>


ros::NodeHandle  nh;
std_msgs::Int32 pwmval;

void motorCb( const std_msgs::Int32& pwmval){
//  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
    digitalWrite(12, HIGH);
    analogWrite(3, pwmval.data);
    digitalWrite(13, LOW);
    analogWrite(11, pwmval.data);
}

ros::Subscriber<std_msgs::Int32> sub("runmotor", &motorCb );

void setup()
{ 
  pinMode(3, OUTPUT);
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
