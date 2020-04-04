#include <Wire.h>
#include <TimerOne.h>
// ros libraries
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
// control
#include "control.h"
// #define DEBUG

#define LOOPTIME 100
#define FORWARD 1
#define REVERSE 0
unsigned long lastMilli = 0;

//PINS NAME
const int PIN_L_IN1 = 4;
const int PIN_L_IN2 = 5;
const int PIN_L_PWM = 3;
const int PIN_R_IN3 = 7;
const int PIN_R_IN4 = 8;
const int PIN_R_PWM = 9;
float throttle = 0; 
float angle = 0; 

void ControlRightMotor(uint8_t dir, uint8_t pwn);
void ControlLeftMotor(uint8_t dir, uint8_t pwn);

ros::NodeHandle nh;
void handle_cmd( const geometry_msgs::Twist& twist);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("uvcrobot/cmd_vel", handle_cmd);
pid_controller pid_z;

void ControlRightMotor(uint8_t dir, uint8_t pwn);
void ControlLeftMotor(uint8_t dir, uint8_t pwn);


void setup()
{
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.subscribe(cmd_vel);
  //Define os pinos como saida
  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_IN3, OUTPUT);
  pinMode(PIN_R_IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    //Sampling period
  if ((millis() - lastMilli) >= LOOPTIME)
  {
    nh.spinOnce();
  }
}

//Control function - Right Motors
void ControlRightMotor(uint8_t dir, uint8_t pwn)
{
  if (dir == FORWARD)
  {
    digitalWrite(PIN_L_IN1, HIGH);
    digitalWrite(PIN_L_IN2, LOW);
  }
  else
  {
    digitalWrite(PIN_L_IN1, LOW);
    digitalWrite(PIN_L_IN2, HIGH);
  }
  analogWrite(PIN_L_PWM, pwn);
}

//Control function - Left Motors
void ControlLeftMotor(uint8_t dir, uint8_t pwn)
{
  if (dir == FORWARD)
  {
    digitalWrite(PIN_R_IN3, LOW);
    digitalWrite(PIN_R_IN4, HIGH);
  }
  else
  {
    digitalWrite(PIN_R_IN3, HIGH);
    digitalWrite(PIN_R_IN4, LOW);
  }
  analogWrite(PIN_R_PWM, pwn);
}


void handle_cmd( const geometry_msgs::Twist& twist)
{
  throttle = twist.linear.x; 
  angle = twist.angular.z; 

  if (twist.linear.x <= 0)
  {
    ControlRightMotor(FORWARD, (uint8_t)abs(throttle));
    ControlLeftMotor(FORWARD, (uint8_t)abs(throttle));
  }
  else if (twist.linear.x > 0){
    ControlRightMotor(REVERSE, (uint8_t)throttle);
    ControlLeftMotor(REVERSE, (uint8_t)throttle);
  }

  #ifdef DEBUG
    Serial.println(throttle);
  #endif
}