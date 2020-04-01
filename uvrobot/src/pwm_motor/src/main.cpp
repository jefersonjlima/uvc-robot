#include <Adafruit_BNO055.h>
#include <TimerOne.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#define LOOPTIME 100
#define FORWARD 0
#define REVERSE 1
unsigned long lastMilli = 0;

//PINS NAME
const int PIN_L_IN1 = 4;
const int PIN_L_IN2 = 5;
const int PIN_L_PWM = 3;
const int PIN_R_IN3 = 7;
const int PIN_R_IN4 = 8;
const int PIN_R_PWM = 9;

//PWM TEST
int count = 0;

void ControlRightMotor(bool dir, uint8_t pwn);
void ControlLeftMotor(bool dir, uint8_t pwn);

void setup()
{
  //Define os pinos como saida
  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_IN3, OUTPUT);
  pinMode(PIN_R_IN4, OUTPUT);
}

void loop()
{
  //Sampling period
  if ((millis() - lastMilli) >= LOOPTIME)
  {
    lastMilli = millis();

    ControlRightMotor(FORWARD, count);
    ControlLeftMotor(REVERSE, count);
    count++;
  }
}

//Control function - Right Motors
void ControlLeftMotor(bool dir, uint8_t pwn)
{
  if (dir)
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
void ControlRightMotor(bool dir, uint8_t pwn)
{
  if (dir)
  {
    digitalWrite(PIN_R_IN3, HIGH);
    digitalWrite(PIN_R_IN4, LOW);
  }
  else
  {
    digitalWrite(PIN_R_IN3, LOW);
    digitalWrite(PIN_R_IN4, HIGH);
  }
  analogWrite(PIN_R_PWM, pwn);
}