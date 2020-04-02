#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TimerOne.h>
// ros libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
// control
#include "control.h"
// #define DEBUG

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

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

ros::NodeHandle nh;
int reverse = 0;
float throttle = 0; 
float angle = 0; 

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  throttle = twist.linear.x; 
  angle = twist.angular.z; 

  if (twist.linear.x <= 0 && reverse == 0)
  {

  }
  else if (twist.linear.x <= 0 && reverse > 0)
  {
  
  }
  else if (twist.linear.x > 0){

    reverse = 0;
  }
  #ifdef DEBUG
    Serial.println(throttle);
  #endif
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("/uvc-robots/cmd_vel", cmdVelCB);

//PWM TEST
int count = 0;
pid_controller pid_z;

void ControlRightMotor(uint8_t dir, uint8_t pwn);
void ControlLeftMotor(uint8_t dir, uint8_t pwn);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup()
{
  Serial.begin(9600);
  #ifdef DEBUG
    Serial.println("Orientation Sensor Test");
    Serial.println("");]
  #endif

  /* Initialise the sensor */

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    #ifdef DEBUG
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    #endif
    while (1);
  }
  else
  {
    #ifdef DEBUG
      Serial.println("OK BNO055!");
    #endif

  }

  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  /* Display some basic information on this sensor */
  #ifdef DEBUG
    displaySensorDetails();
  #endif
  //Define os pinos como saida
  pinMode(PIN_L_PWM, OUTPUT);
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT);
  pinMode(PIN_R_IN3, OUTPUT);
  pinMode(PIN_R_IN4, OUTPUT);
  
  #ifndef DEBUG
    nh.initNode();
    nh.subscribe(subCmdVel);
  #endif
}

void loop()
{
  #ifndef DEBUG
    nh.spinOnce();
  #endif
}

//Control function - Right Motors
void ControlRightMotor(uint8_t dir, uint8_t pwn)
{
  if (dir == REVERSE)
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
  if (dir == REVERSE)
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