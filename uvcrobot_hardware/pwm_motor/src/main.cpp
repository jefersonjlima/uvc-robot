#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// ros libraries
#define USE_USBCON // arduino due parameter
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

// control
#include "control.h"
// #define DEBUG
#define Adafruit_BNO055_ENABLE

#ifdef Adafruit_BNO055_ENABLE
#define BNO055_SAMPLERATE_DELAY_MS (20)
#endif
#define LOOPTIME 100
#define FORWARD 1
#define REVERSE 0
unsigned long lastMilli = 0;

//ARDUINO DUE PINS
const int PIN_L_IN1 = 22;
const int PIN_L_IN2 = 24;
const int PIN_L_PWM = 30;
const int PIN_R_IN3 = 26;
const int PIN_R_IN4 = 29;
const int PIN_R_PWM = 32;
int reverse = 0;
float throttle = 0;
float angle = 0;

#ifdef Adafruit_BNO055_ENABLE
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
#endif

void ControlRightMotor(uint8_t dir, uint8_t pwn);
void ControlLeftMotor(uint8_t dir, uint8_t pwn);
#ifdef Adafruit_BNO055_ENABLE
void Adafruit_BNO055_Details(void);
#endif
ros::NodeHandle nh;
#ifdef Adafruit_BNO055_ENABLE
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("uvcrobot/imu", &imu_msg);
#endif
void handle_cmd( const geometry_msgs::Twist& twist);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("uvcrobot/cmd_vel", handle_cmd);
pid_controller pid_z;

void ControlRightMotor(uint8_t dir, uint8_t pwn);
void ControlLeftMotor(uint8_t dir, uint8_t pwn);


void setup()
{
#ifdef Adafruit_BNO055_ENABLE
  nh.initNode();
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.advertise(imu_pub);
  nh.subscribe(cmd_vel);
#ifdef DEBUG
  Serial.begin(9600);
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
  Adafruit_BNO055_Details();
#endif
#endif
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
#ifdef Adafruit_BNO055_ENABLE
  imu::Quaternion quat = bno.getQuat();
  quat.normalize();
  imu::Vector<3> linear = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angular = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "base_imu";
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();
  imu_msg.angular_velocity.x = angular.x();
  imu_msg.angular_velocity.y = angular.y();
  imu_msg.angular_velocity.z = angular.z();
  imu_msg.linear_acceleration.x = linear.x();
  imu_msg.linear_acceleration.y = linear.y();
  imu_msg.linear_acceleration.z = linear.z();
  imu_pub.publish(&imu_msg);
  delay(BNO055_SAMPLERATE_DELAY_MS);
#endif
  nh.spinOnce();
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

#ifdef Adafruit_BNO055_ENABLE
void Adafruit_BNO055_Details(void)
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
#endif

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
