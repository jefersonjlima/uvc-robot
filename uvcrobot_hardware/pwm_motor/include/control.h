

#ifndef _CONTROL_H
#define _CONTROL_H
#ifndef ARDUINO
#include <stdint.h>
#elif ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

typedef struct pid_controller{
  double kp;   
  double ki; 
  double kd;
  double error;
  float timestamp;
  uint8_t out;
} pid_controller;


#endif