//Adapted from this great blog post http://www.aizac.info/arduino-2560-adk-with-a-9dof-sensor-board-mpu-6050/
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#ifndef Sensors_h
#define Sensors_h

int16_t getAxis(uint8_t axis) ;

enum enumaxes {ax = 0, ay, az, gx, gy, gz, mx, my, mz};
void sensors_update ();
void sensors_init ();
// int16_t getaz(){return az;}
#endif


