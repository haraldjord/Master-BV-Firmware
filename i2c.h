#ifndef _i2c_H /*TODO REMEBER TO INCLUDE THIS, SO HEADER FILE WONT BE INCLUDED MULTIPLE OF TIMES!!!!!*/
#define _i2c_H

#include <stdio.h>
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "main.h"


typedef struct{
  float x;
  float y;
  float z;
}accel_t;

typedef struct{
  float x;
  float y;
  float z;
}gyro_t;

typedef struct{
  float x;
  float y;
  float z;
}mag_t; // TODO read datasheet!

// ICM global variables:
typedef struct{
  accel_t accel;
  gyro_t gyro;
  mag_t mag;
}icm_t;


void set_userbank(uint8_t bank);

void init_imu(void);

void icm_write(uint8_t reg, uint8_t data);

uint8_t icm_read(uint8_t reg);


void set_gyro_sample_rate(uint8_t HzRate);

void set_gyro_lowpass(uint8_t enable, uint8_t mode, uint16_t scale);

void set_accelerometer_sample_rate(uint16_t HzRate);

void set_accelerometer_lowpass(uint8_t enable, uint8_t mode, uint16_t scale);

void read_accel_data(void);

void self_test(void);

#endif