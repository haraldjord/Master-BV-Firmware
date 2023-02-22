#ifndef _i2c_H /*TODO REMEBER TO INCLUDE THIS, SO HEADER FILE WONT BE INCLUDED MULTIPLE OF TIMES!!!!!*/
#define _i2c_H

#include <stdio.h>
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "main.h"
#include "mission.h"


typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
}accel_t;

typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
}gyro_t;

typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
}mag_t; // TODO read datasheet!


typedef struct{
  int16_t x;
  int16_t y;
  int16_t z;
  float sensitivity;
}axiss_t;

// ICM global variables:
typedef struct icm_t{
  axiss_t accel;
  axiss_t gyro;
  axiss_t mag;
}icm_t;

extern icm_t icm;


void set_userbank(uint8_t bank);

void init_imu(void);

void icm_write(uint8_t reg, uint8_t data);

uint8_t icm_read(uint8_t reg);


void set_gyro_sample_rate(uint8_t HzRate);

void set_gyro_lowpass(uint8_t enable, uint8_t mode, uint16_t scale);

void set_accelerometer_sample_rate(uint16_t HzRate);

void set_accelerometer_lowpass(uint8_t enable, uint8_t mode, uint16_t scale);

void read_accel_data(void);

void read_gyro_data(void);

void read_magnetometer_data(void);

void trigger_mag_io(void);

void self_test(void);

void mag_read(uint8_t reg, uint8_t value);

void mag_write(uint8_t reg);

uint8_t mag_read_reg();

#endif