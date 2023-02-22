/*TODO Rename file*/
#include "i2c.h"

icm_t icm;


void set_userbank(uint8_t bank){
  bank = (bank<<4);
  uint8_t REG_BANK_SEL = 0x7f;
  icm_write(REG_BANK_SEL, bank);
}

void init_imu(){
  printf("initialize imu\n");
  set_userbank(0);

  // read chip ID (should be 0xea)
  uint8_t chipID = icm_read(0); 
  printf("chip ID: %x\n", chipID);
  if (chipID != 0xea)
      NRF_LOG_ERROR("Unable to find IMC module");

  icm_write(0x06, 0x80); // Reset internal register amd restore default settings
  nrf_delay_ms(25);
  // set power enable register
  icm_write(0x06, 0x01); // wake up and auto select best available clock source
  nrf_delay_ms(25); // Startup time, see section 3.2 in datasheet.
  icm_write(0x07, 0x00); //Enable accelerometer and gyroscope 

  // Configure gyro and accelerometer:
  set_userbank(2);

  set_gyro_sample_rate(100);
  set_gyro_lowpass(true, 5, 250);         // enable low pass in mode 5, with scale range to +- 250

  set_accelerometer_sample_rate(100);
  set_accelerometer_lowpass(true,5,16);    //enable low pass filter in mode 5, with scale to 16g
  
  set_userbank(0);
  // Configure int pin
  icm_write(0x0f, 0b10111000); //active low, clear on read
  icm_write(0x11, 0x01); // Raw data ready interrupt.
  uint8_t test = icm_read(0x11);


  set_userbank(3);
  icm_write(0x01, 0x4d);
  icm_write(0x02, 0x01);
}


void set_gyro_sample_rate(uint8_t HzRate){
  // Set gyro sample rate in Hz.

  uint8_t calcRate = (1100.0/HzRate)-1 ;
  icm_write(0x00, calcRate);

}


void  set_gyro_lowpass(uint8_t enable, uint8_t mode, uint16_t scale){
  uint8_t scaleVal;
  switch (scale)
  {
  case 250:
    scaleVal = 0b000;
    break;
  case 500:
    scaleVal = 0b010;
    break;
  case 1000:
    scaleVal = 0b100;
    break;
  case 2000:
    scaleVal = 0b110;
    break;
  default:
    NRF_LOG_ERROR("Invalid gyro scale value");
  }
  
  uint8_t value;
  value = (mode<< 3);
  value = (value | enable);
  value = (value | scaleVal);

  icm_write(0x01, value);
}

void set_accelerometer_sample_rate(uint16_t HzRate){
  /*Set sample rate for accelerometer in Hz*/
  
  uint16_t rate = ((1125.0/HzRate) - 1); 
  uint8_t MSBrate =  ((rate>>8) & 0xffff);
  uint8_t LSBrate = (rate & 0xff); 

  icm_write(0x10, MSBrate);       // MSB of accelerometer sample rate
  icm_write(0x11, LSBrate);       // LSB of accelerometer sample rate
}

void set_accelerometer_lowpass(uint8_t enable, uint8_t mode, uint16_t scale){
  uint8_t scaleVal;
  switch (scale)
  {
  case 2:
    scaleVal = 0b000;
    break;
  case 4:
    scaleVal = 0b010;
    break;
  case 8:
    scaleVal = 0b100;
    break;
  case 16:
    scaleVal = 0b110;
    break;
  default:
    NRF_LOG_ERROR("Invalid gyro scale value");
  }
  
  uint8_t value;
  value = (mode<< 3);
  value = (value | enable);
  value = (value | scaleVal);
  
  uint8_t accel_config = 0x14;
  icm_write(accel_config, value);
}


void read_accel_data(void){
  // Identify scale factor:
  set_userbank(2);
  uint8_t scaleFactor = icm_read(0x14);
  scaleFactor = (scaleFactor & 0x06) >> 1;
  switch (scaleFactor){
    case 0:
      icm.accel.sensitivity = 16384;
      break;
    case 1:
      icm.accel.sensitivity = 8192;
      break;
    case 2:
      icm.accel.sensitivity = 4096;
      break;
    case 3:
      icm.accel.sensitivity = 2048;
      break;
      default:
      icm.accel.sensitivity = 1;
      NRF_LOG_ERROR("Invalid scale factor for accelerometer.");   
  }

  set_userbank(0);
  // Read x direction
  uint8_t xMSB = icm_read(0x2d);
  uint8_t xLSB = icm_read(0x2e);
  int16_t tempX = ((xMSB<<8) | xLSB); // save register values as temporarly signed integer with 16 bit.
  icm.accel.x = tempX;
  
  // Read y direction
  uint8_t yMSB = icm_read(0x2f);
  uint8_t yLSB = icm_read(0x30);
  int16_t tempY = (yMSB<<8 | yLSB);
  icm.accel.y = tempY;
  
  
  // Read z direction
  uint8_t zMSB = icm_read(0x31);
  uint8_t zLSB = icm_read(0x32);
  int16_t tempZ = (zMSB<<8 | zLSB);
  icm.accel.z = tempZ;

  printf("accelerometer: %.2f %.2f %.2f\n", (float)icm.accel.x/icm.accel.sensitivity, (float)icm.accel.y/icm.accel.sensitivity, (float)icm.accel.z/icm.accel.sensitivity);
  //printf("MSB: %x %x %x\n", accelerometerData.xMSB, accelerometerData.yMSB, accelerometerData.zMSB);
}

void read_gyro_data(){
  // Identify scale factor:
  set_userbank(2);
  uint8_t scaleFactor = icm_read(0x01);
  scaleFactor = (scaleFactor & 0x06) >> 1;
  switch (scaleFactor){
    case 0:
      icm.gyro.sensitivity = 131;
      break;
    case 1:
      icm.gyro.sensitivity = 65.5;
      break;
    case 2:
      icm.gyro.sensitivity = 32.8;
      break;
    case 3:
      icm.gyro.sensitivity = 16.4;
      break;
      default:
      icm.gyro.sensitivity = 1;
      NRF_LOG_ERROR("Invalid scale factor for accelerometer.");
      }

 set_userbank(0);
  // Read x direction
  uint8_t xMSB = icm_read(0x33);
  uint8_t xLSB = icm_read(0x34);
  int16_t tempX = ((xMSB<<8) | xLSB); // save register values as temporarly signed integer with 16 bit.
  icm.gyro.x = tempX;
  
  // Read y direction
  uint8_t yMSB = icm_read(0x35);
  uint8_t yLSB = icm_read(0x36);
  int16_t tempY = (yMSB<<8 | yLSB);
  icm.gyro.y = tempY;
  
  
  // Read z direction
  uint8_t zMSB = icm_read(0x37);
  uint8_t zLSB = icm_read(0x38);
  int16_t tempZ = (zMSB<<8 | zLSB);
  icm.gyro.z = tempZ;

  printf("gyroscope: %.2f %.2f %.2f\n", ((float)icm.gyro.x)/icm.gyro.sensitivity, ((float)icm.gyro.y)/icm.gyro.sensitivity, ((float)icm.gyro.z)/icm.gyro.sensitivity);
}

void read_magnetometer_data(){
 //TODO configure magnetometer:
}

void trigger_mag_io(void){
    //set_userbank(0);
    uint8_t user = icm_read(0x03);
    icm_write(0x03, (user | 0x20));
    nrf_delay_ms(1);
    icm_write(0x03, user);
}





