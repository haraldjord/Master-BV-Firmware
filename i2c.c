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
  set_gyro_lowpass(true, 5, 250); // enable low pass in mode 5, with scale range to +- 250

  set_accelerometer_sample_rate(100);
  set_accelerometer_lowpass(true,5,16);    //enable low pass filter in mode 5, with scale to 16g
  
  set_userbank(0);
  // TODO something with INT pin...
  icm_write(0x0f, 0x30);
  set_userbank(3);
  icm_write(0x01, 0x4d);
  icm_write(0x02, 0x01);
}

/*void set_userbank(uint8_t bank){
  ret_code_t err_code;
  bank = (bank<<4);

  uint8_t REG_BANK_SEL = 0x7f; // User bank register
  uint8_t reg[2] = {REG_BANK_SEL, bank};
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, ICM_MOTION, reg, sizeof(reg), true);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_ms(50);
}

void icm_write(uint8_t reg, uint8_t data){
  ret_code_t err_code;
  uint8_t write[2] = {reg, data}; // bytes to be written on i2c bus
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, ICM_MOTION, write, sizeof(write), true);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);
}

uint8_t icm_read(uint8_t reg){
  ret_code_t err_code;
  uint8_t data;
  uint8_t send[2];
  send[0] = reg;
  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, ICM_MOTION, send, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  // Read 1 byte from the specified address 
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, ICM_MOTION, &data, sizeof(data));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  return data;
}*/

void set_gyro_sample_rate(uint8_t HzRate){
  // Set gyro sample rate in Hz.

  uint8_t calcRate = (1125.0/HzRate)-1 ;
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
  float sf;
  switch (scaleFactor){
    case 0:
      sf = 16384;
      break;
    case 1:
      sf = 8192;
      break;
    case 2:
      sf = 4096;
      break;
    case 3:
      sf = 2048;
      break;
      default:
      sf = 1;
      NRF_LOG_ERROR("Invalid scale factor for accelerometer.");   
  }

  set_userbank(0);
  // Read x direction
  uint8_t xMSB = icm_read(0x2d);
  uint8_t xLSB = icm_read(0x2e);
  int16_t tempX = ((xMSB<<8) | xLSB); // save register values as temporarly signed integer with 16 bit.
  icm.accel.x = tempX/sf;
  
  // Read y direction
  uint8_t yMSB = icm_read(0x2f);
  uint8_t yLSB = icm_read(0x30);
  int16_t tempY = (yMSB<<8 | yLSB);
  icm.accel.y = tempY/sf;
  
  // Read z direction
  uint8_t zMSB = icm_read(0x31);
  uint8_t zLSB = icm_read(0x32);
  int16_t tempZ = (zMSB<<8 | zLSB);
  icm.accel.z = tempZ/sf;

  printf("accelerometer: %.2f %.2f %.2f\n", icm.accel.x, icm.accel.y, icm.accel.z);
  //printf("MSB: %x %x %x\n", accelerometerData.xMSB, accelerometerData.yMSB, accelerometerData.zMSB);
}


/* TODO DELETE obsolete
void self_test(){
  set_userbank(2);
  // Enable self test and average 8 samples.
  IMU_write(0x15, 0x1f);


  set_userbank(1);
  // Declear some variables
  uint8_t x;
  uint8_t y;
  uint8_t z;
  printf("--------------------------");
  printf("Self Test:");


  // Read x direction
  icm_read(0x15, &x);
  //xData = xx/sf;
  printf("x_axis accelerometer: %d\t", x);

    // read y direction
  icm_read(0x16, &y);
  //yData = yData/sf;
  printf("y_axis accelerometer: %d\t", y);

  // Read z direction
  icm_read(0x17, &z);  
  //zData = zData/sf;
  printf("z_axis accelerometer: %d\t", z);
  printf("\n----------------\n");

}*/
