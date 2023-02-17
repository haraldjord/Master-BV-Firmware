
#include "TMP117.h"

#define ALERT_HIGH_LIMIT 50 // 50 degree celsius as upper limit 
#define ALERT_LOW_LIMIT -10 // -10 degree celsius as lower limit 
#define TMP117Resolution 128 

extern bool TMP117Ready;


void TMP117_init(void){
/*
  uint16_t temp1 = TMP117_read(0x00);
  float temp = temp1;
  temp = temp / 128; // resolution
  printf("Temperature: %.2f\n", temp);
*/
  
  printf("Initiating TMP117:\n");
  // Get TMP117 ID:
  uint16_t ID = TMP117_read(TMP117_DEVICE_ID);
  if (ID != 0x117)
    NRF_LOG_ERROR("TMP117 module not found on I2C bus!");
    
  // soft reset:

  TMP117_write(TMP117_CONFIGURATION, 0x02);
  nrf_delay_ms(25); // Reset duration of 2ms. According to datasheet. 

  uint16_t test = TMP117_read(TMP117_T_HIGH_LIMIT);

  //uint16_t config = TMP117_read(TMP117_CONFIGURATION);
  uint16_t config = 0b000010100000| 0x0000;  // Interupt alert, average 8 measurements, continous conversion mode, see configuration register in datasheet.
  TMP117_write(TMP117_CONFIGURATION, config);

  // Set temp limit for interrupt alert:
  int16_t upper_limit = ALERT_HIGH_LIMIT*TMP117Resolution;
  int16_t lower_limit = ALERT_LOW_LIMIT*TMP117Resolution;
  TMP117_write(TMP117_T_HIGH_LIMIT, upper_limit);
  TMP117_write(TMP117_T_LOW_LIMIT, lower_limit);

}


float TMP117_read_temp(){
  
  uint16_t getValue = TMP117_read(TMP117_TEMP_RESULT); // Read temperature register 
  float temp = getValue;                               //change data type to float
  temp = temp / TMP117Resolution;                                   // Divide by resolution 
  return temp;
}

