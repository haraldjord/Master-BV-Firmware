
#include "I2C.h"


/**
 * @brief Functions for I2C usage.
 */
void i2c_init()
{


  NRF_P0->PIN_CNF[6] = (GPIO_PIN_CNF_DIR_Input   << GPIO_PIN_CNF_DIR_Pos   ) |
                        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos  ) |  // Internal PULLUP
                        (GPIO_PIN_CNF_DRIVE_S0D1  << GPIO_PIN_CNF_DRIVE_Pos );
  
  NRF_P0->PIN_CNF[7] = (GPIO_PIN_CNF_DIR_Input   << GPIO_PIN_CNF_DIR_Pos   ) |
                        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos  ) |  // Internal PULLUP
                        (GPIO_PIN_CNF_DRIVE_S0D1  << GPIO_PIN_CNF_DRIVE_Pos );

  NRF_TWIM0->PSEL.SCL = PIN_SCL;
  NRF_TWIM0->PSEL.SDA = PIN_SDA;

  //NRF_TWIM0->ADDRESS = DEVICE_ADDRESS;
  NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K100 << TWIM_FREQUENCY_FREQUENCY_Pos;
  NRF_TWIM0->SHORTS = 0;
  
}


void i2c_write(uint8_t reg_addr, uint16_t data, uint8_t DEVICE_ADDRESS)
{

  NRF_TWIM0->ADDRESS = DEVICE_ADDRESS;
  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

  uint8_t tx_buf[3];
  NRF_TWIM0->SHORTS = TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos;

  tx_buf[0] = reg_addr;
  tx_buf[1] = data >> 8;
  tx_buf[2] = data;
  NRF_TWIM0->TXD.MAXCNT = 2;//sizeof(tx_buf);
  NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];
/*
  NRF_LOG_INFO("tx_buf[0]: %x", tx_buf[0]);
  NRF_LOG_INFO("tx_buf[1]: %x", tx_buf[1]);
  NRF_LOG_INFO("tx_buf[2]: %x", tx_buf[2]);
*/
  NRF_TWIM0->EVENTS_STOPPED = 0;
  NRF_TWIM0->TASKS_STARTTX = 1;

  while (NRF_TWIM0->EVENTS_STOPPED == 0);


  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;


  
}



uint16_t i2c_read(uint8_t reg_addr, uint8_t DEVICE_ADDRESS)
{

  NRF_TWIM0->ADDRESS = DEVICE_ADDRESS;
  
  NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STOP_Disabled << TWIM_SHORTS_LASTTX_STOP_Pos)       |
                      (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos)  |
                      (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos)        ;

  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos;

  uint8_t tx_buf[1];
  uint8_t rx_buf[2] = {0,0};
  uint16_t result = 0;
  

  tx_buf[0] = reg_addr;
  NRF_TWIM0->TXD.MAXCNT = sizeof(tx_buf);
  NRF_TWIM0->TXD.PTR = (uint32_t)&tx_buf[0];

  NRF_TWIM0->TASKS_STARTTX = 1;

  NRF_TWIM0->RXD.MAXCNT = 1;
  NRF_TWIM0->RXD.PTR = (uint32_t)&rx_buf[0];

  NRF_TWIM0->EVENTS_STOPPED = 0;

  while (NRF_TWIM0->EVENTS_STOPPED == 0);

  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;

  //NRF_LOG_INFO("result0: %x", rx_buf[0]);
  //NRF_LOG_INFO("result1: %x", rx_buf[1]);


  result = (rx_buf[0] << 8) | rx_buf[1];

  //NRF_LOG_INFO("Temperature resultRESULT: %d %x", result, result);

  return result;
}