
#include "TWIM_v2.h"
#include "main.h"


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;


__STATIC_INLINE void data_handler(uint8_t temp)
{
    //printf("Temperature: %x Celsius degrees.\n", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = PIN_SCL,
       .sda                = PIN_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

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
  uint8_t buffer;
  uint8_t send[2];
  send[0] = reg;

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, ICM_MOTION, send, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  /* Read 1 byte from the specified address */
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, ICM_MOTION, &buffer, sizeof(buffer));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  return buffer;
}




uint16_t TMP117_read(uint8_t reg){
  ret_code_t err_code;
  uint8_t write[2];
  write[0] = reg; 
  uint8_t buffer[2];

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, TMP117_module, write, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  /*Read 2 bytes from the specified adress*/
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, TMP117_module, &buffer, sizeof(buffer));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done==false);
  nrf_delay_us(1);

  uint16_t out = ((buffer[0]<<8) |( buffer[1]));
  return out;
}

void TMP117_write(uint8_t reg, uint16_t data){
  ret_code_t err_code;  
  uint8_t write[3] = {reg, (data>>8), (data&0xff)}; 

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, TMP117_module, write, sizeof(write), false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);
}


/*



float readTMP117_v2(void){
  ret_code_t err_code;
  uint8_t tempReg[2];
  tempReg[0] = 0x00; //<Temperature register within TMP117 
  uint8_t buffer[2];

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, TMP117, tempReg, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  //Read 2 bytes from the specified adress
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, TMP117, &buffer, sizeof(buffer));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done==false);
  nrf_delay_us(1);

  //printf("buffer: %d\n", buffer);

  uint16_t bufferd = (buffer[0]>>8 | buffer[1]);
  float temp = ((float)bufferd) * 0.0078125;

  printf("temperature: %.2f\n", temp);
  return temp;

  // Get Temperature
 }


 float readTMP117_offset(void){
  ret_code_t err_code;
  uint8_t reg[2];
  reg[0] = 0x07; //<Temperature register within TMP117
  uint8_t buffer[2];

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, TMP117, reg, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  //Read 2 bytes from the specified adress
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, TMP117, &buffer, sizeof(buffer));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done==false);
  nrf_delay_us(1);

  uint16_t bufferd = (buffer[0]>>8 | buffer[1]);
  float offset = ((float)bufferd) * 0.0078125;

  printf("offset: %.2f\n", offset);
  return offset;
 }






 uint8_t TMP117_ID(){
  ret_code_t err_code;
  uint8_t reg[2];
  reg[0] = 0x0f; //<ID register within TMP117 
  uint8_t buffer[2];

  m_xfer_done = false;
  err_code = nrf_drv_twi_tx(&m_twi, TMP117, reg, 1, false);
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done == false);
  nrf_delay_us(1);

  //Read 2 bytes from the specified adress
  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, TMP117, &buffer, sizeof(buffer));
  APP_ERROR_CHECK(err_code);
  while(m_xfer_done==false);
  nrf_delay_us(1);


  printf("ID: %x", (buffer[0]<<8 | buffer[1]));
 
  return buffer;
 }*/