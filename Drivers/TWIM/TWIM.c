


#include "TWIM.h"

/** @file
 *
 * @defgroup twi TWI program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain TWI/I^2C module with related structures and functions.
 */

nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(0); // Create TWIM instance
extern bool TMP117dataReady;                /**< Flag to signal data from TMP117 temmperature sensor is ready to be read*/
extern bool receiveTMP117;                  /**< Flag to signal waiting for data from TMP117 temmperature sensor is ready to be read*/
uint8_t cnt = 0;                            /**< Counter variable to count the number of received bytes over TWIM*/

/**@brief Handler function for Two Wire Interface Master module
*
* @param[in] p_event Event type
* @param[in] p_context not used
*
*/
nrfx_twim_evt_handler_t twim_Handler(nrfx_twim_evt_t const * p_event, void * p_context){

  if(p_event->type == NRFX_TWIM_EVT_DONE){
    //NRF_LOG_INFO("NRFX_TWIM_EVT_DONE");
    cnt++;
    if(receiveTMP117 && cnt >= 3){
      receiveTMP117 = false;
      TMP117dataReady = true;
      cnt = 0;
      }
   }else if (p_event->type == NRFX_TWIM_EVT_ADDRESS_NACK){
    NRF_LOG_INFO("TWIM HandlerNRFX_TWIM_EVT_ADDRESS_NACK");

   }else if (p_event->type == NRFX_TWIM_EVT_DATA_NACK){
    NRF_LOG_INFO("NRFX_TWIM_EVT_DATA_NACK");

   }else if (p_event->type == NRFX_TWIM_EVT_OVERRUN){
    NRF_LOG_INFO("NRFX_TWIM_EVT_OVERRUN");

   }else if (p_event->type == NRFX_TWIM_EVT_BUS_ERROR){
    NRF_LOG_INFO("NRFX_TWIM_EVT_BUS_ERROR");

   }else
        NRF_LOG_INFO("UNKNOWN Event");

}


void TWIMInit(void)
{
  nrfx_err_t err_code;

  nrfx_twim_config_t twimConfig = NRFX_TWIM_DEFAULT_CONFIG;
  twimConfig.sda = PIN_SDA;
  twimConfig.scl = PIN_SCL;

  err_code = nrfx_twim_init(&m_twim, &twimConfig, twim_Handler, NULL);
  APP_ERROR_CHECK(err_code);
  
  nrfx_twim_enable(&m_twim);
}
/**@snippet [Initialize TWIM module]*/


void TWIMtx(uint8_t address, uint8_t data){

    nrfx_err_t err_code;

    nrfx_twim_xfer_desc_t writeByte = NRFX_TWIM_XFER_DESC_TX(address, &data, sizeof(data));

    err_code = nrfx_twim_xfer(&m_twim, &writeByte, NULL);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [Transmit byte on TWIM module]*/


void TWIMrx(uint8_t address, uint8_t * p_data){

  nrfx_err_t err_code;

  nrfx_twim_xfer_desc_t receive = NRFX_TWIM_XFER_DESC_RX(address, p_data, sizeof(p_data));
  err_code = nrfx_twim_xfer(&m_twim, &receive, NULL);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [receive byte on TWIM module]*/


void TWIMtxrx(uint8_t address, uint8_t txMsg, uint8_t * p_rxMsg){

    nrfx_err_t err_code;

    nrfx_twim_xfer_desc_t readregister = NRFX_TWIM_XFER_DESC_TXRX(address, &txMsg, sizeof(txMsg), p_rxMsg, sizeof(p_rxMsg));
    err_code = nrfx_twim_xfer(&m_twim, &readregister, NULL);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [transmit byte, then receive byte on TWIM module]*/


void TWIMtxtx(uint8_t address, uint8_t data1, uint8_t data2){

    nrfx_err_t err_code;

    nrfx_twim_xfer_desc_t writeWord = NRFX_TWIM_XFER_DESC_TXTX(address, &data1, sizeof(data1), &data2, sizeof(data2));
    err_code = nrfx_twim_xfer(&m_twim, &writeWord, NULL);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [transmit two bytes TWIM module]*/


bool isTWIMbusy(){
return nrfx_twim_is_busy(&m_twim);
}
/**@snippet [Check if TWIM module is busy]*/


/** @} */