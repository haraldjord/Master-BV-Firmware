
#include "SAADC.h"


static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static volatile int m_adc_evtCounter = 0;
extern mission_t mission;

/** @file
 *
 * @defgroup SAADC saadc program file
 * @{
 * @ingroup Drivvers
 *
 * @brief Contain SAADC module with related functions.
 */


/** @brief SAADC event handler
*
* @param[in] p_event saadc event.
*
*/
nrfx_saadc_event_handler_t saadc_handler(nrfx_saadc_evt_t const * p_event)
{
    
      if(p_event->type == NRFX_SAADC_EVT_DONE){
        //NRF_LOG_INFO("NRFX_SAADC_EVT_DONE");

          nrfx_saadc_buffer_convert(p_event->data.done.p_buffer,SAADC_SAMPLES_IN_BUFFER);

          mission.MeasuredData.battery = p_event->data.done.p_buffer[0];
          mission.MeasuredData.pressure = p_event->data.done.p_buffer[1];
          
          if (mission.MeasuredData.battery > 0)
            SAADCdataReady = true;
        }

      else if(p_event->type == NRFX_SAADC_EVT_LIMIT){
        NRF_LOG_INFO("NRFX_SAADC_EVT_LIMIT - High or low limit??");
        }

      else if(p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE){
        NRF_LOG_INFO("NRFX_SAADC_EVT_CALIBRATEDONE");
        }

      else
       NRF_LOG_INFO("Unknown Event: %x",p_event);
      
      if(m_adc_evtCounter % SAADC_CALIBRATION_INTERVAL == 0){
        nrfx_saadc_calibrate_offset();
        m_adc_evtCounter = 0;
        }

      m_adc_evtCounter++; 
}

void saadc_init(){

    ret_code_t err_code;

    // Default SAADC configuration
   nrfx_saadc_config_t saadcConfig;
   
   // SAADC channel configuration
   nrf_saadc_channel_config_t batteryChannelConfig, pressureChannelConfig;


        saadcConfig.resolution = NRFX_SAADC_CONFIG_RESOLUTION;
        saadcConfig.oversample = NRFX_SAADC_CONFIG_OVERSAMPLE;
        saadcConfig.low_power_mode = NRFX_SAADC_CONFIG_LP_MODE;
        saadcConfig.interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY;
    
        // Battery Measurement
        batteryChannelConfig.resistor_p = NRF_SAADC_RESISTOR_DISABLED;        
        batteryChannelConfig.resistor_n = NRF_SAADC_RESISTOR_DISABLED;        
        batteryChannelConfig.gain       = NRF_SAADC_GAIN1_5;                  
        batteryChannelConfig.reference  = NRF_SAADC_REFERENCE_INTERNAL;       
        batteryChannelConfig.acq_time   = NRF_SAADC_ACQTIME_40US;             
        batteryChannelConfig.mode       = NRF_SAADC_MODE_SINGLE_ENDED;        
        batteryChannelConfig.burst      = NRF_SAADC_BURST_ENABLED;            
        batteryChannelConfig.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN0);
        batteryChannelConfig.pin_n      = NRF_SAADC_INPUT_DISABLED;            

        // Pressure Measurement
        pressureChannelConfig.resistor_p = NRF_SAADC_RESISTOR_DISABLED;        
        pressureChannelConfig.resistor_n = NRF_SAADC_RESISTOR_DISABLED;        
        pressureChannelConfig.gain       = NRF_SAADC_GAIN1_5;                  
        pressureChannelConfig.reference  = NRF_SAADC_REFERENCE_INTERNAL;       
        pressureChannelConfig.acq_time   = NRF_SAADC_ACQTIME_40US;             
        pressureChannelConfig.mode       = NRF_SAADC_MODE_SINGLE_ENDED;        
        pressureChannelConfig.burst      = NRF_SAADC_BURST_ENABLED;            
        pressureChannelConfig.pin_p      = (nrf_saadc_input_t)(NRF_SAADC_INPUT_AIN1);
        pressureChannelConfig.pin_n      = NRF_SAADC_INPUT_DISABLED;            

    

  err_code = nrfx_saadc_init(&saadcConfig, saadc_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(0, &batteryChannelConfig);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_saadc_channel_init(1, &pressureChannelConfig);
  APP_ERROR_CHECK(err_code);

  nrfx_saadc_limits_set(0, 0, NRFX_SAADC_LIMITH_DISABLED);

  //Set SAADC buffer 1. The SAADC will start to write to this buffer
  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);    
  APP_ERROR_CHECK(err_code);
  
  //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);    
  APP_ERROR_CHECK(err_code);
}
/** @snippet [Initialize SAADC]*/


 /** @} */