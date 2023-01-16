#ifndef _SAADC_H
#define _SAADC_H

#include "main.h"

/** @file
 *
 * @defgroup SAADC saadc program file
 * @{
 * @ingroup Drivvers
 *
 * @brief Contain SAADC module with related functions.
 */

// SAADC
#define ANALOG_BATTERY (2UL)          /**< ADC Channel 0 - Battery measurement. */
#define ANALOG_PRESSURE (3UL)         /**< ADC Channel 1 - PX3 Pressure measurement. */
#define SAADC_CALIBRATION_INTERVAL 10 /**< Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 10 will make the SAADC calibrate every 10th time the NRF_DRV_SAADC_EVT_DONE is triggered.*/
#define SAADC_SAMPLES_IN_BUFFER 2     /**< Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.*/



extern bool SAADCdataReady; /**< Flag to signal SAADC is ready to be read. */

/** @brief Initialize SAADC
*/
void saadc_init(void);

#endif

/** @} */