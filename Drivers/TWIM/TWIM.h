#ifndef _TWIM_v2_H
#define _TWIM_v2_H

#include <stdio.h>
#include "app_error.h"
#include "nrf_drv_twi.h"


/** @file
 *
 * @defgroup twi TWI program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain TWI/I^2C module with related structures and functions.
 */


/* TWI instance ID. */
#define TWI_INSTANCE_ID 0


#define TMP117_module (0x48)  // Temperature Sensor adresses // Important note, the R/W bit is handled by peripheral, so address is only 7 bit - does not include w.
#define ICM_MOTION    (0x68)  // 9-axis ICM Motion Sensor adresses
#define PIN_SCL       (27UL)    
#define PIN_SDA       (26UL)



__STATIC_INLINE void data_handler(uint8_t temp);
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

/**
 * @brief UART initialization.
 */
void twi_init (void);


/**@brief Transmit one byte to ICM module.
*
* @param[in] address Sensor/module address
* @param[in] data data to be transmitted, register address or configuration data
*/
void icm_write(uint8_t reg, uint8_t data);

/**@brief Read one byte from ICM module
*
* @param[in] regiseter to read value from 
* @param[out] register value
*/
uint8_t icm_read(uint8_t reg);


/**@brief Read two bytes from TMP117 module
*
* @param[in] regiseter to read value from 
* @param[out] register value
*/
uint16_t TMP117_read(uint8_t reg);


/**@brief Transmit two bytes to TMP117 module.
*
* @param[in] address Sensor/module address
* @param[in] data data to be transmitted, register address or configuration data
*/
void TMP117_write(uint8_t reg, uint16_t data);


#endif