#ifndef _TWIM_H_
#define _TWIM_H_

#include "main.h"
#include "nrfx_twim.h"


/** @file
 *
 * @defgroup twi TWI program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain TWI/I^2C module with related structures and functions.
 */


#define TMP117      (0x48) // Temperature Sensor adresses // Important note, the R/W bit is handled by peripheral, so address is only 7 bit - does not include w.
#define ICM_MOTION  (0x68) // 9-axis ICM Motion Sensor adresses
#define HALL_EFFECT (0x30) // Hall Effect Sensor adresses
#define PIN_SCL     (27UL)
#define PIN_SDA     (26UL)

/**@brief Initialization Two Wire Interface Master (TWIM) module
*/
void TWIMInit(void);

/**@brief Transmit one byte on Two Wire Interface Master (TWIM) module.
*
* @param[in] address Sensor/module address
* @param[in] data data to be transmitted, register address or configuration data
*
*/
void TWIMtx(uint8_t, uint8_t);

/**@brief receive one byte on Two Wire Interface Master (TWIM) module.
*
* @param[in] address Sensor/module address
* @param[out] p_data pointer to where received data is stored
*
*/
void TWIMrx(uint8_t, uint8_t *);

/**@brief transmit byte, then receive byte on Two Wire Interface Master (TWIM) module.
*
* @param[in] address Sensor/module address
* @param[in] txMsg data byte to be sent to sensor/module, register address or configuration data
* @param[out] p_rxMsg pointer to where received data is stored
*
*/
void TWIMtxrx(uint8_t, uint8_t, uint8_t *);

/**@brief transmit two bytes on Two Wire Interface Master (TWIM) module.
*
* @details Typically used when configuring/storing data on sensor/module
*
* @param[in] address Sensor/module address
* @param[in] data1 first byte to be sent to sensor/module, register address
* @param[out] data2 second byte to be sent to sensor/module,  configuration data
*
*/
void TWIMtxtx(uint8_t, uint8_t, uint8_t);

/**@brief Check if Two Wire Interface Master (TWIM) module is in a busy state.
*
* @details The module is busy while transmission is ongoing.
*
* @returns bool true if busy, false if not busy
*
*/
bool isTWIMbusy(void);



#endif // _TWIM_H_

/** @} */