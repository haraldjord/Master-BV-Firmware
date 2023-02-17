#ifndef _TWIM_v2_H
#define _TWIM_v2_H

#include <stdio.h>
#include "app_error.h"
#include "nrf_drv_twi.h"
//#include "main.h"


/* TWI instance ID. */
#define TWI_INSTANCE_ID 0

#define TMP117_module      (0x48) // Temperature Sensor adresses // Important note, the R/W bit is handled by peripheral, so address is only 7 bit - does not include w.
#define ICM_MOTION  (0x68) // 9-axis ICM Motion Sensor adresses
//#define HALL_EFFECT (0x30) // Hall Effect Sensor adresses
#define PIN_SCL     (27UL)
#define PIN_SDA     (26UL)



__STATIC_INLINE void data_handler(uint8_t temp);
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);

/**
 * @brief UART initialization.
 */
void twi_init (void);

void icm_write(uint8_t reg, uint8_t data);

uint8_t icm_read(uint8_t reg);

uint16_t TMP117_read(uint8_t reg);

void TMP117_write(uint8_t reg, uint16_t data);

/*
float readTMP117_v2(void);

float readTMP117_offset(void);

uint8_t TMP117_ID(void);
*/

#endif