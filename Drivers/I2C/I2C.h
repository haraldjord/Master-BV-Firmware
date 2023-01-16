#ifndef _I2C_H
#define _I2C_H

#include "main.h"


// I2C
#define TMP117      (0x48) // Temperature Sensor adresses
#define ICM_MOTION  (0x68) // 9-axis ICM Motion Sensor adresses
#define HALL_EFFECT (0x30) // Hall Effect Sensor adresses
#define PIN_SCL       (27UL) // Possibly chanage??
#define PIN_SDA       (26UL) // Possibly chanage??



void i2c_init(void);
void i2c_write(uint8_t , uint16_t, uint8_t);
uint16_t i2c_read(uint8_t, uint8_t);




#endif