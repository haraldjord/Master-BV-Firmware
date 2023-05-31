/*
  Created: spring 2023
    Author: Jordalen 
*/

#ifndef _TMP117_H
#define _TMP117_H


#include <stdio.h>
#include "app_error.h"
#include "TWIM.h"
#include "main.h"

/*Address of registers. See page 23 of datasheet*/
enum TMP117_Register
{
  TMP117_TEMP_RESULT = 0X00,
  TMP117_CONFIGURATION = 0x01,
  TMP117_T_HIGH_LIMIT = 0X02,
  TMP117_T_LOW_LIMIT = 0X03,
  TMP117_EEPROM_UL = 0X04,
  TMP117_EEPROM1 = 0X05,
  TMP117_EEPROM2 = 0X06,
  TMP117_TEMP_OFFSET = 0X07,
  TMP117_EEPROM3 = 0X08,
  TMP117_DEVICE_ID = 0X0F
};

/**@brief Initate TMP117 module
*/
void TMP117_init(void);


/**@brief Get temperature from TMP117 module
*
* @param[out] Temperature 
*/
float TMP117_read_temp(void);


#endif