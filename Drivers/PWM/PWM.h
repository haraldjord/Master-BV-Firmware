#ifndef  _PWM_H
#define  _PWM_H

#include "main.h"


/** @file
 *
 * @defgroup PWM pwm program files
 * @{
 * @ingroup Drivers
 *
 * @brief Contain PWM module with related functions.
 */

// PWM OUTPUT PINS
#define OUTPUT_PIN0 (14UL) /**< Channel 0 Red.*/
#define OUTPUT_PIN1 (12UL) /**< Channel 1 Green.*/
#define OUTPUT_PIN2 (11UL) /**< Channel 2 Blue.*/

/** @brief Initialize PWM module
*/
void pwm_init( void );

/** @brief Set new RGB brightness and/or color
*
* @param[in] REDduty Set red LED brightness as a percentage from 0 % to 100 %
* @param[in] GREENduty Set green LED brightness as a percentage from 0 % to 100 %
* @param[in] BLUEduty Set blue LED brightness as a percentage from 0 % to 100 %
*
*/
void updateLED(uint16_t REDduty, uint16_t GREENduty, uint16_t BLUEduty);

/** @brief Start PWM module
*/
void startLED( void );

/** @brief Stop PWM module
*/
void stopLED( void );

/** @brief PWM module test sequence
*
* @details three for-loops that loop through all the colors of the LED with brightness up to 10 %
*
*/
void testLED( void );

#endif

/** @} */