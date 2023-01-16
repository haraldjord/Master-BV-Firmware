#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"

/** @file
 *
 * @defgroup motor motor program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain motor module with related structures and functions.
 */

 /** @brief Status values returned in reply from command sent to motor interface
 */
typedef enum motorstatus {SUCCESS = 100, LOADED_IN_EEPROM, WRONG_CHECKSUM = 1, INVALID_COMMAND, WRONG_TYPE, INVALID_VALUE, EEPROM_LOCKED, CMD_UNAVAILABLE} status_t;

 /** @brief structure with received values from motor interface
 */
typedef struct{
  bool msgReceived; /**< Flag signal message is received*/
  uint8_t address;  /**< Register address*/
  uint8_t Id;       /**< Motor id*/
  status_t status;  /**< Returned status*/
  long value;       /**< Returned data value in case of a read command*/
}rxMotor_t;

/**@brief Vehicle float up towards the surface - Move piston down
*/
void motorUp(void);

/**@brief Vehicle dive deeper away from the surface - Move piston up
*/
void motorDown(void);

/**@brief stop motor
*/
void motorStop(void);

/**@brief Enable limit switches
*
* @attention important to enable limit switchesd before motor is operated to avoid damages
* @warning if motor is running without limit switches enabled it may hit the switch, break it, and even break the vehcile it self
*/
void motorEnableLimitSwitches(void);

/**@brief Send command to motor controller interface
*
* @param[in] addr Module address
* @param[in] Cmd Command to be executed
* @param[in] Type Command type number
* @param[in] motor Motor or bank number
* @param[in] value Value to be written in case requested by command - MSB first
*
*/
void sendCmd(uint8_t, uint8_t, uint8_t, uint8_t, long);

/**@brief Receive reply message from motor controller interface
*
* @details The function is called by uart_handler if checksum is correct.
*          The data is stored in @ref rxMotor.
*
* @param[in] motorReply data array with received data from uart handler.
*/
void receiveReply(uint8_t*);

/**@brief Move piston to a specific position between 0.0 mm and 55.0 mm
*
* @details Move piston to a specific position between 0.0 mm and 55.0 mm.
*          1 mm linear movement = 51200 (256∗200) MVP
*          estimated MVP range : −2816000.0 (51200∗−55) (5.5 cm) to 0 (0.0 cm)
*          The piston will stop when en limit switch is reached
*/
void setPistonPosition(void);

/**@brief Set motor position counter to zero
*
* @details set position counter to 0, to define reference at surface.
*/
void setReferencePositionToZero(void);

/**@brief request piston position
*
* @returns value Calculated piston position based on position counter value received from motor controller interface
*/
float getPistonPosition(void);

/**@brief request motor speed - internal value [-2048, 2048]
*
* @returns value motor speed measured in internal value [-2048, 2048]
*/
float getMotorSpeed(void);

/**@brief Read axis paramter value
*
* @param[in] parameter axis parameter from which to read its value
* 
* @returns value received axis parameter value
*/
float getAxisParameterValue(uint8_t);

/**@brief Read EEPROM value
*
* @param[in] parameter axis parameter from which to read its EEPROM value
* 
* @returns value received EEPROM value
*/
float getMotorEEPROMValue(uint8_t);

/**@brief Read Firmware version value
* 
* @returns value received Firmware version value in binary form
*/
float getMotorFirmwareVersion(void);

/**@brief Test motor
* 
* @details Enable limit switches, run motor (piston) down for 2 sec,
*          stop, run motor (piston) up for 2 sec, stop.
*/
void MotorTest(void);

/**@brief Initialize and configure motor
* 
* @details Enable limit switches, and set configuration data
*/
void motorInit(void);

 /**@brief Stop motor when surface is reached
 *
 * @details To make motor is powered down to standby current the motor is manualy stopped when bottom
 *          limit switch is triggered, unless running mission.
 *
 */
void stopMotorAtSurface(void);

 /**@brief Set surface reference point to zero
 *
 * @details Move piston position all the way to the bottom (maximum volum, minimum density).
 *          When bottom limit switch is triggered, set reference point to zero in order to define surface level.
 *          This is done to make sure vehicle float to surface, even in case of system reset.
 *
 */
 void setSurfaceReferencePoint(void);

#endif


/** @} */