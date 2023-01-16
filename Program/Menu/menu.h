#ifndef _PRINTMENU_H
#define _PRINTMENU_H

#include "main.h"

/** @file
 *
 * @defgroup menu menu program file
 * @{
 * @ingroup Program
 *
 * @brief Contain Menu module with related functions.
 */

#define RTC_MAX_COUNT 16777216  /**<Maximum value of RTC clock*/

/** @brief menu options and operations enumerated.
*/
enum menu { MAINMENU, MISSIONDATA, CONFIGVEHICLE, TRANSFERDATA, TRANSFER_ALL_FILES, TRANSFER_ONE_FILE, DELETE_ALL_FILES, DELETE_ONE_FILE };

/** @brief file operations enumerated.
*/
enum fileoption { TRANSFER_ALL, TRANSFER_ONE, DELETE_ALL, DELETE_ONE };

/** @brief configurable values enumerated.
*/
enum config { M1DEPTH,      /**< Mission 1 depth*/
              M1TIME,       /**< Mission 1 time*/
              M2DEPTH,      /**< Mission 2 depth*/
              M2TIME,       /**< Mission 2 time*/
              M3DEPTH,      /**< Mission 3 depth*/
              M3TIME,       /**< Mission 3 time*/
              M4DEPTH,      /**< Mission 4 depth*/
              M4TIME,       /**< Mission 4 time*/
              PID_P,        /**< Kp coefficient*/
              PID_I,        /**< Ki coefficient*/
              PID_D,        /**< Kd coefficient*/
              THRESHOLD,    /**< Ki threshold*/
              ATM_PRESSURE  /**< Atmospheric pressure*/
            };



/**@brief function for setting configuration value received from BLE client.
*
*params[in] value received as character, and converted to integer or float depending on the written configuration
*/
void setConfigValue(char *);

/**@brief in Main Menu navigate to sub menu.
*
*params[in] option integer value passed to switch statement to chose sub menu
*/
void mainMenu(int);

/**@brief navigate transferData menu
*
*params[in] option integer value passed to switch statement to choose file operation.
*/
void transferDataMenu(int);


/**@brief navigate Mission Data menu
*
*params[in] option integer value passed to switch statement to choose the configuration to change
*/
void missinDataMenu(int);

/**@brief navigate ConfigVehicle menu
*
*params[in] option integer value passed to switch statement to choose the configuration to change
*/
void configVehicleMenu(int);

/**@brief handle file operation menu
*
*params[in] fileCmd converted to integer and passed to switch statement to choose the file option
*/
void transferData(char *);

/**@brief print main menu over BLE to client.
*/
void printMainMenu(void);

/**@brief print Mission Data menu over BLE to client.
*/
void printMissionDataMenu(void);

/**@brief print Configure Vehicle menu over BLE to client.
*/
void printConfigVehicleMenu(void);

/**@brief print Trasnfer Data menu over BLE to client.
*/
void printTransferDataMenu(void);

/**@brief Transfer all mission log files from SD card over BLE to client.
*/
void transferAllFiles(void);


/**@brief Transfer one mission log files from SD card over BLE to client.
*
* @details Client choose a file to transfer
*
*/
void TransferOneFile(void);

/**@brief delete one mission log files from SD card.
*
* @details Client choose a file to delete
*
*/
void deleteFile(void);

/**@brief delte all mission log files from SD card.
*/
void deleteAllFiles(void);


#endif

/** @} */
