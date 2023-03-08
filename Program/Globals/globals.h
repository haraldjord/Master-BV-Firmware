#ifndef  _GLOBALS_H
#define  _GLOBALS_H
#include "main.h" // necessarily??


// Declaration of gloabal flags, indicated by g_<name>:
extern bool g_testbool;

/*Mission and sensor reading related flags*/
extern bool g_motorStopped;        /**< Flag to signal motor stopped*/
extern bool g_bottomLimit;         /**< Flag to signal when bottom limit switch is reached*/
extern bool g_upperLimit;           /**< Flag to signal when upper limit switch is reached*/
extern bool g_SAADCdataReady;      /**< Flag to signal SAADC data is ready to be read and calculated, and to run PID*/
extern bool g_sampleSensorData;    /**< Flag to signal sampling battery, pressure (SAADC related measurements)*/
extern bool g_sampleIMUdata;       /**< Flag to signal sampling data from IMU (icm chip)*/
extern bool g_missionLogUpdated;   /**< Flag to signal mission log is updated*/
extern bool g_TMP117dataReady;     /**< Flag to signal TMP117 data is ready to be read*/
extern bool g_receiveTMP117;       /**< Flag to signal that data is expected to be received from TMP117*/ // OBSOLETE


/*BLE related global flags*/
extern bool g_isAdvertising;     /**< Flag to signal if advertising or not */
extern bool g_getValue;          /**< Menu is waiting for value from BLE application                 */
extern bool g_transferDataFlag;  /**< Menu is in the process of transfering data to BLE application  */
extern bool g_updateFSM;          /**< Init to true in order to access state machine                  */
extern bool g_sendNUS;           /**< Flag to signal Nordic UART Service to send data                */

extern bool g_readPressureSensor; //  just use use g_sampleSensorData?? 


/* Global variables*/
extern float EMA_alpha;         /**< Exponential Moving Average 'alpha' Coefficient*/

// Global structures: 



#endif