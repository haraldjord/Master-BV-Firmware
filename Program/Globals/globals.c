#include "globals.h"


// global variables are indicated by g_name


//global flags:
bool g_testbool = true;

bool g_motorStopped = false;       
bool g_bottomLimit = false;         
bool g_upperLimit = false;          
bool g_SAADCdataReady = false; 
bool g_sampleSensorData = false;    
bool g_sampleIMUdata = false;       
bool g_missionLogUpdated = false;  
bool g_TMP117dataReady = false;     
bool g_receiveTMP117 = false; // OBSOLETE??
bool g_isAdvertising = false; 

bool g_getValue = false;          /**< Menu is waiting for value from BLE application                 */
bool g_transferDataFlag = false;  /**< Menu is in the process of transfering data to BLE application  */
bool g_updateFSM = true;          /**< Init to true in order to access state machine                  */
bool g_sendNUS = false;           /**< Flag to signal Nordic UART Service to send data                */

bool g_readPressureSensor = false;

/*Global variables*/
float EMA_alpha = 0.2;            /**< Exponential Moving Average 'alpha' Coefficient, initialized to 0.2*/  