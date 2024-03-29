/*
  Created: spring 2020
    Author: Halsos

  Edited: spring 2023 
    Author: Jordalen
*/

#ifndef _MISSION_H_
#define _MISSION_H_

#include "main.h"
#include "timers.h"
#include "icm.h"

/** @file
 *
 * @defgroup PID PID program file
 * @{
 * @ingroup Program
 *
 * @brief Contain mission module with related structures and functions.
 */

#define PID_kP 0.026              /**< Default Kp term*/
#define PID_kI 0.001              /**< Default Ki term*/
#define PID_kD 0.1                /**< Default Kd term*/
#define PID_kI_THRESHOLD 10.0      /**< Default Ki threshold - When measured depth is further away than threshold Ki term is not part of pid calculation*/

#define PID_LIMIT_MIN 0           /**<PID min limit = minimum piston position of 0.0 cm - End limit switches will stop motor at 0.0xx cm*/
#define PID_LIMIT_MAX 0.052       /**<PID max limit = maximum piston position of 5.5 cm - End limit switches will stop motor at 5.1xx cm*/

#define MAX_DEPTH 50.0            /**< Maximum allowable depth [m]*/
#define MIN_DEPTH 0.0             /**< minimum allowable depth [m]*/

#define PSI_RANGE 100.0           /**< Differential sensor range: 14,7 psi to 114,7 psi.*/
#define MEASURED_OFFSET 0.19      /**< A measured pressure offset of ~ 0.19 psi*/
#define PSI_WEATHER_OFFSET 0.123  /**< current offset barometric pressure in PSI. Weight of air pushing on the water*/
#define PSI_1ATM_PRESSURE 14.7    /**< Standard barometric pressure in PSI. Weight of air pushing on the water*/
#define BAR_RANGE 5.89475728      /**< Differential sensor range: 1 bar to 6,89475728 bar*/
#define SAADC_MAX 16320.0 
#define SAADC_MIN 1730
#define PSI_TO_MH2O 0.703070      /**< Convert PSI to Water column (MeterH2O)*/
#define PSI_TO_PASCAL 6894.76     /**< Convert PSI to pascal*/
#define SAADC_VOLTAGE_ERROR 0.022 /**< Average error from calculated SAADC value to actual SAADC value*/
#define PASCAL_MAX 790828.66      /**< Maximum pressure value in pascal*/
#define PASCAL_MIN 101352.90      /**< Minimum pressure value in pascal*/
#define PRESSURE_VOLTAGE_MAX 4.5  /**< Maximum pressure value in volts*/
#define PRESSURE_VOLTAGE_MIN 0.5  /**< Minimum pressure value in volts*/
#define PRESSURE_VOLTAGE_RANGE 4.0 /**< Pressure sensor voltage output range.*/

#define MAX_NR_OF_MISSIONS 4      /**< The maximum number of missions to be configured.*/
#define DEFAULT_M1_DEPTH 1.25      /**< Default mission1 depth in meter */
#define DEFAULT_M1_TIME 180       /**< Default mission1 time in seconds */
#define DEFAULT_M2_DEPTH 0.8      /**< Default mission2 depth in meter */
#define DEFAULT_M2_TIME 180       /**< Default mission1 depth in seconds */



/**@brief structure for holding PID related data*/
typedef struct
{
  float kp;                     /**< Proportional gain coefficient.*/
  float ki;                     /**< Integral gain coefficient.*/
  float kd;                     /**< Derivative gain coefficient.*/
  float output;                 /**< PID output value*/
  float kiThreshold;            /**< Ki threshold - When measured depth is further away than threshold Ki term is not part of pid calculation*/
  float atmosphericPressure;    /**< Atmospheric pressure to help pressure sensor compensate for high or low barometric pressure*/
  float pistonPosition;         /**< Piston position as read from the motor position encoder*/
}pidData_t;

/**@Brief structure for holding relevant PID output data*/
typedef struct
{
  float kp;
  float ki;
  float kd;
  float PID_output;
}pidDataOut_t;



/**@brief structure for holding measured data values*/
typedef struct
{
   uint32_t battery;            /**< raw battery value from SAAADC*/
   float pressureSensorOffset;  /**< pressure sensor offset in meters*/
   float batteryVoltage;        /**< Battery value in volts*/
   float pressure;              /**< raw pressure sensor value from SAAADC*/
   float pressureVoltage;       /**< Pressure value in volts*/
   float psi;                   /**< Pressure value measured in psi*/
   float unfilteredDepth;       /**< unfiltered depth */
   float filteredDepth;         /**< filtered depth */
   float outside_temperature;   /**< Temperature measured by TMP117, attached to pressure housing to measure water temperature*/
}measuredData_t;


/**@brief structure for holding mission specific data*/
typedef struct
{
  float depth;    /**< in meters from 0 to 50. */
  uint32_t time;  /**< in seconds. */

}missiondata_t;

/**@brief structure for mission related structures and data values*/
typedef struct
{
  missiondata_t missionNr[4];     /**< Array of mission data instances*/
  missiondata_t currentMission;   /**< mission data instance to hold the currently running mission data*/
  pidData_t pidData;              /**< PID data instance to hold pid related data*/
  pidDataOut_t pidDataOut;        /**z PID data instance to hold outout from PID calculations */
  measuredData_t MeasuredData;    /**< Measured data instance for holding measured data values*/
  uint8_t nrOfMissions;           /**< Calculated number of missions based on configured mission data*/
  uint8_t missionFinished;        /**< counter to keep track of the number of missions that have finished during a dive*/
  bool running;                   /**< Flag to signal that mission is currently running*/
  uint32_t timeStamp;             /**< Timestamp to keep track of mission duration*/
  
}mission_t;


/**@brief structure for log file related data*/
typedef struct{

  uint32_t filenr;        /**< Hold the filenumber used to write, read, or delete a specific log file*/
  uint32_t nrOfLogfiles;  /**< Total number of log files on SD card*/
  uint32_t latestLogFile; /**< Previous added log file number on SD card*/
  uint8_t filename[];     /**< String array to target a specific file on SD card*/
}logFile_t;

/**@brief structure for holding values of each axis (x,y,z) */
typedef struct{
  int16_t x; /** <TODO: hold 5 x-axis values, once every 0.1 second*/
  int16_t y;
  int16_t z;
  float sensitivity;
}axis_t;

/**@brief structure for holding motion data written to log file. */
typedef struct{
   axis_t accel; /**< hold accelerometer data*/
   axis_t gyro;
   axis_t magn;
}motionData_t;

/**@brief structure for data values written to log file */
typedef struct{

  float timeStamp;        /**< Timestamp to keep track of time during mission*/
  uint8_t missionNr;      /**< Currently running mission id*/
  float setpoint;         /**< Target depth of current mission*/
  float pistonPosition;   /**< current piston position*/
  pidDataOut_t pidDataOut;        /**< Last calculated PID output*/
  float pressureVoltage;  /**< Pressure measured in volts*/
  float filteredDepth;
  float unfilteredDepth;
  float pressurePsi;      /**< Pressure measured in psi*/
  float pressurePascal;   /**< Pressure measured in pascal*/
  float batteryVoltage;   /**< Battery measured in volts*/
  float temperature;      /**< Water temperature measured by TMP117 temperature sensor*/
  motionData_t motionData;/**< 9-axis motion data measured by ICM20948 motion sensor*/
  logFile_t file;         /**< Log file information*/
}missionLog_t;



extern mission_t mission; /**< mission structure instance*/

/**@brief prepare mission related settings.
*
* @details Called before each new dive to calculate the number of missions
*          based on configuration, clear previously stored data, and start/stop timers
*          
*/
void prepareMission(void);

/**@brief Initialize mission module.
*
* @details Set data structures to NULL and populate with default data values.
*          Create pid and configure pid regulator.
*          
*/
void missionInit(void);

/**@brief run mission - measure data, calculate pid, and set piston position accordingly.        
*/
void runMission(void);

/**@brief Calculate pressure and current depth based on raw pressure from SAADC.        
*/
void CalcPressureAndDepth(void);


/**@brief Initialize mission log by setting the structure to zero.
*/
void missionLogInit(void);

/**@brief update mission log structure.
*
* @details In order to get a consisten data set with values represented at the same time instance
*          the mission log is updated with a snapshot of relevant data, then written to SD card.
*
* @note ICM20948 sensor module is not yet configured and is therfor constant 0
* @attention TMP117 sensor module is read but not yet configured so the sampled value is not reliable
*/
void updateMissionLog(void);


#endif  // _MISSION_H_


/** @} */