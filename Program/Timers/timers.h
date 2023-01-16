#ifndef _TIMERS_H_
#define _TIMERS_H_

#include "app_timer.h"
#include "nrfx_timer.h"
#include "main.h"

/** @file
 *
 * @defgroup timers timer program file
 * @{
 * @ingroup Program
 *
 * @brief Contain timer module with related functions.
 */

#define IDLE_TO_SLEEP_TIME APP_TIMER_TICKS(2000*60)        /**< 2 min in idle before going to sleep.*/
#define MOTOR_STOP_TIME APP_TIMER_TICKS(5000)              /**< Wait 5 seconds before the program check if motor need to be stopped.*/
#define UPDATE_MISSIONLOG_TIMER APP_TIMER_TICKS(1000)       /**< Update mission log every 0.5 sec */
#define MEASURE_BATTERY_TIMER APP_TIMER_TICKS(10000)       /**< Measure battery voltage everey 10 sec when not in mission state */
#define SENSORS_SAMPLE_TIMER APP_TIMER_TICKS(500)          /**< Set time between each SAADC and TMP117 sample.*/
#define UPDATE_FSM_TIMER APP_TIMER_TICKS(500)              /**< When not in mission state update FSM within i strict interval. */
#define KHZ_TO_SEC 125000                                  /**< Convert kHz to seconds to set mission timer*/

APP_TIMER_DEF(m_updateFSM_timer_id);              /**< Create a variable to hold the repeated timer m_updateFSM_timer_id. */
APP_TIMER_DEF(m_sleep_timer_id);                  /**< Create a variable to hold the single shot timer m_sleep_timer_id. */
APP_TIMER_DEF(m_motorStop_timer_id);              /**< Create a variable to hold the single shot timer m_motorStop_timer_id. */
APP_TIMER_DEF(m_repeatedBattery_timer_id);        /**< Create a variable to hold the repeated timer m_repeatedBattery_timer_id. */
APP_TIMER_DEF(m_sampleSensorData_timer_id);       /**< Create a variable to hold the repeated timer m_sampleSensorData_timer_id. */
APP_TIMER_DEF(m_updateMissionLog_timer_id);       /**< Create a variable to hold the repeated timer m_updateMissionLog_timer_id. */

/**@brief start timer for FSM updates.
*/
void start_updateFSM_timer_handler(void);

/**@brief stop timer for FSM updates.
*/
void stop_updateFSM_timer_handler(void);

/**@brief start timer for SAADC sample.
*/
void startSampleSensorDatatimer(void);

/**@brief stop timer for SAADC sample.
*/
void stopSampleSensorDatatimer(void);

/**@brief start timer for Idle-to-sleep event.
*/
void startSleepTimer(void);

/**@brief stop timer for Idle-to-sleep event.
*/
void stopSleepTimer(void);

/**@brief start timer for count down until check if motor has stopped.
*/
void startStopMotorTimer(void);

/**@brief start mission timer.
*/
void startMissiontimer(void);

/**@brief stop mission timer.
*/
void stopMissiontimer(void);

/**@brief start timer for measuring battery voltage.
*/
void startBatteryMeasureTimer(void);

/**@brief stop timer for measuring battery voltage.
*/
void stopBatteryMeasureTimer(void);

/**@brief start timer to update mission log.
*/
void startUpdateMissionLogTimer(void);

/**@brief stop timer to update mission log.
*/
void stopUpdateMissionLogTimer(void);


/**@brief Set new mission time.
*
* @param[in] time Update timer during mission with pre-configured mission time
*/
void updateMissiontimer(uint32_t);

/**@brief Uninitialize mission timer.
*/
void uninitMissiontimer(void);


/**@brief Function for Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void);

#endif // _TIMERS_H_


/** @} */