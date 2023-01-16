
#include "timers.h"
#include "FSM.h"
#include "mission.h"


extern FSM_t fsm;
extern mission_t mission;
extern bool sampleSensorData;
extern bool updateFSM;
extern bool missionLogUpdated;
extern bool motorStopped;

/** @file
 *
 * @defgroup timers timer program file
 * @{
 * @ingroup Program
 *
 * @brief Contain timer module with related functions.
 */

/**@brief Create hardware peripheral timer instace - Timer 1.
*/
static const nrfx_timer_t m_missionTimer = NRFX_TIMER_INSTANCE(1); 


/**@brief mission Timer Handler - End current mission when triggered.
*/
nrfx_timer_event_handler_t missionTimerHandler(nrf_timer_event_t event_type, void * p_context)
{
    mission.missionFinished++;
}


/**@brief IDLE-to-Sleep timer handler.*/
void SleepTimerHandler(void)
{
    fsm.IDLEstateTimeout = true;
}

/**@brief stop Motor timer handler.*/
void stopMotorTimerHandler(void)
{
    motorStopped = false;
}

/**@brief Update FSM timer handler.*/
static void updateFSM_timer_handler(void * p_context)
{
  updateFSM = true;
}

/**@brief Measure battery timer handler. */
static void repeatedBattery_timer_handler(void * p_context)
{
    nrfx_saadc_sample();
}

/**@brief Sample SAADC timer handler.*/
static void sampleSensorData_timer_handler(void * p_context)
{
  sampleSensorData = true;
}

/**@brief Update mission log timer handler.*/
static void updateMissionLog_timer_handler(void * p_context)
{
  updateMissionLog();
  missionLogUpdated = true;
}






void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);


    // Create timers.

    // Hardware Mission Timer
    nrfx_timer_config_t missionTimerConfig = NRFX_TIMER_DEFAULT_CONFIG;
    missionTimerConfig.frequency = (nrf_timer_frequency_t) NRF_TIMER_FREQ_125kHz;
    missionTimerConfig.mode      = (nrf_timer_mode_t)      NRF_TIMER_MODE_TIMER;
    missionTimerConfig.bit_width = (nrf_timer_bit_width_t) NRF_TIMER_BIT_WIDTH_32;
    missionTimerConfig.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY;
    missionTimerConfig.p_context = NULL;

    err_code = nrfx_timer_init(&m_missionTimer,&missionTimerConfig,missionTimerHandler);
    APP_ERROR_CHECK(err_code);
    nrfx_timer_extended_compare(&m_missionTimer,
                                NRF_TIMER_CC_CHANNEL0,
                                0,                                    // Initialized to 0 us
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK,   // Stop timer on COMPARE0 Interrupt
                                true); 
    nrfx_timer_disable(&m_missionTimer);


    //Advertising LED timer
    err_code = app_timer_create(&m_updateFSM_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                updateFSM_timer_handler);
    APP_ERROR_CHECK(err_code);

    //Idle-to-Sleep timer
    err_code = app_timer_create(&m_sleep_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                SleepTimerHandler);
    APP_ERROR_CHECK(err_code);

    // stopMotor timer
    err_code = app_timer_create(&m_motorStop_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                stopMotorTimerHandler);
    APP_ERROR_CHECK(err_code);

    // Battery timer
    err_code = app_timer_create(&m_repeatedBattery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                repeatedBattery_timer_handler);
    APP_ERROR_CHECK(err_code);
    
    // Sample sensorData timer
    err_code = app_timer_create(&m_sampleSensorData_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sampleSensorData_timer_handler);
    APP_ERROR_CHECK(err_code);
    
    // Update missionLog timer
    err_code = app_timer_create(&m_updateMissionLog_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                updateMissionLog_timer_handler);
    APP_ERROR_CHECK(err_code);

}
/**@snippet [Timer initialization]*/



void start_updateFSM_timer_handler(){
  ret_code_t err_code = app_timer_start(m_updateFSM_timer_id, UPDATE_FSM_TIMER, NULL);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [start timer for FSM updates]*/


void stop_updateFSM_timer_handler(){
  ret_code_t err_code = app_timer_stop(m_updateFSM_timer_id);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [stop timer for FSM updates]*/


void startSampleSensorDatatimer(){
  ret_code_t err_code = app_timer_start(m_sampleSensorData_timer_id, SENSORS_SAMPLE_TIMER, NULL);  // Toggle every second
  APP_ERROR_CHECK(err_code);
}
/**@snippet [start timer for SensorData sampling]*/


void stopSampleSensorDatatimer(){
  ret_code_t err_code = app_timer_stop(m_sampleSensorData_timer_id);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [stop timer for SensorData sampling]*/



void startSleepTimer(){
  ret_code_t err_code = app_timer_start(m_sleep_timer_id, IDLE_TO_SLEEP_TIME, NULL);  // 2 min Idle time 2000ms*60
  APP_ERROR_CHECK(err_code);
}
/**@snipped [start timer for Idle-to-sleep event]*/



void stopSleepTimer(){
  ret_code_t err_code = app_timer_stop(m_sleep_timer_id);
  APP_ERROR_CHECK(err_code);
}
/**@snipped [stop timer for Idle-to-sleep event]*/


void startMotorStopTimer(){
  ret_code_t err_code = app_timer_start(m_motorStop_timer_id, MOTOR_STOP_TIME, NULL);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [start timer for count down until check if motor has stopped]*/


void startBatteryMeasureTimer(){
  ret_code_t err_code = app_timer_start(m_repeatedBattery_timer_id, MEASURE_BATTERY_TIMER,NULL);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [start timer for measuring battery voltage]*/


void stopBatteryMeasureTimer(){
  ret_code_t err_code = app_timer_stop(m_repeatedBattery_timer_id);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [stop timer for measuring battery voltage]*/


void startUpdateMissionLogTimer(){
  ret_code_t err_code = app_timer_start(m_updateMissionLog_timer_id, UPDATE_MISSIONLOG_TIMER,NULL);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [start timer to update mission log]*/


void stopUpdateMissionLogTimer(){
  ret_code_t err_code = app_timer_stop(m_updateMissionLog_timer_id);
  APP_ERROR_CHECK(err_code);
}
/**@snippet [stop timer to update mission log]*/



void startMissiontimer(){
  nrfx_timer_enable(&m_missionTimer);
}
/**@snippet [start mission timer]*/


void stopMissiontimer(){
  nrfx_timer_disable(&m_missionTimer);
}
/**@snippet [stop mission timer]*/


void updateMissiontimer(uint32_t time){
    time = time*KHZ_TO_SEC;
    nrfx_timer_extended_compare(&m_missionTimer,
                                NRF_TIMER_CC_CHANNEL0,
                                time,
                                NRF_TIMER_SHORT_COMPARE0_STOP_MASK,
                                true); 
}
/**@snippet [Set new mission time]*/


void uninitMissiontimer(){
  nrfx_timer_uninit(&m_missionTimer);
}
/**@snippet [Uninitialize mission timer]*/


/** @} */