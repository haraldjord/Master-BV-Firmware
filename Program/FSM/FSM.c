

#include "FSM.h"

extern mission_t mission;


/** @file
 *
 * @defgroup fsm FSM program file
 * @{
 * @ingroup Program
 *
 * @brief Contain FSM module with related structures and functions.
 */

// FSM event handlers
/**@brief handler to go to initialization state.*/
systemState_t InitStateHandler(void)
{
    return INIT;  /**< Go to INIT state.*/
}

/**@brief handler to go to idle state.*/
systemState_t IdleStateHandler(void)
{
   return IDLE; /**< Go to IDLE state.*/
}

/**@brief handler to go to configuration state.*/
systemState_t ConfigureStateHandler(void)
{
    NRF_LOG_INFO("Go to state: CONFIGURE");
    return CONFIGURE; /**< Go to CONFIGURE state.*/
}

/**@brief handler to go to failure state.
*
* @note Currently not in use as @ref FAILUREstate() is called directly.
*/
systemState_t failureHandler(void){
    NRF_LOG_INFO("Go to state: FAILURE");
    return FAILURE; /**< Go to FAILURE state.*/
}

/**@brief handler to go to mission state.*/
systemState_t MissionStartHandler(void){
    NRF_LOG_INFO("Go to state: Mission");
    return MISSION; /**< Go to MISSION state.*/
}

/**@brief handler to go to PickUp state.*/
systemState_t MissionFinishedHandler(void){
    NRF_LOG_INFO("MissionFinished. Go to state: PickUp");
    return PICKUP; /**< Go to PICKUP state.*/
}

/**@brief handler to go to failure state upon aborted mission.
*
* @note Currently not in use as @ref FAILUREstate() is called directly in case of errors,
*       and mission is aborted only on errors or intended (and accidental) hall effect interrupt.
*/
systemState_t MissionAbortHandler(void){
      NRF_LOG_INFO("SIMULATE MissionAbort go to state: FAILURE");
      return FAILURE; /**< Go to FAILURE state.*/
}

/**@brief handler to go to PickUp state.*/
systemState_t SleepHandler(void){
    return SLEEP; /**< Go to SLEEP state. */
}

/**@brief handler to go to low power state state.
*
* @note Currently not in use as @ref LOWPOWERstate() is called directly.
*/
systemState_t LowPowerHandler(void){
    NRF_LOG_INFO("LOW POWER: Go to state: LOWPOWER")
    return LOWPOWER; /** Go to LOWPOWER state. */
}


/**@brief handler to go to failure state.
*
* @note Deprecated, no longer in use
*/
void waitForEvent(){
   idle_state_handle();
}


/**@brief Initialize Finite state machine after reset.
*/
void INITstate(void){

NRF_LOG_INFO("INIT state");
      
    startBatteryMeasureTimer();
    start_updateFSM_timer_handler();
    fsm.nextState = IdleStateHandler();
    fsm.stateInitialized = false;
    fsm.IDLEstateTimeout = false;
    fsm.BLEgotoIdle      = false;
    fsm.BLEstartMission  = false;
    fsm.hallEffectButton = false;
    mission.MeasuredData.battery = LOW_POWER_THRESHOLD+10; /**< Initialize battery to just above LOWPOWER limit.*/
    mission.MeasuredData.pressureVoltage = SAADC_MIN; /**< Initialize battery to just above LOWPOWER limit.*/
    stopLED();
}


/**@brief Idle state, transistion to sleep mode after 2 minutes
*
* @details BLE is not advertising, but system is in run-mode and ready to transition
*         to next state upon hall effect button interrupt. otherwise it goes back to
*         off-mode after 2 min.
*/
void IDLEstate(){

NRF_LOG_INFO("IDLE state");
    if (!fsm.stateInitialized)
    {
      updateLED(1, 1, 1); /**< White color at minimum brightness. */
      startLED();

      startSleepTimer();
      
      if(BLEconnected())
      {
        disableAdvOnDisconnect();
        BLEdisconnect();
      }
      fsm.stateInitialized = true;

    }
    else if (fsm.IDLEstateTimeout == true){

      fsm.IDLEstateTimeout = false;

      
      if(BLEconnected())
      {
        disableAdvOnDisconnect();
        BLEdisconnect();
      }
      fsm.nextState = SleepHandler();

    }
    else if(fsm.hallEffectButton == true){
      fsm.hallEffectButton = false;
      fsm.stateInitialized = false;
      stopSleepTimer();
      fsm.nextState = ConfigureStateHandler();
      stopLED();
    }
    else
      fsm.nextState = fsm.state;
    

        
}

/****************************

Configuration state

****************************/
/**@brief Configure state, BLE advertise to configre vehicle or transfer file.
*/
void CONFIGUREstate(){

NRF_LOG_INFO("CONFIGURE state");
        if (!fsm.stateInitialized){
          updateLED(0,0,1); /**< blue LED.*/
          startLED();
          
          if(!BLEconnected())
          {
            enableAdvOnDisconnect();
            advertising_start(false);
          }
          fsm.stateInitialized = true;
        }
        else if(fsm.BLEstartMission || fsm.hallEffectButton){
          fsm.hallEffectButton = false;
          fsm.BLEstartMission = false;
          stopLED();

          fsm.nextState = MissionStartHandler();
          fsm.stateInitialized = false;
          
        }
        else if(fsm.BLEgotoIdle){
         fsm.BLEgotoIdle = false;
         
         fsm.nextState = IdleStateHandler();
         fsm.stateInitialized = false;
         stopLED();

        }
        else if(fsm.BLEgotoConfig){
          fsm.BLEgotoConfig = false;
          fsm.nextState = fsm.state;
        }
        else
          fsm.nextState = fsm.state;
}


/****************************

Mission state

****************************/
/**@brief Mission state, BLE is disconnected, and mission is started
*/
void MISSIONstate(){

NRF_LOG_INFO("MISSION state");
    if(!fsm.stateInitialized){
       updateLED(0,10,0); /**< green LED.*/
       startLED();
       
       if(BLEconnected())
       {
          disableAdvOnDisconnect();
          BLEdisconnect();
       }
       else{ stopAdvertising(); }
       prepareMission();
       fsm.stateInitialized = true;
    }
    else if(mission.running == false){
       mission.running = true;
       
       runMission();  // Run mission 0, 1, and 2 if nrOfMission = 3
       startBatteryMeasureTimer();
    }
    else if (mission.missionFinished == mission.nrOfMissions){
      mission.running = false;
      mission.missionFinished = 0;
      
      fsm.nextState = MissionFinishedHandler(); // Go to state PickUp
      fsm.stateInitialized = false;
      stopLED();
    }
    else
      fsm.nextState = fsm.state;
}

/****************************

Vehicle Pick-Up state

****************************/
/**@brief PickUp state, BLE is advertising, and vehicle can be re-configured and/or file transfered
*/
void PICKUPstate(){

  NRF_LOG_INFO("PICKUP state");

    if(!fsm.stateInitialized){
        updateLED(2,2,0);  /**< Yellow LED.*/
        startLED();
        enableAdvOnDisconnect();
        if(!BLEconnected()) 
          advertising_start(false);

        fsm.stateInitialized = true;
    }
    else if(fsm.BLEstartMission){
          fsm.BLEstartMission = false;

          fsm.nextState = MissionStartHandler();
          fsm.stateInitialized = false;
          stopLED();
    }
    else if (fsm.BLEgotoIdle || fsm.hallEffectButton){

       if(BLEconnected())
       {
          disableAdvOnDisconnect();
          BLEdisconnect();
       }else{ stopAdvertising(); }
        fsm.BLEgotoIdle = false;
        fsm.hallEffectButton = false;

        fsm.nextState = IdleStateHandler();
        fsm.stateInitialized = false;
        stopLED();
    }
    else if (fsm.BLEgotoConfig){
        fsm.BLEgotoConfig = false;

        fsm.nextState = ConfigureStateHandler();
        fsm.stateInitialized = false;
        stopLED();
    }
    else
      fsm.nextState = fsm.state;
      
}


/****************************

Sleep state

****************************/
/**@brief Sleep state, SD card is unmounted and system is powered down to off-mode
*/
void SLEEPstate(void){
    NRF_LOG_INFO("SLEEP state");
    unMount();
    sleep_mode_enter();
}

/****************************

Low battery power state

****************************/
void LOWPOWERstate(){
  stopLED();
  NRF_LOG_INFO("LOW Power state");
  NRF_LOG_INFO("Battery: %d",mission.MeasuredData.battery);
  updateLED(10,0,0); /**< Red LED.*/
  startLED();
  mission.pidData.output = 0.000000;
  setPistonPosition(); /**< Float to surface.*/
  unMount(); /**< unmount SD card.*/
  while(1)         // stay in lowPowerState, until reset and change of battery.
  { __WFE(); }     // Risk of losing BV if under water
}                   

/****************************

Failure state

****************************/
void FAILUREstate(){
  stopLED();
  updateLED(10,0,2);  // pink LED
  startLED();
  mission.pidData.output = 0.000000;
  setPistonPosition();
  //motorUp();                        // Float to surface
  unMount();
  NRF_LOG_INFO("FAILURE state");
  //while(1)         // stay in Failure state, until picked up and reset.
  //{ __WFE(); }     // Risk of losing BV if under water
}





void FSM(){

    ret_code_t err_code;
                   
    switch(fsm.state)
    {
      case INIT:
        INITstate();
        break;
      case IDLE:
        IDLEstate();
        break;
      case CONFIGURE:
        CONFIGUREstate();
        break;
      case MISSION:
        MISSIONstate();
        break;
      case PICKUP:
        PICKUPstate();
        break;
      case SLEEP:
        SLEEPstate();
        break;
      case LOWPOWER:
        LOWPOWERstate();
        break;
      case FAILURE:
        FAILUREstate();
        break;
      default:
        NRF_LOG_INFO("UNKNOWN state");
    }
    if(mission.MeasuredData.battery <= LOW_POWER_THRESHOLD)
      fsm.nextState = LowPowerHandler();

    fsm.state = fsm.nextState;
  
}

/** @} */