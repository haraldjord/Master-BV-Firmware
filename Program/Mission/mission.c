
#include"mission.h"

/** @file
 *
 * @defgroup PID PID program file
 * @{
 * @ingroup Program
 *
 * @brief Contain mission module with related structures and functions.
 */

//extern bool motorStopped;         /**< Flag to signal motor stopped*/
//extern bool bottomLimit;          /**< Flag to signal when bottom limit switch is reached             */
//bool SAADCdataReady = false;      /**< Flag to signal SAADC data is ready to be read and calculated, and to run PID*/
//bool sampleSensorData = false;    /**< Flag to signal sampling battery, pressure, and TMP117*/
//bool sampleIMUdata = false;       /**< Flag to signal sampling data from IMU (icm chip)*/
//bool missionLogUpdated = false;   /**< Flag to signal mission log is updated*/
//bool TMP117dataReady = false;     /**< Flag to signal TMP117 data is ready to be read*/
//bool receiveTMP117 = false;       /**< Flag to signal that data is expected to be received from TMP117*/
//float EMA_state = 0.0;         //OBSOLETE   /**< global filter output variable */    
missionLog_t missionLog;          /**< Create mission log instance*/
struct pid_controller ctrlData;   /**< Create PID controller control data instance*/
pid_t pid;                        /**< Create PID controller instnace*/


const float SAADC_pressure_range = SAADC_MAX - SAADC_MIN;
const float bar_pr_saadcValue = BAR_RANGE/SAADC_pressure_range;
const float pascal_range = PASCAL_MAX-PASCAL_MIN;
const float pascal_pr_volts = pascal_range/(PRESSURE_VOLTAGE_MAX-PRESSURE_VOLTAGE_MIN);
 
void missionInit(){

   memset(&mission, 0, sizeof(mission));
   memset(&pid, 0, sizeof(pid));

   mission.missionFinished      = 0;
   mission.nrOfMissions         = 0;
   mission.missionNr[0].depth   = DEFAULT_M1_DEPTH;
   mission.missionNr[0].time    = DEFAULT_M1_TIME;
   mission.missionNr[1].depth   = DEFAULT_M2_DEPTH; 
   mission.missionNr[1].time    = DEFAULT_M2_TIME; 
   mission.missionNr[2].depth   = 0.0; 
   mission.missionNr[2].time    = 0; 
   mission.missionNr[3].depth   = 0.0; 
   mission.missionNr[3].time    = 0; 
   mission.currentMission.depth = mission.missionNr[0].depth;
   mission.currentMission.time  = mission.missionNr[0].time; // timer compare value at current mission time
   mission.MeasuredData.unfilteredDepth = 0.001;  // Initialize to 1 mm
   mission.MeasuredData.filteredDepth = 0.001;    // Initialized to 1mm
   mission.MeasuredData.pressureSensorOffset = 0; // Initialized to 0
     
   mission.pidData.kp = PID_kP;
   mission.pidData.ki = PID_kI;
   mission.pidData.kd = PID_kD;
   mission.pidData.kiThreshold = PID_kI_THRESHOLD;
   mission.pidData.atmosphericPressure = PSI_1ATM_PRESSURE;

   mission.running = false;

   pid = pid_create(&ctrlData, &mission.MeasuredData.filteredDepth, &mission.pidData.output, &mission.currentMission.depth,  PID_kP, PID_kI, PID_kD);
   pid_direction(pid, E_PID_DIRECT);
   pid_limits(pid,PID_LIMIT_MIN,PID_LIMIT_MAX);
   pid_manual(pid); // Disable PID controller

   disablePressureSensor();
   // initiate EMA filter. Might be unnesesarrly, as pressure offset is initateted to zero??
   if (mission.MeasuredData.pressureSensorOffset != NULL){
      mission.MeasuredData.filteredDepth = mission.MeasuredData.pressureSensorOffset;
   }
   else{
      mission.MeasuredData.pressureSensorOffset = 0;
      mission.MeasuredData.filteredDepth = 0;
   }
}
/**@snippet [Mission Init]*/



void prepareMission(){

  pid_tune(pid, mission.pidData.kp, mission.pidData.ki, mission.pidData.kd);
  pid_auto(pid);  /**< Enable PID controller. */

  enablePressureSensor();

  //EMA_state = SAADC_MIN; // OBSOLETE...

  mission.nrOfMissions = 0;
  for(int i = 0; i < MAX_NR_OF_MISSIONS; i++){             // Check number of planned missions
    if(mission.missionNr[i].time != NULL) // mission with zero time, does not count as mission
      mission.nrOfMissions++;
   }

  mission.timeStamp = 0;
  missionLogInit();   // Wipe Log struct
  createMissionLog(); // Create new missionLog file

  stopBatteryMeasureTimer();

  g_SAADCdataReady = false;
  g_sampleSensorData = false;
}
/**@snippet [Prepare mission]*/


void runMission(){

  int timeStamp = 0, oldTimestamp = 0, elapsedTime = 0;
  //uint8_t tempMSB, tempLSB;

  startUpdateMissionLogTimer();
  oldTimestamp = app_timer_cnt_get();

  for(int i = 0; i < mission.nrOfMissions; i++)
  {
      uint8_t missionId = mission.missionFinished;
      mission.currentMission = mission.missionNr[missionId];

      NRF_LOG_INFO("Running mission %d",mission.missionFinished);
      updateMissiontimer(mission.currentMission.time);
      startMissiontimer();  // Current mission time periode
      startSampleSensorDatatimer();
      while(missionId == mission.missionFinished){

          if(g_missionLogUpdated){
            g_missionLogUpdated = false;
            writeMissionLog();
          }

          if(g_sampleSensorData){
            g_sampleSensorData = false;
            nrfx_saadc_sample();
            mission.MeasuredData.outside_temperature = TMP117_read_temp();
            read_accel_data();
            read_gyro_data();
           }

           if(g_SAADCdataReady){
              g_SAADCdataReady = false;
              CalcPressureAndDepth_v2();
              if( fabs(mission.currentMission.depth - mission.MeasuredData.filteredDepth) > mission.pidData.kiThreshold ) /**< If x meters away from target, run as PD regulator, otherwise run as PID regulator. This is so that it regulates quicker towards target */
                pid_tune(pid, mission.pidData.kp, 0, mission.pidData.kd);
              else
                pid_tune(pid, mission.pidData.kp, mission.pidData.ki, mission.pidData.kd);
              
              mission.pidData.pistonPosition = getPistonPosition();
              
              pid_compute(pid);
              timeStamp = app_timer_cnt_get();
              if(timeStamp >= oldTimestamp)      /**<Check for and handle RTC wrap around*/
                elapsedTime = timeStamp - oldTimestamp;
              else
                elapsedTime = timeStamp+RTC_MAX_COUNT - oldTimestamp;
              
              mission.timeStamp += (float)elapsedTime/32.768; // Elapsed time in milliseconds [32.768 KHz LF clock frequency]
              oldTimestamp = timeStamp;
              
              //setPistonPosition(); commented out for dry testing.
                
            }


        // If low Battery, abort mission and go to low power state.
        if(mission.MeasuredData.battery <= LOW_POWER_THRESHOLD){
          stopMissiontimer();
          stopSampleSensorDatatimer();
          stopUpdateMissionLogTimer();
          closeFile();
          LOWPOWERstate();
        }

        

        __WFE();
        /*
        if(fsm.hallEffectButton){
          fsm.hallEffectButton = false;
          i = mission.nrOfMissions;
          mission.missionFinished = mission.nrOfMissions;
         }*/ // hallEffect not working properly... 
      }

     stopMissiontimer();
     stopSampleSensorDatatimer();  
  }
  
  // All missions finished
  disablePressureSensor();
  motorStop();
  motorDown(); /**< In case bottom limit switch is already triggered */
  nrf_delay_ms(1000);
  g_bottomLimit = false;
  motorUp();             
  startMotorStopTimer();
  stopUpdateMissionLogTimer();
  startBatteryMeasureTimer();
  closeFile();
}
/**@snippet [Run Mission]*/




// OBSOLETE
//void CalcPressureAndDepth(void)
//{
//   EMA_state = (EMA_alpha*mission.MeasuredData.pressure) + ((1-EMA_alpha)*EMA_state); /**< Exponential Moving Average (EMA) filtering on RAW pressure voltage*/
//   mission.MeasuredData.batteryVoltage = ((mission.MeasuredData.battery/16383.0)*24.0)+SAADC_VOLTAGE_ERROR;
//   mission.MeasuredData.pressureVoltage = ((EMA_state/16383)*(9.0/2.0))+SAADC_VOLTAGE_ERROR;
//   mission.MeasuredData.psi = ((mission.MeasuredData.pressureVoltage-PRESSURE_VOLTAGE_MIN)*(PSI_RANGE/PRESSURE_VOLTAGE_RANGE)-(mission.pidData.atmosphericPressure-PSI_1ATM_PRESSURE));
//   mission.MeasuredData.pascal = mission.MeasuredData.psi*PSI_TO_PASCAL;
//   mission.MeasuredData.measuredDepth = ((mission.MeasuredData.psi)*PSI_TO_MH2O + mission.MeasuredData.pressureSensorOffset); // correct for offset before EMA filter calculation?
   //printf("-------------------------------------\n\r");
   //printf("Voltage: %f\n\r",mission.MeasuredData.pressureVoltage);
   //printf("psi: %f\n\r",mission.MeasuredData.psi);
   //printf("pascal: %f\n\r",mission.MeasuredData.pascal);
   //printf("RAWpressure: %f\n\r",mission.MeasuredData.pressure);
   //printf("EMA_STATEpressure: %f\n\r",EMA_state);
   //printf("pressureVoltage: %f\n\r",mission.MeasuredData.pressureVoltage);
   //printf("MeasuredDepth in meters: %f\n\r",mission.MeasuredData.measuredDepth);
   //printf("-------------------------------------\n\r");
//}
/**@snippet [Calculate Pressure and Depth]*/

void CalcPressureAndDepth_v2(void){ // TODO rename.
  mission.MeasuredData.pressureVoltage =  ((mission.MeasuredData.pressure/16383)*4.5) + SAADC_VOLTAGE_ERROR;                                  /**< Raw pressure voltage voltage>*/
  mission.MeasuredData.psi = ((mission.MeasuredData.pressureVoltage-PRESSURE_VOLTAGE_MIN)*(PSI_RANGE/PRESSURE_VOLTAGE_RANGE));                /**< Unfiltered psi*/
  mission.MeasuredData.unfilteredDepth = mission.MeasuredData.psi*PSI_TO_MH2O;                                                                /**< Unfiltered depth*/
  mission.MeasuredData.filteredDepth = (EMA_alpha*mission.MeasuredData.unfilteredDepth) + ((1-EMA_alpha)*mission.MeasuredData.filteredDepth); /**< EMA Filtered depth */
  mission.MeasuredData.filteredDepth = mission.MeasuredData.filteredDepth + mission.MeasuredData.pressureSensorOffset;                        /**< Filtered depth corrected for sensor offset*/
  //printf("pressure sensor voltage: %.2f", mission.MeasuredData.pressureVoltage);

  
}
/**@snittep [calculate Pressure and Depth] included depth offset.*/


void missionLogInit(){

  // Set to zero before each new dive
  memset(&missionLog, 0, sizeof(missionLog));
}
/**@snippet [Initialize mission log]*/




void updateMissionLog(){


   if (mission.timeStamp != 0){ // avoid logging when time is zero
     missionLog.timeStamp               = mission.timeStamp;
     missionLog.missionNr               = mission.missionFinished + 1;
     missionLog.setpoint                = mission.currentMission.depth;
     missionLog.pistonPosition          = mission.pidData.pistonPosition;
     missionLog.pidDataOut.PID_output   = mission.pidData.output;
     missionLog.pidDataOut.kp   = mission.pidDataOut.kp;
     missionLog.pidDataOut.ki   = mission.pidDataOut.ki;
     missionLog.pidDataOut.kd   = mission.pidDataOut.kd;
     missionLog.pressureVoltage = mission.MeasuredData.pressureVoltage;
     // missionLog.pressureDepth   = mission.MeasuredData.measuredDepth; OBSOLETE
     missionLog.pressurePsi     = mission.MeasuredData.psi;
     missionLog.unfilteredDepth = mission.MeasuredData.unfilteredDepth;
     missionLog.filteredDepth   = mission.MeasuredData.filteredDepth;
     //missionLog.pressurePascal  = mission.MeasuredData.pascal; OBSOLETE
     missionLog.batteryVoltage  = mission.MeasuredData.batteryVoltage;
     missionLog.temperature     = mission.MeasuredData.outside_temperature;
     /**< ICM motion data is stored as int16_t to reduce space, for external analysis divide by 128 (resolution)*/
     missionLog.motionData.accel.x            = icm.accel.x;
     missionLog.motionData.accel.y            = icm.accel.y;
     missionLog.motionData.accel.z            = icm.accel.z;
     missionLog.motionData.accel.sensitivity  = icm.accel.sensitivity;
     missionLog.motionData.gyro.x             = icm.gyro.x;
     missionLog.motionData.gyro.y             = icm.gyro.y;
     missionLog.motionData.gyro.z             = icm.gyro.z;
     missionLog.motionData.gyro.sensitivity   = icm.gyro.sensitivity;

    }
}
/**@snippet [Update mission log]*/

/** @} */