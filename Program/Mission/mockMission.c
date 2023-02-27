#include "mockMission.h"
#include "main.h"

/*mock Mission tend to test mission and PID firmware in dry condition in debugg mode.*/


//extern bool g_motorStopped;         // Flag to signal motor stopped
//extern bool bottomLimit;          //< Flag to signal when bottom limit switch is reached             
//extern bool SAADCdataReady;      //< Flag to signal SAADC data is ready to be read and calculated, and to run PID
//extern bool sampleSensorData;    //< Flag to signal sampling battery, pressure, and TMP117

//float EMA_state = 0.0;         //OBSOLETE   < global filter output variable     
missionLog_t extern missionLog;          //< Create mission log instance
struct pid_controller extern ctrlData;   //< Create PID controller control data instance
pid_t extern pid;

void testGlobal(){
if(g_motorStopped)
  printf("Hello");
}



void mockmissionInit(){

   memset(&mission, 0, sizeof(mission));
   memset(&pid, 0, sizeof(pid));

   mission.missionFinished      = 0;
   mission.nrOfMissions         = 0;
   mission.missionNr[0].depth   = 1;
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
   mission.pidData.ki = 0.1;//PID_kI;
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


void preparemockmisison(){

  pid_tune(pid, mission.pidData.kp, mission.pidData.ki, mission.pidData.kd);
  pid_auto(pid);  /**< Enable PID controller. */

  mission.nrOfMissions = 0;
  for(int i = 0; i < MAX_NR_OF_MISSIONS; i++){    // Check number of planned missions
    if(mission.missionNr[i].time != NULL)         // mission with zero time, does not count as mission
      mission.nrOfMissions++;
   }

  mission.timeStamp = 0;
  missionLogInit();   // Wipe Log struct
  createMissionLog(); // Create new missionLog file

  stopBatteryMeasureTimer();

  g_SAADCdataReady = false;
  g_sampleSensorData = false;
}




void runmockmission(){
  g_sampleIMUdata = false;
  startSampleIMUdataTimer(); // here used as timer to set new measured depth

  int timeStamp = 0, oldTimestamp = 0, elapsedTime = 0;
  //uint8_t tempMSB, tempLSB;

  startUpdateMissionLogTimer();
  oldTimestamp = app_timer_cnt_get();
  uint8_t missionId = mission.missionFinished;
  mission.currentMission = mission.missionNr[missionId];

  NRF_LOG_INFO("Running mock mission");
  updateMissiontimer(120); //Run mission for x seconds.
  startMissiontimer();  // Current mission time periode
  startSampleSensorDatatimer(); // start mission timer.
  while(missionId == mission.missionFinished){

    if(g_sampleSensorData){ // true by timer every 0.5 seconds 
      g_sampleSensorData = false;
      nrfx_saadc_sample();
      }

    if(g_SAADCdataReady){ // true when SAADC has new data.
        g_SAADCdataReady = false;
        CalcPressureAndDepth_v2();
        if( fabs(mission.currentMission.depth - mission.MeasuredData.filteredDepth) > mission.pidData.kiThreshold ) //< If x meters away from target, run as PD regulator, otherwise run as PID regulator. This is so that it regulates quicker towards target 
          pid_tune(pid, mission.pidData.kp, 0, mission.pidData.kd);
        else
          pid_tune(pid, mission.pidData.kp, mission.pidData.ki, mission.pidData.kd);
        
        mission.pidData.pistonPosition = getPistonPosition();
        
        pid_compute(pid);
        timeStamp = app_timer_cnt_get();
        if(timeStamp >= oldTimestamp)      ///<Check for and handle RTC wrap around
          elapsedTime = timeStamp - oldTimestamp;
        else
          elapsedTime = timeStamp+RTC_MAX_COUNT - oldTimestamp;
        
        mission.timeStamp += (float)elapsedTime/32.768; // Elapsed time in milliseconds [32.768 KHz LF clock frequency]
        oldTimestamp = timeStamp;
        printf("mission timer: %d\n", mission.timeStamp);
        printf("-----------\n");


        //setPistonPosition(); commented out for dry testing.
          
      }

   }
}

void CalcPressureAndDepth_mock(void){
  // mock mission:
  if (!g_sampleIMUdata) //Here used as timer to change depth.
    mission.MeasuredData.filteredDepth = 0.5; // set to a constant when drytesting.
  else
    mission.MeasuredData.filteredDepth = 1.5;
    printf("filtered Depth: %.2f\n", mission.MeasuredData.filteredDepth);
}

}

