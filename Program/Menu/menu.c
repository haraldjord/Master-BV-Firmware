#include "menu.h"

extern enum menu currentMenu;
enum config configVariable;
extern bool getValue;
extern bool transferDataFlag;
extern bool sendNUS;
enum fileoption fileOption;

extern missionLog_t missionLog;

/** @file
 *
 * @defgroup menu menu program file
 * @{
 * @ingroup Program
 *
 * @brief Contain Menu module with related functions.
 */



void setConfigValue(char * value)
{

  float floatValue;
  int   intValue;
  getValue = false;

  switch(configVariable){
    case M1DEPTH:
      floatValue = atof(value);
      if(floatValue > MAX_DEPTH) floatValue = MAX_DEPTH;
      else if(floatValue < MIN_DEPTH) floatValue = MIN_DEPTH;
      mission.missionNr[0].depth = floatValue;
      sendNUS = true;
      break;
    case M1TIME:
      intValue = atoi(value);
      mission.missionNr[0].time = intValue;
      sendNUS = true;
      break;
    case M2DEPTH:
      floatValue = atof(value);
      if(floatValue > MAX_DEPTH) floatValue = MAX_DEPTH;
      else if(floatValue < MIN_DEPTH) floatValue = MIN_DEPTH;
      mission.missionNr[1].depth = floatValue;
      sendNUS = true;
      break;
    case M2TIME:
      intValue = atoi(value);
      mission.missionNr[1].time = intValue;
      sendNUS = true;
      break;
    case M3DEPTH:
      floatValue = atof(value);
      if(floatValue > MAX_DEPTH) floatValue = MAX_DEPTH;
      else if(floatValue < MIN_DEPTH) floatValue = MIN_DEPTH;
      mission.missionNr[2].depth = floatValue;
      sendNUS = true;
      break;
    case M3TIME:
      intValue = atoi(value);
      mission.missionNr[2].time = intValue;
      sendNUS = true;
      break;
    case M4DEPTH:
      floatValue = atof(value);
      if(floatValue > MAX_DEPTH) floatValue = MAX_DEPTH;
      else if(floatValue < MIN_DEPTH) floatValue = MIN_DEPTH;
      mission.missionNr[3].depth = floatValue;
      sendNUS = true;
      break;
    case M4TIME:
      intValue = atoi(value);
      mission.missionNr[3].time = intValue;
      sendNUS = true;
      break;


    case PID_P:
      floatValue = atof(value);
      mission.pidData.kp = floatValue;
      sendNUS = true;
      break;
    case PID_I:
      floatValue = atof(value);
      mission.pidData.ki = floatValue;
      sendNUS = true;
      break;
    case PID_D:
      floatValue = atof(value);
      mission.pidData.kd = floatValue;
      sendNUS = true;
      break;

    case THRESHOLD:
      floatValue = atof(value);
      mission.pidData.kiThreshold = floatValue;
      sendNUS = true;
     break;
   
    case ATM_PRESSURE:
      floatValue = atof(value);
      mission.pidData.atmosphericPressure = floatValue;
      sendNUS = true;
      break;
    default: NRF_LOG_INFO("Unknown value in setConfigValue()");
  }
}
/**@snippet [function for setting configuration value received from BLE client]*/



void mainMenu(int option)
{
  switch(option) {
    case 0:{
      NRF_LOG_INFO("Case 0\n");
      break;
     }
    case 1:
      NRF_LOG_INFO("Option 1\n");
      currentMenu = MISSIONDATA;
      sendNUS = true;
     break;

    case 2:
        NRF_LOG_INFO("Option 2\n"); 
        currentMenu = CONFIGVEHICLE;
        sendNUS = true;
        break;

    case 3: 
        NRF_LOG_INFO("Option 3\n");
        currentMenu = TRANSFERDATA;
        sendNUS = true;
        break;

    case 4:
        NRF_LOG_INFO("Option 4\n");
        fsm.BLEstartMission = true;
        break;

    case 5:
        NRF_LOG_INFO("Option 5\n");
        fsm.BLEgotoIdle = true;
        break;

    case 6:
        NRF_LOG_INFO("Option 6\n");
        fsm.BLEgotoConfig = true;
        break;

    case 9:
        NRF_LOG_INFO("Option 9\n");
        currentMenu = MAINMENU;
        sendNUS = true;
        break;

    default:
       NRF_LOG_INFO("Unknown option: %d\n", option);
    }
}
/**@snippet [Main Menu]*/



void missinDataMenu(int option)
{
  uint16_t msgLength;

  switch(option) {
    case 0:{
      NRF_LOG_INFO("Case 0\n");
      break;
     }
    case 1:
      NRF_LOG_INFO("Option 1\n");
      uint8_t mission1Depth[] = "Type in Mission1 Depth - x.xx (float) - :";
      msgLength = sizeof(mission1Depth);
      nus_send(mission1Depth,msgLength);
      getValue = true;
      configVariable = M1DEPTH;
     break;

    case 2:
      NRF_LOG_INFO("Option 2\n"); 
      uint8_t mission1Time[] = "Type in Mission1 Time - xx (integer) - :";
      msgLength = sizeof(mission1Time);
      nus_send(mission1Time, msgLength);
      getValue = true;
      configVariable = M1TIME;
     break;
        

    case 3: 
      NRF_LOG_INFO("Option 3\n");
      uint8_t mission2Depth[] = "Type in Mission2 depth - x.xx (float) - :";
      msgLength = sizeof(mission2Depth);
      nus_send(mission2Depth, msgLength);
      getValue = true;
      configVariable = M2DEPTH;
     break;

    case 4:
      NRF_LOG_INFO("Option 4\n");
      uint8_t mission2Time[] = "Type in Mission2 Time - xx (integer) - :";
      msgLength = sizeof(mission2Time);
      nus_send(mission2Time,msgLength);
      getValue = true;
      configVariable = M2TIME;
     break;

    case 5:
      NRF_LOG_INFO("Option 5\n");
      uint8_t mission3Depth[] = "Type in Mission3 depth - x.xx (float) - :";
      msgLength = sizeof(mission3Depth);
      nus_send(mission3Depth,msgLength);
      getValue = true;
      configVariable = M3DEPTH;
     break;

    case 6:
      NRF_LOG_INFO("Option 6\n");
      uint8_t mission3Time[] = "Type in Mission3 Time - xx (integer) - :";
      msgLength = sizeof(mission3Time);
      nus_send(mission3Time,msgLength);
      getValue = true;
      configVariable = M3TIME;
     break;

    case 7: NRF_LOG_INFO("Option 7\n");
      uint8_t mission4Depth[] = "Type in Mission4 depth - x.xx (float) - :";
      msgLength = sizeof(mission4Depth);
      nus_send(mission4Depth,msgLength);
      getValue = true;
      configVariable = M4DEPTH;
     break;

    case 8: NRF_LOG_INFO("Option 8\n");
      uint8_t mission4Time[] = "Type in Mission4 Time - xx (integer) - :";
      msgLength = sizeof(mission4Time);
      nus_send(mission4Time,msgLength);
      getValue = true;
      configVariable = M4TIME;
     break;
    
   case 9: NRF_LOG_INFO("Option 9\n");
      currentMenu = MAINMENU;
      sendNUS = true;
     break;

    default:
       NRF_LOG_INFO("Unknown option: %d\n", option);
    }
}
/**@snippet [navigate MissionD ata menu]*/


void configVehicleMenu(int option)
{
  uint16_t msgLength;

  switch(option) {
    case 0:{
      NRF_LOG_INFO("Case 0\n");
      break;
     }
    case 1:
      NRF_LOG_INFO("Option 1\n");
      uint8_t p_msg[] = "Type in P value - x.xxx (float) - :";
      msgLength = sizeof(p_msg);
      nus_send(p_msg,msgLength);
      getValue = true;
      configVariable = PID_P;
     break;

    case 2:
      NRF_LOG_INFO("Option 2\n"); 
      uint8_t i_msg[] = "Type in I value - x.xxx (float) - :";
      msgLength = sizeof(i_msg);
      nus_send(i_msg,msgLength);
      getValue = true;
      configVariable = PID_I;
     break;

    case 3: 
      NRF_LOG_INFO("Option 3\n");
      uint8_t d_msg[] = "Type in D value - x.xxx (float) - :";
      msgLength = sizeof(d_msg);
      nus_send(d_msg,msgLength);
      getValue = true;
      configVariable = PID_D;
     break;

    case 4:
      NRF_LOG_INFO("Option 4\n");
      uint8_t threshold_msg[] = "Type in Ki Threshold value (m) - x.x (float) - :";
      msgLength = sizeof(threshold_msg);
      nus_send(threshold_msg,msgLength);
      getValue = true;
      configVariable = THRESHOLD;
     break;

   case 5:
      NRF_LOG_INFO("Option 5\n");
      uint8_t atmosphericPressure_msg[] = "Type in current atmospheric pressure (psi) - x.x (float) - :";
      msgLength = sizeof(atmosphericPressure_msg);
      nus_send(atmosphericPressure_msg,msgLength);
      getValue = true;
      configVariable = ATM_PRESSURE;
     break;

    case 9:
      NRF_LOG_INFO("Option 9\n");
      currentMenu = MAINMENU;
      sendNUS = true;
     break;

    default:
       NRF_LOG_INFO("Unknown option: %d - in configVehiclemenu\n", option);
    }
}
/**@snippet [navigate ConfigVehicle menu]*/


void transferDataMenu(int option)
{
  uint16_t msgLength;

  switch(option){
    case 0:
      NRF_LOG_INFO("Case 0\n");
      break;
     
    case 1:
      NRF_LOG_INFO("Option 1\n");
      uint8_t file1_msg[] = "Confirm transfer all files: 1 (yes), 0 (No)";
      nus_send(file1_msg, sizeof(file1_msg));
      transferDataFlag = true;
      fileOption = TRANSFER_ALL;
     break;

    case 2:
       NRF_LOG_INFO("Option 2\n"); 
       uint8_t file2_msg[] = "Transfer one file by numbers 1 to xx. 0 to cancel: ";
       nus_send(file2_msg,sizeof(file2_msg));
       transferDataFlag = true;
       fileOption = TRANSFER_ONE;
      break;

    case 3: 
       NRF_LOG_INFO("Option 3\n");
       uint8_t file3_msg[] = "Confirm DELETE all files: 1 (yes), 0 (No)";
       nus_send(file3_msg,sizeof(file3_msg));
       transferDataFlag = true;
       fileOption = DELETE_ALL;
      break;
    case 4:
       NRF_LOG_INFO("Option 4\n"); 
       uint8_t file4_msg[] = "Delete one file by numbers 1 to xx. 0 to cancel: ";
       nus_send(file4_msg,sizeof(file4_msg));
       transferDataFlag = true;
       fileOption = DELETE_ONE;
      break;

    case 9:
        NRF_LOG_INFO("Option 9\n");
        currentMenu = MAINMENU;
        sendNUS = true;
        break;

    default:
       NRF_LOG_INFO("Unknown option: %d\n", option);
    }
NRF_LOG_ERROR("CurrentMenu: %d, sendNUS: %d\n", currentMenu, sendNUS);
}
/**@snippet [navigate Transfer Data menu]*/



void transferData(char * fileCmd){
  int Cmd = 0;

  transferDataFlag = false;
  

  Cmd = atoi(fileCmd);
  uint8_t data[] = "";
  if(Cmd == 0){
    nus_send("Cancelled", sizeof("Cancelled"));
   
    }else{
    switch(fileOption){
      case TRANSFER_ALL:
        if(Cmd == 1) currentMenu = TRANSFER_ALL_FILES;
        else{ nus_send("Cancelled", sizeof("Cancelled")); }
        break;
      case TRANSFER_ONE:
        if(Cmd > 0 && Cmd <= missionLog.file.latestLogFile){
        missionLog.file.filenr = Cmd;
        currentMenu = TRANSFER_ONE_FILE;
        }else {nus_send("Cancelled", sizeof("Cancelled")); }
        break;
      case DELETE_ALL:
        if(Cmd == 1) currentMenu = DELETE_ALL_FILES;
        else { nus_send("Cancelled", sizeof("Cancelled")); }
        break;
      case DELETE_ONE:
        if(Cmd > 0 && Cmd <= missionLog.file.latestLogFile){
        missionLog.file.filenr = Cmd;
        currentMenu = DELETE_ONE_FILE;
        }else {nus_send("Cancelled", sizeof("Cancelled")); }
        break;
      default: NRF_LOG_INFO("TransferData, unknown option: %d",Cmd);
    }
  }
  sendNUS = true;
}
/**@snippet [handle file operation menu]*/


void printMainMenu(){
 
    uint8_t mainMenu[167] = "*****-- Main Menu -- *****\r#Number\tOption\r#1\tSet Mission Data\r#2\tConfigure Vehicle\r#3\tData Files\r#4\tStart Mission\r#5\tGo to Idle\r#6\tGo to Configure\r#9\tReprint Main Menu";
    nus_send(mainMenu, sizeof(mainMenu));
}
/**@snippet [print main menu over BLE to client]*/


void printMissionDataMenu(){

    uint8_t data0[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t data1[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t data2[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t data3[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t missionMenu[] = {0};
    
    uint16_t len0=0, len1=0, len2=0, len3 = 0;
    len0 = sprintf(data0,"*****-- Mission Data --*****\r#Number\tOption\t\tValue\r#1\tMission1 Depth\t%.3f\r",mission.missionNr[0].depth);
    len1 = sprintf(data1,"#2\tMission1 Time\t%d\r#3\tMission2 Depth\t%.3f\r#4\tMission2 Time\t%d\r",mission.missionNr[0].time,mission.missionNr[1].depth,mission.missionNr[1].time);
    len2 = sprintf(data2,"#5\tMission3 Depth\t%.3f\r#6\tMission3 Time\t%d\r#7\tMission4 Depth\t%.3f\r",mission.missionNr[2].depth,mission.missionNr[2].time,mission.missionNr[3].depth);
    len3 = sprintf(data3,"#8\tMission4 Time\t%d\r#9\tMain Menu",mission.missionNr[3].time);

    nus_send(data0, len0);
    nus_send(data1, len1);
    nus_send(data2, len2);
    nus_send(data3, len3);
}
/**@snippet [print Mission Data menu over BLE to client]*/


void printConfigVehicleMenu(){

    uint8_t data0[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t data1[BLE_NUS_MAX_DATA_LEN] = "";
    uint8_t data2[BLE_NUS_MAX_DATA_LEN] = "";
    uint16_t len0=0, len1=0, len2=0;

    len0 = sprintf(data0, "*****-- Configure Vehicle -- *****\r#Number\tOption\tValue\r#1\tP\t\t%.6f\r", mission.pidData.kp);
    len1 = sprintf(data1, "#2\tI\t\t%.6f\r#3\tD\t\t%.6f\r#4\tKi Threshold\t%.6f\r", mission.pidData.ki,mission.pidData.kd,mission.pidData.kiThreshold);
    len2 = sprintf(data2, "#5\tAtmospheric Pressure\t%.6f\r#9\tMainMenu", mission.pidData.atmosphericPressure);

   nus_send(data0, len0);
   nus_send(data1, len1);
   nus_send(data2, len2);
}
/**@snippet [print Tune PID menu over BLE to client]*/


void printTransferDataMenu(){

    uint8_t data0[300] = "";
    uint8_t logContent[3000] = {0};
    uint16_t len0=0, len1=0, len2=0, logLength = 0;

    printMissionLogContent(logContent, &logLength);
    NRF_LOG_ERROR("logContent: %s, logLength: %d",logContent, logLength);

    nus_send(logContent,logLength);

    len0 = sprintf(data0, "Number of Log files: %d\r\n***** -- Data Files -- *****\r#Number\tOption\r#1\tTransfer ALL files\r#2\tTransfer ONE file\r#3\tDelete ALL files\r#4\tDelete ONE file\r#9\tMain Menu\r",missionLog.file.nrOfLogfiles);
    NRF_LOG_ERROR("len0: %d",len0);

    nus_send(data0, len0);
}
/**@snippet [print Trasnfer Data menu over BLE to client]*/


void transferAllFiles(){
    
    printf("Transfering all files .......");

    uint32_t fileSize = 0, totalSize = 0, index = 0, bytesRead = 0, length = 0;
    int startTime = 0, stopTime = 0, elapsedTime = 0;
    float bytesPerSec = 0.0, transferTime = 0.0, totalTime = 0.0;
    uint32_t queue[200];
    char filename[] = {0};
    uint8_t buffer[1] = {0};
    uint8_t data_array[200] = {0};
    uint8_t data[30] = "Transfering all files.......";
    nus_send(data, sizeof(data));
    
    queueLogFilesforTransfer(queue, &totalSize);
    NRF_LOG_ERROR("totalSize: %d",totalSize);

    
  for(int i = 0; i < missionLog.file.nrOfLogfiles; i++)
  {
    startTime = app_timer_cnt_get();

    if(queue[i] == 0)
      i++;

    int len = sprintf(filename, "/%d.txt",queue[i]);
    NRF_LOG_ERROR("queue: %d, Sending %s, len: %d",queue[i], filename, len);

    fileSize = openFileToRead(filename);
    NRF_LOG_ERROR("filename: %s, filesize: %d",filename, fileSize);
    nus_send(filename, len);


    while(index < fileSize && BLEconnected()){
      startTime = app_timer_cnt_get();
      readFromOpenFile(buffer, 1, &bytesRead);
      if(buffer[0] != NULL){
      data_array[length] = buffer[0];
      length += bytesRead;
      }else{ NRF_LOG_INFO("buffer[0]: -%s- = NULL",buffer[0]); }
     
 
      if(buffer[0] == '\n'){
          nus_send(data_array,length);
          length = 0;
          memset(&data_array, 0, sizeof(data_array));
       }
       index++;
       if(app_timer_cnt_get() >= startTime)    /**<Check for and handle RTC wrap around*/
        elapsedTime += app_timer_cnt_get() - startTime;
       else
        elapsedTime += app_timer_cnt_get()+RTC_MAX_COUNT - startTime;
    }closeFile();
    
    nus_send("##",sizeof("##"));
    
    index = 0;

   
    transferTime = elapsedTime/32768.0;
    bytesPerSec = fileSize/(transferTime);
    totalTime += transferTime;
    elapsedTime = 0;

    len = sprintf(data_array, "Filename: %s, Size: %d bytes, TransferTime: %.2f sec, bytes per Second: %.2f Byte/sec",filename, fileSize, transferTime, bytesPerSec);
    nus_send(data_array, len);
    memset(&data_array, 0, sizeof(data_array));
    memset(&filename, 0, sizeof(filename));
  }
    
    bytesPerSec = totalSize/(totalTime);
    length = sprintf(data_array, "Transmission Finished, Total size: %d bytes, Total Time: %.2f sec, Average bytes per Second: %.2f Byte/sec", totalSize, totalTime, bytesPerSec);
    nus_send(data_array, (uint16_t)length);


    transferDataFlag = false;
    currentMenu = MAINMENU;
    sendNUS = true;
}
/**@snippet [Transfer all mission log files from SD card over BLE to client]*/


void TransferOneFile(void){
    printf("Transfering one file.....");

    uint32_t fileSize = 0, index = 0, bytesRead = 0, length = 0;
    int startTime = 0, stopTime = 0, elapsedTime = 0;
    float bytePerSec = 0.0, transferTime = 0.0, totalTime = 0.0;
    uint8_t filename[] = "";
    uint8_t buffer[1] = {0};
    uint8_t data_array[200] = {0};
    uint8_t data[] = "Transfering one file.....";
    nus_send(data, sizeof(data));
    
    uint16_t len = sprintf(filename, "/%d.txt",missionLog.file.filenr);
    fileSize = openFileToRead(filename);
    NRF_LOG_ERROR("filename: %s, filesize: %d",filename, fileSize);
    nus_send(filename, len);
  
    
    while(index < fileSize && BLEconnected()){
      startTime = app_timer_cnt_get();
      readFromOpenFile(buffer, 1, &bytesRead);
      if(buffer[0] != NULL){
      data_array[length] = buffer[0];
      length += bytesRead;
      }else{ NRF_LOG_INFO("buffer[0]: -%s- = NULL",buffer[0]); }
  
      if(buffer[0] == '\n'){
        //if(length >= BLE_NUS_MAX_DATA_LEN){
          nus_send(data_array,length);
          length = 0;
          memset(&data_array, 0, sizeof(data_array));
       }
       index++;
       if(app_timer_cnt_get() >= startTime)      /**<Check for and handle RTC wrap around*/
        elapsedTime += app_timer_cnt_get() - startTime;
       else
        elapsedTime += app_timer_cnt_get()+RTC_MAX_COUNT - startTime;
    }closeFile();
    
    nus_send("##",sizeof("##"));
    

         
    transferTime = elapsedTime/32768.0;     /**< Elapsed time in seconds*/
    bytePerSec = fileSize/(transferTime);
    len = sprintf(data_array, "Filename: %s, Size: %d bytes, TransferTime: %.2f sec, bytes per Second: %.2f B/sec",filename, fileSize, transferTime, bytePerSec);
    nus_send(data_array, len);
    
    transferDataFlag = false;
    currentMenu = MAINMENU;
    sendNUS = true;
}
/**@snippet [Transfer one mission log files from SD card over BLE to client]*/


void deleteFile(void){
    
    printf("Deleting file: %d.txt .......",missionLog.file.filenr);
    uint8_t data[29] = {0};
    
    uint16_t length = sprintf(data,"Deleting file: %d.txt .......",missionLog.file.filenr);
    nus_send(data, length);

    uint8_t returnMsg[200] = {0};
    uint16_t returnMsgLength = 0;
    deleteLogFile(missionLog.file.filenr, returnMsg, &returnMsgLength);
    missionLog.file.nrOfLogfiles = countMissionLog();

    nus_send(returnMsg,returnMsgLength);
    
    transferDataFlag = false;
    currentMenu = MAINMENU;
    sendNUS = true;
}
/**@snippet [delete one mission log files from SD card]*/


void deleteAllFiles(void){

    printf("Deleting all files .......");
    uint8_t data[] = "Deleting all files .......";
    nus_send(data, sizeof(data));

    uint8_t returnMsg[3000] = {0};
    uint16_t returnMsgLength = 0;
    deleteAllLogFiles(&returnMsgLength, returnMsg);
    missionLog.file.nrOfLogfiles = countMissionLog();

    NRF_LOG_ERROR("returnMsg: %s, returnMsgLength: %d",returnMsg, returnMsgLength);
    nus_send(returnMsg,returnMsgLength);
    
    transferDataFlag = false;
    currentMenu = MAINMENU;
    sendNUS = true;
}
/**@snippet [delete all mission log files from SD card]*/


/** @} */
