
#include "SDcard.h"

/** @file
 *
 * @defgroup SDcard SD card program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain SD card module with related structures and functions.
 */

  static FATFS filesystem;
  static DIR directory;
  static FILINFO fileInfo;
  static FIL file;

  extern missionLog_t missionLog;

  FRESULT f_err_code;
  DSTATUS disk_state = STA_NOINIT;  // Not initialized



  /**@brief Initialize FATFS disk I/O interface by providing the block device.
  */
  static diskio_blkdev_t drives[] = 
  {
      DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
  };



void SDcardInit(){

  uint32_t fileNr = 0, cnt = 0;
  diskio_blockdev_register(drives, ARRAY_SIZE(drives));

      NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
        NRF_LOG_INFO("Disk initialization failed.");
        return;
    }
    uint32_t blocks_per_mb = (1024uL * 1024uL) / m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_size;
    uint32_t capacity = m_block_dev_sdc.block_dev.p_ops->geometry(&m_block_dev_sdc.block_dev)->blk_count / blocks_per_mb;
    NRF_LOG_INFO("Capacity: %d MB", capacity);

    NRF_LOG_INFO("Mounting volume...");
    f_err_code = f_mount(&filesystem, "", 1);
    if (f_err_code)
    {
        NRF_LOG_INFO("Mount failed.");
        return;
    }

    NRF_LOG_INFO("\r\n Listing directory: /");
    f_err_code = f_opendir(&directory, "/");
    if (f_err_code)
    {
        NRF_LOG_INFO("Directory listing failed!");
        return;
    }

    do
    {
        f_err_code = f_readdir(&directory, &fileInfo);
        if (f_err_code != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }

        if (fileInfo.fname[0])
        {
            if (fileInfo.fattrib & AM_DIR)  // If directory
            {
                NRF_LOG_RAW_INFO("   <DIR>   %s",(uint32_t)fileInfo.fname);
            }
            else
            {
                NRF_LOG_RAW_INFO("%9lu  %s", fileInfo.fsize, (uint32_t)fileInfo.fname);
                fileNr = atoi(strtok(fileInfo.fname,".TXT"));
                if(fileNr > 0){
                  if(fileNr > missionLog.file.latestLogFile)  
                    missionLog.file.latestLogFile = fileNr;   /**<Highest integer number is the latest created Log file */
                  cnt++;
                }
            }
        }
    }
    while (fileInfo.fname[0]);
    missionLog.file.nrOfLogfiles = cnt;
    NRF_LOG_RAW_INFO("");

}
/**@snippet [Initialize SD card]*/


void openMissionLogDirectory(){

  f_err_code = f_opendir(&directory, "/");
   if (f_err_code)
    {
        NRF_LOG_INFO("Failed to open directory! f_err_code: %x",f_err_code);
        return;
    }
}
/**@snippet [Open missionLog dir and count]*/


uint32_t findLatestMissionLog(){
       uint32_t fileNr = 0, latestLogFile = 0, Logcnt = 0;
      do{
        f_err_code = f_readdir(&directory, &fileInfo);
        if (f_err_code != FR_OK)
        {
            NRF_LOG_ERROR("Directory read failed. f_err_code: %x",f_err_code);
        }

        if (fileInfo.fname[0])  // if filename != NULL
        {
            if (!(fileInfo.fattrib & AM_DIR))  // If != directory
            {
                NRF_LOG_RAW_INFO("%9lu  %s", fileInfo.fsize, (uint32_t)fileInfo.fname);
                fileNr = atoi(strtok(fileInfo.fname,".TXT"));
                Logcnt++;
                if(fileNr > latestLogFile)
                  latestLogFile = fileNr;     /**<Highest integer number is the latest created Log file */
            }
        }
    }
    while (fileInfo.fname[0]);
    missionLog.file.nrOfLogfiles = Logcnt;
    return latestLogFile;
}
/**@snippet [find the latest created log file]*/


void createMissionLog()
{  
    uint32_t bytes_written;
    char text[] = "Time[ms],#Mission,TargetDepth[m],PistonPosition[m],PIDoutput,Pressure[V],MeasuredDepth[m],Psi,Pascal,Battery[V],Temperature,MotionData\r\n";

    openMissionLogDirectory();                             /**<Open missionLog directory */
    uint32_t name = findLatestMissionLog();               /**<and count the number of Log files.*/
    name++;
    sprintf(missionLog.file.filename, "%d.txt",name);     /**<Create a new Log filename as one integer value higher than the previous Log file.*/
    NRF_LOG_RAW_INFO("New filename: %s\n\r",missionLog.file.filename);
    NRF_LOG_RAW_INFO("");
    missionLog.file.latestLogFile = name;                  /**<update latest create log file */
    missionLog.file.nrOfLogfiles++;                        /**<Number of log files increased by one */

  FRESULT f_err_code = f_open(&file, missionLog.file.filename, FA_WRITE | FA_OPEN_APPEND);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Unable to open (to write) or create file: %s.", missionLog.file.filename);
      NRF_LOG_ERROR("Create missionLog, f_open error: %x",f_err_code);
      return;
  }
  f_err_code = f_write(&file, text, sizeof(text)-1, &bytes_written);
  if (f_err_code != FR_OK){ NRF_LOG_INFO("Write failed\r\n.");  }
  else{ NRF_LOG_INFO("bytes written %d.",bytes_written); }
}
/**@snippet [Create log file]*/


void printMissionLogContent(uint8_t returnMsg[], uint16_t * returnLength){
  FRESULT f_err_code = 0;
  uint8_t filename[10] = {0};
  uint8_t buffer[3000] = "Size[Byte], Name\n";
  uint32_t fileNr = 0, nrOfLogFiles = 0;
  uint16_t length = 0;

    openMissionLogDirectory();
    do
    {  
      f_err_code = f_readdir(&directory, &fileInfo);
      if (f_err_code != FR_OK)
      {
          NRF_LOG_INFO("Directory read failed.");
      }
      else{
          strcpy(filename, fileInfo.fname);
          fileNr = atoi(strtok(filename,".txt"));
        if(fileNr > 0){
      
            length = sprintf(buffer,"%s%9lu,  %s\r",buffer, fileInfo.fsize, (uint32_t)fileInfo.fname);
        }
      }
    }while (fileInfo.fname[0]);
    strcpy(returnMsg, buffer);
    *returnLength = length;
    NRF_LOG_RAW_INFO("%sMsglength: %d\n",buffer, length );
}
/**@snippet [print content of mission Log]*/


uint32_t openFileToRead(uint8_t filename[]){
  FRESULT f_err_code = f_open(&file, filename, FA_READ);
  if(f_err_code > 0) NRF_LOG_ERROR("f_open err_code: %d",f_err_code);
  return (uint32_t)file.obj.objsize;
}
/**@snippet [Open file with READ permission]*/


void lseek(uint32_t index){
  FRESULT f_err_code = f_lseek(&file, (unsigned long)index);
  if(f_err_code > 0) NRF_LOG_ERROR("lseek, f_err_code: %x",f_err_code);
}
/**@snippet [go to a position in file]*/


void writeToOpenFile(void* text, uint8_t length, uint32_t * bytes_written){

  f_err_code = f_write(&file, text, length, bytes_written);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Write failed: f_err_code: %d\r\n.",f_err_code);
  }
}
/**@snippet [Write to an already opened file]*/



void readFromOpenFile(void* text, uint8_t length, uint32_t * bytes_read){

  f_err_code = f_read(&file, text, length, bytes_read);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Read failed\r\n.");
  }
  else
  {
      //NRF_LOG_INFO("%d bytes read.", *bytes_read);
  }
}
/**@snippet [Read to an already opened file]*/



void closeFile(){
  (void) f_close(&file);
}
/**@snippet [close an open file]*/


void writeFile(uint8_t filename[], uint8_t text[], uint8_t length, uint32_t * bytes_written)
{
      NRF_LOG_INFO("writeFile - filename: %s. Text: %s", filename, text);

  FRESULT f_err_code = f_open(&file, filename, FA_READ | FA_WRITE | FA_OPEN_APPEND);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Unable to open (to write) or create file: %s.", filename);
      NRF_LOG_ERROR("writeFile, f_open: %x",f_err_code);
      return;
  }
  f_err_code = f_write(&file, text, length, &bytes_written);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Write failed\r\n.");
  }
  else
  {
      NRF_LOG_INFO("%d bytes written.", *bytes_written);
  }
  (void) f_close(&file);
}
/**@snippet [Write to file]*/



void readFile(uint8_t filename[], uint8_t text[], uint8_t length, uint8_t * p_nrBytesRead){

      NRF_LOG_INFO("readFile - filename: %s. Text: %s. length: %d", filename, text, length);


  FRESULT f_err_code = f_open(&file, filename, FA_READ);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Unable to open (to read) or create file: %s.", filename);
      NRF_LOG_ERROR("readFile, f_open: %x",f_err_code);
      return;
  }
  f_err_code = f_read(&file, text, length, p_nrBytesRead);
  if (f_err_code != FR_OK)
  {
      NRF_LOG_INFO("Read failed\r\n.");
  }
  else
  {
      NRF_LOG_INFO("%d bytes read.", *p_nrBytesRead);
  }
  (void) f_close(&file);
}
/**@snippet [Read file]*/




void unMount(){
  
    FRESULT f_err_code = f_mount(NULL,"",0);
    if (f_err_code)
    {
        NRF_LOG_INFO("unmount failed. Error code: %d", f_err_code);
        return;
    }
    else { NRF_LOG_INFO("SDcard Unmounted"); }
}
/**@snippet [Unmount SD card]*/



void writeMissionLog(){
  uint32_t bytes_written, n = 0;
  uint8_t buffer[3000] = {0};
  n += sprintf(&buffer[n],"%.2f,",missionLog.timeStamp);
  n += sprintf(&buffer[n],"%d,",missionLog.missionNr);
  n += sprintf(&buffer[n],"%f,",missionLog.setpoint);
  n += sprintf(&buffer[n],"%f,",missionLog.pistonPosition);
  n += sprintf(&buffer[n],"%f,",missionLog.PIDoutput);
  n += sprintf(&buffer[n],"%f,",missionLog.pressureVoltage);
  n += sprintf(&buffer[n],"%f,",missionLog.pressureDepth);
  n += sprintf(&buffer[n],"%f,",missionLog.pressurePsi);
  n += sprintf(&buffer[n],"%f,",missionLog.pressurePascal);
  n += sprintf(&buffer[n],"%f,",missionLog.batteryVoltage);
  n += sprintf(&buffer[n],"%f,",missionLog.temperature);
  n += sprintf(&buffer[n],"%f\r\n",missionLog.motionData);

  f_err_code = f_write(&file, buffer, n, &bytes_written);
  if (f_err_code != FR_OK) { NRF_LOG_INFO("Write failed - Code: %d\r\n.",f_err_code);}
  else                     { /*NRF_LOG_INFO("%d bytes written.", bytes_written);*/ }
  
  NRF_LOG_FLUSH();
  f_sync(&file);
}
/**@snippet [write the mission log]*/



void deleteLogFile(uint32_t filename, uint8_t returnMsg[], uint16_t * returnLength){

  TCHAR filepath[20] = {0};
  uint8_t buffer[200] = {0};
  uint16_t length = 0;

    sprintf(filepath, "%d.TXT",filename);
    FRESULT f_err_code = f_unlink(filepath);
    if(f_err_code != FR_OK)
    {
        length = sprintf(buffer, "Failed to delete file: %s, FatFS error_code: 0x%x\n",filepath, f_err_code);
        NRF_LOG_ERROR("Failed to delete file: %s - f_err_code: 0x%x",filepath, f_err_code);
    }
    else
      {
      length = sprintf(buffer, "Successfully deleted %s\n",(uint32_t)filepath);
      NRF_LOG_INFO("Successfully deleted %s:- f_err_code: %x",(uint32_t)filepath, f_err_code); 
      }
    strcpy(returnMsg, buffer);
   *returnLength = length;
}
/**@snippet [Delete one log file]*/


void deleteAllLogFiles(uint16_t * returnLength, uint8_t returnMsg[]){

  FRESULT f_err_code = 0;
  TCHAR temp[20] = {0};
  uint8_t buffer[3000] = {0};
  uint32_t fileNr = 0, length = 0, nrDeleted = 0, nrFailed = 0;

    openMissionLogDirectory();
    do
    {  
        f_err_code = f_readdir(&directory, &fileInfo);
        if (f_err_code != FR_OK)
        {
            NRF_LOG_INFO("Directory read failed.");
            return;
        }
        strcpy(temp,fileInfo.fname);
        fileNr = atoi(strtok(temp,".txt"));
        if(fileNr > 0){
             NRF_LOG_RAW_INFO("%s\n", (uint32_t)fileInfo.fname);
            f_err_code = f_unlink(fileInfo.fname);
            if(f_err_code != FR_OK)
              {
                NRF_LOG_INFO("Failed to delete %s",(uint32_t)fileInfo.fname);
                length = sprintf(buffer,"%sFailed to delete %s, FatFS error_code: 0x%x\n",buffer, (uint32_t)fileInfo.fname, f_err_code);
                nrFailed++;
              }
              else{ 
                    NRF_LOG_INFO("Successfully deleted %s",(uint32_t)fileInfo.fname);
                    nrDeleted++;
                  }
        }
    }
    while (fileInfo.fname[0]);
    
    length = sprintf(buffer,"%s\n%d files deleted, %d Failed",buffer, nrDeleted, nrFailed);
    *returnLength = (uint16_t)length;
    strcpy(returnMsg,buffer);
}
/**@snippet [Delete all log files]*/



uint32_t countMissionLog(){

FRESULT f_err_code = 0;
  TCHAR temp[20] = {0};
  uint32_t fileNr = 0, cnt = 0;

  openMissionLogDirectory();
  do
  {  
      f_err_code = f_readdir(&directory, &fileInfo);
      if (f_err_code != FR_OK)
      {
          NRF_LOG_INFO("Directory read failed.");
          return;
      }
      strcpy(temp,fileInfo.fname);
      fileNr = atoi(strtok(temp,".txt"));
      if(fileNr > 0){
          cnt++;
          if(fileNr > missionLog.file.latestLogFile)  /**<Highest integer number is the latest created Log file */
             missionLog.file.latestLogFile = fileNr;
      }
  }while (fileInfo.fname[0]);
  
  return cnt;
}
/**@snippet [Count mission log files]*/


void queueLogFilesforTransfer(uint32_t queue[], uint32_t * totalSize){

  FRESULT f_err_code = 0;
  uint32_t qsize = missionLog.file.nrOfLogfiles;
  static uint32_t queueBuf[200];
  uint8_t filename[10] = "";
  uint32_t fileNr = 0, size = 0, temp = 0;
  uint16_t index = 0;

    openMissionLogDirectory();
    do
    {  
      f_err_code = f_readdir(&directory, &fileInfo);
      if (f_err_code != FR_OK)
      {
          NRF_LOG_INFO("Directory read failed.");
      }
      else{
          strcpy(filename, fileInfo.fname);
          fileNr = atoi(strtok(filename,".txt"));
        if(fileNr > 0){
          NRF_LOG_ERROR("fileNr: %d", fileNr);
          queue[index] = fileNr;
           index++;
           size += fileInfo.fsize;
        }
      }
    }while (fileInfo.fname[0]); 
      *totalSize = size;
    NRF_LOG_RAW_INFO("Total size: %d\n",*totalSize);
    //queueBuf[missionLog.file.nrOfLogfiles] = totalSize;

    //memcpy(queue, queueBuf, qsize );
}
/**@snippet [Create transfering queue]*/


/** @} */
