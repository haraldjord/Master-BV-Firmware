#ifndef _SDCARD_H
#define _SDCARD_H

#include "main.h"

#include "bsp.h"
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

/** @file
 *
 * @defgroup SDcard SD card program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain SD card module with related structures and functions.
 */


// SDcard Data
#define FILE_NAME       "TEST.TXT"
#define TEST_STRING     "SD card test string."

#define SDC_CS_PIN      (22UL)  ///< SDCard chip select (CS) pin.
#define SDC_MOSI_PIN    (23UL)  ///< SDCard serial data in (DI) pin.
#define SDC_MISO_PIN    (24UL)  ///< SDCard serial data out (DO) pin.
#define SDC_SCK_PIN     (25UL)  ///< SDCard serial clock (SCK) pin.


/*
 * @brief  SDCard block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(SDC_MOSI_PIN, SDC_MISO_PIN, SDC_SCK_PIN, SDC_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("BV2020", "SDC", "1.00")
);


/**@brief Initialize SD card and mount disk
*/
void SDcardInit(void);

/**@brief Unmount SD card
*/
void unMount(void);

/**@brief open file and read from it.
*
* @param[int] filename string array contain the filename to read from
* @param[out] text array points to first element of string array in which the read text will be stored
* @param[in] length number of bytes to read
* @param[out] p_nrBytesRead pointer to address where the counted number of bytes read is stored
*
*/
void readFile(uint8_t*, uint8_t*, uint8_t, uint8_t *);

/**@brief open file and read from it.
*
* @param[int] filename string array contain the filename to write to
* @param[in] text string array to be written to file
* @param[in] length number of bytes to write
* @param[out] bytes_written pointer to address where the counted number of bytes written is stored
*
*/
void writeFile(uint8_t*, uint8_t*, uint8_t, uint32_t *);

/**@brief Create log file named one integer higher than the last log file
*/
void createMissionLog(void);

/**@brief Open missionLog directory, and count the number of missionLog files, named [integer].txt
*/
void openMissionLogDirectory(void);

/**@brief Open missionLog directory and find the highest integer name which is the last created log file - name[integer].txt
*/
uint32_t findLatestMissionLog(void);

/**@brief print content of mission Log directory, as a list of log files, to BLE menu.
*
* @param[out] returnMsg string array containing filename
* @param[out] returnLength pointer to which the length of returnMsg is stored
*
*/
void printMissionLogContent(uint8_t*, uint16_t*);

/**@brief Open a file with **read** permission
*
* @param[in] filename String array containing filename to open
*
*/
uint32_t openFileToRead(uint8_t*);

/**@brief find a specific line in an open file
*
* @param[in] index Line number to be accessed
*
*/
void lseek(uint32_t);

/**@brief write data to an already opened file
*
* @param[in] text text to be written to file
* @param[in] length Length of text string to be written
* @param[out] bytes_written return pointer to report how many bytes was written
*
*/
void writeToOpenFile(void*, uint8_t, uint32_t *);

/**@brief read data from an already opened file
*
* @param[out] text bytes read from file is stored in address pointed to by text
* @param[in] length Length of text string to be written
* @param[out] bytes_read return pointer to report how many bytes was read
*
*/
void readFromOpenFile(void*, uint8_t, uint32_t *);

/**@brief Close open file
*/
void closeFile(void);

/**@brief write the mission log structure to a mission log file
*
* @details Create a string array where each value to be written to file is stored. Then write the string to log file.
*/
void writeMissionLog(void);

/**@brief Delete one log file
*
* @details From menu a integer value can be typed in to delete a specific log file
*
* @param[in] filename integer value for the file to be deleted
* @param[out] returnMsg string array to report status after delete operation finished
* @param[out] returnLength pointer that points to address where length of returnMsg is stored
*
*/
void deleteLogFile(uint32_t, uint8_t*, uint16_t*);

/**@brief Delete one log file
*
* @details From menu a integer value can be typed in to delete a specific log file
*
* @param[out] returnLength pointer that points to address where length of returnMsg is stored
* @param[out] returnMsg string array to report status after delete operation finished
*/
void deleteAllLogFiles(uint16_t*, uint8_t*);


/**@brief count the number of log files stored on SD card
*
* @returns cnt Integer valued number of log files
*/
uint32_t countMissionLog(void);

/**@brief Create a queue for trasnfering all log files over BLE
*
* @details in order to transfer all log files a queue must be created,
*          initialy intended to sort the queue and transfer new files first though this step is skipped.
*
* @param[out] queue array holding the queue for file transmission
* @param[out] totalSize The total number og bytes that will be transfered is counted and stored in variable pointed to by totalSize pointer.
*
*/
void queueLogFilesforTransfer(uint32_t*, uint32_t *);

#endif

/** @} */
