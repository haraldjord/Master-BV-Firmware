/*
  Created: spring 2020
    Author: Halsos

  Edited: spring 2023 
    Author: Jordalen
*/

#include "motor.h"
#include "app_uart.h"


/** @file
 *
 * @defgroup motor motor program file
 * @{
 * @ingroup Drivers
 *
 * @brief Contain motor module with related structures and functions.
 */

rxMotor_t rxMotor = {0}; /**< Reply message instance*/
motorSettings_t motorSettings; /**< Motor settings instance*/
extern mission_t mission;

void motorUp(){ // UP towards Surface
   rxMotor.msgReceived = false;
    sendCmd(1,1,0,0,1000);
   while(!rxMotor.msgReceived);
   rxMotor.msgReceived = false;
       NRF_LOG_INFO("motorUp");
}
/**@snippet [Vehicle float up] */


void motorDown(){ // Dive deeper
   rxMotor.msgReceived = false;
    sendCmd(1,2,0,0,1000);
   while(!rxMotor.msgReceived);
   rxMotor.msgReceived = false;
    NRF_LOG_INFO("motorDown");
}
/**@snippet [Vehicle dive deeper] */



void motorStop(){
    rxMotor.msgReceived = false;
    sendCmd(1,3,0,0,0);
    NRF_LOG_INFO("motorStop");
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;
}
/**@snippet [stop motor] */




// Enable right limit switch
void enablePistonBottomLimit(){
  sendCmd(1,5,12,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
}

//Enable left limit switch
void enablePistonTopLimit(){
  sendCmd(1,5,13,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
}

void sendCmd(uint8_t addr, uint8_t Cmd, uint8_t Type, uint8_t motor, long value){

  uint8_t txBuffer[9] = {0};

  txBuffer[0] = addr;
  txBuffer[1] = Cmd;
  txBuffer[2] = Type;
  txBuffer[3] = motor;
  txBuffer[4] = value >> 24;
  txBuffer[5] = value >> 16;
  txBuffer[6] = value >> 8;
  txBuffer[7] = value & 0xff;
  txBuffer[8] = 0;        // CRC calculated and added below
  
  for(int i = 0; i < 8; i++)    // Calculate CRC
      txBuffer[8] += txBuffer[i];
  for(int i = 0; i < 9; i++)    // Send message
    app_uart_put( txBuffer[i]);
}
/**@snippet [Send command to motor] */




void receiveReply(uint8_t motorReply[]){
  rxMotor.address = motorReply[0];
  rxMotor.Id      = motorReply[1];
  rxMotor.status  = motorReply[2];
  rxMotor.value   = (motorReply[4] << 24) | (motorReply[5] << 16) | (motorReply[6] << 8) | motorReply[7];

  switch(rxMotor.status){
    case(SUCCESS): /*NRF_LOG_INFO("TMCL: Successfully executed, no error");*/ break;
    case(LOADED_IN_EEPROM):  NRF_LOG_ERROR("TMCL: Command loaded into TMCL program EEPROM"); break;
    case(WRONG_CHECKSUM): NRF_LOG_ERROR("TMCL: Wrong checksum"); break;
    case(INVALID_COMMAND): NRF_LOG_ERROR("TMCL: Invalid command"); break;
    case(WRONG_TYPE): NRF_LOG_ERROR("TMCL: Wrong type"); break;
    case(INVALID_VALUE): NRF_LOG_ERROR("TMCL: Invalid value"); break;
    case(EEPROM_LOCKED): NRF_LOG_ERROR("TMCL: Configuration EEPROM locked"); break;
    case(CMD_UNAVAILABLE): NRF_LOG_ERROR("TMCL: Command not available"); break;
    default: NRF_LOG_ERROR("Unknown status: %d",rxMotor.status);
  }
  rxMotor.msgReceived = true;
}
/**@snippet [Receive reply message] */



void setPistonPosition(){
  rxMotor.msgReceived = false;
  float PIDoutput = mission.pidData.output;
  
  if(PIDoutput < 0.000) PIDoutput = 0.000; // Already done in PID controler file.
  else if(PIDoutput > 0.055) PIDoutput = 0.055;
  long newPistonPosition = -(PIDoutput*motorSettings.microStep*motorSettings.fullStep*1000);// Position in SI units [meters]     ;51200000.0);
  sendCmd(1,4,0,0,newPistonPosition);
   while(!rxMotor.msgReceived);
   rxMotor.msgReceived = false;

  printf("PistonPos: %f,\n\rPIDout: %f,\n\rnewPistonPos: %ld\n\r",mission.pidData.pistonPosition , PIDoutput, newPistonPosition);
}
/**@snippet [Move piston to position]*/



void setReferencePositionToZero(){
  rxMotor.msgReceived = false;
  
  sendCmd(1,5,1,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
}
/**@snippet [Set position counter to zero] */


float getPistonPosition(){
  rxMotor.msgReceived = false;
  sendCmd(1,6,1,0,0);

  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
  float value = -(rxMotor.value/51200000.0);
  return value;
}
/**@snippet [Get piston position] */


float getMotorSpeed(){
  rxMotor.msgReceived = false;
  sendCmd(1,6,3,0,0);

  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
  float value = rxMotor.value;
  return value;
}
/**@snippet [Get motor speed]*/

  


void motorEnableLimitSwitches(){
  enablePistonBottomLimit();
  enablePistonTopLimit();
}
/**@snippet [Enable limit switches] */


void MotorTest(){

    motorEnableLimitSwitches();
    motorDown();
    nrf_delay_ms(2000);
    motorStop();
    nrf_delay_ms(2000);
    motorUp();
    nrf_delay_ms(2000);
    motorStop();
    NRF_LOG_INFO("Motor test finished");
}
/**@snippet [Test motor]*/


void motorInit(){

    /*See dataSheet TMCM 1161 firmware manual chapter 6 for details regarding velocity in rounds per second and internal speed calculation*/

    motorSettings.pulse_divisor = 1;      // change this variable for setting pulse divider
    motorSettings.microStepSettings = 8; // change this parameter for settings micro step 
    motorSettings.microStep = pow(2, motorSettings.microStepSettings); 
    motorSettings.fullStep = 200;
    motorSettings.v_rps = 2.44;           // change this parameter for setting linear velocity [mm/s] 
    motorSettings.v_pps = motorSettings.v_rps*motorSettings.fullStep*motorSettings.microStep;                     
    motorSettings.v_int = motorSettings.v_pps*pow(2, motorSettings.pulse_divisor)*2048*32/(16*pow(10,6));   
    motorSettings.maxVelocity = motorSettings.v_int;          // internal velocity same as position velocity.            

    motorSettings.StandbyCurrent = 0;         
    motorSettings.maxCurrent = 66;                                                    

    if (motorSettings.v_int > 2047){
        NRF_LOG_ERROR("Internal velocity of motor exceeds physical limmit. Decrease micro step settings and restart controller with new parameters!");
        while(1){} // restart firmware to avoid damage on stepper motor
    }
    else if (motorSettings.v_int < 0){
        NRF_LOG_ERROR("Internal velocity of motor less then zero. Restart controller with new parameters!");
        while(1){} // restart firmware to avoid damage on stepper motor
    }


    rxMotor.msgReceived = false;
    motorEnableLimitSwitches();

    sendCmd(1,5,6,0,motorSettings.maxCurrent);  /**< Set max current -max motor power- (0-255). */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;
    
    sendCmd(1,5,7,0,motorSettings.StandbyCurrent);  /**< Set standby current (0-255) * [4/255 A]. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;

    sendCmd(1,5,214,0,100); /**< Set power-down delay to enter standby current (1-65535) * [10 msec]. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;

    sendCmd(1,5,204,0,500);  /**< Set time for which the motor current is cut off after velocity reaches zero (0 - 65535) [10 msec] 0 means never. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;
    

    /*Set motor speed parameters.*/
    sendCmd(1,5,154,0,motorSettings.pulse_divisor); /**< Set pulse divisor to 1 - make sure to not exeed physical limits. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;

    sendCmd(1,5,140,0,motorSettings.microStepSettings);   /**< Set micro step to 2^n*/
    while (!rxMotor.msgReceived);
    rxMotor.msgReceived = false;

    sendCmd(1,5,4,0,motorSettings.v_int);       /**< Set max positioning speed (integer) [1 - 2047]. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;

    sendCmd(1,5,2,0,motorSettings.maxVelocity); /**< Set max speed in velocity mode (integer) [1 - 2047]. */
    while(!rxMotor.msgReceived);
    rxMotor.msgReceived = false;
  
    setSurfaceReferencePoint();
}
/**@snippet [Initialize and configure motor]*/


float getAxisParameterValue(uint8_t parameter){
rxMotor.msgReceived = false;
  sendCmd(1,6,parameter,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
  return (float)rxMotor.value;
}
/**@snippet [Read axis paramter] */


float getMotorEEPROMValue(uint8_t parameter){
rxMotor.msgReceived = false;
  sendCmd(1,8,parameter,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
  return (float)rxMotor.value;
}
/**@snippet [Read EEPROM value] */



float getMotorFirmwareVersion(){
rxMotor.msgReceived = false;
  sendCmd(1,136,0,0,0);
  while(!rxMotor.msgReceived);
  rxMotor.msgReceived = false;
  return (float)rxMotor.value;
}
/**@snippet [Read Firmware version]*/

 void stopMotorAtSurface(){  

    while(!g_bottomLimit);
    g_bottomLimit = false;
    motorStop();
    g_motorStopped = true;
}
/**@snippet [Stop motor at surface]*/

 void setSurfaceReferencePoint(){ 
    motorDown(); /**< To clear limit switch before re-trigger */
    nrf_delay_ms(1000);
    g_bottomLimit = false;
    motorUp();
    stopMotorAtSurface();
    setReferencePositionToZero();
}
/**@snippet [Set Surface Reference Point]*/


void testPositionMovement(float toPos){
  mission.pidData.output = toPos;
  float atPos;
  atPos = getPistonPosition();
  printf("pos1: %.2f\n", atPos);
  setPistonPosition();
  nrf_delay_ms(10000);
  atPos = getPistonPosition();
  printf("pos2: %.2f\n", atPos);
  



}



/**@snippet [Set position velocity in mm/sec]*/


/** @} */
