#include "PWM.h"


/** @file
 *
 * @defgroup SAADC saadc program file
 * @{
 * @ingroup Drivvers
 *
 * @brief Contain SAADC module with related functions.
 */

 int16_t REDseq_buf[] = {(1 << 15) | 0};    /**< Channel 0: Inverse polarity (bit 15), 500us duty cycle. */
 int16_t GREENseq_buf[] = {(1 << 15) | 0};  /**< Channel 1: Inverse polarity (bit 15), 500us duty cycle. */
 int16_t BLUEseq_buf[] = {(1 << 15) | 1};   /**< Channel 2: Inverse polarity (bit 15), 500us duty cycle. */


void pwm_init(void)
{
     // PWM0 RED pin
    NRF_GPIO->DIRSET = (1 << OUTPUT_PIN0);
    NRF_GPIO->OUTCLR = (1 << OUTPUT_PIN0);
    
    // PWM1 GREEN pin
    NRF_GPIO->DIRSET = (1 << OUTPUT_PIN1);
    NRF_GPIO->OUTCLR = (1 << OUTPUT_PIN1);
    // PWM2 BLUE pin
    NRF_GPIO->DIRSET = (1 << OUTPUT_PIN2);
    NRF_GPIO->OUTCLR = (1 << OUTPUT_PIN2);


    // PWM0
    NRF_PWM0->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_16; // 1 us
    NRF_PWM0->PSEL.OUT[0] = OUTPUT_PIN0;
    NRF_PWM0->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->DECODER     = (PWM_DECODER_LOAD_Common       << PWM_DECODER_LOAD_Pos) | 
                            (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM0->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  
    NRF_PWM0->COUNTERTOP = 2000; // 2ms period
  
  
    NRF_PWM0->SEQ[0].CNT = ((sizeof(REDseq_buf) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[0].ENDDELAY = 0;
    NRF_PWM0->SEQ[0].PTR = (uint32_t)&REDseq_buf[0];
    NRF_PWM0->SEQ[0].REFRESH = 0;
    NRF_PWM0->SHORTS = 0;
  
    NRF_PWM0->ENABLE = 1;
    NRF_PWM0->TASKS_SEQSTART[0] = 1;



    // PWM1
    NRF_PWM1->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_16; // 1 us
    NRF_PWM1->PSEL.OUT[0] = OUTPUT_PIN1;
    NRF_PWM1->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM1->DECODER     = (PWM_DECODER_LOAD_Common       << PWM_DECODER_LOAD_Pos) | 
                            (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM1->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  
    NRF_PWM1->COUNTERTOP = 2000; // 2ms period
  
  
    NRF_PWM1->SEQ[0].CNT = ((sizeof(GREENseq_buf) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM1->SEQ[0].ENDDELAY = 0;
    NRF_PWM1->SEQ[0].PTR = (uint32_t)&GREENseq_buf[0];
    NRF_PWM1->SEQ[0].REFRESH = 0;
    NRF_PWM1->SHORTS = 0;
  
    NRF_PWM1->ENABLE = 1;
    NRF_PWM1->TASKS_SEQSTART[0] = 1;



    // PWM 2
    NRF_PWM2->PRESCALER   = PWM_PRESCALER_PRESCALER_DIV_16; // 1 us
    NRF_PWM2->PSEL.OUT[0] = OUTPUT_PIN2;
    NRF_PWM2->MODE        = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
    NRF_PWM2->DECODER     = (PWM_DECODER_LOAD_Common       << PWM_DECODER_LOAD_Pos) | 
                            (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
    NRF_PWM2->LOOP        = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  
    NRF_PWM2->COUNTERTOP = 2000; // 2ms period
  
  
    NRF_PWM2->SEQ[0].CNT = ((sizeof(BLUEseq_buf) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM2->SEQ[0].ENDDELAY = 0;
    NRF_PWM2->SEQ[0].PTR = (uint32_t)&BLUEseq_buf[0];
    NRF_PWM2->SEQ[0].REFRESH = 0;
    NRF_PWM2->SHORTS = 0;
  
    NRF_PWM2->ENABLE = 1;
    NRF_PWM2->TASKS_SEQSTART[0] = 1;
    
    updateLED(1, 1, 1); // Set minimum brightness
}
/** @snippet [Initialize PWM module]*/


void updateLED(uint16_t REDduty, uint16_t GREENduty, uint16_t BLUEduty)
{ 

  NRF_PWM0->TASKS_STOP = 1;
  NRF_PWM1->TASKS_STOP = 1;
  NRF_PWM2->TASKS_STOP = 1;

   // Multiply dutycycle by 20 to make percenetage to appropriate compare value
   uint16_t REDcompare = REDduty*20;
   uint16_t GREENcompare = GREENduty*20;
   uint16_t BLUEcompare = BLUEduty*20;

   REDseq_buf[0]   = ((1 << 15) | REDcompare);
   GREENseq_buf[0] = ((1 << 15) | GREENcompare);
   BLUEseq_buf[0]  = ((1 << 15) | BLUEcompare);

   NRF_PWM0->SEQ[0].CNT = 1;
   NRF_PWM1->SEQ[0].CNT = 1;
   NRF_PWM2->SEQ[0].CNT = 1;

   NRF_PWM0->SEQ[0].PTR = (uint32_t)&REDseq_buf[0];
   NRF_PWM1->SEQ[0].PTR = (uint32_t)&GREENseq_buf[0];
   NRF_PWM2->SEQ[0].PTR = (uint32_t)&BLUEseq_buf[0];

   NRF_PWM0->TASKS_SEQSTART[0] = 1;
   NRF_PWM1->TASKS_SEQSTART[0] = 1;
   NRF_PWM2->TASKS_SEQSTART[0] = 1;
}
/** @snippet [Set new RGB brightness and/or color]*/


void stopLED( void )
{
  NRF_PWM0->TASKS_STOP = 1;
  NRF_PWM1->TASKS_STOP = 1;
  NRF_PWM2->TASKS_STOP = 1;

  NRF_PWM0->ENABLE = 0;
  NRF_PWM1->ENABLE = 0;
  NRF_PWM2->ENABLE = 0;
}
/** @snippet [Start PWM module] */


void startLED( void )
{
   NRF_PWM0->SEQ[0].CNT = 1;
   NRF_PWM1->SEQ[0].CNT = 1;
   NRF_PWM2->SEQ[0].CNT = 1;

   NRF_PWM0->SEQ[0].PTR = (uint32_t)&REDseq_buf[0];
   NRF_PWM1->SEQ[0].PTR = (uint32_t)&GREENseq_buf[0];
   NRF_PWM2->SEQ[0].PTR = (uint32_t)&BLUEseq_buf[0];

    NRF_PWM0->ENABLE = 1;
    NRF_PWM1->ENABLE = 1;
    NRF_PWM2->ENABLE = 1;

   NRF_PWM0->TASKS_SEQSTART[0] = 1;
   NRF_PWM1->TASKS_SEQSTART[0] = 1;
   NRF_PWM2->TASKS_SEQSTART[0] = 1;
}
/** @snippet [Start PWM module] */


void testLED()
{
  for(int i = 0; i < 10; i++)
    {
      nrf_delay_ms(10);
      updateLED(i,0,0);
    }
    for(int i = 0; i < 10; i++)
    {
      nrf_delay_ms(10);
      updateLED(0,i,0);
    }
    for(int i = 0; i < 10; i++)
    {
      nrf_delay_ms(10);
      updateLED(0,0,i);
    }
}
/** @snippet [PWM module test sequence] */


/** @} */