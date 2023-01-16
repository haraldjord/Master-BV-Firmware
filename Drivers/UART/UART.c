#include "UART.h"

/**
 * @brief Functions for UART usage.
 */
void UARTEinit()
{

  NRF_UART0->ENABLE = UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos; // Configure pins only when UART is disabled

  //NRF_UARTE0->PSEL.RTS = RTS_PIN;
  NRF_UARTE0->PSEL.TXD = TXD_PIN;
  //NRF_UARTE0->PSEL.CTS = CTS_PIN;
  NRF_UARTE0->PSEL.RXD = RXD_PIN;


  // Configugre pins before enabling UARTE
  //NRF_GPIO->DIR |= (1 << RTS_PIN);
  NRF_P0->DIR |= (1 << TXD_PIN);

  NRF_P0->OUTSET = (1 << TXD_PIN); // TXD inactive default 1
  //NRF_GPIO->OUTSET = (1 << RTS_PIN); // RTS inactive default 1

  //NRF_GPIO->DIR &= ~(1 << CTS_PIN);
  NRF_GPIO->DIR &= ~(1 << RXD_PIN);

  NRF_UARTE0->CONFIG = (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos)    |
                       (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos); // Not sure if I want parity bit or not .. Check motor datasheet !!!

  // Baudrate: 115200 baud (actual rate: 9598)
  NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud9600 << UARTE_BAUDRATE_BAUDRATE_Pos;

  //NRF_UARTE0->RXD.MAXCNT = sizeof(p_UARTE_RXbuf)/sizeof(uint8_t) << UARTE_RXD_MAXCNT_MAXCNT_Pos;
  //NRF_UARTE0->TXD.MAXCNT = sizeof(p_UARTE_TXbuf)/sizeof(uint8_t) << UARTE_TXD_MAXCNT_MAXCNT_Pos;

  //NRF_UARTE0->RXD.PTR=(uint32_t)&p_UARTE_RXbuf[0];
  //NRF_UARTE0->TXD.PTR=(uint32_t)&p_UARTE_TXbuf[0];

}

void UARTEreceive()
{

 volatile int8_t UARTE_RXbuf[12] = {0,0,0,0,0,4,0,0,0,0,0,0};

   NRF_UARTE0->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Disabled << UARTE_SHORTS_ENDRX_STARTRX_Pos;

  // Enable UARTE
  NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;

  NRF_UARTE0->RXD.MAXCNT = 9;
  NRF_UARTE0->RXD.PTR=(uint32_t)&UARTE_RXbuf[0];

 // Start Receive
  NRF_UARTE0->EVENTS_RXTO = 0;
  NRF_UARTE0->TASKS_STARTRX = 1;
  NRF_LOG_INFO("STARTRX");
  while(NRF_UARTE0->EVENTS_ENDRX == 0);
  NRF_UART0->TASKS_STOPRX = 1;


while(NRF_UARTE0->EVENTS_RXTO == 0);
  {NRF_LOG_INFO("RXTimeOut");}
  
  NRF_UARTE0->TXD.PTR=(uint32_t)&UARTE_RXbuf[0];
  NRF_UARTE0->TASKS_FLUSHRX;
    NRF_LOG_INFO("Flush");




  // Disable UARTE
  NRF_UART0->ENABLE = UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos;

  NRF_LOG_INFO("Received RXbuf[0] = %x", UARTE_RXbuf[0])
  NRF_LOG_INFO("Received RXbuf[1] = %x", UARTE_RXbuf[1])
  NRF_LOG_INFO("Received RXbuf[2] = %d", UARTE_RXbuf[2])
  NRF_LOG_INFO("Received RXbuf[3] = %x", UARTE_RXbuf[3])
  NRF_LOG_INFO("Received RXbuf[4] = %x", UARTE_RXbuf[4])
  NRF_LOG_INFO("Received RXbuf[5] = %x", UARTE_RXbuf[5])
  NRF_LOG_INFO("Received RXbuf[6] = %x", UARTE_RXbuf[6])
  NRF_LOG_INFO("Received RXbuf[7] = %x", UARTE_RXbuf[7])
  NRF_LOG_INFO("Received RXbuf[8] = %x", UARTE_RXbuf[8])
}


void UARTEsend( volatile uint8_t* p_UARTE_TXbuf )
{



NRF_UARTE0->SHORTS = UARTE_SHORTS_ENDRX_STOPRX_Enabled << UARTE_SHORTS_ENDRX_STOPRX_Pos;

  // Enable UARTE
  NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;
  //uint8_t bytes = (uint8_t)sizeof(p_UARTE_TXbuf)/sizeof(uint8_t);


  NRF_UARTE0->TXD.MAXCNT = 9;//sizeof(p_UARTE_TXbuf)/sizeof(uint8_t) << UARTE_TXD_MAXCNT_MAXCNT_Pos;
  NRF_UARTE0->TXD.PTR=(uint32_t)&p_UARTE_TXbuf[0];

  // Start transmission
  NRF_UARTE0->TASKS_STARTTX = 1;

  // Wait until transmission finish - Should be handled by interrupt so the processor does not have to wait.
  while(NRF_UARTE0->EVENTS_ENDTX == 0);
  
  // Stop transmission
  NRF_UARTE0->EVENTS_TXSTOPPED = 0;
  NRF_UARTE0->TASKS_STOPTX = 1;
  while(NRF_UARTE0->EVENTS_TXSTOPPED == 0);
NRF_LOG_INFO("TXSTOPPED");

  // Disable UARTE
  NRF_UART0->ENABLE = UARTE_ENABLE_ENABLE_Disabled << UARTE_ENABLE_ENABLE_Pos;

  //Full-duplex: Receive reply message
  UARTEreceive();

}




// Configure UART pins to known state before going to OFF mode
void prepareUARTforOFFmode()
{
  NRF_UART0->ENABLE = 0; // Configure pins only when UART is disabled

  //NRF_GPIO->DIR |= (1 << RTS_PIN);
  NRF_GPIO->DIR |= (1 << TXD_PIN);

  NRF_GPIO->OUTSET = (1 << TXD_PIN); // TXD inactive default 1
  //NRF_GPIO->OUTSET = (1 << RTS_PIN); // RTS inactive default 1

  //NRF_GPIO->DIR &= ~(1 << CTS_PIN);
  NRF_GPIO->DIR &= ~(1 << RXD_PIN);
}