#ifndef _UART_H
#define _UART_H

#include "main.h"

// UART
//#define RTS_PIN (5UL)
#define TXD_PIN (11UL) // white
//#define CTS_PIN (7UL)
#define RXD_PIN (12UL) // gray/brown


void UARTEinit(void);
void UARTEreceive(void);
void UARTEsend(volatile uint8_t*);
void prepareUARTforOFFmode(void);


#endif