#ifndef TWI_H
#define TWI_H

//Includes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

const unsigned char ACK = 1;
const unsigned char NACK = 0;


#ifdef __cplusplus
extern "C" {
#endif
//Functions
void TWI_init();
void TWI_start();
void TWI_stop();
unsigned char TWI_read( unsigned char ACKval);
void TWI_write( unsigned char data);

#ifdef __cplusplus
}
#endif
#endif
