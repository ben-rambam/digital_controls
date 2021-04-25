#ifndef TWI_H
#define TWI_H

//Includes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>


#ifdef __cplusplus
extern "C" {
#endif
//Functions
void TWI_init();
void TWI_start();
void TWI_stop();
unsigned char TWI_read( unsigned char ACKval);
unsigned char TWI_write( unsigned char data);

#ifdef __cplusplus
}
#endif
#endif
