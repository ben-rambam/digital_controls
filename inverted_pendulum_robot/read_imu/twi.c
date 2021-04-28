#include "twi.h"
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void printCharactertoUSART(char character);
void printToUSART( char * menuString);
void BCDtoASCII(unsigned char data, char *characterData);


void printCharactertoUSART(char character)
{
  while(! (UCSR0A & (1 << UDRE0)));
  UDR0 = character;
}

void printToUSART( char * menuString)
{
  int len = strlen(&menuString[0]);
  int i = 0;
  for ( i = 0; i<len; i++)
  {
    printCharactertoUSART(menuString[i]);
  }
}

void BCDtoASCII(unsigned char data, char *characterData)
{
   int dec = ((data & 0xF0) >> 4) * 10 + (data & 0x0F);
   itoa(dec, characterData, 10);
}

void TWI_init()
{
  TWSR = 0x00;        //Set the prescale bits to zero
  TWBR = 0x98;        //SLC to 50 Hz ---> 16 MHz clock
  TWCR = (1 << TWEN); //Enable the TWF
}

void TWI_start()
{
  TWCR |= ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
  while(!(TWCR & (1<< TWINT)));
}

void TWI_stop()
{
  TWCR |= ((1<<TWINT) | (1<<TWEN) | (1 << TWSTO));
  _delay_us(100);
}

unsigned char TWI_read( unsigned char ACKval)
{  
  TWCR = ((1<<TWINT) | ( 1<<TWEN) | (ACKval << TWEA));
  while(!(TWCR & (1 << TWINT)));
  return TWDR;
}

void TWI_write( unsigned char data)
{
  TWDR = data;
  TWCR = (( 1<<TWINT) | (1<<TWEN));
  while(!(TWCR & (1 << TWINT)));
}
