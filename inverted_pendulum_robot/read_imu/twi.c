#include <twi.h>

void TWI_init()
{
  TWSR = 0x00;        //Set the prescale bits to zero
  TWBR = 0x98;        //SLC to 50 Hz ---> 16 MHz clock
  TWCR = (1 << TWEN); //Enable the TWF
}

void TWI_start()
{
  TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TNEN));
  while(!(TWCR & (1<< TWINT)));
}

void TWI_stop()
{
  TWCR = ((1<<TWINT) | (1<<TWEN) | (1 << TWST0));
  _delay_us(100);
}

unsigned char TWI_read( unsigned char ACKval)
{
  TWCR = ((1<<TWINT) | ( 1<<TWEN) | (ACK_val << TWEA);
  while(!(TWCR & (1 << TWINT));
  return TWDR;
}

unsigned char TWI_write( unsigned char data)
{
  TWDR = data;
  TWCR = (( 1<<TWINT) | (1<<TWEN));
  while(!(TWCR & (1 << TWINT)));
}
