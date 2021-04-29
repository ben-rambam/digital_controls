//Includes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include "motor_control.h"


int motor_init(enum MotorSelect motorSelect)
{
	if (motorSelect == MOTOR_A)
	{
		// Set the necessary pins to outputs
		//motor A is set to use pin 3 (OC3C) for PWM 
		DDRE |= (1<<PE5);
		//motor A is set to use pin 12 (PB6) for direction
		DDRB |= (1<<PB6);
		TCNT3 = 0;
		
		// freq = 25 kHz as recommended in the L298P motor driver datasheet
		// Set N=1 for the prescaler
		// TOP = (clk_freq/freq/N)-1 = 639
		// COM = 0b10; p.155 Set on bottom, clear on compare
		// WGM = 0b1110; p.145 Fast PWM with ICRn as TOP
		// CS  = 0b001; p.157 No prescaling

		ICR3 = TOP;

		OCR3C = 0;

		TCCR3A = (1<<COM3C1) | (0<<COM3C0) | (1<<WGM31) | (0 << WGM30);
		TCCR3B = (1<<WGM33) | (1<<WGM32) | (0<<CS32) | (1<<CS31) | (1<<CS30);
	}
	else if(motorSelect == MOTOR_B)
	{
		// Set the necessary pins to outputs
		// motor B is set to use pin 11 (OC1A) for PWM
		DDRB |= (1<<PB5);
		// motor B is set to use pin 13 (PB7) for direction
		DDRB |= (1<<PB7);
		TCNT1 = 0;
		
		// freq = 25 kHz as recommended in the L298P motor driver datasheet
		// Set N=1 for the prescaler
		// TOP = (clk_freq/freq/N)-1 = 639
		// COM = 0b10; p.155 Set on bottom, clear on compare
		// WGM = 0b1110; p.145 Fast PWM with ICRn as TOP
		// CS  = 0b001; p.157 No prescaling
		
		ICR1 = TOP;

		OCR1A = 0;

		TCCR1A = (1<<COM1A1) | (0<<COM1A0) | (1<<WGM11) | (0 << WGM10);
		TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
	}
	else
	{
		return 0;
	}
	return 1;
}

int motor_set_dir(enum MotorSelect motorSelect, enum MotorDirection motorDirection)
{
	if(motorSelect == MOTOR_A) //Direction A is on pin 12 (PB6)
	{
		if( motorDirection == MOTOR_DIR_FORWARD )
			PORTB |= (1<<PB6);
		else if( motorDirection == MOTOR_DIR_BACKWARD )
			PORTB &= ~(1<<PB6);
		else //explode the world. This should never occur.
			return 0;
	}
	else if(motorSelect == MOTOR_B) //Direction B is on pin 13 (PB7)
	{
		if( motorDirection == MOTOR_DIR_FORWARD )
			PORTB |= (1<<PB7);
		else if( motorDirection == MOTOR_DIR_BACKWARD )
			PORTB &= ~(1<<PB7);
		else //explode the world. This should never occur.
			return 0;
	}
	else //explode the world. This should never occur.
	{
		return 0;
	}

	return 1;
}

int motor_set_duty_cycle(enum MotorSelect motorSelect, int duty_cycle)
{
	if(motorSelect == MOTOR_A) //Direction A is on pin 12 (PB6)
	{
		uint32_t val = duty_cycle * TOP / 100;
    if(val > TOP)
    {
      val = TOP;
    }
		OCR3C = val;
	}
	else if(motorSelect == MOTOR_B) //Direction B is on pin 13 (PB7)
	{
		uint32_t val = duty_cycle * TOP / 100;
    if(val > TOP)
    {
      val = TOP;
    }
		OCR1A = val;
	}
	else //explode the world. This should never occur.
	{
		return 0;
	}
	return 1;
}
