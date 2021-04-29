#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

//Includes
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

enum MotorDirection
{
	MOTOR_DIR_FORWARD,
	MOTOR_DIR_BACKWARD,
};

enum MotorSelect
{
	MOTOR_A,
	MOTOR_B,
};

static const uint32_t TOP = 639;

#ifdef __cplusplus
  extern "C" {
#endif

//Functions
int motor_init(enum MotorSelect motorSelect);
int motor_set_dir(enum MotorSelect motorSelect, enum MotorDirection motorDirection);
int motor_set_duty_cycle(enum MotorSelect motorSelect, int duty_cycle);

#ifdef __cplusplus
  }
#endif
#endif
