#include "MadgwickAHRS.h"
#include "motor_control.h"
#include "twi.h" 

#define MPU_READ 0xD1
#define MPU_WRITE 0xD0


volatile int32_t risingEdgeCapt;
volatile int32_t fallingEdgeCapt;
volatile int32_t differenceCapt;

const int MPU_ADDR = 0x68;

enum MPU_OUT_REGISTERS
{
  ACCEL_XOUT_H = 0X3B,
  ACCEL_XOUT_L,
  ACCEL_YOUT_H,
  ACCEL_YOUT_L,
  ACCEL_ZOUT_H,
  ACCEL_ZOUT_L,
  TEMP_OUT_H,
  TEMP_OUT_L,
  GYRO_XOUT_H,
  GYRO_XOUT_L,
  GYRO_YOUT_H,
  GYRO_YOUT_L,
  GYRO_ZOUT_H,
  GYRO_ZOUT_L,
  NUM_REGISTERS
};

float accel_x, accel_y, accel_z; // variables for accelerometer raw data
float gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[32]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

float get_yaw()
{
	float top = 2.0f*(q0*q3+q1*q2);
	float bottom = 1.0f - 2.0f*(q2*q2+q3*q3);
	return atan2(top,bottom);
}

float get_pitch()
{
	return asin(2.0f*(q0*q2-q3*q1));
}

float get_roll()
{
	float top = 2.0f*(q0*q1 + q2*q3);
	float bottom = 1.0f-2.0f*(q1*q1+q2*q2);
	return atan2(top,bottom);
}

void setup() {
  Serial.begin(9600);
  sei();
  Serial.print("In Set up");
//  timer5init();


  TWI_init();			// Initialize the TWI
  TWI_start();			// Send a start signal
  TWI_write(MPU_WRITE);		// Send the address to read/write data from/to
  TWI_write(0x6B);		// Set the address
  TWI_write(0);			// Set to zero (wakes up the MPU-6050)
  TWI_stop();			// Send the stop signal
  
  if(!motor_init(MOTOR_A))
    Serial.println("Bad motor enum for init");

  if(!motor_init(MOTOR_B))
	  Serial.println("Bad motor enum for init");
}

void loop() {


  TWI_start();		
  TWI_write(MPU_WRITE);
  TWI_write(ACCEL_XOUT_H);

  TWI_start();
  TWI_write(MPU_READ);

  accel_x = TWI_read(ACK)<<8 | TWI_read(ACK);
  accel_y = -1*(TWI_read(ACK)<<8 | TWI_read(ACK));
  accel_z = -1*(TWI_read(ACK)<<8 | TWI_read(ACK));
  temperature = TWI_read(ACK)<<8 | TWI_read(ACK);
  gyro_x = (TWI_read(ACK)<<8 | TWI_read(ACK))/131.0f*M_PI/180.0f;
  gyro_y = -1*(TWI_read(ACK)<<8 | TWI_read(ACK))/131.0f*M_PI/180.0f;
  gyro_z = -1*(TWI_read(ACK)<<8 | TWI_read(NACK))/131.0f*M_PI/180.0f;

  TWI_stop();

  MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

  float pitch = get_pitch();

  fakeController(pitch);

//  char float_str[32];
//  dtostrf(get_yaw(), 4, 4, float_str);
//  sprintf(tmp_str, " | yaw = %10s", float_str);
//  Serial.print(tmp_str );
//  dtostrf(get_pitch(), 4, 4, float_str);
//  sprintf(tmp_str, " | pitch = %10s", float_str);
//  Serial.print(tmp_str );
//  dtostrf(get_roll(), 4, 4, float_str);
//  sprintf(tmp_str, " | roll = %10s", float_str);
//  Serial.print(tmp_str );
//  Serial.print();
  //Serial.print("Time stamp: ");

  
  //static int prevMillis  = 0;
  //int currentMillis = millis();

  //Serial.print(currentMillis-prevMillis);
  //Serial.print(" ");
  //Serial.print(pitch);
  //Serial.println();
//  Serial.print("Here: ");
//  Serial.println(differenceCapt);
  
  //while((currentMillis - prevMillis) < 10)
  //  currentMillis = millis();
    

 // prevMillis = currentMillis;

}

void fakeController(float pitch)
{
  
  //ok. Let's do this. 
  static int motorSpeed = 0;
  static enum MotorDirection motorDirection = MOTOR_DIR_FORWARD;

  float positivePitch = fabs(pitch);
  motorSpeed = 800*positivePitch;
  motor_set_duty_cycle(MOTOR_A, motorSpeed);
  motor_set_duty_cycle(MOTOR_B, motorSpeed);
    
  if(pitch > 0)
  {
    motorDirection = MOTOR_DIR_BACKWARD;
    
    motor_set_dir(MOTOR_A, motorDirection);
    motor_set_dir(MOTOR_B, motorDirection);
  }
  else if (pitch < 0)
  {
    motorDirection = MOTOR_DIR_FORWARD;
    
    motor_set_dir(MOTOR_A, motorDirection);
    motor_set_dir(MOTOR_B, motorDirection);
  }
//  Serial.print("speed: ");
//  Serial.print(motorSpeed);
//
//  Serial.print("\tdirection: ");
//  Serial.print(motorDirection);

  
}

//void timer5init()
//{
//  TCNT5 = 0; //start at 0
//  TCCR5B |= (1 << CS52);
//  TIMSK5 |= (1 << ICIE5);//enable capture interupts
//  TCCR5B |= (1 << ICNC1); //Enable input capture noise canceller
//  TCCR5B |= (1 << ICES5);// enables interupt rising
//  TIFR5   = 1<<ICF5;
//}
//
//
//
//ISR(TIMER5_CAPT_vect)        
//{
//  
//  if( TCCR5B & (1 << ICES5))
//  {
//    risingEdgeCapt = TCNT5;
//    Serial.print(TCNT5);
//    TCCR5B &= ~(1 << ICES5);// enables interupt falling
//    Serial.print("\nR -");
//    Serial.print(risingEdgeCapt);
//  }
//  else
//  {
//    //calculate time :) anoushka dont know subtracting top=65,536
//    fallingEdgeCapt = TCNT5;
//    Serial.print(TCNT5);
//    differenceCapt = fallingEdgeCapt - risingEdgeCapt;
//    if(differenceCapt < 0)
//    {
//      differenceCapt += 65536;
//    }
//    TCCR5B |= (1 << ICES5);// enables interupt rising
//    Serial.print("\nF");
//    Serial.print(fallingEdgeCapt);
//  }
//
//  TCNT1 = 0;
//  Serial.print(differenceCapt);
//  
//}
