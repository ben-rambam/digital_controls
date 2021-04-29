#include <Wire.h>
#include "MadgwickAHRS.h"
#include "motor-control.hpp"

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
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  if(!motor_init(MOTOR_A))
    Serial.println("Bad motor enum for init");

  if(!motor_init(MOTOR_B))
	  Serial.println("Bad motor enum for init");
  //motor_init(MOTOR_B);
}


void loop() {

 

  /*
  Serial.print("speed: ");
  Serial.print(motorSpeed);
  Serial.print("speed direction: ");
  Serial.print(motorSpeedDirection);
  Serial.print("direction: ");
  Serial.println(motorDirection);
  */

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, MPU_OUT_REGISTERS::NUM_REGISTERS, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accel_x = (Wire.read()<<8 | Wire.read()); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accel_y = -1*(Wire.read()<<8 | Wire.read()); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accel_z = -1*(Wire.read()<<8 | Wire.read()); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = (Wire.read()<<8 | Wire.read())/131.0f*M_PI/180.0f; // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = -1*(Wire.read()<<8 | Wire.read())/131.0f*M_PI/180.0f; // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = -1*(Wire.read()<<8 | Wire.read())/131.0f*M_PI/180.0f; // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  MadgwickAHRSupdateIMU(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z);

  fakeController();

  char float_str[32];

  dtostrf(get_yaw(), 4, 4, float_str);
  sprintf(tmp_str, " | yaw = %s", float_str);
  Serial.print(tmp_str );
  dtostrf(get_pitch(), 4, 4, float_str);
  sprintf(tmp_str, " | pitch = %s", float_str);
  Serial.print(tmp_str );
  dtostrf(get_roll(), 4, 4, float_str);
  sprintf(tmp_str, " | roll = %s", float_str);
  Serial.print(tmp_str );
  Serial.println();

  delay(10);
}

void fakeController()
{

  //ok. Let's do this. 
  static int motorSpeed = 0;
  static enum MotorDirection motorDirection = MOTOR_DIR_FORWARD;

  float positivePitch = fabs(get_pitch());
  motorSpeed = 160*positivePitch;
  motor_set_duty_cycle(MOTOR_A, motorSpeed);
  motor_set_duty_cycle(MOTOR_B, motorSpeed);
    
  if(get_pitch() > 0)
  {
    motorDirection = MOTOR_DIR_BACKWARD;
    
    motor_set_dir(MOTOR_A, motorDirection);
    motor_set_dir(MOTOR_B, motorDirection);
  }
  else if (get_pitch() < 0)
  {
    motorDirection = MOTOR_DIR_FORWARD;
    
    motor_set_dir(MOTOR_A, motorDirection);
    motor_set_dir(MOTOR_B, motorDirection);
  }
  Serial.print("speed: ");
  Serial.print(motorSpeed);

  Serial.print("\tdirection: ");
  Serial.println(motorDirection);
  
}
