#include <Wire.h>
#include "twi.h" 

const int MPU_ADDR = 0x68;
const unsigned char ACK = 1;
const unsigned char NACK = 0;

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

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600);
  
  TWI_init();			// Initialize the TWI
  TWI_start();			// Send a start signal
  TWI_write(MPU_ADDR);		// Send the address to read/write data from/to
  TWI_write(0x68);		// Set the address
  TWI_write(0);			// Set to zero (wakes up the MPU-6050)
  TWI_stop();			// Send the stop signal

  //Wire.begin();
  //Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  //Wire.write(0x6B); // PWR_MGMT_1 register
  //Wire.write(0); // set to zero (wakes up the MPU-6050)
  //Wire.endTransmission(true);

  
}
void loop() {
  //Wire.beginTransmission(MPU_ADDR);
  //Wire.write(ACCEL_XOUT_H); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  //Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  //Wire.requestFrom(MPU_ADDR, MPU_OUT_REGISTERS::NUM_REGISTERS, true); // request a total of 7*2=14 registers
  
  TWI_start();		
  TWI_write(MPU_ADDR);
  TWI_write(ACCEL_XOUT_H);

  TWI_start();
  
  //I am assuming that we don't need to request for number of register to fill. 
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = TWI_read(ACK)<<8 | TWI_read(ACK); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  TWI_read(NACK);				// Tell the TWI to stop reading
  

  TWI_stop();


  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  //accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  //accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  //accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  //temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  //gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  //gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  //gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  // delay
  delay(1000);

}
