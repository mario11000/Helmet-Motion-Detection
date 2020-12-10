#include "MPU6050_HelmetMotion.h"
#include "Arduino.h"

MPU6050::MPU6050(TwoWire &w){
	wire = &w;
}
void MPU6050::begin(){
	writeMPU6050(MPU6050_PWR_MGMT_1,0x00);
	
	this->calculate_IMU_error();
	this->timeLost = 0;
	this->update();
	
	gyroAngleX = 0;
	gyroAngleY = 0;
	
	roll = this->getAccAngleX();
	pitch = this->getAccAngleY();

}
void MPU6050::writeMPU6050(byte reg, byte data){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(reg);
	wire->write(data);
	wire->endTransmission();
}
	
void MPU6050::calculate_IMU_error(){
	// We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
	// Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
	// Read accelerometer values 200 times
	while (c < 200) {
		wire->beginTransmission(MPU6050_ADDR);
		wire->write(0x3B);
		wire->endTransmission(false);
		wire->requestFrom((int)MPU6050_ADDR, 6, (int)true);
		
		AccX = (wire->read() << 8 | wire->read()) / 16384.0 ;
		AccY = (wire->read() << 8 | wire->read()) / 16384.0 ;
		AccZ = (wire->read() << 8 | wire->read()) / 16384.0 ;
		
		// Sum all readings
		AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
		AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
		c++;
	}
	
	//Divide the sum by 200 to get the error value
	AccErrorX = AccErrorX / 200;
	AccErrorY = AccErrorY / 200;
	
	c = 0;
	
	// Read gyro values 200 times
	while (c < 200) {
		wire->beginTransmission(MPU6050_ADDR);
		wire->write(0x43);
		wire->endTransmission(false);
		wire->requestFrom((int)MPU6050_ADDR, 6, (int)true);
		
		GyroX = wire->read() << 8 | wire->read();
		GyroY = wire->read() << 8 | wire->read();
		GyroZ = wire->read() << 8 | wire->read();
	
		// Sum all readings
		GyroErrorX = GyroErrorX + (GyroX / 131.0);
		GyroErrorY = GyroErrorY + (GyroY / 131.0);
		GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
		c++;
	}
	//Divide the sum by 200 to get the error value
	GyroErrorX = GyroErrorX / 200;
	GyroErrorY = GyroErrorY / 200;
	GyroErrorZ = GyroErrorZ / 200;


	xerrGyro = GyroErrorX;
	yerrGyro = GyroErrorY;
	zerr = GyroErrorZ;

	xerrAcc = AccErrorX;
	yerrAcc = AccErrorY;
}
void MPU6050::update(){
	// === Read acceleromter data === //
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);// Start with register 0x3B (ACCEL_XOUT_H)
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 6, (int)true);  // Read 6 registers total, each axis value is stored in 2 registers
	
	//For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
	AccX = (wire->read() << 8 | wire->read()) / 16384.0; // X-axis value
	AccY = (wire->read() << 8 | wire->read()) / 16384.0; // Y-axis value
	AccZ = (wire->read() << 8 | wire->read()) / 16384.0; // Z-axis value
	
	// Calculating Roll and Pitch from the accelerometer data
	accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - xerrAcc; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
	accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - yerrAcc; // AccErrorY ~(-1.58)

	// === Read gyroscope data === //
	previousTime = currentTime;        // Previous time is stored before the actual time read
	currentTime = millis();            // Current time actual time read
	elapsedTime = (currentTime - previousTime - timeLost) / 1000; // Divide by 1000 to get seconds
	
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x43);// Gyro data first register address 0x43
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 6, (int)true);  // Read 4 registers total, each axis value is stored in 2 registers
	
	GyroX = (wire->read() << 8 | wire->read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
	GyroY = (wire->read() << 8 | wire->read()) / 131.0;
	GyroZ = (wire->read() << 8 | wire->read()) / 131.0;
	
	// Correct the outputs with the calculated error values
	GyroX = GyroX - xerrGyro; // GyroErrorX ~(-0.56)
	GyroY = GyroY - yerrGyro; // GyroErrorY ~(2)
	GyroZ = GyroZ - zerr; // GyroErrorZ ~ (-0.8)

	// Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
	gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
	gyroAngleY = gyroAngleY + GyroY * elapsedTime;
	yaw =  yaw + GyroZ * elapsedTime;

	// Complementary filter - combine acceleromter and gyro angle values
	roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
	pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
	
	Serial.println("Current, Previous, lost, Elapsed: ");
	
	Serial.print(currentTime);
	Serial.print(" , ");
	Serial.print(previousTime);
	Serial.print(" , ");
	Serial.print(timeLost);
	Serial.print(" , ");
	Serial.print(elapsedTime);

}