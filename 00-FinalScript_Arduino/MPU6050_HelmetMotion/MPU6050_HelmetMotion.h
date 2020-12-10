#ifndef MPU6050_HelmetMotion
#define MPU6050_HelmetMotion

#include "Arduino.h"
#include "Wire.h"

#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_INITIAL		 0x3B
#define	MPU6050_TEMP		 0x43

class MPU6050{
	public:
		//constructor
		MPU6050(TwoWire &w);
		
		//begin is used to initialize the gyroscope (calibrstion + intial values and errors)
		void begin();
		//update values
		void update();

		
		void writeMPU6050(byte reg,byte data);
		
		//calculate the drift error
		void calculate_IMU_error();
			
		float getAngleX(){ return roll; };
		float getAngleY(){ return pitch; };
		float getAngleZ(){ return yaw; };
		float getAccAngleX(){ return accAngleX; };
		float getAccAngleY(){ return accAngleY; };
		
		float setTimeLoss(float loss){ timeLost = loss; }
	
	private:
		TwoWire *wire;
		
		float AccX, AccY, AccZ;
		float GyroX, GyroY, GyroZ;
		float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
		float roll, pitch, yaw;
		float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
		float elapsedTime, currentTime, previousTime;
		float timeLost;
		int c = 0;
		float xerrGyro, yerrGyro, zerr;
		float xerrAcc, yerrAcc;
};

#endif
