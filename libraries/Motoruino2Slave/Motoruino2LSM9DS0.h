/*
 Artica CC - http://artica.cc

 MotoruinoSlave lib

 based on the SFE_LSM9DS0.h Jim Lindblom @ SparkFun Electronics (beerware license :) )

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 - Start of the project

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */


#ifndef __LSM9DS0_H_
#define __LSM9DS0_H_

#include "Arduino.h"


#include "Motoruino2SPINS.h"
#include "Motoruino2SSPI.h"
#include "Motoruino2AngleKalman.h"

#define IMU_DEBUG 0

#define MAX_SPI_BUFFER 	6
#define CALIB_SAMPLES 	100

// Number of samples acquired before calculation
#define SAMPLES_CALC_G	25		// Gyro @ 760Hz
#define SAMPLES_CALC_A	10		// Accel @ 100Hz
#define SAMPLES_CALC_M	10		// Mag@50Hz

#define RESTRICT_PITCH

////////////////////////////
// LSM9DS0 Values 		  //
////////////////////////////
#define A_CTRL_REG1_G_VALUE   0b11111111	// 760Hz ODR; 100Hz CutOff; Normal Power Mode
#define A_CTRL_REG2_G_VALUE   0b00001001	// Normal Mode; HPF 0.09Hz
#define A_CTRL_REG3_G_VALUE   0b00000000	// All Interrupt Off; Push-Pull; Fifo Wmark disabled
#define A_CTRL_REG4_G_VALUE   0b00110000  	// Continuous ;LSB@Lower Add; 2000dps; Self Test Off
#define A_CTRL_REG5_G_VALUE   0b01000100	// FIFO enabled; HPF disabled; High Pass to FIFO
#define FIFO_CTRL_REG_G_VALUE 0b01011111	// Stream FIFO Mode; WaterMark = 32

#define A_CTRL_REG0_XM_VALUE 0b01000000		// Normal Mode; FIFO enabled; WTM off;
#define A_CTRL_REG1_XM_VALUE 0b01100111		// ODR 100Hz; continuous mode; ZYX enabled
#define A_CTRL_REG2_XM_VALUE 0b00000000		// 773Hz Anti-alias; +-2g; SefTest Normal mode
#define A_CTRL_REG3_XM_VALUE 0b00100000		// Inertial Interrupt generator on INT1
#define A_CTRL_REG4_XM_VALUE 0b00000000 	// No Interrupts on INT2
#define FIFO_CTRL_REG_VALUE  0b01011111		// Stream FIFO Mode; WaterMark = 32

#define A_INT_GEN_1_REG		 	0b01111111
#define A_INT_GEN_1_DURATION	0b01111111
#define A_INT_GEN_1_THS	 	 	0b00011111

#define A_CTRL_REG5_XM_VALUE 0b10010000		// Temperature ON; Mag High Resol; Mag ODR 50Hz
#define A_CTRL_REG6_XM_VALUE 0b00000000 	// Mag scale to +/- 2Gauss
#define A_CTRL_REG7_XM_VALUE 0b00000000 	// HPF Normal Mode; Internal Filter bypass; Low Power: Off; Continuous Mode
#define INT_CTRL_REG_M_VALUE 0b00001000		// int recognition off; push-pull; int active high; not latched; 4D off; int for mag off

#define A_CTRL_REG1_G_VALUE_SLEEP   0b00000000		// All off
#define A_CTRL_REG2_G_VALUE_SLEEP   0b00001001		// Normal Mode; HPF 0.09Hz
#define A_CTRL_REG3_G_VALUE_SLEEP   0b00000000		// All Interrupt Off; Push-Pull; Fifo Wmark disabled
#define A_CTRL_REG4_G_VALUE_SLEEP   0b00110000  	// Continuous ;LSB@Lower Add; 2000dps; Self Test Off
#define A_CTRL_REG5_G_VALUE_SLEEP   0b00000100		// FIFO disabled; HPF disabled; High Pass to FIFO
#define FIFO_CTRL_REG_G_VALUE_SLEEP 0b00000000

#define A_CTRL_REG0_XM_VALUE_SLEEP 0b01000000		// Normal Mode; FIFO enabled; WTM off;
#define A_CTRL_REG1_XM_VALUE_SLEEP 0b01010111		// ODR 50Hz; continuous mode; ZYX enabled
#define A_CTRL_REG2_XM_VALUE_SLEEP 0b00000000		// 773Hz Anti-alias; +-2g; SefTest Normal mode
#define A_CTRL_REG3_XM_VALUE_SLEEP 0b01100000		// Inertial Interrupt generator on INT1
#define A_CTRL_REG4_XM_VALUE_SLEEP 0b01000000 		// No Interrupts on INT2
#define FIFO_CTRL_REG_VALUE_SLEEP  0b01011111		// Stream FIFO Mode; WaterMark = 32

#define A_CTRL_REG5_XM_VALUE_SLEEP 0b00000100		// Temperature OFF; Mag High Resol; Mag ODR 50Hz
#define A_CTRL_REG6_XM_VALUE_SLEEP 0b00000000 	// Mag scale to +/- 2Gauss
#define A_CTRL_REG7_XM_VALUE_SLEEP 0b00000100 	// HPF Normal Mode; Internal Filter bypass; Low Power: on; Continuous Mode
#define INT_CTRL_REG_M_VALUE_SLEEP 0b11101011

class Motoruino2LSM9DS0
{
public:

	Motoruino2LSM9DS0();

	void config(bool _GYRO = true, bool _ACCEL = true, bool _MAG = true);

	bool startFlagIMU; 						// Get IMU Values pooling?

	// ------------------------ CALCULATED DATA -----------------------------------
	// Gyro Integration (in relative degrees * 100)
	short getGyroHeading();	short GetGyroPitch();	short getGyroRoll();

	// Magnetometer Integration (in absolute degrees)
	float  getMagHeading();	float  getMagPitch();	float  getMagRoll();

	// Accelerometer Average Values (float with maximum 2Gs)
	float getAccelAvgZ();	float getAccelAvgX();	float getAccelAvgY();

	// Kalman in degrees (from accelerometer)
	float getAngleY();
	float getAngleX();

	// ------------------------ RAW DATA -----------------------------------
	// Get the raw values from the accelerometer, gyroscope and magnetometer for the different axis
	short getAccelerometerX();
	short getAccelerometerY();
	short getAccelerometerZ();

	short getGyroscopeX();
	short getGyroscopeY();
	short getGyroscopeZ();

	short getMagnetometerX();
	short getMagnetometerY();
	short getMagnetometerZ();

	// ------------------------ FUNCTIONS -----------------------------------

	// Resets the current heading referential
	void resetGyro(bool _Heading, bool _Pitch, bool _Roll);

	bool calibrateAngle();		// Calibrate Accelerometer Angle - Kalman Filter
	bool calibrateGyro(); 		// Calibrate Gyroscope
	bool calibrateMag(); 		// Calibrate GMagnetometer

	// Sleep functions
	void wake(bool _GYRO, bool _ACCEL, bool _MAG);
	void sleep();

	unsigned short getTemp();

	// ------------------------ POOLING FUNCTIONS -----------------------------------
	// pooling function for Data Update (integration)
	void updateGyroscope();
	void updateAccelerometer();
	void updateMagnetometer();
	void updateKalman();


private:

	Motoruino2SSPI spi;

	// Create the Kalman instances
	Kalman kalmanX;
	Kalman kalmanY;

	double gyroXangle, gyroYangle; // Angle calculate using the gyro only
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
	float headingDegrees;
	unsigned long timer;

	// Gyroscope Stream FIFO Memory (to don't lose any sample)
	volatile short streamGyroX[SAMPLES_CALC_G+1];
	volatile short streamGyroY[SAMPLES_CALC_G+1];
	volatile short streamGyroZ[SAMPLES_CALC_G+1];
	volatile unsigned char StreamGyroPointer;

	// Gyroscope processed angle - "were it is turned"
	volatile long rollX, rollXAngle;
	volatile long pitchY, pitchYAngle;
	volatile long headingZ, headingZAngle;

	// Gyroscope calibration Data
	volatile short rollError;
	volatile short pitchError;
	volatile short headingError;

	// Accelerometer Stream FIFO Memory (to don't lose any sample)
	volatile short streamAccelX[SAMPLES_CALC_A+1];
	volatile short streamAccelY[SAMPLES_CALC_A+1];
	volatile short streamAccelZ[SAMPLES_CALC_A+1];
	volatile unsigned char streamAccelPointer;

	// Last updated Raw Values Received
	volatile short gyroRawX,  gyroRawY,  gyroRawZ;
	volatile float magRawX,  magRawY,  magRawZ;
	volatile short accelRawX, accelRawY, accelRawZ;

	// Accelerometer processed angle - "accumulated average values"
	volatile float accelXAvg, accelYAvg, accelZAvg;

	volatile int calibrationSamples;

	volatile long tempDataX,tempDataY,tempDataZ;
	unsigned char bufferSPI[MAX_SPI_BUFFER];
	unsigned char fifo_src[2];

	// pooling startup flag
	unsigned char initializationEnded;

	// Config functions
	void initGyro();
	void initAccel();
	void initMag();

	// SPI Functions to read and Write
	void writeReg(int cs_pin, unsigned char reg, unsigned char value);
	void readReg(int cs_pin, unsigned char reg, unsigned char len, unsigned char *data);

};

extern Motoruino2LSM9DS0				imuLSM9DS0;

#endif
