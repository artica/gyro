/*
 Artica CC - http://artica.cc

 MotoruinoSlave lib

 based on the SFE_LSM9DS0 Jim Lindblom @ SparkFun Electronics (beerware license :) )

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


#include "Motoruino2LSM9DS0.h"
#include "Motoruino2SSPI.h"

#define _GYRO_180_VALUE 21000

// ADDRESSES DEFINES
////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			0x0F

#define A_CTRL_REG1_G			0x20
#define A_CTRL_REG2_G			0x21
#define A_CTRL_REG3_G			0x22
#define A_CTRL_REG4_G			0x23
#define A_CTRL_REG5_G			0x24

#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27

#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D

#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06

#define STATUS_REG_M		0x07

#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D

#define WHO_AM_I_XM			0x0F

#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E

#define A_CTRL_REG0_XM		0x1F

#define A_CTRL_REG1_XM		0x20
#define A_CTRL_REG2_XM		0x21
#define A_CTRL_REG3_XM		0x22
#define A_CTRL_REG4_XM		0x23
#define A_CTRL_REG5_XM		0x24
#define A_CTRL_REG6_XM		0x25
#define A_CTRL_REG7_XM		0x26

#define STATUS_REG_A		0x27

#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F

#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

Motoruino2LSM9DS0				imuLSM9DS0;

Motoruino2LSM9DS0::Motoruino2LSM9DS0()
{
	initializationEnded = 0;
}

void Motoruino2LSM9DS0::config(bool _GYRO, bool _ACCEL, bool _MAG)
{
	unsigned long start = 0;

	if (IMU_DEBUG) Serial.begin(115200);

	// SPI Chip-Select Setup
    pinMode(_CS_ACCEL_MAG, OUTPUT);
    pinMode(_CS_GYRO,  OUTPUT);
    digitalWrite(_CS_ACCEL_MAG, HIGH);
    digitalWrite(_CS_GYRO,  HIGH);

    StreamGyroPointer = 0;

   	spi.config();
   
      //Delay before start 50ms
    start = millis();
    while (millis() - start < 50);

    if (_GYRO) initGyro();
    if (_ACCEL) initAccel();
    if (_MAG) initMag();

    startFlagIMU = false;
    if ((_GYRO) || (_ACCEL) || (_MAG)) startFlagIMU = true;

    initializationEnded = 1;

}

void Motoruino2LSM9DS0::initGyro()
{
	 // Gyro
	writeReg(_CS_GYRO, A_CTRL_REG1_G, A_CTRL_REG1_G_VALUE);
	writeReg(_CS_GYRO, A_CTRL_REG2_G, A_CTRL_REG2_G_VALUE);
	writeReg(_CS_GYRO, A_CTRL_REG3_G, A_CTRL_REG3_G_VALUE);
	writeReg(_CS_GYRO, A_CTRL_REG4_G, A_CTRL_REG4_G_VALUE);
	writeReg(_CS_GYRO, A_CTRL_REG5_G, A_CTRL_REG5_G_VALUE);
	delay(20);

	writeReg(_CS_GYRO, FIFO_CTRL_REG_G, FIFO_CTRL_REG_G_VALUE);

	// Reset stuff
	rollError = 0;	pitchError = 0;	headingError = 0;
	rollX = 0;		pitchY = 0;	headingZ = 0;

}

void Motoruino2LSM9DS0::initAccel()
{
    // Accelerometer
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG0_XM, A_CTRL_REG0_XM_VALUE);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG1_XM, A_CTRL_REG1_XM_VALUE);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG2_XM, A_CTRL_REG2_XM_VALUE);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG3_XM, A_CTRL_REG3_XM_VALUE);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG4_XM, A_CTRL_REG4_XM_VALUE);
    delay(20);
    writeReg(_CS_ACCEL_MAG, FIFO_CTRL_REG, FIFO_CTRL_REG_VALUE);

}

void Motoruino2LSM9DS0::initMag()
{
	// Magnetometer
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG5_XM, A_CTRL_REG5_XM_VALUE);
	writeReg(_CS_ACCEL_MAG, A_CTRL_REG6_XM, A_CTRL_REG6_XM_VALUE);
	writeReg(_CS_ACCEL_MAG, A_CTRL_REG7_XM, A_CTRL_REG7_XM_VALUE);
	writeReg(_CS_ACCEL_MAG, INT_CTRL_REG_M, INT_CTRL_REG_M_VALUE);
}


void Motoruino2LSM9DS0::sleep()
{
	initializationEnded = 0;

	writeReg(_CS_ACCEL_MAG, INT_GEN_1_REG,		A_INT_GEN_1_REG);
	writeReg(_CS_ACCEL_MAG, INT_GEN_1_DURATION,	A_INT_GEN_1_DURATION);
	writeReg(_CS_ACCEL_MAG, INT_GEN_1_THS,		A_INT_GEN_1_THS);

	 // Gyro
	writeReg(_CS_GYRO, A_CTRL_REG1_G, A_CTRL_REG1_G_VALUE_SLEEP);
	writeReg(_CS_GYRO, A_CTRL_REG2_G, A_CTRL_REG2_G_VALUE_SLEEP);
	writeReg(_CS_GYRO, A_CTRL_REG3_G, A_CTRL_REG3_G_VALUE_SLEEP);
	writeReg(_CS_GYRO, A_CTRL_REG4_G, A_CTRL_REG4_G_VALUE_SLEEP);
	writeReg(_CS_GYRO, A_CTRL_REG5_G, A_CTRL_REG5_G_VALUE_SLEEP);
	delay(20);

	writeReg(_CS_GYRO, FIFO_CTRL_REG_G, FIFO_CTRL_REG_G_VALUE_SLEEP);

    // Accelerometer
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG0_XM, A_CTRL_REG0_XM_VALUE_SLEEP);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG1_XM, A_CTRL_REG1_XM_VALUE_SLEEP);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG2_XM, A_CTRL_REG2_XM_VALUE_SLEEP);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG3_XM, A_CTRL_REG3_XM_VALUE_SLEEP);
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG4_XM, A_CTRL_REG4_XM_VALUE_SLEEP);
    delay(20);
    writeReg(_CS_ACCEL_MAG, FIFO_CTRL_REG, FIFO_CTRL_REG_VALUE_SLEEP);

	// Magnetometer
    writeReg(_CS_ACCEL_MAG, A_CTRL_REG5_XM, A_CTRL_REG5_XM_VALUE_SLEEP);
	writeReg(_CS_ACCEL_MAG, A_CTRL_REG6_XM, A_CTRL_REG6_XM_VALUE_SLEEP);
	writeReg(_CS_ACCEL_MAG, A_CTRL_REG7_XM, A_CTRL_REG7_XM_VALUE_SLEEP);
	writeReg(_CS_ACCEL_MAG, INT_CTRL_REG_M, INT_CTRL_REG_M_VALUE_SLEEP);
}

void Motoruino2LSM9DS0::wake(bool _GYRO, bool _ACCEL, bool _MAG)
{
    if (_GYRO) initGyro();
    if (_ACCEL) initAccel();
    if (_MAG) initMag();

    initializationEnded = 1;
}


// ---------------------------------------------------------------------
// Gyroscope calibration - slide effect
// TODO: Review a better way to calibrate - EEPROM stuff

bool Motoruino2LSM9DS0::calibrateGyro()
{
	unsigned long start =  millis();
	unsigned char InitState = 0;

	// Ensures that the initialization flag is on to update Gyro...
	InitState = initializationEnded;
	initializationEnded = 1;

	if (IMU_DEBUG) Serial.println("CALIBRATING");

	// DUMY SAMPLES - Let stabilize the cap....
	calibrationSamples = CALIB_SAMPLES;
	start = millis();
	while ((calibrationSamples) && ((millis() - start < 20000)))
	{
		updateAccelerometer();
		updateGyroscope();
		updateMagnetometer();
	}

	// Reset stuff
	rollError = 0;	pitchError = 0;	headingError = 0;
	rollX = 0;		pitchY = 0;	headingZ = 0;

	// REAL SAMPLES
	calibrationSamples = CALIB_SAMPLES;
	start = millis();
	while ((calibrationSamples) && ((millis() - start < 20000)))
	{
		updateAccelerometer();
		updateGyroscope();
		updateMagnetometer();
	}

	// If not it has exited by timeout ...
	if (!calibrationSamples)
	{
		rollError = (rollX / (CALIB_SAMPLES));
		pitchError = (pitchY / (CALIB_SAMPLES));
		headingError = (headingZ / (CALIB_SAMPLES));
	}

	rollX = 0;		pitchY = 0;		headingZ = 0;

	if (IMU_DEBUG) Serial.println("END CALIBRATING");

	initializationEnded = InitState;

	if (!calibrationSamples)
		return true;
	else
		return false;
}
// ---------------------------------------------------------------------
// Magnetometer offset/scale alpha beta correction
bool Motoruino2LSM9DS0::calibrateMag()
{
	return false;
}

// ---------------------------------------------------------------------
// StartupAngle Kalman Calibration
bool Motoruino2LSM9DS0::calibrateAngle()
{
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
  		double roll  = atan2(getAccelAvgY(), getAccelAvgZ()) * RAD_TO_DEG;
  		double pitch = atan(-getAccelAvgX() / sqrt(getAccelAvgY() * getAccelAvgY() + getAccelAvgZ() * getAccelAvgZ())) * RAD_TO_DEG;
	#else // Eq. 28 and 29
  		double roll  = atan(getAccelAvgY() / sqrt(getAccelAvgX() * getAccelAvgX() + getAccelAvgZ() * getAccelAvgZ())) * RAD_TO_DEG;
  		double pitch = atan2(-getAccelAvgX(), getAccelAvgZ()) * RAD_TO_DEG;
	#endif

  	// Set starting angle
  	kalmanX.setAngle(roll);
  	kalmanY.setAngle(pitch);

 	gyroXangle = roll;
  	gyroYangle = pitch;

  	return true;
}


// ---------------------------------------------------------------------
// Function called by pooling on the main function
void Motoruino2LSM9DS0::updateGyroscope()
{

	if (!initializationEnded) return;

	readReg(_CS_GYRO, FIFO_SRC_REG_G, 1, fifo_src);

	if (fifo_src[0] & 0b01000000) if (IMU_DEBUG) Serial.println("OVF GYRO");

	// ** GET GYRO VALUES -> Test if FIFO is empty ***
	if ((fifo_src[0] & 0b00011111))
	{
		// Get Gyro
		readReg(_CS_GYRO, OUT_X_L_G, 6, bufferSPI);

		gyroRawX = ((bufferSPI[1] << 8) | bufferSPI[0]);
		gyroRawY = ((bufferSPI[3] << 8) | bufferSPI[2]);
		gyroRawZ = ((bufferSPI[5] << 8) | bufferSPI[4]);

		streamGyroX[StreamGyroPointer] = gyroRawX;
		streamGyroY[StreamGyroPointer] = gyroRawY;
		streamGyroZ[StreamGyroPointer++] = gyroRawZ;

		// Reached the end without update the Values
		// -> Record only the last Value and Discard All Others
		if (StreamGyroPointer > 32)
		{
			if (IMU_DEBUG) Serial.println("NO_GYRO");

			StreamGyroPointer = 1;
			streamGyroX[0] = gyroRawX;
			streamGyroY[0] = gyroRawY;
			streamGyroZ[0] = gyroRawZ;
		}

	}

	// ** PROCESS GYRO VALUES Process Buffer Recorded Data @ uC SAMPLE TIME FREQUENCY **
	if (StreamGyroPointer >= SAMPLES_CALC_G)
	{
		//Serial.print("\t G:");Serial.println((fifo_src[0] & 0b00011111));

		// Add all elements of the recorded Stream
		tempDataX = 0; tempDataY = 0; tempDataZ = 0;
		for (int i = StreamGyroPointer ; i--; )
		{
			tempDataX += streamGyroX[i];
			tempDataY += streamGyroY[i];
			tempDataZ += streamGyroZ[i];
		}

		// Divide by the Steam and update the Heading, Pitch and Roll
		tempDataX = tempDataX / StreamGyroPointer;
		tempDataY = tempDataY / StreamGyroPointer;
		tempDataZ = tempDataZ / StreamGyroPointer;

		rollX += (tempDataX * 0.92) - rollError;
		pitchY += (tempDataY * 0.92) - pitchError;
		headingZ += (tempDataZ * 0.92) - headingError;




		if (rollX > _GYRO_180_VALUE)
			rollX =  rollX - (long)_GYRO_180_VALUE*2;
		else if (rollX < -_GYRO_180_VALUE)
			rollX =  rollX + (long)_GYRO_180_VALUE*2;

		if (pitchY > _GYRO_180_VALUE)
			pitchY = pitchY - (long)_GYRO_180_VALUE*2;
		else if (pitchY < -_GYRO_180_VALUE)
			pitchY =  pitchY + (long)_GYRO_180_VALUE*2;

		if (headingZ > _GYRO_180_VALUE)
			headingZ =  headingZ - (long)_GYRO_180_VALUE*2;
		else if (headingZ < -_GYRO_180_VALUE)
				headingZ =  headingZ + (long)_GYRO_180_VALUE*2;

		headingZAngle = (long)(headingZ * 180 / _GYRO_180_VALUE);
		pitchYAngle = (long)(pitchY * 180 / _GYRO_180_VALUE);
		rollXAngle = (long)(rollX * 180 / _GYRO_180_VALUE);


		StreamGyroPointer = 0;

		calibrationSamples--;

    }
}


void Motoruino2LSM9DS0::updateAccelerometer()
{
	if (!initializationEnded) return;

	readReg(_CS_ACCEL_MAG, FIFO_SRC_REG, 1, fifo_src);

	if (fifo_src[0] & 0b01000000) if (IMU_DEBUG) Serial.println("OVF ACCEL");

	if ((fifo_src[0] & 0b00011111))
	{
		// Get Accel
		readReg(_CS_ACCEL_MAG, OUT_X_L_A, 6, bufferSPI);
		accelRawX = ((bufferSPI[1] << 8) | bufferSPI[0]);
		accelRawY = ((bufferSPI[3] << 8) | bufferSPI[2]);
		accelRawZ = ((bufferSPI[5] << 8) | bufferSPI[4]);

		streamAccelX[streamAccelPointer] = accelRawX;
		streamAccelY[streamAccelPointer] = accelRawY;
		streamAccelZ[streamAccelPointer++] = accelRawZ;

		// Reached the end without update the Values
		// -> Record only the last Value and Discard All Others
		if (streamAccelPointer > 32)
		{
			if (IMU_DEBUG) Serial.println("NO_ACCEL");

			streamAccelPointer = 1;
			streamAccelX[0] = accelRawX;
			streamAccelY[0] = accelRawY;
			streamAccelZ[0] = accelRawZ;
		}
	}

	// ** PROCESS ACCEL VALUES Process Buffer Recorded Data @ uC SAMPLE TIME FREQUENCY **
	if (streamAccelPointer >= SAMPLES_CALC_A)
	{

		// Add all elements of the recorded Stream
		tempDataX = 0; tempDataY = 0; tempDataZ = 0;
		for (int i = streamAccelPointer; i--; )
		{
			tempDataX += streamAccelX[i];
			tempDataY += streamAccelY[i];
			tempDataZ += streamAccelZ[i];
		}

		// Divide by the Steam and update the Heading, Pitch and Roll
		accelXAvg = (float) tempDataX / streamAccelPointer;
		accelYAvg = (float) tempDataY / streamAccelPointer;
		accelZAvg = (float) tempDataZ / streamAccelPointer;

		accelXAvg =  (accelXAvg * 2)/32768;
		accelYAvg =  (accelYAvg * 2)/32768;
		accelZAvg =  (accelZAvg * 2)/32768;

		streamAccelPointer = 0;

    }
}


void Motoruino2LSM9DS0::updateKalman()
{
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  	timer = micros();

//	#ifdef RESTRICT_PITCH // Eq. 25 and 26
//  		double roll  = atan2(getAccelAvgY(), getAccelAvgZ()) * RAD_TO_DEG;
//  		double pitch = atan(-getAccelAvgX() / sqrt(getAccelAvgY() * getAccelAvgY() + getAccelAvgZ() * getAccelAvgZ())) * RAD_TO_DEG;
//	#else // Eq. 28 and 29
//  		double roll  = atan(getAccelAvgY() / sqrt(getAccelAvgX() * getAccelAvgX() + getAccelAvgZ() * getAccelAvgZ())) * RAD_TO_DEG;
//  		double pitch = atan2(-getAccelAvgX(), getAccelAvgZ()) * RAD_TO_DEG;
//	#endif

  	// Changes made:
  	double roll  = atan2(getAccelAvgY(), getAccelAvgZ()) * RAD_TO_DEG;
  	double pitch = atan2(-getAccelAvgX(), getAccelAvgZ()) * RAD_TO_DEG;

  	double gyroXrate = getGyroRoll() / 131.0; // Convert to deg/s
  	double gyroYrate = GetGyroPitch() / 131.0; // Convert to deg/s

//#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
//#else

  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
//#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

}


void Motoruino2LSM9DS0::updateMagnetometer()
{
	if (!initializationEnded) return;

	readReg(_CS_ACCEL_MAG, STATUS_REG_M , 1, fifo_src);

	// ** GET ACCEL VALUES -> Test if FIFO is empty ***
	if ((fifo_src[0] & 0b00001000))
	{
		// Get Mag
		readReg(_CS_ACCEL_MAG, OUT_X_L_M, 6, bufferSPI);
		magRawX = ((bufferSPI[1] << 8) | bufferSPI[0]);
		magRawY = ((bufferSPI[3] << 8) | bufferSPI[2]);
		magRawZ = ((bufferSPI[5] << 8) | bufferSPI[4]);

		magRawX =  (magRawX * 2)/32768;
		magRawY =  (magRawY * 2)/32768;
		magRawZ =  (magRawZ * 2)/32768;

	}
}


// Scale to same values as in Gyrosocope
float Motoruino2LSM9DS0::getMagHeading()
{
	float heading;
	if (magRawY > 0)
	{
		heading = 90 - (atan(magRawX / magRawY) * (180 / PI));
	}
	else if (magRawY < 0)
	{
		heading = - (atan(magRawX / magRawY) * (180 / PI));
	}
	else // hy = 0
	{
	if (magRawX < 0)
		heading = 180;
	else
		heading = 0;
	}

//	// Calculate the angle of the vector y,x
//	  float heading = (atan2(MagRaw_y,MagRaw_x) * 180) / PI;
//
//	  // Normalize to 0-360
//	  if (heading < 0)
//	  {
//	    heading = 360 + heading;
//	  }

	return (float) heading;
}
float Motoruino2LSM9DS0::getMagPitch()
{
	float pitch;

	pitch = atan2(magRawX, sqrt(magRawY * magRawY) + (magRawZ * magRawZ));
	pitch *= 180.0 / PI;
	return (float) pitch;
}
float Motoruino2LSM9DS0::getMagRoll()
{
	float  roll;
	roll = atan2(magRawY, sqrt(magRawX * magRawX) + (magRawZ * magRawZ));
	roll *= 180.0 / PI;
	return (float) roll;
}

// Get the current heading, pitch and roll for motoruino (in degrees) -> Scaled the values (div 68)
short Motoruino2LSM9DS0::getGyroHeading(){	return headingZAngle;}
short Motoruino2LSM9DS0::GetGyroPitch(){	return pitchYAngle;}
short Motoruino2LSM9DS0::getGyroRoll(){		return rollXAngle;}

// Get the current Accelerometer calculated values
float Motoruino2LSM9DS0::getAccelAvgY(){	return (float) accelYAvg;}
float Motoruino2LSM9DS0::getAccelAvgX(){	return (float) accelXAvg;}
float Motoruino2LSM9DS0::getAccelAvgZ(){	return (float) accelZAvg;}

// Get the current kalman filter inclination and Orientation degrees
float Motoruino2LSM9DS0::getAngleY(){		return (float) kalAngleY;}
float Motoruino2LSM9DS0::getAngleX(){ 		return (float) kalAngleX;}

// Resets the current heading referential
void Motoruino2LSM9DS0::resetGyro(bool _Heading, bool _Pitch, bool _Roll)
{
	if (_Roll) rollX = 0;
	if (_Pitch) pitchY = 0;
	if (_Heading) headingZ = 0;
}

// Get the raw values from the accelerometer and gyroscope for the different axis
short Motoruino2LSM9DS0::getAccelerometerX(){	return accelRawX;}
short Motoruino2LSM9DS0::getAccelerometerY(){	return accelRawY;}
short Motoruino2LSM9DS0::getAccelerometerZ(){	return accelRawZ;}

short Motoruino2LSM9DS0::getGyroscopeX(){		return gyroRawX;}
short Motoruino2LSM9DS0::getGyroscopeY(){		return gyroRawY;}
short Motoruino2LSM9DS0::getGyroscopeZ(){		return gyroRawZ;}

short Motoruino2LSM9DS0::getMagnetometerX(){		return magRawX;}
short Motoruino2LSM9DS0::getMagnetometerY(){		return magRawY;}
short Motoruino2LSM9DS0::getMagnetometerZ(){		return magRawZ;}

unsigned short Motoruino2LSM9DS0::getTemp()
{
	unsigned char temp[2]; // We'll read two bytes from the temperature sensor into temp
	readReg(_CS_ACCEL_MAG, OUT_TEMP_L_XM,  2, temp); // Read 2 bytes, beginning at OUT_TEMP_L_M
	return ((((unsigned short) temp[1] << 8) | temp[0] ) & 0x0FFF);
}

// Aux functions for the SPI Comm to the Chip
void Motoruino2LSM9DS0::writeReg(int cs_pin, unsigned char reg, unsigned char value)
{
    reg = reg & 0x7F;               // bit7-0 (write)
    reg = reg & 0xBF;               // bit6-0 (address not-inc)

    digitalWrite(cs_pin, LOW);
    spi.transfer(reg);
    spi.transfer(value);
    digitalWrite(cs_pin, HIGH);
}

void Motoruino2LSM9DS0::readReg(int cs_pin, unsigned char reg, unsigned char len, unsigned char *data)
{
	unsigned char i;

    reg = reg | 0x80;               // bit7-1 (read)
    reg = reg | 0x40;               // bit6-1 (address auto-inc)

    digitalWrite(cs_pin, LOW);
    Motoruino2SSPI::transfer(reg);

    for (i = len; i--; ){
        *data = spi.transfer(0x00);
        data ++;
    }

    digitalWrite(cs_pin, HIGH);
}

