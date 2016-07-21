/*
 Artica CC - http://artica.cc

 Motoruino 2 Slave IMU functions Interface Lib

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Creation of IMU Slave Interface Libs

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include "Motoruino2IMU.h"
#include "Motoruino2Comm.h"
#include "Motoruino2.h"

// ---------------------------------------------------------------------
// Initialization and configuration


// Default constructor
Motoruino2IMU::Motoruino2IMU()
{
	Serial.begin(115200);
	motoruino2Comm.commConfig();
	_initialized = true;
}

bool Motoruino2IMU::getTemperature(short * data)
{
	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyShort(GTMP, data);
}

// ----------------------------- Config IMU----------------------------------
bool Motoruino2IMU::config_imu(bool Gyro, bool Accel, bool Mag)
{

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) Gyro);		// Gyro ON
	motoruino2Comm.addCharData( (char) Accel);		// Accel ON
	motoruino2Comm.addCharData( (char) Mag);		// Mag ON

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(STTI);
}

// ----------------------------- CALIBRATE IMU FUNCTIONS ----------------------------------
bool Motoruino2IMU::calibrate(enum _ReceivedOrder cmd, int delay)
{

	// Prepare the Parameters
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	// set the command
	motoruino2Comm.commBuff[0] = (char) cmd;

	// Send the Command and expect 2 bytes
	if (!motoruino2Comm.sendData(motoruino2Comm.commBuff, delay, motoruino2Comm.commBuff, 1 + 1)) return false;

	// Validate the received message
	if ((enum _ReceivedOrder) motoruino2Comm.commBuff[0] != cmd) return false;

	if ((motoruino2Comm.commBuff[1] != (char) true) ) return false;

	return true;
}

bool Motoruino2IMU::calibrateGyro()
{
	return calibrate(CALG, 7000);
}

bool Motoruino2IMU::calibrateAngle()
{
	return calibrate(CALA, 0);
}

bool Motoruino2IMU::calibrateMag()
{
	return calibrate(CALM, 6500);
}

// ----------------------------- RESET GYROSCOPE ----------------------------------
bool Motoruino2IMU::resetGyro(bool _heading, bool _pitch, bool _roll)
{
	char param = 0;

	param = ((_heading)? 0x01 : 0x00) |  ((_pitch)? 0x02 : 0x00) | ((_roll)? 0x04 : 0x00);

	// Prepare the Params
	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	motoruino2Comm.addCharData( (char) param);		// Mag ON

	// Send it and validate reply
	return motoruino2Comm.sendDataReplyBool(RSTG);
}

// ----------------------------- GET MAGNETOMETER HEADING/PITCH/ROLL ----------------------------------
float Motoruino2IMU::getMagHeading()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GMAH, &reply))
		return reply;
	else
		return 0xFFFF;
}

float Motoruino2IMU::getMagPitch()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GMAP, &reply))
		return reply;
	else
		return 0xFFFF;

}

float Motoruino2IMU::getMagRoll()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GMAR, &reply))
		return reply;
	else
		return 0xFFFF;

}


// ----------------------------- GET GYRO HEADING/PITCH/ROLL ----------------------------------
short Motoruino2IMU::getGyroHeading()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYH, &reply))
		return reply;
	else
	{
		return 0xFFFF;
	}
}

short Motoruino2IMU::getGyroPitch()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYP, &reply))
		return reply;
	else
		return 0xFFFF;

}
short Motoruino2IMU::getGyroRoll()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYR, &reply))
		return reply;
	else
		return 0xFFFF;
}


// ----------------------------- GET ACCELEROMETER AVG ----------------------------------
float Motoruino2IMU::getAccelAvgX()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GAVX, &reply))
		return reply;
	else
		return 0xFFFF;
}

float Motoruino2IMU::getAccelAvgY()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GAVY, &reply))
		return reply;
	else
		return 0xFFFF;

}

float Motoruino2IMU::getAccelAvgZ()
{
	float reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GAVZ, &reply))
		return reply;
	else
		return 0xFFFF;

}

// ----------------------------- GET ANGLE ----------------------------------
float Motoruino2IMU::getAngleX()
{
	float reply = 0;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GAGX, &reply))
		return reply;
	else
		return 0xFFFF;
}

float Motoruino2IMU::getAngleY()
{
	float reply = 0;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyFloat(GAGY, &reply))
		return reply;
	else
		return 0xFFFF;

}



// ----------------------------- GET ACCELEROMETER XYZ  ----------------------------------
// Get the raw values from the accelerometer and gyroscope for the different axis
short Motoruino2IMU::getAccelerometerX()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GACX, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getAccelerometerY()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GACY, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getAccelerometerZ()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GACZ, &reply))
		return reply;
	else
		return 0xFFFF;
}

// ----------------------------- GET GYROSCOPE XYZ  ----------------------------------
short Motoruino2IMU::getGyroscopeX()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYX, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getGyroscopeY()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYY, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getGyroscopeZ()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GGYZ, &reply))
		return reply;
	else
		return 0xFFFF;
}

// ----------------------------- GET MAGNETOMETER XYZ  ----------------------------------
short Motoruino2IMU::getMagnetometerX()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GMAX, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getMagnetometerY()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GMAY, &reply))
		return reply;
	else
		return 0xFFFF;
}

short Motoruino2IMU::getMagnetometerZ()
{
	short reply;

	motoruino2Comm.startNewComm(ADRRESS_SLAVE);

	if (motoruino2Comm.sendDataReplyShort(GMAZ, &reply))
		return reply;
	else
		return 0xFFFF;
}



