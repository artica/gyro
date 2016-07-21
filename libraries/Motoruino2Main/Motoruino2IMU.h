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

#ifndef _MOTORUINO2IMU_H_
#define _MOTORUINO2IMU_H_

#include "Arduino.h"
#include "Motoruino2Protocol.h"


class Motoruino2IMU
{
	public:
		// ---------------------------------------------------------------------
		// Initialization and configuration
		// Default constructor
		Motoruino2IMU() ;

		bool config_imu(bool Gyro = true, bool Accel = true, bool Mag = true);

		bool calibrateGyro();
		bool calibrateAngle();
		bool calibrateMag();

		// ---------------------------------------------------------------------
		// Calculated Data Values

		float getMagHeading();
		float getMagPitch();
		float getMagRoll();

		bool resetGyro(bool _heading, bool _pitch, bool _roll);
		short getGyroHeading();
		short getGyroPitch();
		short getGyroRoll();

		float getAccelAvgX();
		float getAccelAvgY();
		float getAccelAvgZ();

		float getAngleX();
		float getAngleY();

		// Get the raw values from the accelerometer and gyroscope for the different axis
		short getAccelerometerX();
		short getAccelerometerY();
		short getAccelerometerZ();

		short getGyroscopeX();
		short getGyroscopeY();
		short getGyroscopeZ();

		short getMagnetometerX();
		short getMagnetometerY();
		short getMagnetometerZ();

		// Get temperature
		bool getTemperature(short * data);

	private:

		bool calibrate(enum _ReceivedOrder cmd, int delay);
		int debug_var;
		bool _initialized;

};
#endif
