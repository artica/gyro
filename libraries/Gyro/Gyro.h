/*
 Artica CC - http://artica.cc

 Gyro Compelte Robot Lib:
  - Motoruino2 Board
  - BumperGyro Board
  - IRSensor Included (if needed to use with Sharp Sensor)
  - GyroSerialCommand Included (if needed to use with Bluetooth and Android App)
  - GyroServoControl Inlcuded (if needed to use with servomotors)

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */


#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"
#include <Motoruino2.h>

#include "GyroBumper.h"
#include "GyroIRSensor.h"
#include "GyroServoControl.h"
#include "GyroSerialCommand.h"


class Gyro
{ 
	public:
		// Default constructor
		// This creates a Gyro object with the default configuration
		Gyro();

		// Different components that form a Gyro
		GyroBumper bumpers;
		Motoruino2 motoruino2;
		GyroIRSensor distanceSensor;
		GyroServoControl neck;
};

#endif
