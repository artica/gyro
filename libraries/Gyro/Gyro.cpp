
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


#include "Gyro.h"

Gyro::Gyro()
{
	// @TODO inicializar as comunicaçes I2C (v2.0)
	// @TODO inicializar Bumpers (v2.0)
	// @TODO inicializar IRSensors (v2.0)
	// @TODO inicializar ServoControls (v2.0)

}

/*
Gyro::Gyro(int maximumServo, int minimumServo, int delServo, int incServo, int servoPin, bool typeServo, int bumperLeftPin, int bumperRightPin, int irPin, int motorA0, int motorA1, int motorB0, int motorB1)
{
	neck = ServoRange(maximumServo, minimumServo, delServo, incServo, servoPin, typeServo);
	bumpers = Bumpers(bumperLeftPin, bumperRightPin);
	distanceSensor = IR(irPin);
	motors = DifferentialMotor(motorA0, motorA1, motorB0, motorB1);
}

*/
