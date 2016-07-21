/*
 Artica CC - http://artica.cc

 Gyro Bumper "Master Wire" Lib
 - Communicates with I2C/Wire with the Bumper Gyro PCB
 - Uses 2 APDS-9960 Sensors on Front (Gyro eyes - Left and Right)
 - Uses 1 APDS-9960 Sensor on bottom (Center Bottom Sensor)
 - Uses 2 Line Sensors (Left and Right) QRE1113-Analog from Sparkfun
 - Have one RGB Led on bottom

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef GyroBumper_h
#define GyroBumper_h

#include "Arduino.h"

#include "BumperProtocol.h"

#define BUMPER_ADDRESS 	4			// Slave I2C address


class GyroBumper
{
	public:
		// ---------------------------------------------------------------------
		// Initialization and configuration
		GyroBumper();

		// Configure bumpers
		//void begin( /* TODO: Set flags for low consumption */ );
		void begin(	_BumperConfig lconfig,
					_BumperConfig cconfig,
					_BumperConfig rconfig );

		// ---------------------------------------------------------------------
		// Bumper sensor methods

		void setTriggerBumper(unsigned char trigg);

		bool getLeftProximity(unsigned short * proximity);
		bool getLeftGesture(_GestureState * gestureState);
		bool getLeftRed(unsigned short * red);
		bool getLeftGreen(unsigned short * green);
		bool getLeftBlue(unsigned short * blue);
		bool getLeftAmbient(unsigned short * ambient);
		bool getLeftLine(unsigned short * line);
		bool setLeftLED(unsigned char value);
		bool CheckLeftBumper();

		bool getRightProximity(unsigned short * proximity);
		bool getRightGesture(_GestureState * gestureState);
		bool getRightRed(unsigned short * red);
		bool getRightGreen(unsigned short * green);
		bool getRightBlue(unsigned short * blue);
		bool getRightAmbient(unsigned short * ambient);
		bool getRightLine(unsigned short * line);
		bool setRightLED(unsigned char value);
		bool CheckRightBumper();

		bool getCenterProximity(unsigned short * proximity);
		bool getCenterGesture(_GestureState * gestureState);
		bool getCenterRed(unsigned short * red);
		bool getCenterGreen(unsigned short * green);
		bool getCenterBlue(unsigned short * blue);
		bool getCenterAmbient(unsigned short * ambient);
		bool setCenterLED(unsigned char value);

		bool setRGB(unsigned char red, unsigned char green, unsigned char blue);

		
	private:

		enum _ReceivedOrderBumper OrderRequestBumper;
		enum _ProcessLaterOrdersBumper CommReplyLaterBumper;
		enum _GestureState	GestureState;

		bool _initialized;

		unsigned short trigger;

};

#endif
