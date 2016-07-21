/*
 Artica CC - http://artica.cc

 Gyro IR Sensor Lib To use with Sharp IR sensor

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */


#ifndef GyroIRSensor_h
#define GyroIRSensor_h
#include "Arduino.h"

#define _DEFAULT_IR_PIN A0

class GyroIRSensor
{
	public:
		// ---------------------------------------------------------------------
		// Initialization and configuration
		// Default constructor, uses analog pin FARRUSCO_DEFAULT_IR_PIN as default
		GyroIRSensor();
		// Custom constructor, uses a specific analog pin to read the data
		GyroIRSensor(int pinIR);
		// Configure a specific pin for the IR sensor
		void Begin(int pinIR);

		// ---------------------------------------------------------------------
		// IR sensor methods
		// Get the raw value read form the IR sensor
		int GetValue();
		// Get the distance (in centimetres) read from the IR sensor
		float GetDistance();
		
	private:
		// Configures all the necessary variables for the class
		void configure( int pinIR );
		// Initializes the input pin for the IR sensor
		void initialize();
		
		// Pin used for the IR sensor
		int _pinIR;
		// Is the sensor initialized?
		// The initialization occurs when Begin is called, or when it is used the first time
		bool _initialized;
};

#endif
