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

#include "GyroIRSensor.h"

// ---------------------------------------------------------------------
// Initialization and configuration

// Default constructor, uses analog pin 0 as default
GyroIRSensor::GyroIRSensor()
{
	configure(_DEFAULT_IR_PIN);
}
 
// Custom constructor, uses a specific analog pin to read the data
GyroIRSensor::GyroIRSensor(int pin)
{
	configure(pin);
}

// Initialize the IR sensor in the assigned pin
void GyroIRSensor::Begin(int pinIR)
{
	configure(pinIR);
}

// Configure the necessary values to initialize the IRSensor class
void GyroIRSensor::configure( int pinIR )
{
	_initialized = false;
	// Base configuration for the ir sensor
	_pinIR = pinIR;
}

// Initializes all the necessary pins so they can be used properly
void GyroIRSensor::initialize()
{	
	// Declare the IR pin as Input
	pinMode(_pinIR, INPUT);
	_initialized = true;	
}

//Returns the value read by the distance sensor
int GyroIRSensor::GetValue()
{
	// Make sure everything is initialized before trying to read data 
	if (!_initialized) initialize();
	return(analogRead(_pinIR));
}

//Returns the value read by the distance sensor in centimeters
float GyroIRSensor::GetDistance()
{
	// Calculates the distance based on the analog value read
	float distance = (4800/(GetValue()-20));
	return(distance);
}
