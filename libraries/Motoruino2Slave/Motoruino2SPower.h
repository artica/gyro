/*
 Artica CC - http://artica.cc

 MotoruinoSlave Power lib


 It will control the Sleep and Wake functions and also measure the battery level

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

#ifndef Motoruino2SPower_h
#define Motoruino2SPower_h

#include "Arduino.h"
#include "Motoruino2SPINS.h"


class Motoruino2SPower
{

	public:
		// Constructor
		Motoruino2SPower();

		// Config
		void config();

		// Get Battery Value
		short getBattery();

		// Set Generic Output PWM
		void setOutput(unsigned char pwmValue);
		unsigned char getOutput();

		static volatile void sleep();

		static volatile char flagWDT;
		static volatile char flagPinChange;

	private:

		volatile unsigned char outputValue;

};

extern Motoruino2SPower 	power;		// Power Functions

#endif


