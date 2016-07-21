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

#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/power.h>


#include "Motoruino2SPINS.h"
#include "Motoruino2SPower.h"
#include "Motoruino2LSM9DS0.h"




Motoruino2SPower 	power;		// Power Functions

volatile char Motoruino2SPower::flagWDT = 0;
volatile char Motoruino2SPower::flagPinChange = 0;

void wake_func(void);

Motoruino2SPower::Motoruino2SPower()
{
	config();
}

void Motoruino2SPower::config()
{

	outputValue = 0;

	flagWDT = 0;
	flagPinChange = 0;

	// OUTPUT PIN as OFF
	pinMode(GP_OUT, OUTPUT);
	analogWrite(GP_OUT, 0);

	// PERIFERAL SUPPLY Turned ON (5V to devices) - Sleep Pin
	pinMode(SLEEP_CTL, OUTPUT);
	digitalWrite(SLEEP_CTL, HIGH);

	// WAKE UP PIN
	pinMode(WAKE_PIN, INPUT_PULLUP);

}


short Motoruino2SPower::getBattery()
{
	return analogRead(BATTERY_ADC);
}

void Motoruino2SPower::setOutput(unsigned char pwmValue)
{
	outputValue = pwmValue;
	analogWrite(GP_OUT, outputValue);
}

unsigned char Motoruino2SPower::getOutput()
{
	return outputValue;

}








