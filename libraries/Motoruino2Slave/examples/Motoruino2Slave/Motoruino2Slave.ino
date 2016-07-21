/*
 Artica CC - http://artica.cc

 MotoruinoSlave lib

 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!! PROGRAM ONLY WITH 3V3 FTDI !!!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!! Arduino Pro Mini 3.3V 8MHz !!!!!!!!!!!!!!!!!!
 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

#include "Metro.h"
#include "Wire.h"
#include "TimerOne.h"

#include "Motoruino2SProtocol.h"
#include "Motoruino2SPINS.h"
#include "Motoruino2LSM9DS0.h"
#include "Motoruino2SMotor.h"
#include "Motoruino2SPower.h"
#include "Motoruino2SSPI.h"
#include "Motoruino2SPinChangeInt.h"

#define SERIAL_DEBUG	true

// ------------------------- DEFINES -------------------------
#define BLINKTIME 			500	// Printout Debug Flag (ms)
#define NAVIGATIONTIME 		10		// Pooling timer for navigation (ms)
#define COMMTIME 		10		// Pooling timer for navigation (ms)

#define BUFFER_SIZE 	10			// I2C Buffer size
#define WIRE_ADDRESS 	2			// Slave I2C address

#define WHEEL_DISTANCE	110		// in mm


// Metro Update
Metro MetroUpdateTime = Metro(BLINKTIME);
Metro MetroNavigationTime = Metro(NAVIGATIONTIME);
Metro MetroCommTime = Metro(COMMTIME);

// Wire Comm
char receivedBuffer[BUFFER_SIZE];
char sendBuffer[BUFFER_SIZE];
unsigned char sendLength = 0;

// Pooling Flags
bool readyFlagToReply = false;			// Finished processing the reply?

// Navigation
bool startNavigateToAngle = false;		// Goto Angle? (IMU)
bool startNavigateToDistance = false;	// Goto Distance? (ENCODERS)
bool startRotateToAngle = false; 		// Rotate Angle? (ENCODERS)
bool startReachEncoderAngle = false;

short goalAngle = 0;
short goalSpeed = 0;
long goalDistance = 0;
short goalSpeedl = 0;
short goalSpeedr = 0;


// Aux Global Vars
long Data = 0;
enum Motoruino2SMotor::EncoderMode mode;
short DataShortAux;
float DataFloatAux;


// Function Prototypes
////void commConfig(unsigned int address);
//
///* Clear the reset flag. */
//MCUSR &= ~(1<<WDRF);
//
///* In order to change WDE or the prescaler, we need to
// * set WDCE (This will allow updates for 4 clock cycles).
// */
//WDTCSR |= (1<<WDCE) | (1<<WDE);
//
///* set new watchdog timeout prescaler value */
//WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
//
///* Enable the WD interrupt (note no reset). */
//WDTCSR |= _BV(WDIE);

// --------------------- SETUP ----------------------
void setup() {

	// Brown out Voltage Off
//   MCUCR |= (1<<BODS) | (1<<BODSE);
//   MCUCR &= ~(1<<BODSE);  // must be done right before sleep

	if (SERIAL_DEBUG)
		Serial.begin(115200);

	// needed to connect to IMU
	spi.config();

	// Communication Configuration
	commConfig();

	// Object Configurations
	motor.config(-1, 1);

	// Set Sleep Mode and OutputPower Pins
	power.config();

	//if (SERIAL_DEBUG) Serial.println(":calibrating...");
	imuLSM9DS0.config(true, true, true);
	//imuLSM9DS0.calibrateGyro();
	//imuLSM9DS0.calibrateAngle();

	// Pools/Flags init
	startNavigateToAngle = false;// Flag that indicates if the motoruino is currently heading to an angle
	startNavigateToDistance = false;// Flag that indicates if the motoruino is currently moving to a distance
	readyFlagToReply = false;// Flag indicates that there are messages to the Master

	// Main Values init
	goalAngle = 0;
	goalSpeed = 0;
	Data = 0;								// Aux Data

	motor.setPWMMotorSpeed(0, 0);
	//motor.setPWMMotorSpeed(255,255);
	//motor.startEncoders(Motoruino2SMotor::FAST_ENCODER, 300, 4.75);
	//motor.setRPMMotorSpeed(30, 30);

	// set Motor 1 20
	//motor.SetSpeed(1,-20);
	//motor.SetSpeed(2,-20);

	imuLSM9DS0.startFlagIMU = true;

	if (SERIAL_DEBUG)
		Serial.println(":started");

}

bool outstate = false;

float oldX = 0.0;
float oldY = 0.0;
float aux = 0.0;

// --------------------- LOOP ----------------------
void loop() {

	wdt_reset();

	// outstate = !outstate;
	// digitalWrite(GP_OUT, outstate);

	// gotoSleep();

	if (digitalRead(SLEEP_CTL) == HIGH) {

		// POOLING Functions
		// - IMU
		if (imuLSM9DS0.startFlagIMU) {
			imuLSM9DS0.updateAccelerometer();
			UpdateComm();
			imuLSM9DS0.updateGyroscope();
			UpdateComm();
			imuLSM9DS0.updateMagnetometer();
			UpdateComm();
			imuLSM9DS0.updateKalman();
		}

		UpdateComm();

		// - Navigation
		if (MetroNavigationTime.check() == 1) {

			if ((startNavigateToAngle) && (imuLSM9DS0.startFlagIMU))
				if (ReachAngle(goalAngle, motor.pwmGoalSpeed))
					startNavigateToAngle = false;

			UpdateComm();

			if ((startNavigateToDistance) && (motor.isEncoderDefined))
				if (ReachDistance(goalDistance, motor.pwmGoalSpeed))
					startNavigateToDistance = false;

//			UpdateComm();
//
//			if ((startRotateToAngle) && (motor.isEncoderDefined))
//				if (RotateAngle(goalAngle, motor.pwmGoalSpeed))
//					startRotateToAngle = false;

			UpdateComm();

			if ((startReachEncoderAngle) && (motor.isEncoderDefined))
				if (ReachEncoderAngle(motor.angle, motor.angleLeftMotorPwm, motor.angleRightMotorPwm))
				{
					startReachEncoderAngle = false;
				}




		}

		UpdateComm();

		if (motor.motorBlocked())
			Serial.println("B");

		// TEST Functions to print
		if (MetroUpdateTime.check() == 1) {

//		Serial.print("ADC:");
			//	Serial.println(motor.getADC());

 // 		Serial.print("H:"); Serial.print(imuLSM9DS0.getGyroHeading());Serial.print("  ");
//		Serial.print("M:"); Serial.print(imuLSM9DS0.getMagHeading());Serial.print("  ");
//		Serial.print("T:"); Serial.print(imuLSM9DS0.getTemp());
//
//	  	Serial.print("|\t");
//
//
//
//		Serial.print("A:");Serial.print(imuLSM9DS0.getAngleX()); Serial.print("\t");
//	  	Serial.print(imuLSM9DS0.getAngleY()); Serial.print("\t");

//		aux = imuLSM9DS0.getAccelAvgX();
//		if ((aux - oldX)>0.01)
//		{
//			Serial.println("FRONT");
//		} else if ((aux - oldX)<-0.01)
//		{
//			Serial.println("BACK");
//		}
//	  	oldX = aux;
//
//	  	aux = imuLSM9DS0.getAccelAvgY();
//	  	if ((aux - oldY)>0.01)
//		{
//	  		Serial.println("LEFT");
//		} if ((aux - oldY)<-0.01)
//		{
//			Serial.println("RIGHT");
//		}
//		oldY = aux;

//	  	Serial.print("|\t");
//
//	Serial.print(motor.distance1);Serial.print(" ");
//		Serial.print(motor.distance2);Serial.print(" ");

//					Serial.print(motor.frequency1);Serial.print("\t ");
//			 Serial.print(motor.desiredFrequency1);Serial.print("\t ");
//			 Serial.print(motor.motorSpeed1);Serial.print("\t ");
//
//			 Serial.print(motor.distance1);Serial.print("\t ");
//			 Serial.print(motor.distance2);Serial.print("\t ");
			 /*
			 Serial.println("");
			 motor.setPWMMotorSpeed(255,255);*/
//
			//motor.ResetDistance(1);
			//motor.ResetDistance(2);
//			Serial.println();
		}

	} else {
		if (MetroUpdateTime.check() == 1)
			Serial.println("Low Power");
	}

}

// POWER FUNCTIONS

volatile void gotoSleep() {

	// Send everything to Low Power
	digitalWrite(SLEEP_CTL, LOW);

	// Send IMU to Sleep (interrupt pin on WAKE_PIN)
	imuLSM9DS0.sleep();

//	cli();

	// Enable global Sleep Pin
	sleep_enable();

	// Attach the Function to Wakeup - TODO WDT interrupt
	attachPinChangeInterrupt(WAKE_PIN, wake_func, CHANGE);

	// Set the most efficient mode
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);

	// Brown out disable
//	sleep_bod_disable();
//	sei();

	// GOTO Sleep
	sleep_cpu();

	/* wake up here */
	sleep_disable();

	// Turn off the Wake-Up Pin Interrupt
	detachPinChangeInterrupt(WAKE_PIN);

	//re-enable periferals
	power_all_enable();

	// wake-up the IMU
	imuLSM9DS0.config();

	Serial.print(" X: ");
	Serial.print(power.flagPinChange);
	Serial.print(" W: ");
	Serial.println(power.flagWDT);

}

// -----------------------------------------------------------
// ------ ISR Function Wake-up Function ----------------------
// -				ON PIN CHANGE
// -----------------------------------------------------------
void wake_func(void) {
	if (power.flagPinChange == 0) {
		power.flagPinChange = 1;
	} else {
		Serial.println("Pin Change without sleep");
	}
	sleep_disable();
	PCintPort::detachInterrupt(WAKE_PIN);
	Serial.println("CHANGE");
}
// -----------------------------------------------------------
// ------ ISR Function Wake-up Function ----------------------
// -				WDT
// -----------------------------------------------------------
ISR (WDT_vect)
{
	if(power.flagWDT == 0)
	{
		power.flagWDT = 1;
	}
	else
	{
		Serial.println("WDT Overrun!!!");
	}
}

