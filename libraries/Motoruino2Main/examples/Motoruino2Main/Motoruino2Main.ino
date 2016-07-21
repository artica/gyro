/*
 Artica CC - http://artica.cc

 Motoruino 2 Main Libs

 V0.0 - Serras and Tarquinio - 23/11/2015
 	 Made to test the Motoruino 2 Board and corresponding included sensors

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
#include <Wire.h>
#include <Metro.h>
#include <Servo.h>

#include "Motoruino2Protocol.h"
#include "Motoruino2Comm.h"
#include "Motoruino2IMU.h"
#include "Motoruino2Motor.h"
#include "Motoruino2Power.h"
#include "Motoruino2.h"

#define ADRRESS_SLAVE 2


#define LED13 13

// Define the rotation (chose between 1 or -1)
#define M1_SIGNAL -1
#define M2_SIGNAL 1

// The Motoruino Object
Motoruino2 motoruino;

long longvar;
long start_timeout;
int Side = 1;

// the setup routine runs once when you press reset:
void setup() {    

	// Config the BaudRate for Debug USB Port and Comm Port (Serial1)
	Serial.begin(115200);
	Serial1.begin(9600);

	Serial.println("Main Started");

	// Define Major Pins
	pinMode(LED13, OUTPUT);

	// global Timeout initialization
	start_timeout = 0;

	// Wait for secondary to start
	delay(100);

	// motoruino Configuration
	motoruino.config_imu(true, true, true);
	motoruino.config_motor(M1_SIGNAL,M2_SIGNAL);
	motoruino.config_power();

	motoruino.setSpeedPWM(0,0);

	// Calibrate the Gyroscope for correct readings, and wait for reply
	digitalWrite(LED13, LOW);

	if (motoruino.calibrateGyro())
		Serial.println("Calibrated OK");
	else
		Serial.println("Not Calibrated");

	motoruino.calibrateAngle();

	digitalWrite(LED13, HIGH);

	motoruino.setSpeedPWM(255,255);

	// Current Limit for Motor
	motoruino.startMotorTrigger(30);

	// Wait
	delay(1000);

}

unsigned char aux = 0;
unsigned short value = 0;

// the loop routine runs over and over again forever:
void loop() {

	delay(100);

	// Gyroscope Turn to Angle
	//motoruino.turnToDegree(-100,255);

	// Motor current Value
	motoruino.getCurrentValue(&value);
	Serial.print(value);Serial.print('\t');

	// Gyroscope Heading
	Serial.println(motoruino.getGyroHeading());

	motoruino.getAngleX();

//	Motoruino.Motors.TurnTo( 0, 255);

//	start_timeout = millis();
//	while (millis() - start_timeout < 2000)
//	{
//		Motoruino.Motors.TurnTo( 0, 255);
//		delay(10);
//		Serial.println(Motoruino.IMU.GetHeading());
//	}

//	start_timeout = millis();
//	while (millis() - start_timeout < 2000)
//	{
//		Motoruino.Motors.TurnTo( Side * -9000, 255);
//		delay(10);
//		Serial.println(Motoruino.IMU.GetHeading());
//	}
//
//
//	start_timeout = millis();
//	while (millis() - start_timeout < 2000)
//	{
//		Motoruino.Motors.TurnTo( 18000, 255);
//		delay(10);
//		Serial.println(Motoruino.IMU.GetHeading());
//	}
//
//
//	start_timeout = millis();
//	while (millis() - start_timeout < 2000)
//	{
//		Motoruino.Motors.TurnTo( Side * 9000, 255);
//		delay(10);
//		Serial.println(Motoruino.IMU.GetHeading());
//	}


}
