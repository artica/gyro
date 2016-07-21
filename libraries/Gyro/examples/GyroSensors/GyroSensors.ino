/*
 Artica CC - http://artica.cc

 Gyro Sensors - Serras and Tarquinio

 This App is mainly to serve as a start point - to test the Gyro Functions

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include <Gyro.h>
#include <Motoruino2.h>
#include <Wire.h>
#include <Metro.h>
#include <SPI.h>
#include <Servo.h>

#define DEBUG_PIN 13

// Define the rotation (chose between 1 or -1)
#define M1_SIGNAL -1
#define M2_SIGNAL 1

Gyro gyro;

Metro blinkMetro = Metro(500);

int debug = 0;

void setup() 
{

    Serial.begin(115200);
    Serial1.begin(9600);

    Serial.println("Le starting");

    pinMode(DEBUG_PIN, OUTPUT); 

    delay(200);

    gyro.bumpers.begin( SENSOR_PROXIMITY,  SENSOR_PROXIMITY, SENSOR_PROXIMITY);

    // Bumpers Configuration
    gyro.bumpers.setTriggerBumper(100);

	// Motoruino2 Configuration
	gyro.motoruino2.config_imu(true, true, true);
	gyro.motoruino2.config_motor(M1_SIGNAL,M2_SIGNAL);
	gyro.motoruino2.config_power();

	gyro.motoruino2.setSpeedPWM(0,0);

	gyro.motoruino2.startEncoder(Motoruino2Motor::SLOW_ENCODER, 300, 50.0);

	//gyro.motoruino2.setSpeedPWM(0,0);

    if (gyro.motoruino2.calibrateGyro())
        Serial.println("Calibrated OK");
    else
        Serial.println("Not Calibrated");

    gyro.motoruino2.calibrateAngle();

    // Reset Gyroscope
    gyro.motoruino2.resetGyro(true,true,true);

    // Maximum ADC Current Value allowed to motor
    gyro.motoruino2.startMotorTrigger(30);

    //gyro.motoruino2.setSpeedPWM(255,255);

    //gyro.motoruino2.setSpeedRPMRight(60);

    gyro.bumpers.setCenterLED(0);

    gyro.motoruino2.setSpeedPWM(0,0);

	// -- MOVE TO DISTANCE --
	gyro.motoruino2.moveToDistance( 2000, 255);

    Serial.print("Started");

}

unsigned short leftProx, leftRed, leftGreen, leftBlue, leftAmbient, leftLine, leftGesture;
unsigned short rightProx, rightRed, rightGreen, rightBlue, rightAmbient, rightLine, rightGesture;
unsigned short centerProx, centerRed, centerGreen, centerBlue, centerAmbient, centerGesture;

unsigned short value, heading;
unsigned int count = 0;

void loop()
{
	if (blinkMetro.check() == 1)
	{

		Serial.print(count++);Serial.print('\t');




		// -- GOTO DEGREE --
		//gyro.motoruino2.turnToDegree( 90, 255);

		// -- CURRENT LIMIT --
		//gyro.motoruino2.getCurrentValue(&value); Serial.print(value);Serial.print('\t');

		// -- BATTERY VALUE --
		//gyro.motoruino2.getBatteryValue(&value); Serial.print(value * 66 / 1024);Serial.print('\t');

		// -- IMU GYROSCOPE HEADING 2D --
		heading = gyro.motoruino2.getGyroHeading();
		Serial.print((short)heading ); Serial.print('\t');

		// -- BUMPERS FUNCTION --
		gyro.bumpers.getLeftProximity((unsigned short *) &leftProx);	Serial.print(leftProx);	Serial.print('\t');
		gyro.bumpers.getCenterProximity((unsigned short *) &centerProx);	Serial.print(centerProx);	Serial.print('\t');
		gyro.bumpers.getRightProximity((unsigned short *) &rightProx);	Serial.print(rightProx);	Serial.print('\t');
		gyro.bumpers.getLeftLine((unsigned short *) &leftLine);			Serial.print(leftLine);	Serial.print('\t');
		gyro.bumpers.getRightLine((unsigned short *) &rightLine);			Serial.print(rightLine);	Serial.print('\t');

//		Serial.print(gyro.bumpers.CheckLeftBumper());Serial.print('\t');
//		Serial.print(gyro.bumpers.CheckRightBumper());Serial.print('\t');
//
//		gyro.bumpers.setRGB(0,255,255);
//
//		gyro.bumpers.getLeftProximity((unsigned short *) &leftProx);	Serial.print(leftProx);	Serial.print('\t');
//		gyro.bumpers.getLeftGesture((_GestureState *) &leftGesture);	Serial.print(leftGesture);	Serial.print('\t');
//		gyro.bumpers.getLeftRed((unsigned short *) &leftRed);			Serial.print(leftRed);	Serial.print('\t');
//		gyro.bumpers.getLeftGreen((unsigned short *) &leftGreen);		Serial.print(leftGreen);	Serial.print('\t');
//		gyro.bumpers.getLeftBlue((unsigned short *) &leftBlue);			Serial.print(leftBlue);	Serial.print('\t');
//		gyro.bumpers.getLeftAmbient((unsigned short *) &leftAmbient);	Serial.print(leftAmbient);	Serial.print('\t');
//		gyro.bumpers.getLeftLine((unsigned short *) &leftLine);			Serial.print(leftLine);	Serial.print('\t');
//
//		gyro.bumpers.setRGB(255,0,255);
//
//		gyro.bumpers.getRightProximity((unsigned short *) &rightProx);	Serial.print(rightProx);	Serial.print('\t');
//		gyro.bumpers.getRightGesture((_GestureState *) &rightGesture);	Serial.print(rightGesture);	Serial.print('\t');
//		gyro.bumpers.getRightRed((unsigned short *) &rightRed);			Serial.print(rightRed);	Serial.print('\t');
//		gyro.bumpers.getRightGreen((unsigned short *) &rightGreen);		Serial.print(rightGreen);	Serial.print('\t');
//		gyro.bumpers.getRightBlue((unsigned short *) &rightBlue);			Serial.print(rightBlue);	Serial.print('\t');
//		gyro.bumpers.getRightAmbient((unsigned short *) &rightAmbient);	Serial.print(rightAmbient);	Serial.print('\t');
//		gyro.bumpers.getRightLine((unsigned short *) &rightLine);			Serial.print(rightLine);	Serial.print('\t');
//
//		gyro.bumpers.setRGB(255,255,0);
//
//		gyro.bumpers.getCenterProximity((unsigned short *) &centerProx);	Serial.print(centerProx);	Serial.print('\t');
//		gyro.bumpers.getCenterGesture((_GestureState *) &centerGesture);	Serial.print(centerGesture);	Serial.print('\t');
//		gyro.bumpers.getCenterRed((unsigned short *) &centerRed);			Serial.print(centerRed);	Serial.print('\t');
//		gyro.bumpers.getCenterGreen((unsigned short *) &centerGreen);		Serial.print(centerGreen);	Serial.print('\t');
//		gyro.bumpers.getCenterBlue((unsigned short *) &centerBlue);			Serial.print(centerBlue);	Serial.print('\t');
//		gyro.bumpers.getCenterAmbient((unsigned short *) &centerAmbient);	Serial.print(centerAmbient);	Serial.print('\t');

		Serial.println();
	}

}


