/*
 Artica CC - http://artica.cc

 MotoruinoSlave lib

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

#define ANGLE_OK 5				// Reach the Angle: default: 1deg (1*100)
#define ANGLE_MEDIUM 60			// Angle from which it will go to full speed on navigation; default: 80deg (80 * 100)

bool ReachAngle(long Angle, short Speed) {

	long Heading = (long) imuLSM9DS0.getGyroHeading();
	long Speed_Aux = 0;

	//Serial.print(Heading);Serial.print(" ");

	// Reset Heading -> Positions Heading on Zero and Replace the Angle

	Angle = Angle - Heading;

	if (Angle > 180)
		Angle = Angle - 360;
	else if (Angle < -180)
		Angle = Angle + 360;

	//Serial.println(Angle);

	// GOTO Positive Angle
	if (Angle > 0) {
		if (Angle > ANGLE_MEDIUM) {
			//motor.setPWMMotorSpeed(Speed, 0);
			motor.setPWMMotorSpeed(0, Speed);
		} else {
			Speed_Aux = ((long) Speed * Angle) / (long) (-1 * ANGLE_MEDIUM)
					+ (long) 255;
			if (Speed_Aux > 255)
				Speed_Aux = 255;

			//motor.setPWMMotorSpeed(Speed, (byte) Speed_Aux);
			motor.setPWMMotorSpeed((byte) Speed_Aux, Speed);
		}

	}
	// GOTO Positive Angle
	else {

		if (Angle < -ANGLE_MEDIUM) {
			motor.setPWMMotorSpeed(Speed, 0 );

		} else {

			Speed_Aux = ((long) Speed * Angle) / (long) (ANGLE_MEDIUM)
					+ (long) 255;

			if (Speed_Aux > 255)
				Speed_Aux = 255;

			motor.setPWMMotorSpeed(Speed, (byte) Speed_Aux);
		}

	}

	if ((Angle > -ANGLE_OK) && (Angle < ANGLE_OK))
		return true;
	else
		return false;
}




bool ReachDistance(signed long Distance, short Speed) {
	volatile signed long Distance_1 = motor.getDistance(1);
	volatile signed long Distance_2 = motor.getDistance(2);
	volatile signed long CurrentDistance = 0;

//	Serial.print(Distance); Serial.print(" ");
////
////	Serial.print(Speed); Serial.print(" ");
////
//	Serial.print(Distance_2); Serial.print(" "); Serial.println(Distance_1);

	CurrentDistance = Distance_1 ;

	if (Distance == 0)
	{
		motor.setPWMMotorSpeed(0, 0);
		return true;
	}

	if (Distance > 0) {
		if (Distance > CurrentDistance) {

			if (Distance_1 > Distance_2)
			{
				motor.setPWMMotorSpeed(Speed * 0.9, Speed );
			}
			else
			{
				motor.setPWMMotorSpeed(Speed , Speed * 0.90);
			}


			return false;
		} else {
			motor.setPWMMotorSpeed(0, 0);
			return true;
		}
	} else {
		if (Distance < CurrentDistance) {

			if (Distance_1 < Distance_2)
			{
				motor.setPWMMotorSpeed(-Speed* 0.90, -Speed );
			}
			else
			{
				motor.setPWMMotorSpeed(-Speed , -Speed* 0.90 );
			}

			return false;
		} else {
			motor.setPWMMotorSpeed(0, 0);
			return true;
		}
	}

}

const float Pi = 3.14159;

float constraintAngle(float angle)
{
	float x = fmod(angle, 2*Pi);

	if (x<0)
	{
		x+=2*Pi;
	}

	return x;

}

bool ReachEncoderAngle(float angle, short leftSpeed, short rightSpeed)
{
	/*
	 * Wheel Speed: v = 2*pi*r/t (r=Wheel Radius)
	 * l=distance between wheels ; R=rotation distance (Major Radius);
	 * Vr and Vl = Right and Left Wheel Velocity (m/s)
	 *
	 * Angular Velocity (rad/s): w * R = v => w * (R + l/2) = Vr ; w * (R - l/2) = Vl
	 *
	 * R = l/2 * (Vl + Vr) / (Vl - Vr) : when Vl = Vr, then R=00 (infinite!)
	 *
	 * w = (Vr - Vl) / l
	 *
	 */
//	Serial.print(angle);Serial.print(" ");
//
//	Serial.print(leftSpeed);Serial.print(" ");
//
//	Serial.println(rightSpeed);



	angle = constraintAngle(angle);

	motor.setPWMMotorSpeed(leftSpeed, rightSpeed);

	if ( leftSpeed == rightSpeed) return false;			// Exception!!! divide by zero

	float dl = motor.getDistance(1);
	float dr = motor.getDistance(2);

	//short w = (leftSpeed - rightSpeed)

	bool positiveRotation = dr > dl;

	float value = ((dr-dl) / WHEEL_DISTANCE);

	//Serial.println(value);

	if (positiveRotation && (value >= angle))
	{
		motor.setPWMMotorSpeed(0, 0);
		return true;
	}
	else if (!positiveRotation && (value <= (angle-(2*Pi))))
	{
		motor.setPWMMotorSpeed(0, 0);
		return true;
	}

	return false;
}


bool RotateAngle(long Angle, short Speed) {
	long Distance1 = motor.getDistance(1);
	long Distance2 = motor.getDistance(2);
	long CurrentDistance = 0;

	// TODO





	return false;

}
