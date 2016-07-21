/*
 Artica CC - http://artica.cc

 Motoruino 2 Main Libs

 V0.0 - Serras and Tarquinio - 23/11/2015
	Extend Motoruino 2 Board to included sensors

 This Software is under The MIT License (MIT)

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#ifndef _MOTORUINO2_H_
#define _MOTORUINO2_H_
#include "Arduino.h"

#include "Motoruino2IMU.h"
#include "Motoruino2Motor.h"
#include "Motoruino2Power.h"

#define ADRRESS_SLAVE 2

class Motoruino2 : public Motoruino2IMU, public Motoruino2Motor, public Motoruino2Power
{
public:
	Motoruino2() ;

	// *** Objects ***

	// *** Navigation Functions ****
	// -> All functions can bee pooled to know if it reached the goal !!!
	//

	// TODO: Not Implemented yet
	bool moveForwardArc (int angularSpeed, unsigned int distance);
	// Turn to a degree
	bool turnToDegree(short angle, short speed);

	// Moves straight
	bool moveToDistance (signed long distance, unsigned short speed );

	// Rotate Angle using Encoder
	bool reachEncoderAngle(float angle, short speedl, short speedr);


	// Turning rotates the wheels in opposite directions in order to change the heading
	bool turnLeft (int angle = 0);
	bool turnRight (int angle = 0);
	// Stops the movement completely
	void stopMove();


	// TODO: Backup and Restore Data
	bool saveData(unsigned char * buffer, unsigned char size);
	bool getData(unsigned char * buffer, unsigned char size);

private:



};

#endif /* _MOTORUINO2_H_ */
