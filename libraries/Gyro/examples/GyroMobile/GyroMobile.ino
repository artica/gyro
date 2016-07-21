/*
 Artica CC - http://artica.cc

 Gyro Mobile - Tarquinio and Serras

 This App is to integrate with mobile App developed to Android platform using
 the Bluetooth HC-05 Xbee Module on Motoruino2 platform

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
#include <GyroSerialCommand.h>
#include <Wire.h>
#include <Metro.h>
#include <SPI.h>
#include <Servo.h>
//#include <Tone.h>

#define DEBUG_PIN 13

// Define the rotation (chose between 1 or -1)
#define M1_SIGNAL -1
#define M2_SIGNAL 1


Gyro gyro;

GyroSerialCommand serial_command;    // Command interpreter

unsigned long last_update = 0;

int command_timeout = 250;

Metro log_metro = Metro(50);

int debug = 0;
void setup() 
{

    Serial.begin(9600);
    Serial1.begin(9600);

    Serial.println(F("Le starting"));    
    serial_command.addCommand("diff",  commandDifferential);
    serial_command.addCommand("traj",  commandTrajectory);
    serial_command.addCommand("tone",  commandTone);
    pinMode(DEBUG_PIN, OUTPUT); 

    delay(100);

    gyro.bumpers.begin( SENSOR_PROXIMITY,  SENSOR_PROXIMITY, SENSOR_PROXIMITY);


    // Bumpers Configuration
    gyro.bumpers.setTriggerBumper(100);

	// Motoruino2 Configuration
	gyro.motoruino2.config_imu(true, true, true);
	gyro.motoruino2.config_motor(M1_SIGNAL,M2_SIGNAL);
	gyro.motoruino2.config_power();

	gyro.motoruino2.setSpeedPWM(0,0);

	gyro.motoruino2.startEncoder(Motoruino2Motor::FAST_ENCODER, 300, 50.0);

	// Current Limit for Motor
	gyro.motoruino2.startMotorTrigger(60);

	gyro.motoruino2.resetDistance();

    if (gyro.motoruino2.calibrateGyro())
        Serial1.println("Calibrated OK");
    else
        Serial1.println("Not Calibrated");

    gyro.motoruino2.resetGyro(true,true,true);


}

void blinkDebug()
{
    digitalWrite(DEBUG_PIN, HIGH);
    delay(20);
    digitalWrite(DEBUG_PIN, LOW);
}

int hexValue(char value)
{                
    if (value == 0) return -1;
    if (value>='0' && value<='9') return value-'0';
    if (value>='a' && value<='z') return value-'a'+10;
    if (value>='A' && value<='Z') return value-'A'+10;
    return 0;
}  

int readHexTriple(char* buffer, int* pos)
{
    if ( hexValue(buffer[(*pos)]) == -1 || hexValue(buffer[(*pos)+1]) == -1 || hexValue(buffer[(*pos)+2]) == -1) return -1; 
    int result = hexValue(buffer[(*pos)])*256 + hexValue(buffer[(*pos)+1])*16 + hexValue(buffer[(*pos)+2]);
    (*pos) += 4;
    return result;
}

void commandTrajectory() 
{
    char *arg;
    
    Serial.println(F("We're in commandTrajectory"));
    arg = serial_command.next();
    if (arg == NULL) 
    {
        Serial.print(F("No argument!"));
        return;
    }
    
    Serial.print("TRAJ:"); Serial.println(arg);
    
    int pos = 0;
    
    byte move_count = readHexTriple(arg, &pos);

    Serial.print("move_count:"); Serial.println(move_count);
    
    int move_direction[move_count];
    int move_delay[move_count];
    
    for (int i=0; i<move_count; i++)
    {
        int next_angle = readHexTriple(arg, &pos);
        int next_delay = readHexTriple(arg, &pos);
        
        // If we don't get both balid values, stop here
        if (next_angle == -1 || next_delay==-1) 
        {
            move_count = i;
            break;
        }
        
        move_direction[i] = next_angle;
        move_delay[i] = next_delay;
        
        Serial.print("a:"); Serial.print(move_direction[i]);
        Serial1.print("a:"); Serial1.print(move_direction[i]);


        if ( move_direction[i] > 180) move_direction[i] -= 360;
        
        //move_direction[i] *= 100;

        Serial.print("  "); Serial.println(move_direction[i]);
        Serial1.print("  "); Serial1.println(move_direction[i]);

    }
    
    // traj 05012001002003004005
    
    // Quadrado, 2s lado
    // traj 004 000 7D0 05A 7D0 0B4 7D0 10E 7D0 

    // Rectangulo, 2s lado curto, 4s lado comprido
    // traj 004 000 FA0 05A 7D0 0B4 FA0 10E 7D0 

    // Frente-tras, 3s lado
    // traj 004,000;BB8,0B4;BB8,000;BB8,0B4;BB8,000;BB8,0B4;BB8 


    // counter-clockwise, 500ms 45graus
    // traj 001 02D 1F4


    gyro.motoruino2.resetGyro(true,true,true);
    Serial1.println("traj 0");

    for (int i=1; i<move_count; i++)
    {
        Serial1.print("TurnTo:"); Serial1.println(move_direction[i]);
        Serial.print("TurnTo:"); Serial.println(move_direction[i]);

        // Block while not on the correct angle
        while (!gyro.motoruino2.turnToDegree( move_direction[i], 255));

        Serial.print("nhec:"); Serial.println(abs(move_direction[i] - gyro.motoruino2.getGyroHeading()));
        //Serial.print("nhec:"); Serial.println(abs(move_direction[i] ));
        
//        int delay_adjust = move_delay[i] * (1.0+ abs(move_direction[i] - gyro.motoruino2.getGyroHeading()) / 9000  );
//        delay( delay_adjust );


        Serial.print("Forward:"); Serial.println(move_delay[i]);

        // Block while not reach distance
        gyro.motoruino2.resetDistance();
        delay(100);
        while (!gyro.motoruino2.moveToDistance(move_delay[i] / 40, 255)) delay(10);

        //motoruino.moveForward(500, 100);

        Serial1.print("traj "); Serial1.println(i+1);
    }
    
    gyro.motoruino2.setSpeedPWM(0, 0);
    
}

void commandDifferential() 
{
    //Serial.println(F("We're in commandDifferential"));
    char *arg;   
    arg = serial_command.next();
    if (arg == NULL) 
    {
        Serial.print(F("No argument 1"));
        return;
    }
    int left_motor = atoi(arg);

    arg = serial_command.next();
    if (arg == NULL) 
    {
        Serial.print(F("No argument 2"));
        return;
    }
    int right_motor = atoi(arg);
    
    
    arg = serial_command.next();
    if (arg == NULL) 
    {
        Serial.print(F("No argument 3"));
        return;
    }
    int neck_angle = atoi(arg);
    
    Serial.print("moving:"); Serial.print(left_motor);
    Serial.print(','); Serial.print(right_motor);
    Serial.print("  debug:"); Serial.println(debug);
    
    gyro.motoruino2.setSpeedPWM(left_motor, right_motor);
    gyro.neck.MoveTo(neck_angle);
    
    // diff 200 200

    last_update = millis();
}

void commandTone()
{
    Serial.println(F("We're in commandTone"));
    char *arg;
    arg = serial_command.next();
    if (arg == NULL)
    {
        Serial.print(F("No tone defined"));
        return;
    }

    Serial.print(F("tone: "));  Serial.print(arg[0]);

    switch(arg[0])
    {
        case '0': /* tone.stop(); */ break;
    }
 }


void loop()
{
    serial_command.readSerial();     // Read new incoming serial commands
    
    if (millis()-last_update > command_timeout) gyro.motoruino2.stopMove();
    
    if (log_metro.check() && true)
    {
        Serial1.print("sen "); Serial1.print(gyro.bumpers.CheckLeftBumper());
        Serial1.print(' '); Serial1.print(gyro.bumpers.CheckRightBumper());
        Serial1.print(' '); Serial1.print(gyro.distanceSensor.GetValue()); // GetDistance()
        Serial1.print(' '); Serial1.print(0);
        Serial1.print(' '); Serial1.println(gyro.motoruino2.getGyroHeading());
    }

}


