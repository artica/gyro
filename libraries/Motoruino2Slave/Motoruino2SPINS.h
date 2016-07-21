/*
 Artica CC - http://artica.cc

 MotoruinoSlave Hardware Abstration Layer

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
#ifndef _MOTORUINO2HAL_H_
#define _MOTORUINO2HAL_H_
#include "Arduino.h"

// DEFINITIONS OF THE POWER PINS
#define WAKE_PIN 		4		//
#define SLEEP_CTL 		7		//
#define BATTERY_ADC 	A7

// DEFINITIONS OF THE GPOUT PINS
#define GP_OUT 			A1

#define DEBUG A1

// DEFINITIONS OF THE MOTOR PINS
#define ENC1_INT 		2		// Encoder M1 Interrupt activated In
#define ENC1_IO 		A3		// Encoder M1 IO In -> OLD Pin4 -> See LSM330DLC.h -> Now RX
#define ENC2_INT 		3		// Encoder M2 Interrupt activated In
#define ENC2_IO 		A2		// M2 PWM output

#define M1_IN1 			5		// M1 PWM output
#define M1_IN2 			6		// M1 PWM output
#define M2_IN1 			10		// 9 M2 PWM output
#define M2_IN2 			9		// 10 M2 PWM output

#define MOTOR_ADC 		A6		// ADC for the L298P Resistor for ShortCircuit/Consumption Detection

// DEFINITIONS OF THE IMU PINS
#define _CS_ACCEL_MAG	8
#define _CS_GYRO  		A0

#endif
