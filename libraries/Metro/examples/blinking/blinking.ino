/*
 This code will blink an LED attached to pin 13 on and off. 
 It will stay on for 0.25 seconds.
 It will stay off for 1 second.
 */
#include <Metro.h> //Include Metro library
#define LED 13 // Define the led's pin

//Create a variable to hold theled's current state
int mainState = HIGH;

// Instanciate a metro object and set the interval to 250 milliseconds (0.25 seconds).
Metro ledMetro = Metro(250); 

void setup()
{
  pinMode(LED,OUTPUT);
  digitalWrite(LED,mainState);
}

void loop()
{

  if (ledMetro.check() == 1) { // check if the metro has passed its interval .
    if (mainState==HIGH)  mainState=LOW;
    else mainState=HIGH;
    
    digitalWrite(LED,mainState);
  }
}
