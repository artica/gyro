
void BumperReader() 
{
  if(digitalRead(bumperLeft) == HIGH)   BLeft = LOW;
  else BLeft = HIGH;
  
  if (digitalRead(bumperRight) == HIGH) BRight = LOW;
  else BRight = HIGH;

//  Serial.print(BLeft);
//  Serial.print("  ");
//  Serial.println(BRight);
}

