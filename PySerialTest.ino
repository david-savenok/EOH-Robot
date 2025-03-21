#include <stdlib.h>

char serial;
#define RELAY1  2                      
void setup()
{    
  Serial.begin(9600);
  pinMode(RELAY1, OUTPUT);       
}

void loop()
{
  if(Serial.available() > 0)
  {
      serial = Serial.read();
      Serial.println( serial, HEX);
      if (serial=='s')
      {
        digitalWrite(RELAY1,0);           
        Serial.println("Light ON");
        delay(2000);                                      
        digitalWrite(RELAY1,1);          
        Serial.println("Light OFF");
        delay(2000);
      }
   } 
}