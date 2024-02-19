#include <SoftwareSerial.h>

#define RX 3
#define TX 2

SoftwareSerial btSerial(RX, TX); // RX | TX
 
void setup() 
{
  Serial.begin(9600);
  Serial.println("ready");
  btSerial.begin(38400);      
  Serial.println("btSerial started at 38400");
}
 
void loop()
{
  if (Serial.available())
    btSerial.write(Serial.read());
  
  if (btSerial.available())
    Serial.write(btSerial.read());
}