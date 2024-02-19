#include <SoftwareSerial.h>

// HC05 pins
#define RX 2
#define TX 3

SoftwareSerial btSerial(RX, TX);

typedef struct _PacketData_
{
  byte lxAxisValue_;
  byte lyAxisValue_;
  byte rxAxisValue_;
  byte ryAxisValue_;
} PacketData;
PacketData data;

//---------------------------------------------------------------------------------------------------------------------
///
/// values from 0 to 254, 127 is center
///
/// @param 
///
/// @return none
//
void setup()
{
  btSerial.begin(38400);
  
  Serial.begin(9600);
  Serial.println("Serial started");
}

//---------------------------------------------------------------------------------------------------------------------
///
/// This function is used to map 0-1023 joystick value to 0-254. hence 127 is the center value which we send.
/// It also adjust the deadband in joystick.
/// Jotstick values range from 0-1023. But its center value is not always 511. It is little different.
/// So we need to add some deadband to center value. in our case 500-530. Any value in this deadband range is mapped to center 127.
///
/// @param 
///
/// @return none
//
// 
int mapAndAdjustJoystickDeadBandValues(int value, bool reverse)
{
  if (value >= 530)
    value = map(value, 530, 1023, 127, 254);

  else if (value <= 500)
    value = map(value, 500, 0, 127, 0);  

  else
    value = 127;

  if (reverse)
    value = 254 - value;

  return value;
}

//---------------------------------------------------------------------------------------------------------------------
///
/// values from 0 to 254, 127 is center
///
/// @param 
///
/// @return none
//
void loop()
{
  data.lxAxisValue_ = mapAndAdjustJoystickDeadBandValues(analogRead(A0), false);
  data.lyAxisValue_ = mapAndAdjustJoystickDeadBandValues(analogRead(A1), false);
  data.rxAxisValue_ = mapAndAdjustJoystickDeadBandValues(analogRead(A2), false);
  data.ryAxisValue_ = mapAndAdjustJoystickDeadBandValues(analogRead(A3), false);

  String dataString;
  dataString = dataString + data.lxAxisValue_ + ","  + data.lyAxisValue_ + "," + data.rxAxisValue_ + "," + data.ryAxisValue_ + "\n";
  
  Serial.println(data.lxAxisValue_);
  Serial.println(data.lyAxisValue_);
  Serial.println(data.rxAxisValue_);
  Serial.println(data.ryAxisValue_);
  Serial.println("now sending");
  btSerial.print(dataString); // send to HC05
  Serial.println("should be send");
  delay(500);
}
