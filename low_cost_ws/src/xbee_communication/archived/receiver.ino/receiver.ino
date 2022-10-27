#include <SoftwareSerial.h>

SoftwareSerial XBee(2, 3); // RX, TX

void setup()
{
  XBee.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  if (XBee.available())
  {
    Serial.println(XBee.readString());
  }
}
