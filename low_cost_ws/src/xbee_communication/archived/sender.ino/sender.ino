#include <SoftwareSerial.h>  
              
SoftwareSerial XBee(2, 3); // RX, TX 

char msg[] = "hello world!";
//char msg = 'a';

void setup()
{
  Serial.begin(9600);                        
  XBee.begin(9600);
}

void loop()
{
  XBee.println(msg);
  //xbeeSerial.println();
  delay(1000);
}
