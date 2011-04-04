
#include <NewSoftSerial.h>

NewSoftSerial mySerial(3, 4);

void setup()  
{
  Serial.begin(57600);
  Serial.print("NewSoftSerial Verision");
  Serial.println(NewSoftSerial::library_version());
  // set the data rate for the NewSoftSerial port
  mySerial.begin(57600);
  mySerial.enable_timer0(false);
}

void loop()                     // run over and over again
{

  if (mySerial.available()) {
      Serial.print((char)mySerial.read());
  }
  if (Serial.available()) {
      mySerial.print((char)Serial.read());
  }
  
  if (mySerial.overflow()) {
     Serial.println("!!!Overflow!!!"); 
  }
}
