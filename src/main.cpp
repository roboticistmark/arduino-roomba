#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

/// ArduinoRoomb(rx,tx,dd)
ArduinoRoomba roomba(3,4,5);
void setup() {
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(13, OUTPUT);
  roomba.init();
}

void loop() {
  digitalWrite(13, HIGH);   // set the LED on
  delay(200);              // wait for a second
  digitalWrite(13, LOW);    // set the LED off
  delay(2000);              // wait for a second
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();

	// IMPORTANT!!!!!!!!!!
	// The Arduino main() must NEVER NEVER NEVER return
	return 0;
}

