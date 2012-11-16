/* $HeadURL$
 * $Id$
 * This main is included so that you can use the Arduino .ino files for development.
 *   They assume that you have something that already defines a main() using setup() and loop()
 *   This file is enabled if you use the UNO/Mega build profile
 *
 *   Alternatively, you can just put the code you want to call here directly
 *   If you use the Arduino style, you'll need to comment the setup() and loop() definition or you will
 *   get a naming conflict.
 *
 *   The build profiles are found by right-clicking on the project > Build Configurations > Set Active
 *   You can enable/disable files from a build by right clicking on the file > Resource Configruations > Exclude from Build
 *   The output .hex file will be put into a new folder with the same name as the Build Configuration
 *   Don't forget to update the AVR settings under Project > Properties > AVR
 *   	You will need the right architecture (ATMega 328P for an UNO at 16000000hz) and
 *   	the right serial port under the programmer
 */
#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

// ArduinoRoomba(rx,tx,dd)
/// Default arduino pin configuration, RX on pin 3, TX on pin 4, DD on pin 5

const int ledpin = 13;

void setup() {
	// put something that gets run at the beginnning
	pinMode(ledpin, OUTPUT);
}

void loop() {
	// put something that gets run multiple times
	// this should make a quick non-blocking blink
	if(0 == millis() % 1000 ) {
		digitalWrite(ledpin, HIGH);
	}
	if(500 == millis() % 1000) {
		digitalWrite(ledpin, HIGH);
	}
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

