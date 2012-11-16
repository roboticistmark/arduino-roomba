/* This is included so that you can use the Arduino .ino files
 *   it is enabled if you use the UNO/Mega build profile
 *   Alternatively, you can just put the code you want to call here
 *   If you use the Arduino style, you'll need to uncomment the setup() and loop() definition
 */
#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

// ArduinoRoomba(rx,tx,dd)
/// Default arduino pin configuration, RX on pin 3, TX on pin 4, DD on pin 5

void setup() {
	// put something that gets run at the beginnning
}

void loop() {
	// put something that gets run multiple times

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

