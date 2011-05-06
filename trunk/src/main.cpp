#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

// ArduinoRoomb(rx,tx,dd)
/// Default arduino pin configuration, RX on pin 3, TX on pin 4, DD on pin 5


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

