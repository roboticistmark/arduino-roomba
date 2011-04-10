#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

/// ArduinoRoomb(rx,tx,dd)
ArduinoRoomba roomba(3,4,5);
void setup() {
	Serial.begin(57600);
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:

  pinMode(13, OUTPUT);
  Serial.println("Sending init commands");
  roomba.init();
  roomba.safeMode();
}

void loop() {
	Serial.print("Sensors(");
	Serial.print(roomba.updateSensors(1));
	Serial.println("):");
	//Serial.println(roomba.getSensorDirect(7), DEC);
	for (int i = 0; i < 10; i++) {
	Serial.print(roomba.getSensorData(1,i));
	}
	Serial.println();
	delay(500);
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

