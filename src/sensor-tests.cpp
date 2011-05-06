/*
 * sensor-tests.cpp
 *
 *  Created on: May 6, 2011
 *      Author: foley
 */

#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

// ArduinoRoomb(rx,tx,dd)
/// Default arduino pin configuration, RX on pin 3, TX on pin 4, DD on pin 5
ArduinoRoomba roomba(3,4,5);
void setup() {
	Serial.begin(roombaConst::SerialFast);
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:

  pinMode(13, OUTPUT);
  Serial.println("Sending init commands");
  roomba.init();
  roomba.safeMode();
}

//--- Sensor distance thresholds
#define sensor1thresh 510
#define sensor2thresh 510
#define sensor3thresh 510

void updatedistSensors(){
	 int sensor1 = analogRead(A0);  //sensor in front to the left
	 int sensor2 = analogRead(A1);  //sensor in front to the right
	 int sensor3 = analogRead(A2);  //sensor in the back

	 Serial.print("sensor1=[");
	 Serial.print(sensor1);
	 Serial.print("]");
	 Serial.print("sensor2=[");
	 Serial.print(sensor2);
	 Serial.print("]");
	 Serial.print("sensor3=[");
	 Serial.print(sensor3);
	 Serial.println("]");
}

void loop() {
	//Serial.print("Sensors(");
	roomba.updateSensors(1);
	//Serial.println("):");

	if (roomba.bumpLeft()) { Serial.print("bumpLeft ");}
	if (roomba.bumpRight()) { Serial.print("bumpRight ");}
	if (roomba.dropLeft()) { Serial.print("dropLeft ");}
	if (roomba.dropRight()) { Serial.print("dropRight ");}
	if (roomba.dropCaster()) { Serial.print("dropCaster ");}
	if (roomba.cliffLeft()) { Serial.print("cliffLeft ");}
	if (roomba.cliffFrontLeft()) { Serial.print("cliffFrontLeft ");}
	if (roomba.cliffFrontRight()) { Serial.print("cliffFrontRight ");}
	if (roomba.cliffRight()) { Serial.print("cliffRight ");}
	if (roomba.chargingAvailable()) {
		Serial.print("charger ");
		roomba.start();
		while(1) {
			Serial.print("Charge:");
			Serial.println(roomba.chargingState(), HEX);
			Serial.print("Buttons:");
			Serial.println(roomba.button(), HEX);
			if (roomba.button() & roombaMask::button_play) break;
			delay(500);
		}
	}

	Serial.println();
	updatedistSensors();
	delay(100);
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

