/*
 * $Id: RobbiRobot.pde 420 2011-03-27 14:15:14Z foley $
 * $HeadURL: https://projects.cs.ru.is/svn/honnun2011/Projects/InteractiveArt/Group04/RobbiRobot/RobbiRobot.pde $ 
 * --------------
 * Firmware to control Robbi Robot
 * LHI-HIR Honnun 2011 Collaboration
 * 
 * Arduino pin 3 (RX) is connected to Roomba TXD
 * Arduino pin 4 (TX) is connected to Roomba RXD
 * If you are using a normal roomba via the cable and not a Create, then you
 * also need to connect this:
 *    Arduino pin 5 (DD) is conencted to Roomba DD
 *
 * 3 IR Range sensors are connected
 *   http://www.adafruit.com/index.php?main_page=product_info&cPath=35&products_id=164
 * Arduino pin A0 is Sensor 1 (Left)
 * Arduino pin A1 is Sensor 2 (Right)
 * Arduino pin A2 is Sensor 3 (Rear)
 *
 * 3 BlinkIC LEDs are connected
 *   http://www.sparkfun.com/products/9000
 * Arduino Pin A4 is SDA (I2C d)
 * Arduino Pin A5 is SCK (I2C c)
 * 
 * Currently it reads from one sensor and I created a "switch" that
 * tells it to mowe when it detects someone and stop when it does not
 * Now it just drives straight when it "sees" something currently one
 * blinkM led is working and blinks fadely and random. For now thats
 * the only thing ve want it to do.
 *
 * Based upon code "roomba.pde"
 * by Chris P. Gilmer <chris.gilmer@gmail.com>
 *   http://svn.chrisgilmer.net/viewvc.cgi/project/anomalyrobotics/arduino/roomba/roomba.pde?view=markup
 */

#include <WProgram.h>
#include <stdlib.h>
#include<ArduinoRoomba.h>
#include<virtualfunctionfix.h>

//--- Include All Libraries
#include <NewSoftSerial.h> // Needed since it is referenced by RoombaSerial
#include <ArduinoRoomba.h>

// Header file sets things up
#include "RobbiRobot.h"
#include "Wire.h"
#include "BlinkM_funcs.h"
	
//--- Arduino Onboard Serial Ports and Digital Pin
#define rxPin 3
#define txPin 4
#define ddPin 5
	
//--- Define an LED pin
const int ledPin = 13;

//--- Sensor distance thresholds
#define sensor1thresh 510
#define sensor2thresh 510
#define sensor3thresh 510

	
//--- Add the Roomba object
ArduinoRoomba roomba(rxPin,txPin,ddPin);

//--- Finite State machine
int presence = 0;  //
int state = 0;     //

//--- Sensor inputs
boolean sensor1on = false; //For multiple sensors
boolean sensor2on = false;
boolean sensor3on = false; 

//--- Helper functions
void blink(int blinks)  {
  for(int i=0; i<blinks; i++) {
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(600);
  }
}

//--- Setup Code
void setup() {
  BlinkM_begin();
  BlinkM_stopScript(blinkm_addr);  // turn off startup script
  BlinkM_setRGB(blinkm_addr, 0xff, 0x00, 0x00);

  // serial port for debugging
  Serial.begin(57600);
  Serial.println("$Id: RobbiRobot.pde 420 2011-03-27 14:15:14Z foley $");
  Serial.print("Robi Robot Initializing.");
  boolean setup = false;
  //--- Set the digital pins as outputs
  pinMode(ledPin, OUTPUT);	
  //--- Turn on/off LED to indicate init function
  while(! setup) {
	  roomba.init();
	  //roomba.fullMode();  // ignore cliff sensors
	  roomba.safeMode();  // try not to fall off table, unfortunately this means
                    // that it will shut down when it hits and edge
	  if (roomba.OImode() == roombaConst::ModeSafe) {setup = true;}
	  Serial.print(".");
  }
  Serial.println(".done.");
  // Green, we're ready to go!
  BlinkM_setRGB(blinkm_addr, 0x00, 0xff, 0x00);
    //-- Put the LED's back to cycling on hues
  //  BlinkM_stopScript(blinkm_addr);  // turn off startup script
  BlinkM_playScript(blinkm_addr,11,0,0);

}

void bounceCliff() {
 // cliff sensors give a true if they can see the floor
 // if we see a 0, then it can't see the floor
  Serial.print("Cliff sensors(L,LF,R,RF):");
  int cl = roomba.cliffLeft();
  int cfl = roomba.cliffFrontLeft();
  int cr = roomba.cliffRight();
  int cfr = roomba.cliffFrontRight();
  Serial.print(cl);
  Serial.print(cfl);
  Serial.print(cr);
  Serial.print(cfr);
  Serial.println();
  boolean cliffright = cr && cfr;
  boolean cliffleft = cl && cfl;  
  if (cliffright && cliffleft) {
     Serial.print("I've fallen off the table, help me!"); 
    //roomba.stopMoving();
  }
  else {
     if(cliffleft) {
       Serial.print("Cliff on my left, turning right.");
        //roomba.spinRight(); // Spins the left wheel so the robot turns
     }     
     if(cliffright) {
       Serial.print("Cliff on my right, turning left.");
       //roomba.spinLeft(); // Spins the left wheel so the robot turns
     }
  }
}

 void updatedistSensors(){
  int sensor1 = analogRead(A0);  //sensor in front to the left
  int sensor2 = analogRead(A1);  //sensor in front to the right
  int sensor3 = analogRead(A2);  //sensor in the back
  sensor1on = (sensor1 > sensor1thresh);
  sensor2on = (sensor2 > sensor2thresh);
  sensor3on = (sensor3 > sensor3thresh);
 
  Serial.print("sensor1=[");
  Serial.print(sensor1);
  Serial.print("]");
  Serial.print("sensor2=[");
  Serial.print(sensor2);
  Serial.print("]");
  Serial.print("sensor3=[");
  Serial.print(sensor3);
  Serial.println("]");
  
  if (sensor1on||sensor2on||sensor3on) {
    if (sensor1on) {
      presence = RIGHT_SENSOR;
    }
    else if (sensor2on) {
      presence = LEFT_SENSOR;
    }
    else if (sensor3on) {
      presence = REAR_SENSOR;
    }
    Serial.print("(Presence");
    Serial.print(presence);
    Serial.println(")");
  }
  else {
    presence = 0;
    Serial.println("(Nothing)");
  }
}

void bumpertest() {
  if(roomba.bumpLeft()) {
    Serial.println("LeftBump");
    //roomba.turnRScript();
    //roomba.runScript();

  }
  if(roomba.bumpRight()) {
    Serial.println("RightBump");
    //roomba.turnLScript();
    //roomba.runScript();
  }
}

void checkNcharge() {
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


}

boolean debounce = true;
//--- Loop Code
void loop() {
  //--- Turn on/off LED to indicate sensor function
  roomba.updateSensors(1);
  updatedistSensors();
  checkNcharge();

  //--- begin of blinkM loop
  //  bri_val = analogRead(bri_pot_pin);    // read the brightness pot
  // hue_val = analogRead(hue_pot_pin);    // read the hue pot
 
  // set blinkms with hue & bri, saturation is max
  // BlinkM_fadeToHSB( blinkm_addr, hue_val, 255, bri_val );
  bounceCliff();
  bumpertest();
  delay(100);  // wait a bit because we don't need to go fast

  Serial.print("(state ");
  Serial.print(state);
  Serial.print(")");
  if (state == 0){    // nothing is happening
      debounce = true;
      //roomba.stopMoving();
      //  BlinkM_stopScript(blinkm_addr);  // turn off startup script
      BlinkM_playScript(blinkm_addr,11,0,0);

      Serial.println("stoppar");

    if (presence > 0) {
      state = 1;
      Serial.println("keyrir burt");    
    }
  }
  if (state == 1){
    // Presence should still be set at this point, now we move
    // Check RobbieRobot.h for what these mean
    BlinkM_stopScript(blinkm_addr);  // turn off startup script

  if (debounce) {
    debounce = false;
    switch(presence) {
  case RIGHT_SENSOR:
    BlinkM_setRGB(blinkm_addr, 0x90, 0x90, 0x00);
    Serial.println("Something to my right.  Turning left!");
    //roomba.turnLScript();
    //roomba.runScript();
    break;
  case LEFT_SENSOR:
    BlinkM_setRGB(blinkm_addr, 0x00, 0x90, 0x90);
    Serial.println("Something to my left.  Turning right!");
    //roomba.turnLScript();
    //roomba.runScript();
    break;
  case REAR_SENSOR:
    BlinkM_setRGB(blinkm_addr, 0x90, 0x00, 0x90);
    Serial.println("Something behind me.  Running away!");
    roomba.drive(400, 0x8000); // Tells the robot to go forvard
    //delay(3000); /// Puts on delay to increase the forward distance, the higher the number gets the further away he drives from you. 5000 is about 1 meter
    break;
    }
  }
  if (presence == ALL_CLEAR) {
      state = 0;
      debounce = true;
    }
  }
  Serial.print("mode ");
  Serial.println(roomba.OImode(), DEC);
  delay(200);  // wait a bit because we don't need to go fast
  //Serial.print("Distance:");
  //Serial.println(roomba.getDistance());
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



