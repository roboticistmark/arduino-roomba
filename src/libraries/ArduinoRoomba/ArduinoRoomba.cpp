/*!  \file ArduinoRoomba.cpp
 	$Id: ArduinoRoomba.h 15 2011-04-02 23:14:50Z joe.foley@gmail.com $
	$HeadURL: https://arduino-roomba.googlecode.com/svn/trunk/src/libraries/ArduinoRoomba/ArduinoRoomba.h $
	\author  Joe Foley <foley@mit.edu>, <foley@ru.edu>
	Copyright (C) 2011 Joe Foley.  Use is subject to license conditions

	Inspired and Derived heavily from code by
	Mike McCauley (mikem@open.com.au)
 	Based upon the Roomba library at
 	http://www.open.com.au/mikem/arduino/Roomba


 	\par Open Source Licensing GPL V2
	This is the appropriate option if you want to share the source code of your
	application with everyone you distribute it to, and you also want to give them
	the right to share who uses it. If you wish to use this software under Open
	Source Licensing, you must contribute all your source code to the open source
	community in accordance with the GPL Version 2 when your application is
	distributed. See http://www.gnu.org/copyleft/gpl.html
*/

#include "WProgram.h"
#include "ArduinoRoomba.h"
#include "NewSoftSerial.h"
#include "PString.h"

//namespace roombaMask { }
//namespace roombaConst { }

/////////////////////////////////////////////////////////////////////
/// \class ArduinoRoomba ArduinoRoomba.cpp <ArduinoRoomba.cpp>
///
//  \brief Support for iRobot Roomba and Create platforms via NewSoftSerial using the iRobot Open Interface (OI) protocol.
//
/// The iRobot Roomba and Create platforms support a serial port through which you can control and
/// interrogate the device. The protocol implemented here conforms to the Open Interface protocol described in the
/// iRobot Open Interface Command Reference. Not all commands are supported on both platforms. Differences are
/// noted in the API
///
/// The Roomba and Create is equipped with a min-din serial port socket, while the Create
/// is also equipped with a 25-pin D connector called the Cargo Bay Connector. The
/// pins on the Cargo Bay Connector include the serial port, battery, digital inputs and
/// outputs, and an analog input.
///
/// In order to communicate with a Roomba, you must create an instance of the Roomba class and then call its
/// instance methods to send commmands and receive sensor messages. You can also request continuous
/// streams of sensor data to be sent by the Roomba. The Roomba also emits messages on its
/// serial port from time to time as described below.

ArduinoRoomba::ArduinoRoomba(uint8_t rxPin, uint8_t txPin, uint8_t ddPin)
{
#if defined(UBRR1H)
	/// if defined, we have a hardware Serial1
	/// TODO: fix the pointer vs. struct problem so that both versions are compatible
	this->sci = &Serial1;
#else
	// setup the NewSoftSerial port
	// this is probably wrong.  Fix it later
	this->sci = new NewSoftSerial::NewSoftSerial(rxPin, txPin);
#endif

	/// setup instance variables to remember which pins used
	this->_rxPin = rxPin;
	this->_txPin = txPin;
	this->_ddPin = ddPin;

	// Set dd as output so we can use it to wake Roomba.
	// sciSerial has already setup rx and tx pins
	pinMode(ddPin, OUTPUT);
}

void ArduinoRoomba::grabSerial()
{
	// Just listen on the serial port
	/// Roomba/Create is at 56K baud by default
	const int bufsize = 5;
	char buffer[bufsize];
	PString mystring(buffer, sizeof(buffer));
	this->sci->begin(57600);
	Serial.begin(57600);
	Serial.println("Grabbing serial.");
	char c;
	int i = 0;
	while(true) {
		if(this->sci->available()) {
			c = this->sci->read();
			mystring.print(c);
			i++;
		}
		if (i >= bufsize) {
			Serial.print(mystring);
			mystring.begin(); // reset
			i = 0;
		}

	}


}

/// Send an int, useful if the int is more than 8 bits
void ArduinoRoomba::sendint16(int16_t outint)
{
	this->sci->print((outint & 0xff00) >> 8);
	this->sci->print(outint & 0xff);
}


void ArduinoRoomba::init()
{
	// RoombaSerial puts manual delays between instructions
	// we can do better.

	/// Roomba/Create is at 56K baud by default
	this->sci->begin(57600);

	/// Wake up if off
	//this->wake();

	/// Tell it to prepare to receive commands
	this->start();
	/// We must wait for the tone to play before continuing
	delay(500);

	/// Now we must slow down or the UNO can't keep up
	this->baud(roombaConst::Baud38400);
}

int ArduinoRoomba::getSensorData(uint8_t sensorCode, uint8_t index)
{
  if(sensorCode == 0){ return this->sensorbytes_0[index]; }
  if(sensorCode == 1){ return this->sensorbytes_1[index]; }
  if(sensorCode == 2){ return this->sensorbytes_2[index]; }
  else { return this->sensorbytes_3[index]; } // 3 or greater
}

int ArduinoRoomba::getSensorDirect(uint8_t packetCode) {
  // call for individual addressed packets instead of using groupings
  this->sci->print(142, BYTE);
  this->sci->print(packetCode, BYTE);  // sensor packet 1, 10 bytes
  // don't do anything until we see data
  while(! this->sci->available()) {}
  return(this->sci->read());
}

void ArduinoRoomba::wake()
{
	/// The wake code is only needed on the SCI interface, really
	/// This will take it out of standby
	/// Harmess on a robot that is awake
	 digitalWrite(this->_ddPin, HIGH);
	 delay(100);
	 digitalWrite(this->_ddPin, LOW);
	 delay(500);
	 digitalWrite(this->_ddPin, HIGH);
	  delay(2000);
}


void ArduinoRoomba::start(void)
{
	this->sci->print(roombaCmd::START, BYTE);
}

/// Change the OI interface speed.  Particularly important for NewSoftSerial usage
void ArduinoRoomba::baud(roombaConst::Baud baud)
{
	this->sci->print(roombaCmd::BAUD, BYTE);
	this->sci->print(baud, BYTE);
	delay(100); /// OI specification says to wait 100ms before continuing
}

void ArduinoRoomba::control(void)
{
	this->sci->print(roombaCmd::CONTROL, BYTE);
}

void ArduinoRoomba::safeMode(void)
{
	this->sci->print(roombaCmd::SAFE, BYTE);
}

void ArduinoRoomba::fullMode(void)
{
	this->sci->print(roombaCmd::FULL, BYTE);
}

void ArduinoRoomba::power(void)
{
	this->sci->print(roombaCmd::POWER, BYTE);
}

void ArduinoRoomba::spot(void)
{
	this->sci->print(roombaCmd::SPOT, BYTE);
}

void ArduinoRoomba::demo(roombaConst::Demo demo)
{
	this->sci->print(roombaCmd::DEMO, BYTE);
	this->sci->print(demo, BYTE);
}


// now routines for getting data

/** Reads at most len bytes and stores them to dest
 * If successful, returns true.
 * If there is a timeout, returns false
 * Blocks until all bytes are read
 * Caller must ensure there is sufficient space in dest
 */
bool ArduinoRoomba::getData(char* dest, uint8_t len)
{
  while (len-- > 0)
  {
    unsigned long startTime = millis();
    while (this->sci->available())
    {
      // Look for a timeout
      if (millis() > startTime + read_timeout)
        return false; // Timed out
    }
    *dest++ = this->sci->read();
  }
  return true;
}

//void ArduinoRoomba::updateSensors(uint8_t sensorCode) {
//
//  //--- Safe the sensor values
//  if(sensorCode > 3) {
//    sensorCode = 3;
//  }
//
//  //this->sci.print(roombaCmd::SENSORS, BYTE);
//  this->sci.print(142, BYTE);
//  this->sci.print(sensorCode, BYTE);  // sensor packet 1, 10 bytes
//
//  if(! this->sci.available()) { }
//  int i = 0;
//  while(this->sci.available()) {
//    int c = this->sci.read();
//
//    if(sensorCode == 0){
//      this->sensorbytes_0[i++] = c;
//    }
//    if(sensorCode == 1){
//      this->sensorbytes_1[i++] = c;
//    }
//    if(sensorCode == 2){
//      this->sensorbytes_2[i++] = c;
//    }
//    if(sensorCode == 3){
//      this->sensorbytes_3[i++] = c;
//    }
//  }
//}

void ArduinoRoomba::updateSensors(uint8_t sensorCode) {
  //this->sci.print(roombaCmd::SENSORS, BYTE);
  this->sci->print(142, BYTE);
  this->sci->print(sensorCode, BYTE);  // sensor packet 1, 10 bytes
  this->getData(this->sensorbytes_1, 10);
}

bool ArduinoRoomba::bumpRight(void) {
  if(this->sensorbytes_1[0] & 0x01){
    return true;
  }
  return false;
}


