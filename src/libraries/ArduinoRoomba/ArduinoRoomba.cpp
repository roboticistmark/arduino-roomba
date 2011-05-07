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
//#include "PString.h"

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

ArduinoRoomba::ArduinoRoomba(uint8_t rxPin, uint8_t txPin, uint8_t ddPin):
_velocityLimit(500),
_radiusLimit(2000)
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
/*
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
*/

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
	/// Wait one second for the startup message to go blasting through if it does
	delay(1000);
	this->sci->flush();

	/// Wake up if off
	//this->wake();

	/// Tell it to prepare to receive commands
	this->start();

	/// Now we must slow down or the UNO can't keep up
	this->baud(roombaConst::Baud38400);
	this->sci->begin(38400);

}

int ArduinoRoomba::getSensorData(uint8_t sensorCode, uint8_t index)
{
  if(sensorCode == 0){ return this->_sensorbytes_0[index]; }
  if(sensorCode == 1){ return this->_sensorbytes_1[index]; }
  if(sensorCode == 2){ return this->_sensorbytes_2[index]; }
  else { return this->_sensorbytes_3[index]; } // 3 or greater
}

uint8_t ArduinoRoomba::getSensorDirect(uint8_t packetCode) {
  // call for individual addressed packets instead of using groupings
  this->sci->print(142, BYTE);
  this->sci->print(packetCode, BYTE);  // sensor packet 1, 10 bytes
  // don't do anything until we see data
  unsigned long endTime = millis() + _readTimeout;
  while (! this->sci->available())
    {
      // Look for a timeout
      if (millis() > endTime) {
        Serial.println("(TIMEOUT)");
        return(-1); // Timed out
      }
    }
  	  /// needed if we're going multiple packets
    //endTime = millis() + _readTimeout;
    return(sci->read());
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
	delay(100);
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
      if (millis() > startTime + _readTimeout)
        return false; // Timed out
    }
    *dest++ = this->sci->read();
  }
  return true;
}

/// Grab data off the port.  We also return how many bytes we read.
int8_t ArduinoRoomba::updateSensors(uint8_t sensorCode) {
	// Update the best we can, remember to allow for timeouts
  if(sensorCode > 6) {
    sensorCode = 6;
  }

  this->sci->print(roombaCmd::SENSORS, BYTE);
  this->sci->print(sensorCode, BYTE);  // sensor packet 1, 10 bytes

  // now we care about packet length and which array
  char* array_p;
  uint8_t length;
  switch(sensorCode) {

  /// is this a pointer assignment or a copy?  I hate c++.
  case 0:
	  array_p = this->_sensorbytes_0;
	  length = 26;
	  break;
  case 1:
	  array_p = this->_sensorbytes_1;
	  length = 10;
	  break;
  case 2:
	  array_p = this->_sensorbytes_2;
	  length = 6;
	  break;
  case 3:
	  array_p = this->_sensorbytes_3;
	  length = 10;
	  break;
  case 4:
	  /// Unimplemented due to storage restrictions
//	  array_p = this->_sensorbytes_4;
//	  length = 14;
  case 5:
//	  array_p = this->_sensorbytes_5;
//	  length = 12;
  case 6:
//	  array_p = this->_sensorbytes_6;
//	  length = 52;
  default:
	  /// something is wrong, we should never end up here
	  /// perhaps an exception?  Too expensive.
	  return(-1);
  }
  unsigned long endTime = millis() + _readTimeout;
  {  /// scoping for exit code
	int i = 0;
    for (; i < length; i++){
	  while (! this->sci->available())
	  {
		  // Look for a timeout
		  if (millis() > endTime) {
			  Serial.print("(SensorsUpdate:TIMEOUT at ");
			  Serial.print(i);
			  Serial.println(")");
			  return(i);
		  }
	  }
  	  /// needed if we're going multiple packets
	  array_p[i] = sci->read();
	  endTime = millis() + _readTimeout;
  }
  /// Hopefully all went well!
  return(i);
  }
}

bool ArduinoRoomba::bumpLeft(void) {
	return(this->_sensorbytes_1[roombaConst::P1BumpsAndWheelDrops] & roombaMask::bump_left);
}
bool ArduinoRoomba::bumpRight(void) {
	return(this->_sensorbytes_1[roombaConst::P1BumpsAndWheelDrops] & roombaMask::bump_right);
}

bool ArduinoRoomba::dropLeft(void) {
  return(this->_sensorbytes_1[roombaConst::P1BumpsAndWheelDrops] & roombaMask::drop_left);
}
bool ArduinoRoomba::dropRight(void) {
  return(this->_sensorbytes_1[roombaConst::P1BumpsAndWheelDrops] & roombaMask::drop_right);
}
bool ArduinoRoomba::dropCaster(void) {
  return(this->_sensorbytes_1[roombaConst::P1BumpsAndWheelDrops] & roombaMask::drop_caster);
}

bool ArduinoRoomba::cliffLeft(void) {
  return(this->_sensorbytes_1[roombaConst::P1CliffLeft]);
}
bool ArduinoRoomba::cliffFrontLeft(void) {
  return(this->_sensorbytes_1[roombaConst::P1CliffFrontLeft]);
}
bool ArduinoRoomba::cliffFrontRight(void) {
  return(this->_sensorbytes_1[roombaConst::P1CliffFrontRight]);
}
bool ArduinoRoomba::cliffRight(void) {
  return(this->_sensorbytes_1[roombaConst::P1CliffRight]);
}

uint8_t ArduinoRoomba::button(void) {
	return(this->getSensorDirect(roombaConst::SensorButtons));
}

bool ArduinoRoomba::chargingAvailable(void) {
	return(this->getSensorDirect(roombaConst::SensorChargingSourcesAvailable) > 0);
}


uint8_t ArduinoRoomba::chargingState(void) {
	return(this->getSensorDirect(roombaConst::SensorChargingState));
}

uint8_t ArduinoRoomba::OImode(void) {
	return(this->getSensorDirect(roombaConst::SensorOIMode));
}

void ArduinoRoomba::drive(int16_t velocity, int16_t radius) {

	//--- The high and low bytes sent to the roomba
	uint8_t v_byte_high;
	uint8_t v_byte_low;
	uint8_t r_byte_high;
	uint8_t r_byte_low;

	//--- Safe the velocity values and convert them
	if(velocity >= this->_velocityLimit) {
		velocity = this->_velocityLimit;
	} else if(velocity <= -this->_velocityLimit) {
		  velocity = -this->_velocityLimit;
	}
	v_byte_low = velocity & 0xFF;
	v_byte_high = velocity >> 8;

	//--- Safe the radius values and convert them
	// Forward case
	if(radius == 32768 || radius == -32768) {
		r_byte_high = 128;
		r_byte_low = 0;
	}
	// Counter-clockwise
	else if(radius == 1) {
		r_byte_high = 0;
		r_byte_low = 1;
	}
	// Clockwise
	else if(radius == -1) {
		r_byte_high = 255;
		r_byte_low = 255;
	}
	// General data
	else {
		if(radius >= this->_radiusLimit) {
			radius = this->_radiusLimit;
		} else if(radius <= -this->_radiusLimit) {
			radius = -this->_radiusLimit;
		}
	r_byte_low = radius & 0xFF;
	r_byte_high = radius >> 8;
	}
	this->sci->print(roombaCmd::DRIVE, BYTE);   // DRIVE
	  this->sci->print(v_byte_high, BYTE);
	  this->sci->print(v_byte_low,  BYTE);
	  this->sci->print(r_byte_high, BYTE);
	  this->sci->print(r_byte_low,  BYTE);

}


void ArduinoRoomba::spinLeft(void) {

  int velocity = 200;
  int radius = 1;
  this->drive(velocity, radius);
	}

void ArduinoRoomba::spinRight(void) {

  int velocity = 200;
  int radius = -1;
  this->drive(velocity, radius);
}

void ArduinoRoomba::stopMoving(void) {

  int velocity = 0;
  int radius = 0;
  this->drive(velocity, radius);
}


