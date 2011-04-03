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

namespace roombaMask { }
namespace roombaConst { }

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



void ArduinoRoomba::start() {

	int a;
	int b;
	a = roombaMask::bump_right;
	b = roombaConst::Baud300;
	a = a + 1;
	b = b + 1;
}

