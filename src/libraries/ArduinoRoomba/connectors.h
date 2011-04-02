/*
 * connectors.h
 *  $HeadURL$
 *  $Id$
 *
 *  Created on: Mar 26, 2011
 *      Author: foley
 */

#ifndef CONNECTORS_H_
#define CONNECTORS_H_

namespace CreateConnector {
/// Pinouts on Create DB-25
typedef enum {
	RXD = 1,
	/// 0-5V serial input to Create
	TXD = 2,
	/// 0-5V serial output from Create
	PowerControlToggle = 3,
	/// Turns Create on or off on a low-to-high transition
	AnalogInput = 4,
	/// 0-5V analog input to Create
	DigitalInput1 = 5,
	/// 0-5V digital input to Create
	DigitalInput3 = 6,
	/// 0-5V digital input to Create
	DigitalOutput1 = 7,
	/// 0-5V, 20ma digital output from Create
	Switched5V = 8,
	/// Provides a regulated 5V 100mA supply and analog
	///   reference voltage when Create is switched on
	Vpwr = 9,
	/// Create battery voltage (unregulated), 0.5A
	SwitchedVpwr1 = 10,
	/// Provides battery power @ 1.5A when Create is powered on
	SwitchedVpwr2 = 11,
	/// Provides battery power @ 1.5A when Create is powered on
	SwitchedVpwr3 = 12,
	/// Provides battery power @ 1.5A when Create is powered on
	RobotCharging = 13,
	/// When Create is charging, this pin is high (5V)
	GND1 = 14,
	/// Create battery ground
	DeviceDetectNBaudRateChangePin = 15,
	/// 0-5V digital input to Create which can also be used to
	///   change the baud rate to 19200 (see below)
	GND2 = 16,
	/// Create battery ground
	DigitalInput0 = 17,
	/// 0-5V digital input to Create
	DigitalInput2 = 18,
	/// 0-5V digital input to Create
	DigitalOutput0 = 19,
	/// 0-5V, 20mA digital output from Create
	DigitalOutput2 = 20,
	/// 0-5V, 20mA digital output from Create
	GND3 = 21,
	/// Create battery ground
	LowSideDriver0 = 22,
	/// 0.5A low side driver from Create
	LowSideDriver1 = 23,
	/// 0.5A low side driver from Create
	LowSideDriver2 = 24,
	/// 1.5A low side driver from Create
	GND4 = 25,
	/// Create battery ground
} CreateCargoBayPins;
}

namespace RoombaConnector {
typedef enum {
	Vpwr1 = 1,
	// Create battery + (unregulated)
	Vpwr2 = 2,
	// Create battery + (unregulated)
	RXD = 3,
	// 0-5V Serial input to Create
	TXD = 4,
	// 0-5V Serial output from Create
	BRC = 5,
	// Baud Rate Change
	GND1 = 6,
	// Create battery ground
	GND2 = 7,
	// Create battery ground
} RoombaMiniDIN;
}



#endif /* CONNECTORS_H_ */
