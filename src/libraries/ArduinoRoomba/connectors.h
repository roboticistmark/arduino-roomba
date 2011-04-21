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

namespace createConnector {
/// Pinouts on Create DB-25
typedef enum {
	/// 0-5V serial input to Create
	RXD = 1,
	/// 0-5V serial output from Create
	TXD = 2,
	/// Turns Create on or off on a low-to-high transition
	PowerControlToggle = 3,
	/// 0-5V analog input to Create
	AnalogInput = 4,
	/// 0-5V digital input to Create
	DigitalInput1 = 5,
	/// 0-5V digital input to Create
	DigitalInput3 = 6,
	/// 0-5V, 20ma digital output from Create
	DigitalOutput1 = 7,
	/// Provides a regulated 5V 100mA supply and analog
	///   reference voltage when Create is switched on
	Switched5V = 8,
	/// Create battery voltage (unregulated), 0.5A
	Vpwr = 9,
	/// Provides battery power @ 1.5A when Create is powered on
	SwitchedVpwr1 = 10,
	/// Provides battery power @ 1.5A when Create is powered on
	SwitchedVpwr2 = 11,
	/// Provides battery power @ 1.5A when Create is powered on
	SwitchedVpwr3 = 12,
	/// When Create is charging, this pin is high (5V)
	RobotCharging = 13,
	/// Create battery ground
	GND1 = 14,
	/// 0-5V digital input to Create which can also be used to
	///   change the baud rate to 19200 (see below)
	DeviceDetectNBaudRateChangePin = 15,
	/// Create battery ground
	GND2 = 16,
	/// 0-5V digital input to Create
	DigitalInput0 = 17,
	/// 0-5V digital input to Create
	DigitalInput2 = 18,
	/// 0-5V, 20mA digital output from Create
	DigitalOutput0 = 19,
	/// 0-5V, 20mA digital output from Create
	DigitalOutput2 = 20,
	/// Create battery ground
	GND3 = 21,
	/// 0.5A low side driver from Create
	LowSideDriver0 = 22,
	/// 0.5A low side driver from Create
	LowSideDriver1 = 23,
	/// 1.5A low side driver from Create
	LowSideDriver2 = 24,
	/// Create battery ground
	GND4 = 25,
} RoombaPin;
}

namespace roombaConnector {
typedef enum {
	/// Create battery + (unregulated)
	Vpwr1 = 1,
	/// Create battery + (unregulated)
	Vpwr2 = 2,
	/// 0-5V Serial input to Create
	RXD = 3,
	/// 0-5V Serial output from Create
	TXD = 4,
	/// Baud Rate Change
	BRC = 5,
	/// Create battery ground
	GND1 = 6,
	/// Create battery ground
	GND2 = 7,
} RoombaPin;
}



#endif /* CONNECTORS_H_ */
