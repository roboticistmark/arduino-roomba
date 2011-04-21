/*
 * RoombaConst.h
 *
 *  Created on: Apr 21, 2011
 *      Author: foley
 */

#ifndef ROOMBACONST_H_
#define ROOMBACONST_H_
namespace roombaMask {
//! \enum Masks for LEDs on top of create and roomba
enum {
	led_none = 0x0,
	led_play = 0x2,
	led_advance = 0x8,
};

/// Masks for digitalOut
enum {
	dout0 = 0x1,
	dout1 = 0x2,
	dout2 = 0x4,
};

/// Masks for drivers
enum {
	driver_0 = 0x1,
	driver_1 = 0x2,
	driver_2 = 0x4,
};

/// Roomba-specific masks
enum {
	side_brush = 0x1,
	vacuum = 0x2,
	main_brush = 0x4,
};

/// Masks for bumps and wheel-drop sensor packet id 7
enum {
	bump_right = 0x1,
	bump_left = 0x2,
	drop_right = 0x4,
	drop_left = 0x8,
	drop_caster = 0x10,
};

/**
 * Masks for driver over-currents Packet ID 13.  Roomba, use SIDE_BRUSH,  VACUUM, MAIN_BRUSH
 */
enum {
	ld1 = 0x1,
	ld0 = 0x2,
	ld2 = 0x4,
	right_wheel = 0x8,
	left_wheel = 0x10,
};

/// Masks for buttons sensor packet ID 18
enum {
	/// Create
	button_play = 0x1,
	button_advance = 0x4,
	/// Roomba
	button_max = 0x1,
	button_clean = 0x2,
	button_spot = 0x4,
	button_power = 0x8,
};

/// Masks for digital inputs sensor packet ID 32
enum {
	din0 = 0x1,
	din1 = 0x2,
	din2 = 0x4,
	din3 = 0x8,
	din_detect = 0x10,
};

/// Masks for charging sources sensor packet ID 34
enum {
	internal_charger = 0x1,
	home_base = 0x2,
};
}

namespace roombaConst {
/// \enum Baud
/// Demo types to pass to Roomba::baud()
typedef enum
{
	Baud300    = 0,
	Baud600    = 1,
	Baud1200   = 2,
	Baud2400   = 3,
	Baud4800   = 4,
	Baud9600   = 5,
	Baud14400  = 6,
	Baud19200  = 7,
	Baud28800  = 8,
	Baud38400  = 9,
	Baud57600  = 10,
	Baud115200 = 11,
} Baud;

/// \enum Demo
/// Demo types to pass to Roomba::demo()
typedef enum
{
	DemoAbort = -1,
	DemoCover = 0,
	DemoCoverAndDock = 1,
	DemoSpotCover = 2,
	DemoMouse = 3,
	DemoDriveFigureEight = 4,
	DemoWimp = 5,
	DemoHome = 6,
	DemoTag = 7,
	DemoPachelbel = 8,
	DemoBanjo = 9,
} Demo;

/// \enum Drive
/// Special values for radius in Roomba::drive()
typedef enum
{
	DriveStraight                = 0x8000,
	DriveInPlaceClockwise        = 0xFFFF,
	DriveInPlaceCounterClockwise = 0x0001,
} Drive;

/// \enum StreamCommand
/// Values to pass to Roomba::streamCommand()
typedef enum
{
	StreamCommandPause  = 0,
	StreamCommandResume = 1,
} StreamCommand;

/// \enum EventType
/// Values to pass to Roomba::waitEvent()
/// Note that you will need the unsigned inverse to get lack of event

typedef enum
{
	EventTypeWheelDrop       = 1,
	EventTypeFronWheelDrop   = 2,
	EventTypeLeftWheelDrop   = 3,
	EventTypeRightWheelDrop  = 4,
	EventTypeBump            = 5,
	EventTypeLeftBump        = 6,
	EventTypeRightBump       = 7,
	EventTypeVirtualWall     = 8,
	EventTypeWall            = 9,
	EventTypeCliff           = 10,
	EventTypeLeftCliff       = 11,
	EventTypeFrontLeftCliff  = 12,
	EventTypeFrontRightCliff = 13,
	EventTypeRightCliff      = 14,
	EventTypeHomeBase        = 15,
	EventTypeAdvanceButton   = 16,
	EventTypePlayButton      = 17,
	EventTypeDigitalInput0   = 18,
	EventTypeDigitalInput1   = 19,
	EventTypeDigitalInput2   = 20,
	EventTypeDigitalInput3   = 21,
	EventTypeModePassive     = 22,
} EventType;

/// \enum IRCommand
/// Values for sensor packet ID 27
typedef enum
{
	// Remote control:
	IR_Left                   = 129,
	IR_Forward                = 130,
	IR_Right                  = 131,
	IR_Spot                   = 132,
	IR_Max                    = 133,
	IR_Small                  = 134,
	IR_Medium                 = 135,
	IR_LargeClean             = 136,
	IR_Pause                  = 137,
	IR_Power                  = 138,
	IR_ArcForwardLeft         = 139,
	IR_ArcForwardRight        = 140,
	IR_DriveStop              = 141,
	// Scheduling Remote:
	IR_SendAll                = 142,
	IR_SeekDock               = 143,
	// Home Base:
	IR_Reserved1              = 240,
	IR_RedBuoy                = 248,
	IR_GreenBuoy              = 244,
	IR_ForceField             = 242,
	IR_RedGreenBuoy           = 252,
	IR_RedBuoyForceField      = 250,
	IR_GreenBuoyForceField    = 246,
	IR_RedGreenBuoyForceField = 254,
} IRCommand;

/// \enum ChargeState
/// Values for sensor packet ID 21
typedef enum
{
	Charge_NotCharging            = 0,
	Charge_ReconditioningCharging = 1,
	Charge_FullChanrging          = 2,
	Charge_TrickleCharging        = 3,
	Charge_Waiting                = 4,
	Charge_Fault                  = 5,
} ChargeState;

/// \enum Mode
/// Values for sensor packet ID 35
typedef enum
{
	ModeOff     = 0,
	ModePassive = 1,
	ModeSafe    = 2,
	ModeFull    = 3,
} Mode;

/// \enum Sensor
/// Values for sensor packet IDs to pass to getSensors() and getSensorsList()
typedef enum
{
	Sensors7to26                   = 0,
	Sensors7to16                   = 1,
	Sensors17to20                  = 2,
	Sensors21to26                  = 3,
	Sensors27to34                  = 4,
	Sensors35to42                  = 5,
	Sensors7to42                    = 6,
	SensorBumpsAndWheelDrops       = 7,
	SensorWall                     = 8,
	SensorCliffLeft                = 9,
	SensorCliffFrontLeft           = 10,
	SensorCliffFrontRight          = 11,
	SensorCliffRight               = 12,
	SensorVirtualWall              = 13,
	SensorOvercurrents             = 14,
	//	SensorUnused1                  = 15,
	//	SensorUnused2                  = 16,
	SensorIRByte                   = 17,
	SensorButtons                  = 18,
	SensorDistance                 = 19,
	SensorAngle                    = 20,
	SensorChargingState            = 21,
	SensorVoltage                  = 22,
	SensorCurrent                  = 23,
	SensorBatteryTemperature       = 24,
	SensorBatteryCharge            = 25,
	SensorBatteryCapacity          = 26,
	SensorWallSignal               = 27,
	SensorCliffLeftSignal           = 28,
	SensorCliffFrontLeftSignal      = 29,
	SensorCliffFrontRightSignal     = 30,
	SensorCliffRightSignal          = 31,
	SensorUserDigitalInputs        = 32,
	SensorUserAnalogInput          = 33,
	SensorChargingSourcesAvailable = 34,
	SensorOIMode                   = 35,
	SensorSongNumber               = 36,
	SensorSongPlaying              = 37,
	SensorNumberOfStreamPackets    = 38,
	SensorVelocity                 = 39,
	SensorRadius                   = 40,
	SensorRightVelocity            = 41,
	SensorLeftVelocity             = 42,
} Sensor;
}

namespace roombaNotes {
	/// we use some of the lilypond notation
	enum {
		// G0 49.0hz
		g0 = 31, gis0,
		a1, ais1, b1, c1, cis1,	d1,	dis1, e1, f1, fis1,	g1,	gis1,
		a2, ais2, b2, c2, cis2,	d2,	dis2, e2, f2, fis2,	g2,	gis2,
		a3, ais3, b3, c3, cis3,	d3,	dis3, e3, f3, fis3,	g3,	gis3,
		a4, ais4, b4, c4, cis4,	d4,	dis4, e4, f4, fis4,	g4,	gis4,
		a5, ais5, b5, c5, cis5,	d5,	dis5, e5, f5, fis5,	g5,	gis5,
		a6, ais6, b6, c6, cis6,	d6,	dis6, e6, f6, fis6,	g6,	gis6,
		a7, ais7, b7, c7, cis7,	d7,	dis7, e7, f7, fis7,	g7,	gis7,
		a8, ais8, b8, c8, cis8,	d8,	dis8, e8, f8, fis8,	g8,
		// G8 12543.9hz
	};

}

namespace roombaCmd {
	enum {
		START = 128,
		BAUD = 129,
		CONTROL = 130,
		SAFE = 131,
		FULL = 132,
		POWER = 133, // Create
		MAX = 133, // Roomba
		SPOT = 134,
		COVER = 135,  // Create
		CLEAN = 135,  // Roomba
		DEMO = 136,
		DRIVE = 137,
		LOW_DRV = 138,
		LEDS = 139,
		SONG = 140,
		PLAY_SONG = 141,
		SENSORS = 142,
		COVER_DOCK = 143,
		PWM_LOW_DRV = 144,
		DRIVE_DIRECT = 145,
		DIGITAL_OUT = 147,
		STREAM = 148,
		QUERY_LIST = 149,
		PAUSE_STREAM = 150,
		RESUME_STREAM = 150,
		SEND_IR = 151,
		SCRIPT = 152,
		PLAY_SCRIPT = 153,
		SHOW_SCRIPT = 154,
		WAIT_TIME = 155,
		WAIT_DIST = 156,
		WAIT_ANGLE = 157,
		WAIT_EVENT = 158,

	};

}


#endif /* ROOMBACONST_H_ */
