/*!  \file ArduinoRoomba.h
 	$Id$
	$HeadURL$
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

#ifndef ArduinoRoomba_h
#define ArduinoRoomba_h

#include "WProgram.h"

namespace masks {
//! Masks for LEDs on top of create and roomba
enum {
  led_none = 0x0,
  led_play = 0x2,
  led_advance = 0x8,
};

/// Masks for digitalOut()
enum {
  dout0 = 0x1,
  dout1 = 0x2,
  dout2 = 0x4,
};

/// Masks for drivers()
enum {
  driver_0 = 0x1,
  driver_1 = 0x2,
  driver_2 = 0x4,
};

/// Roomba Specific masks
enum {
  side_brush = 0x1,
  vacuum = 0x2,
  main_brush = 0x4,
};

/// Masks for bumps and wheeldrops sensor packet id 7
enum {
  bump_right = 0x1,
  bump_left = 0x2,
  wheeldrop_right = 0x4,
  wheeldrop_left = 0x8,
  wheeldrop_caster = 0x10,
};

/// Masks for driver over-currents Packet ID 13.  Roomba, use SIDE_BRUSH,  VACUUM, MAIN_BRUSH
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



/////////////////////////////////////////////////////////////////////
/// \class ArduinoRoomba ArduinoRoomba.h <ArduinoRoomba.h>
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
///
/// \par Other Roomba messages
///
/// When iRobot Create powers up and after a reset, it sends a message like this on its serial port:
/// \code
/// bl-start
/// 2006-09-12-1137-L
/// RDK by iRobot!
/// MC9S12E128
/// 2006-11-20-1731-L
/// battery-current-quiescent-raw 524  battery-current-zero 510
///
/// 2006-11-20-1731-L
/// \endcode
///
/// While charging it will send a message like this each second:
/// \code
/// bat:   min 3  sec 21  mV 15558  mA 1491  deg-C 24
/// \endcode
///
/// To enter the factory test menu for the IRobot Create, hold down the (>) and (>>|)
/// buttons then press and hold the Power button until the assending and descending tones play and then stop.
/// You wil see some messages emitted on teh serial port.
/// Press the right-right arrow button to cycle through the tests.
///


class ArduinoRoomba
{

public:
  /// \def read_timeout
  /// Read timeout in milliseconds.
  /// If we have to wait more than this to read a char
  /// when we are expecting one, then something is wrong.
  static const int read_timeout = 200;

  
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
	IRCommandLeft                   = 129,
	IRCommandForward                = 130,
	IRCommandRight                  = 131,
	IRCommandSpot                   = 132,
	IRCommandMax                    = 133,
	IRCommandSmall                  = 134,
	IRCommandMedium                 = 135,
	IRCommandLargeClean             = 136,
	IRCommandPause                  = 137,
	IRCommandPower                  = 138,
	IRCommandArcForwardLeft         = 139,
	IRCommandArcForwardRight        = 140,
	IRCommandDriveStop              = 141,
	// Scheduling Remote:
	IRCommandSendAll                = 142,
	IRCommandSeekDock               = 143,
	// Home Base:
	IRCommandReserved1              = 240,
	IRCommandRedBuoy                = 248,
	IRCommandGreenBuoy              = 244, 
	IRCommandForceField             = 242,
	IRCommandRedGreenBuoy           = 252,
	IRCommandRedBuoyForceField      = 250,
	IRCommandGreenBuoyForceField    = 246,
	IRCommandRedGreenBuoyForceField = 254,
    } IRCommand;
  
    /// \enum ChargeState
    /// Values for sensor packet ID 21
    typedef enum
    {
	ChargeStateNotCharging            = 0,
	ChargeStateReconditioningCharging = 1,
	ChargeStateFullChanrging          = 2,
	ChargeStateTrickleCharging        = 3,
	ChargeStateWaiting                = 4,
	ChargeStateFault                  = 5,
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

    
    /// Constructor. You can have multiple simultaneous Roomba if that makes sense.
    ArduinoRoomba();

    //// Getting Started Commands
    ///////////////////////////////////////

    /// Starts the Open Interface and sets the mode to Passive. 
    /// You must send this before sending any other commands.
    /// Initialises the serial port to the baud rate given in the constructor
    void start();
    
    /// Converts the specified baud code into a baud rate in bits per second
    /// \param[in] baud Baud code, one of Roomba::Baud
    /// \return baud rate in bits per second
    uint32_t baudCodeToBaudRate(Baud baud);

    /// Changes the baud rate
    /// Baud is on of the Roomba::Baud enums
    void baud(Baud baud);

    /// MODES
    //////////////////////////////////////
    /// Sets the OI to Safe mode.
    /// In Safe mode, the cliff and wheel drop detectors work to prevent Roomba driving off a cliff
    void safeMode();

    /// Sets the OI to Full mode.
    /// In Full mode, the cliff and wheel drop detectors do not stop the motors: you are responsible
    // for full control of the Roomba
    void fullMode();

    /// Demo Commands
    //////////////////////////////////////
    /// Starts the requirested built-in demo
    /// \param[in] demo The demo number. One of Roomba::Demo
    void demo(Demo demo);

    /// Starts the Cover demo
    /// Changes mode to Passive
    void cover();

    /// Starts the Cover and Dock demo
    /// Changes mode to Passive
    void coverAndSeekDock();

    /// Starts the Spot Cover demo
    /// Changes mode to Passive
    void spot();

    /// Actuator Commands
    //////////////////////////////////////

    /// Starts the Roomba driving with a specified wheel velocity and radius of turn
    /// \param[in] velocity Speed in mm/s (-500 to 500 mm/s)
    /// \param[in] radius Radius of the turn in mm. (-2000 to 2000 mm). 
    /// Any of the special values in enum Roomba::Drive may be used instead of a radius value
    void drive(int16_t velocity, int16_t radius);

    /// Starts the Roomba driving with a specified velocity for each wheel
    /// Create only. No equivalent on Roomba.
    /// \param[in] leftVelocity Left wheel velocity in mm/s (-500 to 500 mm/s)
    /// \param[in] rightVelocity Right wheel velocity in mm/s (-500 to 500 mm/s)
    void driveDirect(int16_t leftVelocity, int16_t rightVelocity);

    /// Controls the LEDs on the Create
    /// \param[in] leds Bitmask specifying which LEDs to activate. ORed combination of ROOMBA_MASK_LED_*
    /// \param[in] powerColour The colour of the Power LED. 0 to 255. 0 = green, 255 = red, 
    /// intermediate values are intermediate colours
    /// \param[in] powerIntensity Power LED intensity. 0 to 255. 0 = off, 255 = full intensity
    void leds(uint8_t leds, uint8_t powerColour, uint8_t powerIntensity);

    /// Sets the digital output pins on the Cargo Bay Connector of the Create
    /// Create only. No equivalent on Roomba.
    /// \param[in] out Mask specifiying which outputs to enable. ORed value ROOMBA_MASK_DIGITAL_OUT_*
    void digitalOut(uint8_t out);

    /// Sets the duty cycle for PWM outputs on the low side drivers. These can be use for PWM driving of
    /// motors, lights etc.
    /// Create only. No equivalent on Roomba.
    /// \param[in] dutyCycle0 Duty cycle for low side driver 0. 0 to 128.
    /// \param[in] dutyCycle1 Duty cycle for low side driver 1. 0 to 128.
    /// \param[in] dutyCycle2 Duty cycle for low side driver 2. 0 to 128.
    void pwmLowSideDrivers(uint8_t dutyCycle0, uint8_t dutyCycle1, uint8_t dutyCycle2); 

    /// Sets the low side drivers on or off. On the Romba, these control the 3 motors.
    /// \param[in] out Bitmask of putputs to enable. ORed value ROOMBA_MASK_DRIVER_*
    void lowSideDrivers(uint8_t out);

    /// Sends the requested byte out of the low side driver 1 (pin 23 on the Cargo Bay Connector).
    /// low side driver 1 can be used to drive an IR transmitter to send commands to other Roombas and Creates. 
    /// Create only. No equivalent on Roomba.
    /// \param[in] data Data byte to transmit
    void sendIR(uint8_t data);

    /// Defines a song which can later be played with playSong()
    /// \param[in] songNumber Song number for this song. 0 to 15
    /// \param[in] notes Array of note/duration pairs. See Open Interface manual for details. 2 bytes per note, 
    /// first byte is the note and the second is the duration
    /// \param[in] len Length of notes array in bytes, so this will be twice the number of notes in the song
    void song(uint8_t songNumber, const uint8_t* notes, int len);

    /// Plays a song that has previously been defined by song()
    /// \param[in] songNumber The song number to play. 0 to 15
    void playSong(uint8_t songNumber);

    /// Input Commands
    /////////////////////////////////////////////
    /// Reads the sensor data for the specified sensor packet ID. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure len agrees with the expected 
    /// length of the sensor data. See the Open Interface mamual for details on sensor packet lengths.
    /// Roomba.h defines various enums and defines for decoding sensor data.
    /// Blocks untill all len bytes are read or a read timeout occurs.
    /// \param[in] packetID The ID of the sensor packet to read from Roomba::Sensor
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of sensor data bytes to read
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool sensors(uint8_t packetID, uint8_t* dest, uint8_t len);

    /// Reads the sensor data for the specified set of sensor packet IDs. Note that different sensor packets have 
    /// different lengths, and it is the callers responsibilty to make sure len agrees with the expected 
    /// length of the sensor data. See the Open Interface mamual for details on sensor packet lengths.
    /// Blocks until all len bytes are read or a read timeout occurs.
    /// Create only. No equivalent on Roomba.
    /// \param[in] packetIDs Array of IDs  from Roomba::Sensor of the sensor packets to read
    /// \param[in] numPacketIDs number of IDs in the packetIDs array
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of sensor data bytes to read and store to dest.
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool queryList(uint8_t* packetIDs, uint8_t numPacketIDs, uint8_t* dest, uint8_t len);

    /// Requests that a stream of sensor data packets be sent by the Roomba.
    /// See the Open Interface manual for details on the resutting data.
    /// The packets will be sent every 15ms.
    /// You can use pollSensors() to receive sensor data streams.
    /// Create only. No equivalent on Roomba.
    /// See the Open Interface maual for more details and limitations.
    /// \param[in] packetIDs Array specifying sensor packet IDs from Roomba::Sensor to be sent.
    /// \param[in] len Number of IDs in packetIDs
    void stream(const uint8_t* packetIDs, int len);

    /// Pause or resume a stream of sensor data packets previously requested by stream()
    /// Create only. No equivalent on Roomba.
    /// \param[in] command One of Roomba::StreamCommand
    void pauseResumeStream(StreamCommand command);


    /// Script Commands
    /////////////////////////////////////////////////////////////////
    /// Defines a command script which can later be executed with playScript(). You can clear the script by calling 
    /// script(NULL, 0);
    /// Create only. No equivalent on Roomba.
    /// \param[in] script Array containing a sequence of Roomba OI commands.
    /// \param[in] len Length of the script in bytes.
    void script(const uint8_t* script, uint8_t len);

    /// Executes a previously defined script, 
    /// the last one specified by script()
    /// Create only. No equivalent on Roomba.
    void playScript();

    /// Reads a the contents of the script most recently specified by a call to script().
    /// Create only. No equivalent on Roomba.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len The maximum number of bytes to place in dest. If the script is actually longer than len
    /// only len bytes will be written
    /// \return The actual number of bytes in the script, even if this is more than len. By calling 
    /// getScript(NULL, 0), you can determine how many bytes would be required to store the script.
    uint8_t showScript(uint8_t* dest, uint8_t len);



    /// Wait Commands
    ////////////////////////////////////////////////////////////////
    /// Tells the Roomba to wait for a specified time.
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] ticks The number of ticks to wait. Each tick is 15ms
    void waitTime(uint8_t ticks);

    /// Causes Roomba to wait until it has travelled the distance specified. 
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] mm Distance to wait for in mm
    void waitDistance(int16_t mm);

    /// Causes Roomba to wait until it has rotated through the specified angle.
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// This command is intended for use in scripting only.
    /// Create only. No equivalent on Roomba.
    /// \param[in] degrees Angle to wait for in degrees
    void waitAngle(int16_t degrees);

    /// Cause the Create to wait for a specified event.
    /// Roomba will not react to any inputs until the wait has completed. 
    /// Note that this does not cause the host arduino to wait, it only sends the wait comman to the Roomba
    /// Create only. No equivalent on Roomba.
    /// \param[in] type Event type to wait for. One of Roomba::EventType
    void waitEvent(EventType type);


    /// Low level Helper Functions
    /////////////////////////////////////////////////////////////////////
    /// Low level funciton to read len bytes of data from the Roomba
    /// Blocks untill all len bytes are read or a read timeout occurs.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Number of bytes to read
    /// \return true if all len bytes were successfully read. Returns false in the case of a timeout 
    /// on reading any byte.
    bool getData(uint8_t* dest, uint8_t len);

    /// NonOI support functions
    ///////////////////////////////////////////////////////////////
    /// Polls the serial input for data belonging to a sensor data stream previously requested with stream().
    /// As sensor data is read it is appended to dest until at most len bytes are stored there. 
    /// When a complete sensor stream has been read with a correct checksum, returns true. 
    /// See the Open Interface manual for details on how the sensor data will be encoded in dest.
    /// Discards characters that are not part of a stream, such as the messages the Roomba 
    /// sends at startup and while charging.
    /// Create only. No equivalent on Roomba.
    /// \param[out] dest Destination where the read data is stored. Must have at least len bytes available.
    /// \param[in] len Max number of sensor data bytes to store to dest
    /// \return true when a complete stream has been read, and the checksum is correct. The sensor data
    /// (at most len bytes) will have been stored into dest, ready for the caller to decode.
    bool pollSensors(uint8_t* dest, uint8_t len);



    /// Resets the Roomba. 
    /// It will emit its startup message
    /// Caution, this may take several seconds to complete
    void reset();


    //// Roomba Only Commands
    /////////////////////////////////////////////////////////
    /// Causes roomba to immediately 
    /// seek the docking station.
    /// No equivalent for Create.
    void dock();

    /// Puts a Roomba in sleep mode.
    /// Roomba only, no equivalent for Create.
    void power();


	

private:
    /// \enum PollState
    /// Values for _pollState
    typedef enum
    {
	PollStateIdle         = 0,
	PollStateWaitCount    = 1,
	PollStateWaitBytes    = 2,
	PollStateWaitChecksum = 3,
    } PollState;

    /// The baud rate to use for the serial port
    uint32_t        _baud;
	
    /// The serial port to use to talk to the Roomba
    HardwareSerial* _serial;
    
    /// Variables for keeping track of polling of data streams
    uint8_t         _pollState; /// Current state of polling, one of Roomba::PollState
    uint8_t         _pollSize;  /// Expected size of the data stream in bytes
    uint8_t         _pollCount; /// Num of bytes read so far
    uint8_t         _pollChecksum; /// Running checksum counter of data bytes + count

};

#endif  // ArduinoRoomba_H
