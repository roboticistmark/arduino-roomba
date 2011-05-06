/* 
 * Include file for Robbie Robot
 * Useful constants go here
 *
 */
#ifndef RobbieRobot_h
#define RobbieRobot_h
	

//---begin of BlinkM global code
//#include "BlinkM_funcs.h"

//Broadcast is 0x00
//Default address is 0x09
#define blinkm_addr 0x00

//-- analog in pins used for brightness & hue
//#define bri_pot_pin 0
//#define hue_pot_pin 1

//byte bri_val;
//byte hue_val;
//--- end of blinkM

//--- Presence/Sensor indicators
#define ALL_CLEAR 0
#define RIGHT_SENSOR 1
#define LEFT_SENSOR 2
#define REAR_SENSOR 3

#endif
