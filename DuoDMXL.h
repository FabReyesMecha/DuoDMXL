/*
DuoDMXL v.0.2.1
MX-64AR Half Duplex USART/RS-485 Communication Library
-----------------------------------------------------------------------------
Target Boards:
	Redebar Duo
	Particle Photon (not tested)
	Arduino Leonardo (not tested)
	or any other board with two hardware serial ports (soft serial not tested)

DuoDMXL.h:
	Definitions of constants and methods for class DynamixelClass

	Initially based on Savage's DynamixelSerial Library
	http://savageelectronics.blogspot.jp/2011/01/arduino-y-dynamixel-ax-12.html
-------------------------------------------------------------------------------

Copyright(c) 2016 Fabian Eugenio Reyes Pinner
Created by Fabian E. Reyes Pinner on 2016-06-01 (August 1st, 2016)

This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

-----------------------------------------------------------------------------
 Log:

 2016-12-22:	v0.2.1	Modified int to unsigned long in reading serial timeout
 											Marked constants specific to MX-64 and AX-12
 2016-12-21:	v.0.2
 2016-06-01:	v.0.1

 TODO:

-----------------------------------------------------------------------------

 Contact: 	burgundianvolker@gmail.com
 Author:		Fabian Eugenio Reyes Pinner (Fabian Reyes)
 */

#ifndef DuoDMXL_h
#define DuoDMXL_h

	// EEPROM AREA  ///////////////////////////////////////////////////////////
#define EEPROM_MODEL_NUMBER_L				0
#define EEPROM_MODEL_NUMBER_H				1
#define EEPROM_VERSION						2
#define EEPROM_ID							3
#define EEPROM_BAUD_RATE					4
#define EEPROM_RETURN_DELAY_TIME			5
#define EEPROM_CW_ANGLE_LIMIT_L				6
#define EEPROM_CW_ANGLE_LIMIT_H				7
#define EEPROM_CCW_ANGLE_LIMIT_L    		8
#define EEPROM_CCW_ANGLE_LIMIT_H    		9
#define EEPROM_LIMIT_TEMPERATURE			11
#define EEPROM_DOWN_LIMIT_VOLTAGE			12
#define EEPROM_UP_LIMIT_VOLTAGE				13
#define EEPROM_MAX_TORQUE_L					14
#define EEPROM_MAX_TORQUE_H					15
#define EEPROM_RETURN_LEVEL					16
#define EEPROM_ALARM_LED					17
#define EEPROM_ALARM_SHUTDOWN				18
#define EEPROM_TURN_OFFSET_L				20			//MX-64
#define EEPROM_TURN_OFFSET_H				21			//MX-64
#define EEPROM_RESOLUTION_DIV				22			//MX-64

// RAM AREA  //////////////////////////////////////////////////////////////
#define RAM_TORQUE_ENABLE					24
#define RAM_LED								25
#define	RAM_CW_COM_MAR						26			//AX-12
#define	RAM_CCW_COM_MAR						27			//AX-12
#define	RAM_CW_COM_SLOPE					28			//AX-12
#define	RAM_CCW_COM_SLOPE					29			//AX-12
#define RAM_D_GAIN							26			//MX-64
#define RAM_I_GAIN							27			//MX-64
#define RAM_P_GAIN							28			//MX-64
#define RAM_GOAL_POSITION_L					30
#define RAM_GOAL_POSITION_H					31
#define RAM_GOAL_SPEED_L            		32
#define RAM_GOAL_SPEED_H            		33
#define RAM_TORQUE_LIMIT_L					34
#define RAM_TORQUE_LIMIT_H					35
#define RAM_PRESENT_POSITION_L				36
#define RAM_PRESENT_POSITION_H				37
#define RAM_PRESENT_SPEED_L					38
#define RAM_PRESENT_SPEED_H					39
#define RAM_PRESENT_LOAD_L					40
#define RAM_PRESENT_LOAD_H					41
#define RAM_PRESENT_VOLTAGE					42
#define RAM_PRESENT_TEMPERATURE				43
#define RAM_REGISTERED_INSTRUCTION 			44
#define RAM_MOVING							46
#define RAM_LOCK							47
#define RAM_PUNCH_L							48
#define RAM_PUNCH_H							49
#define RAM_CURRENT_L						68			//MX-64
#define RAM_CURRENT_H						69			//MX-64
#define RAM_TORQUE_CONTROL					70			//MX-64
#define RAM_GOAL_TORQUE_L					71			//MX-64
#define RAM_GOAL_TORQUE_H					72			//MX-64
#define RAM_GOAL_ACCEL						73			//MX-64

// Status Return Levels ///////////////////////////////////////////////////////////////
#define RETURN_NONE							0
#define RETURN_READ							1
#define RETURN_ALL							2

// Instruction Set ///////////////////////////////////////////////////////////////
#define DMXL_PING                 			1
#define DMXL_READ_DATA              		2
#define DMXL_WRITE_DATA             		3
#define DMXL_REG_WRITE              		4
#define DMXL_ACTION                 		5
#define DMXL_RESET                  		6
#define DMXL_SYNC_WRITE             		131

// Specials ///////////////////////////////////////////////////////////////
#define OFF                         		0
#define ON                          		1
#define DMXL_RESET_LENGTH					2
#define DMXL_ACTION_LENGTH					2
#define DMXL_GOAL_SP_LENGTH         		7
#define DMXL_ACTION_CHECKSUM				250
#define BROADCAST_ID                		254
#define DMXL_START                  		255
#define TIME_OUT                    		50        	//[ms] Waiting time for the incomming data from servomotor
#define COOL_DOWN							1			//cool down period (milliseconds) before sending another command to the dynamixel servomotor
#define Tx_MODE                     		1
#define Rx_MODE                     		0

//Length of commands
#define LENGTH_READ							4			//All read functions requiere only a length of 4 (Instruction + command adress + length of data + checksum)
#define ONE_BYTE							1
#define TWO_BYTES							2

#include <inttypes.h>

class DynamixelClass {
	
	private:

		unsigned char Checksum;
		unsigned char Direction_Pin = D15;
		unsigned char Incoming_Byte;
		int Error_Byte;

		unsigned char dataLSB;
		unsigned char dataMSB;
		int data;

		int read_error(void);
		int read_information(void);

	public:

	//General functions
		int sendWord(unsigned char ID, unsigned char address, int params, int noParams);
		int readWord(unsigned char ID, unsigned char address, int noParams);

		void begin(long baud, unsigned char directionPin);
		void begin(long baud);
		void end(void);

		int reset(unsigned char ID);
		int ping(unsigned char ID);
		void action(void);

	//EEPROM Area Instructions
		int readModel(unsigned char ID);
		int readFirmware(unsigned char ID);
		int setID(unsigned char ID, unsigned char newID);
		int readID(unsigned char ID);
		int setBD(unsigned char ID, long baud);
		int setBDTable(unsigned char ID, unsigned char baud);
		int readBD(unsigned char ID);
		int setRDT(unsigned char ID, unsigned char RDT);
		int readRDT(unsigned char ID);
		int setCWAngleLimit(unsigned char ID, int limit);
		int readCWAngleLimit(unsigned char ID);
		int setCCWAngleLimit(unsigned char ID, int limit);
		int readCCWAngleLimit(unsigned char ID);
		int setTempLimit(unsigned char ID, unsigned char Temperature);
		int readTempLimit(unsigned char ID);
		int setLowVoltageLimit(unsigned char ID, unsigned char lowVoltage);
		int readLowVoltageLimit(unsigned char ID);
		int setHighVoltageLimit(unsigned char ID, unsigned char highVoltage);
		int readHighVoltageLimit(unsigned char ID);
		int setMaxTorque(unsigned char ID, int MaxTorque);
		int readMaxTorque(unsigned char ID);
		int setSRL(unsigned char ID, unsigned char SRL);
		int readSRL(unsigned char ID);
		int setAlarmLED(unsigned char ID, unsigned char alarm);
		int readAlarmLED(unsigned char ID);
		int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
		int readShutdownAlarm(unsigned char ID);
		int setMultiTurnOffset(unsigned char ID, int offset);
		int readMultiTurnOffset(unsigned char ID);
		int setResolutionDivider(unsigned char ID, unsigned char divider);
		int readResolutionDivider(unsigned char ID);

	//RAM Area Instructions
		int torqueEnable(unsigned char ID, bool Status);
		int torqueEnableStatus(unsigned char ID);
		int ledStatus(unsigned char ID, bool Status);
		int setGainD(unsigned char ID, int gain);
		int readGainD(unsigned char ID);
		int setGainI(unsigned char ID, int gain);
		int readGainI(unsigned char ID);
		int setGainP(unsigned char ID, int gain);
		int readGainP(unsigned char ID);
		int move(unsigned char ID, int Position);
		int moveSpeed(unsigned char ID, int Position, int Speed);
		int setMovingSpeed(unsigned char ID, int speed);
		int readMovingSpeed(unsigned char ID);
		int setTorqueLimit(unsigned char ID, int torque);
		int readTorqueLimit(unsigned char ID);
		int readPosition(unsigned char ID);
		int readSpeed(unsigned char ID);
		int readLoad(unsigned char ID);
		int readVoltage(unsigned char ID);
		int readTemperature(unsigned char ID);
		int registeredStatus(unsigned char ID);
		int moving(unsigned char ID);
		int lockEEPROM(unsigned char ID);
		int setPunch(unsigned char ID, int Punch);
		int readPunch(unsigned char ID);
		int readCurrent(unsigned char ID);
		int torqueControl( unsigned char ID, bool enable);
		int readTorqueControl( unsigned char ID);
		int setGoalTorque(unsigned char ID, int torque);
		int setGoalAccel(unsigned char ID, unsigned char accel);

	//Custom functions
		void configureServo(unsigned char ID, unsigned char newID, long baud);
		void setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
		void setWheelMode(unsigned char ID, bool enable);
		void setJointMode(unsigned char ID);
		void setDIP(unsigned char ID, int gainD, int gainI, int gainP);
		int findByBaudRate(long baudRate);
		int findByID(unsigned char id, unsigned char directionPin);
		void findServo(unsigned char directionPin);
};

extern DynamixelClass Dynamixel;

#endif //DuoDMXL_h
