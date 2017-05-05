/*
DuoDMXL v.1.0
MX-64AR Half Duplex USART/RS-485 Communication Library
-----------------------------------------------------------------------------
Target Boards:
	Redbear Duo
	Particle Photon (not tested)
	Arduino Leonardo (not tested)
	or any other board with two hardware serial ports (soft serial not tested)

DuoDMXL.cpp:
	Methods for class DynamixelClass
Dependencies:
	DuoDMXL.h
	Arduino.h

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

2017-05-05:		v.1.0	Improved communication safety in readInformation() using the flag _response_within_timeout
						User can change the baudrate in the same session, without reseting the microcontroller
2017-05-04: 	v.0.3	Status Return Level (SRL) can now be changed by the user
					   	TIME_OUT and COOL_DOWN are accessible to the user
2017-04-13: 	v.0.2.2	Added extra comments.
					   	changed name read_information() to readInformation()
2016-12-22:		v0.2.1	Modified int to unsigned long in reading serial timeout
					   	Marked constants specific to MX-64 and AX-12
2016-12-21:		v.0.2
2016-06-01:		v.0.1

 TODO:

-----------------------------------------------------------------------------

 Contact: 	burgundianvolker@gmail.com
 Author:	Fabian Eugenio Reyes Pinner (Fabian Reyes)
 */

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "DuoDMXL.h"
#include <math.h>

// Macro for the selection of the Serial Port
//Serial refers to the USB port and is reserved for communication with a PC
//Serial1 corresponds to the second hardware serial port (where available)
#define sendData(args)  (Serial1.write(args))   // Write Over Serial
#define availableData() (Serial1.available())   // Check Serial Data Available
#define readData()      (Serial1.read())        // Read Serial Data
#define peekData()      (Serial1.peek())        // Peek Serial Data
#define beginCom(args)  (Serial1.begin(args))   // Begin Serial Comunication
#define endCom()        (Serial1.end())         // End Serial Comunication
#define serialFlush()	(Serial1.flush())		// Wait until data has been written

// Macro for Timing
#define delayus(args) (delayMicroseconds(args))  // Delay Microseconds

// Macro for Comunication Flow Control
#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

// Private Methods //////////////////////////////////////////////////////////////

//Read response from the servo and return the error (if any)
//Function inherited from Savage's library. DEPRECATED by readInformation()
int DynamixelClass::read_error(void)
{
	unsigned long startTime = millis();
	unsigned long processTime;
	int lengthMessage;

	//Do nothing until we have the start bytes, ID, and length of the message
	while( (availableData() <=4) && (((processTime = millis()) - startTime) <= TIME_OUT) ){}

	while (availableData() > 0){
		Incoming_Byte = readData();										//First byte of the header
		if ( (Incoming_Byte == 255) && (peekData() == 255) ){
			readData();                                    				//Second byte of the header
			readData();                                    				//Dynamixel ID
			lengthMessage = readData();									//Length of the message

			while( !availableData() ){}
			Error_Byte = readData();                       				//Error

			for(int i=0; i< (lengthMessage-1); i++){
				while( !availableData() ){}
				readData();												//Read rest of the message. The last byte should be the checksum
			}

			delay(COOL_DOWN);
			return (Error_Byte);
		}
	}

	delay(COOL_DOWN);
	return(-1);											 				//No servo Response
}

//General function to read the status package from the servo
//TODO: Add timer to all 'while( !availableData() ){}'
int DynamixelClass::readInformation(void)
{
	unsigned long startTime = millis();
	int processTime, lengthMessage;

	_response_within_timeout = true;									//Reset flag

	//DuoDMXL < v.1.0
	//Do nothing until we have the start bytes, ID, and length of the message
	//while( (availableData() <=4) && (((processTime=(int) millis()) - startTime) <= TIME_OUT) ){}

	//This version works, but it is hard to read and may lead to errors
	//Do nothing until we have the start bytes, ID, and length of the message
	//while(  (availableData() <=4)  && (_response_within_timeout = (bool) (((processTime=(int) millis()) - startTime) <= TIME_OUT)) ){}

	//DuoDMXL v.1.0
	//Do nothing until we have the start bytes, ID, and length of the message. Even at 9600bps, it should take around 31.25[ms] for the four bytes to arrive
	while(  (availableData() <=4)  &&  _response_within_timeout){
		processTime = (int) millis();
		processTime = processTime - startTime;							//time since this function started

		_response_within_timeout = (bool) (processTime <= TIME_OUT);	//is the communication within the allowed time?
	}

	//Even if there was a TIME_OUT, if there are available bytes in the buffer, read them
	//DuoDMXL < v.1.0
	//while (availableData() > 0){

	//Only proceed if there was no problem in the communication
	while (availableData() > 0 && _response_within_timeout){
		Incoming_Byte = readData();										//First byte of the header

		//For now, DuoDMXL assumes there are no problems in the rest of the communication. TODO: put safeguard here
		if ( (Incoming_Byte == 255) && (peekData() == 255) ){
			readData();                                    				//Second byte of the header
			readData();                                    				//Dynamixel ID
			lengthMessage = readData();									//Length of the message

			while( !availableData() ){}									//Do nothing until the next byte is in the buffer
			Error_Byte = readData();                       				//Error

			if((lengthMessage-2) == 0){
				data = Error_Byte;										//No data is returned. Send Error_Byte. This is the common response when sending commands
			}
			else if( (lengthMessage-2) == 1 ){
				while( !availableData() ){}
				dataLSB = readData();            						//LSB of the data
				data = (int) dataLSB;
			}
			else if((lengthMessage-2) == 2){
				while( !availableData() ){}
				dataLSB = readData();            						//LSB of the data
				while( !availableData() ){}
				dataMSB = readData();									//MSB of the data
				data = dataMSB << 8;
				data = data + dataLSB;
			}
			else{
				data = LENGTH_INCORRECT;								//The length was not correct or there was some problem
			}

			while( !availableData() ){}
			readData();													//checksum

			delay(COOL_DOWN);
			return (data);
		}
	}

	//If there was a TIME_OUT and not enough bytes, there was an error in communication
	delay(COOL_DOWN);
	return (NO_SERVO_RESPONSE);											// No servo Response
}

// Public Methods //////////////////////////////////////////////////////////////

//Function to set the value of a servo's address. noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need to send
int DynamixelClass::sendWord(uint8_t ID, uint8_t address, int param, int noParams){
	uint8_t param_MSB, param_LSB;
	param_MSB = param >> 8;
	param_LSB = param;

	uint8_t length = noParams + 3;				//instruction(address) + noParams + 2 = noParams + 3

	if(noParams == ONE_BYTE){
		Checksum = (~(ID + length + DMXL_WRITE_DATA + address + param_LSB))&0xFF;
	}
	else if(noParams == TWO_BYTES){
		Checksum = (~(ID + length + DMXL_WRITE_DATA + address + param_LSB + param_MSB))&0xFF;
	}

	switchCom(Direction_Pin,Tx_MODE);
	sendData(DMXL_START);
	sendData(DMXL_START);
	sendData(ID);
	sendData(length);
	sendData(DMXL_WRITE_DATA);
	sendData(address);
	sendData(param_LSB);

	if(noParams == TWO_BYTES){
		sendData(param_MSB);
	}

	sendData(Checksum);
	serialFlush();
	switchCom(Direction_Pin,Rx_MODE);

	//Depending on the current value of statusReturnLevel read or skip the status package
	//if writting a word, then we only expect a package when statusReturnLevel is RETURN_ALL or PING was used (PING not currently supported)
	if(statusReturnLevel == RETURN_ALL){
		return(readInformation());
	}
	else{
		return(NO_ERROR);
	}
}

//Function to read the value of a servo's address. noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need
int DynamixelClass::readWord(uint8_t ID, uint8_t address, int noParams){

	Checksum = (~(ID + LENGTH_READ + DMXL_READ_DATA + address + noParams))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(DMXL_START);
	sendData(DMXL_START);
	sendData(ID);
	sendData(LENGTH_READ);
	sendData(DMXL_READ_DATA);
	sendData(address);
	sendData(noParams);
	sendData(Checksum);
	serialFlush();
	switchCom(Direction_Pin,Rx_MODE);

	//Depending on the current value of statusReturnLevel read or skip the status package
	//if reading a word, then we only expect a package when statusReturnLevel is RETURN_ALL or RETURN_READ
	if(statusReturnLevel == RETURN_NONE){
		return(NO_ERROR);
	}
	else{
		return(readInformation());
	}
}

//Initialize communication with the servos with a user-defined pin for the data direction control
//By default, always set status return level as RETURN_ALL
void DynamixelClass::begin(long baud, uint8_t directionPin){
	Direction_Pin = directionPin;
	setDPin(Direction_Pin, OUTPUT);
	beginCom(baud);
}

//Initialize communication with the servos with a pre-defined pin (D15) for the data direction control
void DynamixelClass::begin(long baud){
	setDPin(Direction_Pin, OUTPUT);
	beginCom(baud);
}

//End communication
void DynamixelClass::end(){
	endCom();
}

//Function inherited from Savage's library. NOT fully tested or supported by DuoDMXL yet
int DynamixelClass::reset(uint8_t ID){
	Checksum = (~(ID + DMXL_RESET_LENGTH + DMXL_RESET))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(DMXL_START);
	sendData(DMXL_START);
	sendData(ID);
	sendData(DMXL_RESET_LENGTH);
	sendData(DMXL_RESET);
	sendData(Checksum);
	serialFlush();
	switchCom(Direction_Pin,Rx_MODE);

	return (read_error());
}

//Function inherited from Savage's library. Not fully tested or supported by DuoDMXL yet
int DynamixelClass::ping(uint8_t ID){
	Checksum = (~(ID + DMXL_READ_DATA + DMXL_PING))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
	sendData(DMXL_START);
	sendData(DMXL_START);
	sendData(ID);
	sendData(DMXL_READ_DATA);
	sendData(DMXL_PING);
	sendData(Checksum);
	serialFlush();
	switchCom(Direction_Pin,Rx_MODE);

  return (read_error());
}

//Function inherited from Savage's library. Not fully tested or supported by DuoDMXL yet
void DynamixelClass::action(){
	switchCom(Direction_Pin,Tx_MODE);
    sendData(DMXL_START);                // Send Instructions over Serial
    sendData(DMXL_START);
    sendData(BROADCAST_ID);
    sendData(DMXL_ACTION_LENGTH);
    sendData(DMXL_ACTION);
    sendData(DMXL_ACTION_CHECKSUM);
	serialFlush();
	switchCom(Direction_Pin,Rx_MODE);
}

//Function to read the servo model. EEPROM Address 0(x00) and 1(0x01)
int DynamixelClass::readModel(uint8_t ID){
	return(readWord(ID, EEPROM_MODEL_NUMBER_L, TWO_BYTES));
}

//Function to read the version of the firmware. EEPROM Address 2(0x02)
int DynamixelClass::readFirmware(uint8_t ID){
	return(readWord(ID, EEPROM_VERSION, ONE_BYTE));
}

//Function to set the ID of the servo. EEPROM Address 3(0x03)
int DynamixelClass::setID(uint8_t ID, uint8_t newID){
	return(sendWord(ID, EEPROM_ID, newID, ONE_BYTE));
}

//Function to read the ID of the servo. EEPROM Address 3(0x03)
int DynamixelClass::readID(uint8_t ID){
	return(readWord(ID, EEPROM_ID, ONE_BYTE));
}

//Function to set baudrate. EEPROM Address 4(0x04).
//The baudrate change takes effect immediately. You need to use end() and begin() with the new baudrate
int DynamixelClass::setBD(uint8_t ID, long baud){
	uint8_t Baud_Rate = round((2000000.0/(float) baud) -1);

	return(sendWord(ID, EEPROM_BAUD_RATE, Baud_Rate, ONE_BYTE));
}

//Function to set baudrate based on the manual's table. EEPROM Address 4(0x04)
//The baudrate change takes effect immediately. You need to use end() and begin() with the new baudrate
int DynamixelClass::setBDTable(uint8_t ID, uint8_t baud){
	return(sendWord(ID, EEPROM_BAUD_RATE, baud, ONE_BYTE));
}

//Function to read the setting of the baudrate. EEPROM Address 4(0x04)
int DynamixelClass::readBD(uint8_t ID){
	return(readWord(ID, EEPROM_BAUD_RATE, ONE_BYTE));
}

//Set the Return Delay Time (RDT) in microseconds. EEPROM Address 5(0x05)
int DynamixelClass::setRDT(uint8_t ID, uint8_t RDT){
	return(sendWord(ID, EEPROM_RETURN_DELAY_TIME, RDT/2, ONE_BYTE));
}

//Read the Return Delay Time (RDT) value. EEPROM Address 5(0x05)
int DynamixelClass::readRDT(uint8_t ID){
	return(readWord(ID, EEPROM_RETURN_DELAY_TIME, ONE_BYTE));
}

//Set the value for the CW Angle limit. EEPROM Address 6(0x06) and 7(0x07)
int DynamixelClass::setCWAngleLimit(uint8_t ID, int limit){
	return(sendWord(ID, EEPROM_CW_ANGLE_LIMIT_L, limit, TWO_BYTES));
}

//Read the value for the CW Angle limit. EEPROM Address 6(0x06) and 7(0x07)
int DynamixelClass::readCWAngleLimit(uint8_t ID){
	return(readWord(ID, EEPROM_CW_ANGLE_LIMIT_L, TWO_BYTES));
}

//Set the value for the CCW Angle limit. EEPROM Address 8(0x08) and 9(0x09)
int DynamixelClass::setCCWAngleLimit(uint8_t ID, int limit){
	return(sendWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, limit, TWO_BYTES));
}

//Read the value for the CCW Angle limit. EEPROM Address 8(0x08) and 9(0x09)
int DynamixelClass::readCCWAngleLimit(uint8_t ID){
	return(readWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, TWO_BYTES));
}

//Set the limit temperature. EEPROM Address 11(0x0B)
int DynamixelClass::setTempLimit(uint8_t ID, uint8_t Temperature){
	return(sendWord(ID, EEPROM_LIMIT_TEMPERATURE, Temperature, ONE_BYTE));
}

//Read the limit temperature. EEPROM Address 11(0x0B)
int DynamixelClass::readTempLimit(uint8_t ID){
	return(readWord(ID, EEPROM_LIMIT_TEMPERATURE, ONE_BYTE));
}

//Set the lowest voltage limit. EEPROM Address 12(0x0C)
int DynamixelClass::setLowVoltageLimit(uint8_t ID, uint8_t lowVoltage){
	return(sendWord(ID, EEPROM_DOWN_LIMIT_VOLTAGE, lowVoltage, ONE_BYTE));
}

//Read the lowest voltage limit. EEPROM Address 12(0x0C)
int DynamixelClass::readLowVoltageLimit(uint8_t ID){
	return(readWord(ID, EEPROM_DOWN_LIMIT_VOLTAGE, ONE_BYTE));
}

//Set the highest voltage limit. EEPROM Address 13(0x0D)
int DynamixelClass::setHighVoltageLimit(uint8_t ID, uint8_t highVoltage){
	return(sendWord(ID, EEPROM_UP_LIMIT_VOLTAGE, highVoltage, ONE_BYTE));
}

//Read the highest voltage limit. EEPROM Address 13(0x0D)
int DynamixelClass::readHighVoltageLimit(uint8_t ID){
	return(readWord(ID, EEPROM_UP_LIMIT_VOLTAGE, ONE_BYTE));
}

//Set the maximum torque. EEPROM Address 14(0x0E) and 15(0x0F)
int DynamixelClass::setMaxTorque(uint8_t ID, int MaxTorque){
	return(sendWord(ID, EEPROM_MAX_TORQUE_L, MaxTorque, TWO_BYTES));
}

//Read the maximum torque. EEPROM Address 14(0x0E) and 15(0x0F)
int DynamixelClass::readMaxTorque(uint8_t ID){
	return(readWord(ID, EEPROM_MAX_TORQUE_L, TWO_BYTES));
}

//Set the Status Return Level. EEPROM Address 16(0x10).
//DuoDMXL assumes that upon reset, all servos have RETURN_ALL. If the value was changed in a previos session, use setSRL(BROADCAST_ID, 2)
int DynamixelClass::setSRL(uint8_t ID, uint8_t SRL){

	//Send the new desired status return level
	int error = sendWord(ID, EEPROM_RETURN_LEVEL, SRL, ONE_BYTE);
	//change the current value of statusReturnLevel
	statusReturnLevel = SRL;

	return(error);
}

//Read the Status Return Level value. EEPROM Address 16(0x10)
int DynamixelClass::readSRL(uint8_t ID){
	return(readWord(ID, EEPROM_RETURN_LEVEL, ONE_BYTE));
}

//Set Alarm LED. EEPROM Address 17(0x11)
int DynamixelClass::setAlarmLED(uint8_t ID, uint8_t alarm){
	return(sendWord(ID, EEPROM_ALARM_LED, alarm, ONE_BYTE));
}

//Read Alarm LED value. EEPROM Address 17(0x11)
int DynamixelClass::readAlarmLED(uint8_t ID){
	return(readWord(ID, EEPROM_ALARM_LED, ONE_BYTE));
}

//Set Shutdown alarm. EEPROM Address 18(0x12)
int DynamixelClass::setShutdownAlarm(uint8_t ID, uint8_t SALARM){
	return(sendWord(ID, EEPROM_ALARM_SHUTDOWN, SALARM, ONE_BYTE));
}

//Read Shutdown alarm value. EEPROM Address 18(0x12)
int DynamixelClass::readShutdownAlarm(uint8_t ID){
	return(readWord(ID, EEPROM_ALARM_SHUTDOWN, ONE_BYTE));
}

//Set the multi-turn offset values. EEPROM ADDRESS: 20(0x14) and 21(0x15)
int DynamixelClass::setMultiTurnOffset(uint8_t ID, int offset){
	return(sendWord(ID, EEPROM_TURN_OFFSET_L, offset, TWO_BYTES));
}

//Read the multi-turn offset values. EEPROM ADDRESS: 20(0x14) and 21(0x15)
int DynamixelClass::readMultiTurnOffset(uint8_t ID){
	return(readWord(ID, EEPROM_TURN_OFFSET_L, TWO_BYTES));
}

//Set the resolution divider value. EEPROM ADDRESS: 22(0x16)
int DynamixelClass::setResolutionDivider(uint8_t ID, uint8_t divider){
	return(sendWord(ID, EEPROM_RESOLUTION_DIV, divider, ONE_BYTE));
}

//Read the resolution divider value. EEPROM ADDRESS: 22(0x16)
int DynamixelClass::readResolutionDivider(uint8_t ID){
	return(readWord(ID, EEPROM_RESOLUTION_DIV, ONE_BYTE));
}

//FUNCTIONS TO ACCESS COMMANDS IN THE RAM AREA

//Function to turn ON or OFF torque. RAM Address 24(0x18)
int DynamixelClass::torqueEnable( uint8_t ID, bool Status){
	return(sendWord(ID, RAM_TORQUE_ENABLE, (int) Status, ONE_BYTE));
}

//Function to check if the servo generates torque. RAM Address 24(0x18)
int DynamixelClass::torqueEnableStatus( uint8_t ID){
	return(readWord(ID, RAM_TORQUE_ENABLE, ONE_BYTE));
}

//Function to turn ON or OFF the servo's LED. RAM Address 25(0x19)
int DynamixelClass::ledStatus(uint8_t ID, bool Status){
	return(sendWord(ID, RAM_LED, (int) Status, ONE_BYTE));
}

//Function to set the value of the Derivative gain. RAM Address 26(0x1A)
int DynamixelClass::setGainD(uint8_t ID, int gain){
	return(sendWord(ID, RAM_D_GAIN, gain, ONE_BYTE));
}

//Function to read the value of the Derivative gain. RAM Address 26(0x1A)
int DynamixelClass::readGainD(uint8_t ID){
	return(readWord(ID, RAM_D_GAIN, ONE_BYTE));
}

//Function to set the value of the Integral gain. RAM Address 27(0x1B)
int DynamixelClass::setGainI(uint8_t ID, int gain){
	return(sendWord(ID, RAM_I_GAIN, gain, ONE_BYTE));
}

//Function to read the value of the Integral gain. RAM Address 27(0x1B)
int DynamixelClass::readGainI(uint8_t ID){
	return(readWord(ID, RAM_I_GAIN, ONE_BYTE));
}

//Function to set the value of the Proportional gain. RAM Address 28(0x1C)
int DynamixelClass::setGainP(uint8_t ID, int gain){
	return(sendWord(ID, RAM_P_GAIN, gain, ONE_BYTE));
}

//Function to read the value of the Proportional gain. RAM Address 28(0x1C)
int DynamixelClass::readGainP(uint8_t ID){
	return(readWord(ID, RAM_P_GAIN, ONE_BYTE));
}

//Function to move servo to a specific position. RAM Address 30(0x1E) and 31(0x1F)
int DynamixelClass::move(uint8_t ID, int Position){
	return(sendWord(ID, RAM_GOAL_POSITION_L, Position, TWO_BYTES));
}

//Function inherited from Savage's library. Not fully tested or supported by DuoDMXL yet
int DynamixelClass::moveSpeed(uint8_t ID, int Position, int Speed){
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

	Checksum = (~(ID + DMXL_GOAL_SP_LENGTH + DMXL_WRITE_DATA + RAM_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;

	switchCom(Direction_Pin,Tx_MODE);
    sendData(DMXL_START);                // Send Instructions over Serial
    sendData(DMXL_START);
    sendData(ID);
    sendData(DMXL_GOAL_SP_LENGTH);
    sendData(DMXL_WRITE_DATA);
    sendData(RAM_GOAL_POSITION_L);
    sendData(Position_L);
    sendData(Position_H);
    sendData(Speed_L);
    sendData(Speed_H);
    sendData(Checksum);
	serialFlush();;
	switchCom(Direction_Pin,Rx_MODE);

    return (read_error());               // Return the read error
}

//Function to set the desired moving speed. RAM Address 32(0x20) and 33(0x21)
int DynamixelClass::setMovingSpeed(uint8_t ID, int speed){
	return(sendWord(ID, RAM_GOAL_SPEED_L, speed, TWO_BYTES));
}

//Function to read the desired moving speed. RAM Address 32(0x20) and 33(0x21)
int DynamixelClass::readMovingSpeed(uint8_t ID){
	return(readWord(ID, RAM_GOAL_SPEED_L, TWO_BYTES));
}

//Function to set the value of the goal torque. RAM Address 34(0x22) and 35(0x23)
int DynamixelClass::setTorqueLimit(uint8_t ID, int torque){
	return(sendWord(ID, RAM_TORQUE_LIMIT_L, torque, TWO_BYTES));
}

//Function to read the value of the goal torque. RAM Address 34(0x22) and 35(0x23)
int DynamixelClass::readTorqueLimit(uint8_t ID){
	return(readWord(ID, RAM_TORQUE_LIMIT_L, TWO_BYTES));
}

//Read the actual position. RAM Address 36(0x24) and 37(0x25)
int DynamixelClass::readPosition(uint8_t ID){
	return(readWord(ID, RAM_PRESENT_POSITION_L, TWO_BYTES));
}

//Read the actual speed. RAM Address 38(0x26) and 39(0x27)
int DynamixelClass::readSpeed(uint8_t ID){
	return(readWord(ID, RAM_PRESENT_SPEED_L, TWO_BYTES));
}

//Read the load. RAM Address 40(0x28) and 41(0x29)
int DynamixelClass::readLoad(uint8_t ID){
	return(readWord(ID, RAM_PRESENT_LOAD_L, TWO_BYTES));
}

//Function to read the voltage. RAM Address 42(0x2A)
int DynamixelClass::readVoltage(uint8_t ID){
	readWord(ID, RAM_PRESENT_VOLTAGE, ONE_BYTE);
}

//Function to read the Temperature. RAM Address 43(0x2B)
int DynamixelClass::readTemperature(uint8_t ID){
	readWord(ID, RAM_PRESENT_TEMPERATURE, ONE_BYTE);
}

//Check if there is an instruction registered. RAM Address 44(0x2C)
int DynamixelClass::registeredStatus(uint8_t ID){
	return(readWord(ID, RAM_REGISTERED_INSTRUCTION, ONE_BYTE));
}

//Check if goal position command is being executed (Address 0x30?). RAM Address 46(0x2E)
int DynamixelClass::moving(uint8_t ID){
	return(readWord(ID, RAM_MOVING, ONE_BYTE));
}

//Locks the EEPROM. RAM Address 47(0x2F)
int DynamixelClass::lockEEPROM(uint8_t ID){
	return(sendWord(ID, RAM_LOCK, 1, ONE_BYTE));
}

//RAM Address 48(0x30) and 49(0x31)
int DynamixelClass::setPunch(uint8_t ID, int Punch){
	return(sendWord(ID, RAM_PUNCH_L, Punch, TWO_BYTES));
}

//RAM Address 48(0x30) and 49(0x31)
int DynamixelClass::readPunch(uint8_t ID){
	return(readWord(ID, RAM_PUNCH_L, TWO_BYTES));
}

//Function to read the current. RAM ADDRESS: 68(0x44) and 69(0x45)
int DynamixelClass::readCurrent(uint8_t ID){
	return(readWord(ID, RAM_CURRENT_L, TWO_BYTES));
}

//Torque control mode enable. RAM ADDRESS: 70(0x46)
int DynamixelClass::torqueControl( uint8_t ID, bool enable){
	return(sendWord(ID, RAM_TORQUE_CONTROL, (int) enable, ONE_BYTE));
}

//Read the Torque control mode status. RAM ADDRESS: 70(0x46)
int DynamixelClass::readTorqueControl( uint8_t ID){
	return(readWord(ID, RAM_TORQUE_CONTROL, ONE_BYTE));
}

//Function to set the goal torque. RAM ADDRESS: 71(0x47) and 72(0x48)
int DynamixelClass::setGoalTorque(uint8_t ID, int torque){
	return(sendWord(ID, RAM_GOAL_TORQUE_L, torque, TWO_BYTES));
}

//Function to set goal acceleration/ RAM ADDRESS: 73(0x49)
int DynamixelClass::setGoalAccel(uint8_t ID, uint8_t accel){
	return(sendWord(ID, RAM_GOAL_ACCEL, accel, ONE_BYTE));
}

//CUSTOM FUNCTIONS---------------------------------------------------------

//Configure both ID and Baudrate of the servo. By changing the baudrate, the communications will restart automatically
//The next time time you call the servos you need to use the NEW baudrate
void DynamixelClass::configureServo(uint8_t ID, uint8_t newID, long baud){

	setID(ID, newID);
	setBD(newID, baud);

	//End communications and restart with new baudrate
	end();
	begin(baud, Direction_Pin);
}

//Set both angle limits.
void DynamixelClass::setAngleLimit(uint8_t ID, int CWLimit, int CCWLimit){
	sendWord(ID, EEPROM_CW_ANGLE_LIMIT_L, CWLimit, TWO_BYTES);
	sendWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, CCWLimit, TWO_BYTES);
}

//Function to set both limits to 0. The servo is functioning in wheel mode
void DynamixelClass::setWheelMode(uint8_t ID, bool enable){
	 if (enable){
		 setAngleLimit(ID, 0, 0);}
	 else{
		 setAngleLimit(ID, 0, 4095);}
}

//Function to set the servo as joint mode. Equivalent to setWheelMode(ID, false)
void DynamixelClass::setJointMode(uint8_t ID){
 	 setAngleLimit(ID, 0, 4095);
}

//Function to set all gains
void DynamixelClass::setDIP(uint8_t ID, int gainD, int gainI, int gainP){
	setGainD(ID, gainD);
	setGainI(ID, gainI);
	setGainP(ID, gainP);
}

//Function to find the ID of the servo, if you have the correct baudrate. Assume begin() has been called
int DynamixelClass::findByBaudRate(long baudRate){
	int foundID;

	for(int i=0; i<254; i++){												//Search every ID possible
		if( (foundID=readID(i))!=-1 ){										//If we get anything but a communication error, return the value
			return(foundID);
		}
	}

	return(NO_SERVO_RESPONSE);												//Return error if nothing found
}

//Function to find the baudrate to communicate with the servo, if you have the correct ID. Assume begin() has NOT been called
int DynamixelClass::findByID(uint8_t id, uint8_t directionPin){
	int foundID;
	long roundedBaudRate;

	for(int i=0; i<=254; i++){
		roundedBaudRate = (2000000)/(i+1);									//Search every possible baudrate
		begin(roundedBaudRate, directionPin);								//begin communication

		if( (foundID=readID(id)) == id ){									//if we get the same ID as the one we assumed correct
			return(readBD(id));												//return the baudrate
		}

		end();																//end communication
	}

	return(NO_SERVO_RESPONSE);												//If nothing found, return error
}

//Find the servo without having any information. Assume begin() has NOT been called
void DynamixelClass::findServo(uint8_t directionPin){
	int error;
	long roundedBaudRate;
	for(uint8_t i=0; i<=254; i++){									//Try every baudrate
	   roundedBaudRate = (2000000)/(i+1);
	   begin(roundedBaudRate, directionPin);

	   for(uint8_t j=0; j<254; j++){									//Try every ID

	     if( (error=readID(j)) != -1){										//If we get anything but an error
	       //digitalWrite(led1, HIGH);
	       Serial.print("Attempting ID: ");
	       Serial.print(j);
	       Serial.print(", attempted baud rate is: ");
	       Serial.print(i);
	       Serial.print(", and the returned baudrate is: ");
	       Serial.println(readBD(j));
	     }
	     else{
	       Serial.print("Attempted ID: ");
	       Serial.print(j);
	       Serial.print(", attempted baud rate is: ");
	       Serial.println(i);
	       }
	   }
	   end();
	 }
}

//Change time out period (waiting time for status package). Unit is [ms]. The maximum value is 255
void DynamixelClass::changeTimeOut(uint8_t newTimeOut){
	TIME_OUT = newTimeOut;
}

//Change cool down period (time between sending commands). Unit is [ms]. The maximum value is 65,535 (i.e., 65.535 seconds)
void DynamixelClass::changeCoolDown(uint16_t newCoolDown){
	COOL_DOWN = newCoolDown;
}

DynamixelClass Dynamixel;
