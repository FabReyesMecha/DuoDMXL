/*
DuoDMXL v2.0
MX-64AR Half Duplex USART/RS-485 Communication Library
-----------------------------------------------------------------------------
Target Boards:
	Redbear Duo
	Particle Photon (not tested)
	Arduino Leonardo
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

2018-02-26		v2.0	Stable Version
 						All functions tested
 						Added new example _05-Move_multiple_servos
2018-02-19:		v1.8	sendWords() works correctly
2018-02-19:		v1.7	Tested more throughoughly readWords(). Timming tests
 						Improved sendWord(). Correct use when using ping(), BROADCAST_ID, and SRL
2018-02-17:		v1.6.1	set/get methods for _directionPin and _baudrateDMXL
						Add servoIntroduction() and printPC/printlnPC macros
2018-02-16:		v1.6	Add waitData() functions. Avoid blocking if no response is obtained
2018-02-03:		v1.5	Add setAng() functions for specifying desired units
2018-01-15:		v1.4	Add multi-compatibility functions instead of macros
2018-01-09:		v1.3	Add automatic selection of Pins
2017-10-27:		v1.2	Created readWords() for bulk reading values from several servos
2017-10-24:		v1.1	Created setBoardSRL() to change the board's SRL. Assumes all servos have the same SRL value
						sendWord() and readWord() now send whole arrays instead of byte-by-byte
						sendWords() function created to send information to all servos quickly using REG_WRITE and ACTION
						ping(), reset() and action() functions now supported
2017-05-05:		v1.0	Improved communication safety in readInformation() using the flag _response_within_timeout
						User can change the baudrate in the same session, without reseting the microcontroller
2017-05-04: 	v0.3	Status Return Level (SRL) can now be changed by the user
						TIME_OUT and COOL_DOWN are accessible to the user
2017-04-13: 	v0.2.2	Added extra comments
						changed name read_information() to readInformation()
2016-12-22:		v0.2.1	Modified int to unsigned long in reading serial timeout
						Marked constants specific to MX-64 and AX-12
2016-12-21:		v0.2
2016-06-01:		v0.1

 TODO:
	-Save SRL in the EEPROM
	-Test reset()
-----------------------------------------------------------------------------

 Contact: 	burgundianvolker@gmail.com
 Author:	Fabian Eugenio Reyes Pinner (Fabian Reyes)
 */

#include "DuoDMXL.h"

// Macro for Comunication Flow Control
#if (PLATFORM_ID==88) || defined(SPARK)
	#define setDPin(DirPin,Mode)	(pinMode(DirPin,Mode))     	// Select the Switch to TX/RX Mode Pin
	#define switchCom(DirPin,Mode) 	(digitalWrite(DirPin,Mode))	// Switch to TX/RX Mode
	#define printPC(value) 			(Serial.print(value))  		// Print to the PC
	#define printlnPC(value) 		(Serial.println(value))  	// Print to the PC
#else
	#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
	#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode
	#define printPC(value) 			(Serial.print(value))  		// Print to the PC
	#define printlnPC(value) 		(Serial.println(value))  	// Print to the PC
#endif

// Private Methods //////////////////////////////////////////////////////////////

//General function to read the status package from the servo
int DynamixelClass::readInformation(void)
{
	unsigned long startTime = millis();
	int processTime, lengthMessage, dataLength;

	//Reset flag
	_response_within_timeout = true;

	//Reset the buffer
	memset(RESPONSE, 0, 64);

	//DuoDMXL v.1.6. Do nothing until we have the start bytes, ID, length of the message (four bytes). Even at 9600bps, it should take around 3.33[ms] for the four bytes to arrive
	waitData(4, (int) TIME_OUT);

	//DuoDMXL v.1.0+. Only proceed if there was no problem in the communication
	while ( (availableData() > 0) && _response_within_timeout){
		Incoming_Byte = readData();										//First byte of the header

		//For now, DuoDMXL assumes there are no problems in the rest of the communication
		if ( (Incoming_Byte == 255) && (peekData() == 255) ){

			//Fill buffer for debugging
			RESPONSE[0] = Incoming_Byte;
			RESPONSE[1] = readData();                                  	//Second byte of the header
			RESPONSE[2] = readData();                                  	//Dynamixel ID
			RESPONSE[3] = lengthMessage = readData();					//Length of the message
			dataLength = lengthMessage-2;

			waitData((int) TIME_OUT);									//Do nothing until the next byte is in the buffer
			RESPONSE[4] = Error_Byte = readData();                     	//Error

			if(dataLength == 0){
				data = Error_Byte;										//No data is returned. Send Error_Byte. This is the common response when sending commands
			}
			else if( dataLength == 1 ){
				waitData((int) TIME_OUT);

				dataLSB = readData();            						//LSB of the data
				data = (int) dataLSB;

				RESPONSE[5] = dataLSB;
			}
			else if(dataLength == 2){
				waitData(1, TIME_OUT);

				dataLSB = readData();            						//LSB of the data
				dataMSB = readData();									//MSB of the data
				data = dataMSB << 8;
				data = data + dataLSB;

				RESPONSE[5] = dataLSB;
				RESPONSE[6] = dataMSB;
			}
			else{
				data = LENGTH_INCORRECT;								//The length was not correct or there was some problem
			}

			waitData((int) TIME_OUT);
			RESPONSE[MIN_RETURN_LEN + dataLength] = readData();			//checksum

			delayms(COOL_DOWN);
			return (data);
		}
	}

	//If there was a TIME_OUT and not enough bytes, there was an error in communication
	delayms(COOL_DOWN);
	return (NO_SERVO_RESPONSE);											// No servo Response
}

//Wait for a certain number of bytes 'length', for a certain amount of time 'timeLimit' [ms]
bool DynamixelClass::waitData(int length, int timeLimit){

	unsigned long startTime = millis();
	int processTime;
    bool response_within_timeout = true;

    while(  (availableData() <=length)  &&  response_within_timeout){
        processTime = (int) millis();
        processTime = processTime - startTime;                            //time since this function started

        response_within_timeout = (bool) (processTime <= timeLimit );    //is the communication within the allowed time?
    }

	//Set global flag
	_response_within_timeout = response_within_timeout;
    return response_within_timeout;
}

//Wait for one byte, for a certain amount of time 'timeLimit' [ms]
bool DynamixelClass::waitData(int timeLimit){
    return waitData(0, timeLimit);
}

//keep waiting until there is at least one byte in the buffer
bool DynamixelClass::waitData(){
    while( !availableData() ){}
	return true;
}

//-----------Public Methods-------------------------------------------------

/*
Function to set (write) the value of a servo's address.
noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need to send
In general, instruction should be either DMXL_WRITE_DATA or DMXL_REG_WRITE
*/
int DynamixelClass::sendWord(uint8_t ID, uint8_t address, int param, int noParams, uint8_t instruction){
	uint8_t param_MSB, param_LSB, length, lengthPackage;
	uint8_t *package = NULL;

	param_MSB = param >> 8;
	param_LSB = param;

	//Based on numbers of parameters, decide size of outgoing package and calculate checksum
	if(noParams == 0){
		length = 2;									//instruction + checksum = 2
		Checksum = (~(ID + length + instruction))&0xFF;

		lengthPackage = 6;
		package = new uint8_t[lengthPackage];

		package[5] = Checksum;
	}
	else if(noParams == ONE_BYTE){
		length = 4;									//instruction + address + param_LSB + checksum = noParams + 3 = 4
		Checksum = (~(ID + length + instruction + address + param_LSB))&0xFF;

		lengthPackage = 8;
		package = new uint8_t[lengthPackage];

		package[5] = address;
		package[6] = param_LSB;
		package[7] = Checksum;
	}
	else if(noParams == TWO_BYTES){
		length = 5;									//instruction + address + param_LSB + param_MSB + checksum = noParams + 3 = 5
		Checksum = (~(ID + length + instruction + address + param_LSB + param_MSB))&0xFF;

		lengthPackage = 9;
		package = new uint8_t[lengthPackage];

		package[5] = address;
		package[6] = param_LSB;
		package[7] = param_MSB;
		package[8] = Checksum;
	}

	//Rest of the package, common for either a 0, ONE_BYTE or TWO_BYTES package
	package[0] = DMXL_START;
	package[1] = DMXL_START;
	package[2] = ID;
	package[3] = length;
	package[4] = instruction;

	switchCom(_directionPin,Tx_MODE);
	sendDataBuff(package, lengthPackage);
	serialFlush();
	switchCom(_directionPin,Rx_MODE);

	//free(package);
	delete[] package;

	/*
	DuoDMXL v.1.7+. Usage of ping() seems to take priority over usage of BROADCAST_ID
	If writting a word, then we only expect a package when PING was used, or statusReturnLevel is RETURN_ALL and not using broadcast ID
	*/

	//if( (ID!=BROADCAST_ID) && ((statusReturnLevel==RETURN_ALL) || (instruction == DMXL_PING))  ){ //Works correctly when using ping(ID). Communication error if using ping(BROADCAST_ID)
	if( (instruction == DMXL_PING) || ((statusReturnLevel==RETURN_ALL) && (ID!=BROADCAST_ID)) ){
		return(readInformation());
	}
	else{
		return(NO_ERROR);
	}
}

//Function to set the value of a several servos' address. noIDs is how many servos we are communicating with
//params is an array with values (one for each servo). noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need to send per servo
int DynamixelClass::sendWords(uint8_t IDs[], uint8_t noIDs, uint8_t address, int params[], int noParams){

	// ----Alternative using REG_WRITE and ACTION (sometimes hangs)----------

	// //1- Disable Status Packet for all servos. Since I am using the BROADCAST_ID I am not expecting a status return
	// uint8_t oldStatusReturnLevel = statusReturnLevel;
	// if(oldStatusReturnLevel != RETURN_READ){
	// 	sendWord(BROADCAST_ID, EEPROM_RETURN_LEVEL, RETURN_READ, ONE_BYTE, DMXL_WRITE_DATA);
	// 	statusReturnLevel = RETURN_READ;
	// }
	//
	// //2- Write params[i] to the registry using DMXL_REG_WRITE
	// for(uint8_t i=0; i<noIDs; i++){
	// 	sendWord(IDs[i], address, params[i], noParams, DMXL_REG_WRITE);
	// }
	//
	// //3- Send DMXL_ACTION to all servos
	// action(BROADCAST_ID);
	//
	// //Re-enable the old status return level. Since I am using the BROADCAST_ID I am not expecting a status return
	// if(oldStatusReturnLevel != RETURN_READ){
	// 	sendWord(BROADCAST_ID, EEPROM_RETURN_LEVEL, oldStatusReturnLevel, ONE_BYTE, DMXL_WRITE_DATA);
	// 	statusReturnLevel = oldStatusReturnLevel;
	// }
	//
	// return(NO_ERROR);

	//-------------Alternative using sync_write-----------------------------

	//1- Prepare information
	uint8_t length = (noParams+1)*noIDs + 4;				//instruction + address + noParams + (1 + noParams)*noIDs + checksum = (1 + noParams)*noIDs + 4
	uint8_t lengthPackage = length + 4;						//DMXL_START + DMXL_START + BROADCAST_ID + length + {package}
	uint8_t package[lengthPackage] = {};
	uint16_t tempChecksum = 0;

	//2- Buffer to hold all the information
	package[0]=DMXL_START;
	package[1]=DMXL_START;
	package[2]=BROADCAST_ID;
	package[3]=length;
	package[4]=DMXL_SYNC_WRITE;
	package[5]=address;
	package[6]=noParams;

	for(uint8_t i=0; i<noIDs; i++){
		//Save ID
		package[7 + i*(1 + noParams)] = IDs[i];

		//Save LSB and MSB
		for(uint8_t j=0; j<noParams; j++){
			package[7 + i*(1 + noParams) + 1 + j] = (uint8_t) ( params[i] >> (8*j) );
		}
	}

	//Obtain cheksum and save it in the last position of the buffer. Checksum starts at the ID
	for(uint8_t i=2; i<(lengthPackage-1); i++){
		tempChecksum += package[i];
	}
	Checksum = (~tempChecksum)&0xFF;
	package[lengthPackage-1] = Checksum;

	//Send the package
	switchCom(_directionPin,Tx_MODE);
	sendDataBuff(package, lengthPackage);
	serialFlush();
	switchCom(_directionPin,Rx_MODE);
}

//Function to read the value of a servo's address. noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need
int DynamixelClass::readWord(uint8_t ID, uint8_t address, int noParams){

	Checksum = (~(ID + LENGTH_READ + DMXL_READ_DATA + address + noParams))&0xFF;

	//Prepare a buffer with all information
	uint8_t package[8] = {DMXL_START, DMXL_START, ID, LENGTH_READ, DMXL_READ_DATA, address, noParams, Checksum};

	switchCom(_directionPin,Tx_MODE);
	sendDataBuff(package, 8);
	serialFlush();
	switchCom(_directionPin,Rx_MODE);

	//Depending on the current value of statusReturnLevel read or skip the status package
	//if reading a word, then we only expect a package when statusReturnLevel is RETURN_ALL or RETURN_READ
	if(statusReturnLevel==RETURN_NONE){
		return(NO_ERROR);
	}
	else{
		return(readInformation());
	}
}

//Function to read the value of a several servos. noParams should be ONE_BYTE or TWO_BYTES, depending on how many bytes we need
//This function assumes address and noParams is the same for all servos. It is necessary to pass a previously allocated array
void DynamixelClass::readWords(uint8_t IDs[], uint8_t noIDs, uint8_t address, int noParams, int *response){

	//1- Prepare information
	uint8_t length = 3*noIDs + 3;					//instruction + 0x00 + noParams1 + ID1 + address1 + ... + noParamsN + IDN + addressN + Checksum
	uint8_t lengthPackage = length + 4;				//DMXL_START + DMXL_START + BROADCAST_ID + length + ...
	uint8_t package[lengthPackage] = {};
	uint16_t tempChecksum = 0;

	//2- I need to create a buffer to hold all the information
	package[0] = DMXL_START;
	package[1] = DMXL_START;
	package[2] = BROADCAST_ID;
	package[3] = length;
	package[4] = DMXL_BULK_READ;
	package[5] = 0x00;

	for(uint8_t i=0; i<noIDs; i++){
		package[6 + i*3] = noParams;
		package[6 + i*3 + 1] = IDs[i];
		package[6 + i*3 + 2] = address;
	}

	//3- Obtain cheksum and save it in the last position of the buffer. Checksum starts at the ID
	for(uint8_t i=2; i<(lengthPackage-1); i++){
		tempChecksum += package[i];
	}

	Checksum = (~tempChecksum)&0xFF;
	package[lengthPackage-1] = Checksum;

	//4- Send the package
	switchCom(_directionPin,Tx_MODE);
	sendDataBuff(package, lengthPackage);
	serialFlush();
	switchCom(_directionPin,Rx_MODE);

	//5- Read the return package and save it
	for(uint8_t i=0; i<noIDs; i++){
		response[i] = readInformation();
	}

}

//Initialize communication with the servos with a user-defined pin for the data direction control
//By default, always set status return level as RETURN_ALL. TODO: Read SRL from either the servos, or the EEPROM
void DynamixelClass::begin(long baud, uint8_t directionPin){

	//Save private variables
	setBaudrateDMXL(baud);
	setDirectionPin(directionPin);

	//Start connection using the selected pin
	setDPin(_directionPin, OUTPUT);
	beginCom(baud);
	delayms(DMXLSERIAL_TIMEOUT);

	//Set the SRL to RETURN_ALL
	setSRL(BROADCAST_ID, RETURN_ALL);
}

//Initialize communication with the servos with a pre-defined pin (D15) for the data direction control
void DynamixelClass::begin(long baud){

	//Save private variables
	setBaudrateDMXL(baud);

	//Start communication with the default pin
	setDPin(_directionPin, OUTPUT);
	beginCom(baud);
	delayms(DMXLSERIAL_TIMEOUT);

	//Set the SRL to RETURN_ALL
	setSRL(BROADCAST_ID, RETURN_ALL);
}

//End communication
void DynamixelClass::end(){
	endCom();
}

//Function inherited from Savage's library
int DynamixelClass::reset(uint8_t ID){

	// -----------old method. Preserved for reference
	Checksum = (~(ID + LENGTH_RESET + DMXL_RESET))&0xFF;

	uint8_t package[6] = {DMXL_START, DMXL_START, ID, LENGTH_RESET, DMXL_RESET, Checksum};

	switchCom(_directionPin,Tx_MODE);
	sendDataBuff(package, 6);
	serialFlush();
	switchCom(_directionPin,Rx_MODE);

	//Upon reset, SRL should be set to RETURN_ALL
	statusReturnLevel = RETURN_ALL;

	return(readInformation());

	// -----------new method
	// //Upon reset, SRL should be set to RETURN_ALL
	// statusReturnLevel = RETURN_ALL;
	// return(sendWord(ID, 0, 0, 0, DMXL_RESET));
}

//Ping a servo
int DynamixelClass::ping(uint8_t ID){
	return(sendWord(ID, 0, 0, 0, DMXL_PING));
}

//Send the ACTION Instruction
void DynamixelClass::action(uint8_t ID){
	sendWord(ID, 0, 0, 0, DMXL_ACTION);
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
	return(sendWord(ID, EEPROM_ID, newID, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to read the ID of the servo. EEPROM Address 3(0x03)
int DynamixelClass::readID(uint8_t ID){
	return(readWord(ID, EEPROM_ID, ONE_BYTE));
}

//Function to set baudrate. EEPROM Address 4(0x04).
//The baudrate change takes effect immediately. You need to use end() and then begin() with the new baudrate
int DynamixelClass::setBD(uint8_t ID, long baud){
	uint8_t Baud_Rate = round( (2000000.0/((float) baud)) -1 );
	return(sendWord(ID, EEPROM_BAUD_RATE, Baud_Rate, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to set baudrate based on the manual's table. EEPROM Address 4(0x04)
//The baudrate change takes effect immediately. You need to use end() and begin() with the new baudrate
int DynamixelClass::setBDTable(uint8_t ID, uint8_t baud){
	return(sendWord(ID, EEPROM_BAUD_RATE, baud, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to read the setting of the baudrate. EEPROM Address 4(0x04)
int DynamixelClass::readBD(uint8_t ID){
	return(readWord(ID, EEPROM_BAUD_RATE, ONE_BYTE));
}

//Set the Return Delay Time (RDT) in microseconds. EEPROM Address 5(0x05)
int DynamixelClass::setRDT(uint8_t ID, uint8_t RDT){
	return(sendWord(ID, EEPROM_RETURN_DELAY_TIME, RDT/2, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the Return Delay Time (RDT) value. EEPROM Address 5(0x05)
int DynamixelClass::readRDT(uint8_t ID){
	return(readWord(ID, EEPROM_RETURN_DELAY_TIME, ONE_BYTE));
}

//Set the value for the CW Angle limit. EEPROM Address 6(0x06) and 7(0x07)
int DynamixelClass::setCWAngleLimit(uint8_t ID, int limit){
	return(sendWord(ID, EEPROM_CW_ANGLE_LIMIT_L, limit, TWO_BYTES, DMXL_WRITE_DATA));
}

//Read the value for the CW Angle limit. EEPROM Address 6(0x06) and 7(0x07)
int DynamixelClass::readCWAngleLimit(uint8_t ID){
	return(readWord(ID, EEPROM_CW_ANGLE_LIMIT_L, TWO_BYTES));
}

//Set the value for the CCW Angle limit. EEPROM Address 8(0x08) and 9(0x09)
int DynamixelClass::setCCWAngleLimit(uint8_t ID, int limit){
	return(sendWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, limit, TWO_BYTES, DMXL_WRITE_DATA));
}

//Read the value for the CCW Angle limit. EEPROM Address 8(0x08) and 9(0x09)
int DynamixelClass::readCCWAngleLimit(uint8_t ID){
	return(readWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, TWO_BYTES));
}

//Set the limit temperature. EEPROM Address 11(0x0B)
int DynamixelClass::setTempLimit(uint8_t ID, uint8_t Temperature){
	return(sendWord(ID, EEPROM_LIMIT_TEMPERATURE, Temperature, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the limit temperature. EEPROM Address 11(0x0B)
int DynamixelClass::readTempLimit(uint8_t ID){
	return(readWord(ID, EEPROM_LIMIT_TEMPERATURE, ONE_BYTE));
}

//Set the lowest voltage limit. EEPROM Address 12(0x0C)
int DynamixelClass::setLowVoltageLimit(uint8_t ID, uint8_t lowVoltage){
	return(sendWord(ID, EEPROM_DOWN_LIMIT_VOLTAGE, lowVoltage, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the lowest voltage limit. EEPROM Address 12(0x0C)
int DynamixelClass::readLowVoltageLimit(uint8_t ID){
	return(readWord(ID, EEPROM_DOWN_LIMIT_VOLTAGE, ONE_BYTE));
}

//Set the highest voltage limit. EEPROM Address 13(0x0D)
int DynamixelClass::setHighVoltageLimit(uint8_t ID, uint8_t highVoltage){
	return(sendWord(ID, EEPROM_UP_LIMIT_VOLTAGE, highVoltage, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the highest voltage limit. EEPROM Address 13(0x0D)
int DynamixelClass::readHighVoltageLimit(uint8_t ID){
	return(readWord(ID, EEPROM_UP_LIMIT_VOLTAGE, ONE_BYTE));
}

//Set the maximum torque. EEPROM Address 14(0x0E) and 15(0x0F)
int DynamixelClass::setMaxTorque(uint8_t ID, int MaxTorque){
	return(sendWord(ID, EEPROM_MAX_TORQUE_L, MaxTorque, TWO_BYTES, DMXL_WRITE_DATA));
}

//Read the maximum torque. EEPROM Address 14(0x0E) and 15(0x0F)
int DynamixelClass::readMaxTorque(uint8_t ID){
	return(readWord(ID, EEPROM_MAX_TORQUE_L, TWO_BYTES));
}

//Set the Status Return Level. EEPROM Address 16(0x10).
//DuoDMXL assumes that upon reset, all servos have RETURN_ALL. If the value was changed in a previos session, use setSRL(BROADCAST_ID, 2) or setBoardSRL(SRL)
//The change in SRL takes place beginning with the NEXT communication. Even if sending RETURN_NONE, you may still get a status return
int DynamixelClass::setSRL(uint8_t ID, uint8_t SRL){

	//Send the new desired status return level
	int error = sendWord(ID, EEPROM_RETURN_LEVEL, SRL, ONE_BYTE, DMXL_WRITE_DATA);
	statusReturnLevel = SRL;

	return(error);
}

//Read the Status Return Level value. EEPROM Address 16(0x10)
int DynamixelClass::readSRL(uint8_t ID){
	return(readWord(ID, EEPROM_RETURN_LEVEL, ONE_BYTE));
}

//Forcefully set SRL value saved in the DUO, instead of the servo's SRL value
int DynamixelClass::setBoardSRL(uint8_t SRL){
	//change the current value of statusReturnLevel
	statusReturnLevel = SRL;
	return(NO_ERROR);
}

//Set Alarm LED. EEPROM Address 17(0x11)
int DynamixelClass::setAlarmLED(uint8_t ID, uint8_t alarm){
	return(sendWord(ID, EEPROM_ALARM_LED, alarm, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read Alarm LED value. EEPROM Address 17(0x11)
int DynamixelClass::readAlarmLED(uint8_t ID){
	return(readWord(ID, EEPROM_ALARM_LED, ONE_BYTE));
}

//Set Shutdown alarm. EEPROM Address 18(0x12)
int DynamixelClass::setShutdownAlarm(uint8_t ID, uint8_t SALARM){
	return(sendWord(ID, EEPROM_ALARM_SHUTDOWN, SALARM, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read Shutdown alarm value. EEPROM Address 18(0x12)
int DynamixelClass::readShutdownAlarm(uint8_t ID){
	return(readWord(ID, EEPROM_ALARM_SHUTDOWN, ONE_BYTE));
}

//Set the multi-turn offset values. EEPROM ADDRESS: 20(0x14) and 21(0x15)
int DynamixelClass::setMultiTurnOffset(uint8_t ID, int offset){
	return(sendWord(ID, EEPROM_TURN_OFFSET_L, offset, TWO_BYTES, DMXL_WRITE_DATA));
}

//Read the multi-turn offset values. EEPROM ADDRESS: 20(0x14) and 21(0x15)
int DynamixelClass::readMultiTurnOffset(uint8_t ID){
	return(readWord(ID, EEPROM_TURN_OFFSET_L, TWO_BYTES));
}

//Set the resolution divider value. EEPROM ADDRESS: 22(0x16)
int DynamixelClass::setResolutionDivider(uint8_t ID, uint8_t divider){
	return(sendWord(ID, EEPROM_RESOLUTION_DIV, divider, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the resolution divider value. EEPROM ADDRESS: 22(0x16)
int DynamixelClass::readResolutionDivider(uint8_t ID){
	return(readWord(ID, EEPROM_RESOLUTION_DIV, ONE_BYTE));
}

//FUNCTIONS TO ACCESS COMMANDS IN THE RAM AREA

//Function to turn ON or OFF torque. RAM Address 24(0x18)
int DynamixelClass::torqueEnable( uint8_t ID, bool Status){
	return(sendWord(ID, RAM_TORQUE_ENABLE, (int) Status, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to check if the servo generates torque. RAM Address 24(0x18)
int DynamixelClass::torqueEnableStatus( uint8_t ID){
	return(readWord(ID, RAM_TORQUE_ENABLE, ONE_BYTE));
}

//Function to turn ON or OFF the servo's LED. RAM Address 25(0x19)
int DynamixelClass::ledStatus(uint8_t ID, bool Status){
	return(sendWord(ID, RAM_LED, (int) Status, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to set the value of the Derivative gain. RAM Address 26(0x1A)
int DynamixelClass::setGainD(uint8_t ID, int gain){
	return(sendWord(ID, RAM_D_GAIN, gain, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to read the value of the Derivative gain. RAM Address 26(0x1A)
int DynamixelClass::readGainD(uint8_t ID){
	return(readWord(ID, RAM_D_GAIN, ONE_BYTE));
}

//Function to set the value of the Integral gain. RAM Address 27(0x1B)
int DynamixelClass::setGainI(uint8_t ID, int gain){
	return(sendWord(ID, RAM_I_GAIN, gain, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to read the value of the Integral gain. RAM Address 27(0x1B)
int DynamixelClass::readGainI(uint8_t ID){
	return(readWord(ID, RAM_I_GAIN, ONE_BYTE));
}

//Function to set the value of the Proportional gain. RAM Address 28(0x1C)
int DynamixelClass::setGainP(uint8_t ID, int gain){
	return(sendWord(ID, RAM_P_GAIN, gain, ONE_BYTE, DMXL_WRITE_DATA));
}

//Function to read the value of the Proportional gain. RAM Address 28(0x1C)
int DynamixelClass::readGainP(uint8_t ID){
	return(readWord(ID, RAM_P_GAIN, ONE_BYTE));
}

//Function to move servo to a specific position. RAM Address 30(0x1E) and 31(0x1F)
int DynamixelClass::move(uint8_t ID, int Position){
	return(sendWord(ID, RAM_GOAL_POSITION_L, Position, TWO_BYTES, DMXL_WRITE_DATA));
}

//Function to move servos to a specific positions. RAM Address 30(0x1E) and 31(0x1F)
int DynamixelClass::move(uint8_t IDs[], uint8_t noIDs, int Positions[]){
	return(sendWords(IDs, noIDs, RAM_GOAL_POSITION_L, Positions, TWO_BYTES));
}

//Function inherited from Savage's library. Not fully tested or supported by DuoDMXL yet
// int DynamixelClass::moveSpeed(uint8_t ID, int Position, int Speed){
//     char Position_H,Position_L,Speed_H,Speed_L;
//     Position_H = Position >> 8;
//     Position_L = Position;                // 16 bits - 2 x 8 bits variables
//     Speed_H = Speed >> 8;
//     Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables
//
// 	Checksum = (~(ID + DMXL_GOAL_SP_LENGTH + DMXL_WRITE_DATA + RAM_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H))&0xFF;
//
// 	switchCom(_directionPin,Tx_MODE);
//     sendData(DMXL_START);                // Send Instructions over Serial
//     sendData(DMXL_START);
//     sendData(ID);
//     sendData(DMXL_GOAL_SP_LENGTH);
//     sendData(DMXL_WRITE_DATA);
//     sendData(RAM_GOAL_POSITION_L);
//     sendData(Position_L);
//     sendData(Position_H);
//     sendData(Speed_L);
//     sendData(Speed_H);
//     sendData(Checksum);
// 	serialFlush();;
// 	switchCom(_directionPin,Rx_MODE);
//
//     return (read_error());               // Return the read error
// }

//Function to set the desired moving speed. RAM Address 32(0x20) and 33(0x21)
int DynamixelClass::setMovingSpeed(uint8_t ID, int speed){
	return(sendWord(ID, RAM_GOAL_SPEED_L, speed, TWO_BYTES, DMXL_WRITE_DATA));
}

//Function to read the desired moving speed. RAM Address 32(0x20) and 33(0x21)
int DynamixelClass::readMovingSpeed(uint8_t ID){
	return(readWord(ID, RAM_GOAL_SPEED_L, TWO_BYTES));
}

//Function to set the value of the goal torque. RAM Address 34(0x22) and 35(0x23)
int DynamixelClass::setTorqueLimit(uint8_t ID, int torque){
	return(sendWord(ID, RAM_TORQUE_LIMIT_L, torque, TWO_BYTES, DMXL_WRITE_DATA));
}

//Function to read the value of the goal torque. RAM Address 34(0x22) and 35(0x23)
int DynamixelClass::readTorqueLimit(uint8_t ID){
	return(readWord(ID, RAM_TORQUE_LIMIT_L, TWO_BYTES));
}

//Read the actual position of one servo. RAM Address 36(0x24) and 37(0x25)
int DynamixelClass::readPosition(uint8_t ID){
	return(readWord(ID, RAM_PRESENT_POSITION_L, TWO_BYTES));
}

//Read the actual position of several servos. RAM Address 36(0x24) and 37(0x25)
void DynamixelClass::readPosition(uint8_t IDs[], uint8_t noIDs, int *positions){
	readWords(IDs, noIDs, RAM_PRESENT_POSITION_L, TWO_BYTES, positions);
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
	return(readWord(ID, RAM_PRESENT_VOLTAGE, ONE_BYTE));
}

//Function to read the Temperature. RAM Address 43(0x2B)
int DynamixelClass::readTemperature(uint8_t ID){
	return(readWord(ID, RAM_PRESENT_TEMPERATURE, ONE_BYTE));
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
	return(sendWord(ID, RAM_LOCK, 1, ONE_BYTE, DMXL_WRITE_DATA));
}

//RAM Address 48(0x30) and 49(0x31)
int DynamixelClass::setPunch(uint8_t ID, int Punch){
	return(sendWord(ID, RAM_PUNCH_L, Punch, TWO_BYTES, DMXL_WRITE_DATA));
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
	return(sendWord(ID, RAM_TORQUE_CONTROL, (int) enable, ONE_BYTE, DMXL_WRITE_DATA));
}

//Read the Torque control mode status. RAM ADDRESS: 70(0x46)
int DynamixelClass::readTorqueControl( uint8_t ID){
	return(readWord(ID, RAM_TORQUE_CONTROL, ONE_BYTE));
}

//Function to set the goal torque. RAM ADDRESS: 71(0x47) and 72(0x48)
int DynamixelClass::setGoalTorque(uint8_t ID, int torque){
	return(sendWord(ID, RAM_GOAL_TORQUE_L, torque, TWO_BYTES, DMXL_WRITE_DATA));
}

//Function to set goal acceleration/ RAM ADDRESS: 73(0x49)
int DynamixelClass::setGoalAccel(uint8_t ID, uint8_t accel){
	return(sendWord(ID, RAM_GOAL_ACCEL, accel, ONE_BYTE, DMXL_WRITE_DATA));
}

//CUSTOM FUNCTIONS---------------------------------------------------------

//Function to move servo to a specific angle [deg]. RAM Address 30(0x1E) and 31(0x1F)
int DynamixelClass::setAng(uint8_t ID, float angle){
	//transform angle into 0-4095 values
	int Position;
	Position = (int) ( map(angle, 0.0, 360.0, 0.0, 4095.0) );
	Position = constrain(Position, 0, 4095);

	return(sendWord(ID, RAM_GOAL_POSITION_L, Position, TWO_BYTES, DMXL_WRITE_DATA));
}

//Function to move servo to a specific angle [deg]. RAM Address 30(0x1E) and 31(0x1F)
//unit specify the input units: 'b': bit-value, 'd': degrees, 'r': radians
int DynamixelClass::setAng(uint8_t ID, float angle, char unit){

	int Position, error;

	if(unit == 'b'){
		Position = (int) angle;
		error = move(ID, Position);
	}
	else if(unit == 'd'){
		error = setAng(ID, angle);
	}
	else if(unit == 'r'){
		Position = (int) ( map(angle, 0.0, 2*M_PI, 0.0, 4095.0) );
		Position = constrain(Position, 0, 4095);
		error = move(ID, Position);
	}
	else{
		error = -2;
	}

	return(error);
}

//Set the direction pin of the serial communication with DMXL
void DynamixelClass::setDirectionPin(uint8_t pin){
	_directionPin = pin;
}

//Get the direction pin of the serial communication with DMXL
uint8_t DynamixelClass::getDirectionPin(){
	return(_directionPin);
}

//Set the baudrate of the serial communication with DMXL without ending/beginning serial
void DynamixelClass::setBaudrateDMXL(long baud){
	_baudrateDMXL = baud;
}

//Get the baudrate of the serial communication with DMXL
long DynamixelClass::getBaudrateDMXL(){
	return(_baudrateDMXL);
}

//Configure both ID and Baudrate of the servo. By changing the baudrate, the communications will restart automatically
//The next time time you call the servos you need to use the NEW baudrate
void DynamixelClass::configureServo(uint8_t ID, uint8_t newID, long baud){

	setID(ID, newID);
	setBD(newID, baud);

	//End communications and restart with new baudrate
	end();
	begin(baud, _directionPin);
}

//Set both angle limits.
void DynamixelClass::setAngleLimit(uint8_t ID, int CWLimit, int CCWLimit){
	sendWord(ID, EEPROM_CW_ANGLE_LIMIT_L, CWLimit, TWO_BYTES, DMXL_WRITE_DATA);
	sendWord(ID, EEPROM_CCW_ANGLE_LIMIT_L, CCWLimit, TWO_BYTES, DMXL_WRITE_DATA);
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
	       printPC("Attempting ID: ");
	       printPC(j);
	       printPC(", attempted baud rate is: ");
	       printPC(i);
	       printPC(", and the returned baudrate is: ");
	       printlnPC(readBD(j));
	     }
	     else{
	       printPC("Attempted ID: ");
	       printPC(j);
	       printPC(", attempted baud rate is: ");
	       printlnPC(i);
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

//Print information about the servo. Use carefully since it uses the Serial port
void DynamixelClass::servoIntroduction(uint8_t ID){
	printlnPC("------------------------------------------");

	printPC("Hi, I am a servo model ");
	printPC(ID);
	printPC(" with Firmware version ");
	printPC(readFirmware(ID));
	printlnPC(", my ID is ");
	printPC(readID(ID));
	printPC(", communicating at a baudrate of ");
	printPC(readBD(ID));
	printPC(", with a RDT of ");
	printlnPC(readRDT(ID));

	printPC("CW limit set as ");
	printPC(readCWAngleLimit(ID));
	printPC(", CCW limit set as ");
	printlnPC(readCCWAngleLimit(ID));

	printPC("The temperature limit, lowest voltage limit, highest voltage limit, and max torque are: ");
	printPC(readTempLimit(ID));
	printPC(", ");
	printPC(readLowVoltageLimit(ID));
	printPC(", ");
	printPC(readHighVoltageLimit(ID));
	printPC(", ");
	printlnPC(readMaxTorque(ID));

	printPC("My Status return level, Alarm LED, and shutdown alarm settings are: ");
	printPC(readSRL(ID));
	printPC(", ");
	printPC(readAlarmLED(ID));
	printPC(", ");
	printlnPC(readShutdownAlarm(ID));

	printPC("The multi-turn offset setting and resolution divider are: ");
	printPC(readMultiTurnOffset(ID));
	printPC(", ");
	printlnPC(readResolutionDivider(ID));

	printPC("The DIP gains are: ");
	printPC(readGainD(ID));
	printPC(", ");
	printPC(readGainI(ID));
	printPC(", and ");
	printlnPC(readGainP(ID));

	printPC("The value of moving speed is: ");
	printlnPC(readMovingSpeed(ID));
	printPC("The value of torque limit (goal torque) is: ");
	printlnPC(readTorqueLimit(ID));

	printPC("My present load is: ");
	printPC(readLoad(ID));
	printPC("I am operating at a voltage of ");
	printPC(readVoltage(ID));
	printPC(" and a temperature of ");
	printlnPC(readTemperature(ID));

	printPC("Is there a function waiting to be executed in Registered?: ");
	printlnPC(registeredStatus(ID));

	printPC("Is a moving (goal_position) command being executed?: ");
	printlnPC(moving(ID));

	printlnPC("------------------------------------------");
}

//--------------------Multi-compatibility functions------------------------

void DynamixelClass::sendData(uint8_t val){

	#if (PLATFORM_ID==88) || defined(SPARK)
		Serial1.write(val);								//For Duo or Photon
	#else
		Serial1.write(val);								//For Leonardo
	#endif
}

void DynamixelClass::sendDataBuff(uint8_t* buff, uint8_t len){

	#if (PLATFORM_ID==88) || defined(SPARK)
		Serial1.write(buff, len);						//For Duo or Photon
	#else
		Serial1.write(buff, len);						//For Leonardo
	#endif
}

// Check Serial Data Available
int DynamixelClass::availableData(void){

	#if (PLATFORM_ID==88) || defined(SPARK)
		return Serial1.available();
	#else
		return Serial1.available();
	#endif
}

// Read Serial Data
uint8_t DynamixelClass::readData(void){

	#if (PLATFORM_ID==88) || defined(SPARK)
		return Serial1.read();
	#else
		return Serial1.read();
	#endif
}

// Peek Serial Data
uint8_t DynamixelClass::peekData(void){

	#if (PLATFORM_ID==88) || defined(SPARK)
		return Serial1.peek();
	#else
		return Serial1.peek();
	#endif
}

// Begin Serial Comunication
void DynamixelClass::beginCom(long speed){

	#if (PLATFORM_ID==88) || defined(SPARK)
		Serial1.begin(speed);
	#else
		Serial1.begin(speed);
	#endif
}

// End Serial Comunication
void DynamixelClass::endCom(void){

	#if (PLATFORM_ID==88) || defined(SPARK)
		Serial1.end();
	#else
		Serial1.end();
	#endif
}

// Wait until data has been written
void DynamixelClass::serialFlush(void){

	#if (PLATFORM_ID==88) || defined(SPARK)
		Serial1.flush();
	#else
		Serial1.flush();
	#endif
}

// Macro for Timing. Delay milliseconds
void DynamixelClass::delayms(unsigned int ms){

	#if (PLATFORM_ID==88) || defined(SPARK)
		delay(ms);
	#else
		delay(ms);
	#endif
}

// Macro for Timing. Delay Microseconds
void DynamixelClass::delayus(unsigned int us){

	#if (PLATFORM_ID==88) || defined(SPARK)
		delayMicroseconds(us);
	#else
		delayMicroseconds(us);
	#endif
}

DynamixelClass Dynamixel;
