/*
 * Copyright (c) 2016 Fabian Eugenio Reyes Pinner (Fabian Reyes)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 */

 /*
  * In this example, the information (for example, configured values in its EEPROM or RAM area) of a servo will be obtained and displayed into the terminal.
  * It is assumed that the ID of the servo is '1' and the baudrate configured is '57600'. This may be different of your setup, so just change the necessary
  * parameters.
  */

SYSTEM_MODE(MANUAL);  //do not connect to the cloud. Communicate with the servo immediately

//The DuoDMXL library allows communication with the servo
#include <DuoDMXL.h>

int led1 = D7;      //onboard LED
int dataPin = D15;  //Pin used to control Data flow between the DUO and Dynamixel. By default we use the D15 Pin which is closes to the TX and RX pins
int servoID = 1;
long baud = 57600;  //Baudrate for communication with Dynamixel servos (Depending of your servo, this may be 1Mbps by default)

void servoIntroduction(unsigned char servoID);  //custom function

void setup() {
  pinMode(led1, OUTPUT);              //prepare the on-board LED

  Serial.begin(115200);               //Start communication with the PC;
  Dynamixel.begin(baud, dataPin);     // Initialize the servo at Baud Rate (baud) and Pin Control (dataPin)

  delay(1000);                        //Let's wait a little bit

}

void loop() {

    digitalWrite(led1, HIGH);               //turn on on-board LED to show that the self-introduction has started

    servoIntroduction(servoID);             //print the information of the servo with ID = servoID

    digitalWrite(led1, LOW);

    delay(1000);                            //wait one second and show the information again
}

//Basic information about the servo
void servoIntroduction(uint8_t servoID){
  Serial.println("------------------------------------------");

  Serial.print("Hi, I am a servo model ");
  Serial.print(Dynamixel.readModel(servoID));
  Serial.print(" with Firmware version ");
  Serial.print(Dynamixel.readFirmware(servoID));
  Serial.print(", my ID is ");
  Serial.print(Dynamixel.readID(servoID));
  Serial.print(", communicating at a baudrate of ");
  Serial.print(Dynamixel.readBD(servoID));
  Serial.print(", with a RDT of ");
  Serial.println(Dynamixel.readRDT(servoID));

  Serial.print("CW limit set as ");
  Serial.print(Dynamixel.readCWAngleLimit(servoID));
  Serial.print(", CCW limit set as ");
  Serial.println(Dynamixel.readCCWAngleLimit(servoID));

  Serial.print("The temperature limit, lowest voltage limit, highest voltage limit, and max torque are: ");
  Serial.print(Dynamixel.readTempLimit(servoID));
  Serial.print(", ");
  Serial.print(Dynamixel.readLowVoltageLimit(servoID));
  Serial.print(", ");
  Serial.print(Dynamixel.readHighVoltageLimit(servoID));
  Serial.print(", ");
  Serial.println(Dynamixel.readMaxTorque(servoID));

  Serial.print("My Status return level, Alarm LED, and shutdown alarm settings are: ");
  Serial.print(Dynamixel.readSRL(servoID));
  Serial.print(", ");
  Serial.print(Dynamixel.readAlarmLED(servoID));
  Serial.print(", ");
  Serial.println(Dynamixel.readShutdownAlarm(servoID));

  Serial.print("The multi-turn offset setting and resolution divider are: ");
  Serial.print(Dynamixel.readMultiTurnOffset(servoID));
  Serial.print(", ");
  Serial.println(Dynamixel.readResolutionDivider(servoID));

  Serial.print("The DIP gains are: ");
  Serial.print(Dynamixel.readGainD(servoID));
  Serial.print(", ");
  Serial.print(Dynamixel.readGainI(servoID));
  Serial.print(", and ");
  Serial.println(Dynamixel.readGainP(servoID));

  Serial.print("The value of moving speed is: ");
  Serial.println(Dynamixel.readMovingSpeed(servoID));
  Serial.print("The value of torque limit (goal torque) is: ");
  Serial.println(Dynamixel.readTorqueLimit(servoID));

  Serial.print("My present load is: ");
  Serial.println(Dynamixel.readLoad(servoID));
  Serial.print("I am operating at a voltage of ");
  Serial.print(Dynamixel.readVoltage(servoID));
  Serial.print(" and a temperature of ");
  Serial.println(Dynamixel.readTemperature(servoID));

  Serial.print("Is there a function waiting to be executed in Registered?: ");
  Serial.println(Dynamixel.registeredStatus(servoID));

  Serial.print("Is a moving (goal_position) command being executed?: ");
  Serial.println(Dynamixel.moving(servoID));

  Serial.println("------------------------------------------");
}
