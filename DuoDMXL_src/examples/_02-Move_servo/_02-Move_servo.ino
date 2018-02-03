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
  * In this example, we will move the servo to 0[deg], 180[deg], and finally 360[deg].
  * Also, we will print the current position to the terminal.
  *
  * This example introduces the functions Dynamixel.move(), Dynamixel.setAng(), and Dynamixel.readPosition()
  */

SYSTEM_MODE(MANUAL);  //do not connect to the cloud. Communicate with the servo immediately

//The DuoDMXL library allows communication with the servo
#include <DuoDMXL.h>

int led1 = D7;      //onboard LED
int dataPin = D15;  //Pin used to control Data flow between the DUO and Dynamixel. By default we use the D15 Pin which is closest to the TX and RX pins
int servoID = 1;

long baud = 57600;  //Baudrate for communication with Dynamixel servos (Depending of your servo, this may be 1Mbps by default)

void setup() {
  pinMode(led1, OUTPUT);                  //prepare the on-board LED

  Serial.begin(115200);                   //Start communication with the PC;
  Dynamixel.begin(baud, dataPin);         //Initialize the servo at Baud Rate (baud) and Pin Control (dataPin)

  delay(1000);                            //Let's wait a little bit
  digitalWrite(led1, HIGH);               //turn on on-board LED to show that the loop has started
}

void loop() {
  int actualPos;

  //move to 0[deg] (bit-wise value 0)
  Dynamixel.move(servoID, 0);
  delay(1000);                            //wait one second for the servo to reach its position
  actualPos = Dynamixel.readPosition(servoID);
  Serial.print("The actual position is: ");
  Serial.println(actualPos);

  //move to 180[deg] (bit-wise value 2048). In this case, we can use the setAng() function
  Dynamixel.setAng(servoID, 180.0, 'd');  //equivalent to 'Dynamixel.move(servoID, 2048)'
  delay(1000);                            //wait one second for the servo to reach its position
  actualPos = Dynamixel.readPosition(servoID);
  Serial.print("The actual position is: ");
  Serial.println(actualPos);

  //move to 360[deg] (bit-wise value 4095)
  Dynamixel.move(servoID, 4095);
  delay(1000);                            //wait one second for the servo to reach its position
  actualPos = Dynamixel.readPosition(servoID);
  Serial.print("The actual position is: ");
  Serial.println(actualPos);

}
