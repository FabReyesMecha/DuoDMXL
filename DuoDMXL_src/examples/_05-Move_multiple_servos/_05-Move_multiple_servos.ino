/*
 * Copyright (c) 2018 Fabian Eugenio Reyes Pinner (Fabian Reyes)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPY
  * This example introduces the functions Dynamixel.move() and Dynamixel.readPosition() updated in DuoDMXL v1.2 which supports faster communication with multiple servos within the same call
  */

SYSTEM_MODE(MANUAL);  //do not connect to the cloud. Communicate with the servo immediately

//The DuoDMXL library allows communication with the servo
#include <DuoDMXL.h>

int led1 = D7;      //onboard LED
int dataPin = D15;  //Pin used to control Data flow between the DUO and Dynamixel. By default we use the D15 Pin which is closest to the TX and RX pins
int errorMove, servoPos, servoCurrent, servoTorqueEnableStatus, gainD, gainI, gainP;

const uint8_t noServos = 2;                   //number of servos
uint8_t servoIDs[noServos] = {2,4};           //IDs of the servos

long baud = 57600;                            //Baudrate for communication with Dynamixel servos (Depending of your servo, this may be 1Mbps by default)

void printRow(int [], uint8_t);               //Prototype of function to print a row of values

void setup() {
  pinMode(led1, OUTPUT);                  //prepare the on-board LED

  Serial.begin(115200);                   //Start communication with the PC;
  delay(3000);                            //Wait for the microcontroller to be recognized by the PC
  
  Dynamixel.begin(baud, dataPin);         //Initialize the servo at Baud Rate (baud) and Pin Control (dataPin) 
}

void loop() {
  digitalWrite(led1, HIGH);               //turn on on-board LED to show that the loop has started
  int pos[noServos] = {};                 //buffer for current position

  //---------------------------------move to 0[deg] (bit-wise value 0)
  Serial.println("------------------------------");
  
  Serial.println("Moving servos to 0");
  int desPos[noServos] = {0, 0};          //buffer for desired position
  Dynamixel.move(servoIDs, noServos, desPos);
  delay(1000);

  Serial.println("Positions are");
  Dynamixel.readPosition(servoIDs, noServos, pos);
  printRow(pos, noServos);

  //---------------------------------move to 180[deg] (bit-wise value 2048)
  Serial.println("------------------------------");
  
  Serial.println("Moving servos to 2048");
  int desPos2[noServos] = {2048, 2048};          //buffer for desired position
  Dynamixel.move(servoIDs, noServos, desPos2);
  delay(1000);

  Serial.println("Positions are");
  Dynamixel.readPosition(servoIDs, noServos, pos);
  printRow(pos, noServos);

  //---------------------------------move to 360[deg] (bit-wise value 4095)
  Serial.println("------------------------------");
  
  Serial.println("Moving servos to 4095");
  int desPos3[noServos] = {4095, 4095};          //buffer for desired position
  Dynamixel.move(servoIDs, noServos, desPos3);
  delay(1000);

  Serial.println("Positions are");
  Dynamixel.readPosition(servoIDs, noServos, pos);
  printRow(pos, noServos);
}

void printRow(int values[], uint8_t noValues){

  Serial.print("{");
  for (int i=0; i<(noValues-1); i++){
    Serial.print(values[i]);
    Serial.print(",");
  }
  Serial.print(values[noValues-1]);
  Serial.println("}");
  
}

