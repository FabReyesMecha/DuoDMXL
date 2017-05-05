# DuoDMXL
## Libary for servo control with Duo (Redbear) or Photon (Particle)

For the documentation visit [DuoDMXL](https://fabreyesmecha.github.io/DuoDMXL/).

DuoDMXL is a library for controlling Dynamixel servos using a Duo (or Photon) as main microcontroller. This library allows the user to write or read all of the possible registers in the servomotors. Specifically, it is meant for MX-64 servomotors. Other models like AX have different EEPROM registers, but the library can be adapted.

Initially, the library was based on [Savage Electronics Library](http://savageelectronics.blogspot.jp/2011/01/arduino-y-dynamixel-ax-12.html), but eventually started to diverge from the original. The original library used timers for several functions, which would lead to troubles using other microcontrollers, different than the Arduino UNO.

The main difference is that communication is delegated to the basic functions:

1. `sendWord()` which writes a value to a register of the servo.
2. `readWord()` which reads the current value of a register of the servo.
3. `readInformation()` which reads a response (set of bytes) from the servomotors. It could be requested information or a simple no-error response.

and all other functions call these basic functions with the appropiate parameters. This not only decreases the code size, but also allows for easier creation of user-defined functions.

![alt text](DuoDMXL_hardware/communicationHierarchy.png "Communication Overview")

The Dynamixel servos have their own protocol for communication, which you can check in the communication section of the [ROBOTIS manual](http://support.robotis.com/en/). Depending on the servos you are communicating with, you need to transform the half-duplex UART signal of the Duo (or Photon) unto a TTL signal for some servos (e.g., MX-64T) which can be done with a tri-state buffer, or use a RS-485 transceiver for servos that use RS-485 bus (e.g., MX-64AR).

The DuoDMXL repository includes eagle schematics and board layouts for two types of 'shields'. The **Duo Tri-state Buffer Shield** is used for half-duplex communication. It takes the TX and RX signal from the DUO (or Photon) and the signal of a control pin, and changes it into communication with only one line of data. The **Duo RS-485 Shield** is used to communicate with a RS-485 transceiver. It takes the TX and RX signal from the DUO (or Photon) and the signal of a control pin and outputs differential communication through the two signals D+ and D- (also called A and B). The library works equally with both hardware setups.

![alt text](DuoDMXL_hardware/Duo_pinout.png "Electronic Setup of Duo and servomotors")

Prerequisites:

1. Hardware:
    * RS-485 transceiver or Tri-state buffer
2. Software dependencies:
    * None (as of version 1.0)

Installation:
This library is mainly intended to be used with the Arduino IDE. Just download the source code in *DuoDMXL_src/* directory and move it to a new directory *.../documents/Arduino/libraries/DuoDMXL/*. Next time you restart the Arduino IDE, the new library DuoDMXL will be detected automatically.

## Log:

[2017-05-05] Stable release: **DuoDMXL** v1.0
[2017-05-04] Uploaded **DuoDMXL** v0.3
