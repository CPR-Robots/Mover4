Mover4
 
Test program to show how to interface with the Commonplace Robotics Mover4 robot arm


Version 0.4	  
May 14th, 2013    
info@cpr-robots.com
 

Necessary hardware: USB2CAN adapter (2013 version) and Mover4 robot arm, both available via www.cpr-robots.com
 
The program has to be started in a terminal window.
Written and tested on Ubuntu 11.10, Eclipse 3.7.0 with CDT
Libraries: boost, ncurses
 

Functionalities: 
The program establishes a connection to the robot via a virtual com port of the USB2CAN driver
The robot can move the four joints.
This com port has to be set in cpr_RS232CAN.cpp, preset is /dev/ttyUSB0.
 
Usage:
1. Start with ./Mover4
2. Reset Errors and load current joint values with 'p'
3. Enable Motors with 'o'
4. Move Robot with 'q' to 'f'
5. If necessary set joints to zero with 'z', afterwards again reset and enable the motors

Attention: this software is a demo program only! For performing applications more attention on error handling and recovery should be paid!
 
Troubleshooting:
* The rights for the USB adapter have to be correct. When connecting is not possible try to start with sudo or change rights for ttyUSB0.


Changes:
Version 0.4:
	* Feature: When reseting the joints the current positions are copied to the setPoint position. Zeroing is not necessary before start
	* Debug: Screen is erased every cycle, less visualization errors
	* Debug: EvaluateBuffer has been updated, a wrong cast in computing the checksum byte was the reason for not admitted messages.
Version 0.3:
	* Updated communication to the USB2CAN bridge with Checksum bits

