Mover4
 
Test program to show how to interface with the Commonplace Robotics Mover4 robot arm


Version 0.5	  
May 16th, 2013    
info@cpr-robots.com
 

Necessary hardware: USB2CAN adapter (2013 version) and Mover4 robot arm, both available via www.cpr-robots.com
 
The program has to be started in a terminal window.
Written and tested on Ubuntu 11.10, gcc
Libraries: boost, ncurses
 

Functionalities: 
- establish a connection to the robot via a virtual com port of the USB2CAN driver. (to be set in cpr_RS232CAN.cpp, preset is /dev/ttyUSB0)
- establish a communication loop, joint positions are read and set, motors can be reset, enabled and zeroed
- move the single joints or cartesian directions with the keyboard
 

Usage:
1. Start with ./Mover4
2. Reset Errors and load current joint values with 'p' (maybe press space to update the screen)
3. Enable Motors with 'o'
4. Move Robot with 'q' to 'f' in joint mode
5. Or move the robot with 'z' to 'k' in cartesian mode. XYZ can be set, B stay in at the current value.
6. If necessary set joints to zero with 'z', afterwards again reset and enable the motors


Attention: this software is a demo program only! For performing applications more attention on error handling and recovery should be paid!
 
Troubleshooting:
* The rights for the USB adapter have to be correct. When connecting is not possible try to start with sudo or change rights for ttyUSB0.
* The screen does not alway update automatically. Try pressing space bar.


Changes:
Version 0.5:
	* Added 4 axis kinematic for the robot. The robot can be controlled in XYZ now. B ist held, but cannot be directed (but via J4).  
Version 0.4:
	* Feature: When reseting the joints the current positions are copied to the setPoint position. Zeroing is not necessary before start
	* Debug: Screen is erased every cycle, less visualization errors
	* Debug: EvaluateBuffer has been updated, a wrong cast in computing the checksum byte was the reason for not admitted messages.
Version 0.3:
	* Updated communication to the USB2CAN bridge with Checksum bits


To be done:
	* The screen sometimes gets messed up



