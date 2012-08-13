Mover4
 
Test program to show how to interface with the Commonplace Robotics Mover4 robot arm


Version 0.2	   
August 13th, 2012    
info@cpr-robots.com
 


Necessary hardware: USB2CAN adapter and Mover4 robot arm, both available via www.cpr-robots.com
 
The program has to be started in a terminal window.
 

Written and tested on Ubuntu 11.10, Eclipse 3.7.0 with CDT
 
Libraries: boost, ncurses
 

Functionalities: 
The program establishes a connection to the robot via a virtual com port of the USB2CAN driver
 
This com port has to be set in cpr_RS232CAN.cpp, preset is /dev/ttyUSB0.
 
Then the communication to the robot is established, Joints are resettet and motors enabled
 
With the keyboard you can move the single joints 
Also you can set the joints to zero. 

Attention: this software is a demo program only! For performing applications more attention on error handling and recovery should be paid!
 