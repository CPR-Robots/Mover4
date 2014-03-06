Mover4
 
Test program to show how to interface with the Commonplace Robotics Mover4 robot arm


Version 0.6	  
March 3rd, 2014    
info@cpr-robots.com
 

Necessary hardware: Mover4 robot arm and CAN adapter, either PCAN-USB (white) or CAN2USB (black) , both available via www.cpr-robots.com
When using with the PCAN-USB adapter the according device driver v7.10 or higher has to be installed, see the according section below.
 
The program has to be started in a terminal window.
Written and tested on Ubuntu 11.10, gcc
Libraries: boost, ncurses, (libpcan for the PCAN-USB adapter)
 

Functionalities: 
	- establish a connection to the robot via USB adapter
	- in cpr_mover4.cpp you have to choose the interface, either cpr_PCAN or cpr_RS232CAN
		- for cpr_RS232CAN: a virtual com port is opened (to be set in cpr_RS232CAN.cpp, preset is /dev/ttyUSB0)
		- for PCAN-USB: the driver has to be installed before
	- establish a communication loop, Joint positions are read and set, motors can be reset, enabled and zeroed
	- move the single joints or cartesian directions with the keyboard
 

Usage:
1. Start with ./Mover4
2. Reset Errors and load current joint values with 'p' (maybe press space to update the screen)
3. Enable Motors with 'o'
4. Move Robot with 'q' to 'f' in joint mode
5. Or move the robot with 'z' to 'k' in cartesian mode. XYZ can be set, B stay in at the current value.
6. If necessary set joints to zero with 'z', afterwards again reset and enable the motors


Attention: this software is a demo program only! For performing applications more attention on error handling and recovery should be paid!
 

Installation of the PCAN-USB device driver:
	- Download the current driver package from http://www.peak-system.com/fileadmin/media/linux/index.htm
	- Extract driver package
	- change into the directory, e.g. cd peak-linux-driver-7.10
	- make clean
	- We want to install the chardev version of the driver: make NET=NO_NETDEV_SUPPORT
	- sudo make install
	- reboot
	- the command cat \proc\pcan should show:

	*------------- PEAK-System CAN interfaces (www.peak-system.com) -------------
	*------------- Release_20140121_n (7.10.0) Mar  3 2014 14:13:14 --------------
	*---------------- [mod] [isa] [pci] [dng] [par] [usb] [pcc] -----------------
	*--------------------- 1 interfaces @ major 249 found -----------------------
	*n -type- ndev --base-- irq --btr- --read-- --write- --irqs-- -errors- status
	32    usb -NA- ffffffff 255 0x001c 00000000 00000000 0000000c 00000000 0x0000

	Especially below the ndev word there should be -NA-


Troubleshooting:
* For CAN2USB adapter: The rights for the USB adapter have to be correct. When connecting is not possible try to start with sudo or change rights for ttyUSB0.
* The screen does not alway update automatically. Try pressing space bar.


Changes:
Version 0.6:
	* Added support for the PEAK Systeme PCAN-USB adapter
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



