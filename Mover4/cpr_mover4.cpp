/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-13, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Mover4
 * Version 0.3	   April 21st, 2013    info@cpr-robots.com
 *
 * Test program to show how to interface with the Commonplace Robotics Mover4 robot arm
 * Necessary hardware: USB2CAN adapter and Mover4 robot arm, both available via www.cpr-robots.com
 * The program has to be started in a terminal window.
 * Written and tested on Ubuntu 11.10, Eclipse 3.7.0 with CDT
 * Libraries: boost, ncurses
 *
 * Functionalities: The program establishes a connection to the robot via a virtual com port of the USB2CAN driver
 * This com port has to be set in cpr_RS232CAN.cpp, preset is /dev/ttyUSB0.
 * Then the communication to the robot is established, Joints are resettet and motors enabled
 * With the keyboard you can move the single joints now
 * Also you can set the joints to zero.
 *
 * ToDo: thread safety is completely missing up till now
 */



#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include <boost/lexical_cast.hpp>
#include <iostream>


#include "cpr_RS232CAN.h"
#include "cpr_KinematicMover.h"
#include "cpr_InputKeyboard.h"

using namespace std;





class cprMover4HW
{
public:
  	cprMover4HW();
	void DoComm(void);
	void ResetJointsToZero();
	void ResetError();
	void EnableMotors();
	void DisableMotors();

private:
  	
	void Wait(int ms);

	cpr_KinematicMover kin;		// stores the robot joint position and does kinematic computations
	cpr_RS232CAN itf;		// provides the hardware interface to the robots CAN bus via a virtual com port
	cpr_InputKeyboard keyboard;	// reads keys to move the joints; a minimal interface

	// robot state
	robotState state;
	int jointIDs[4];
	int nrOfJoints;

	int InvKin(void);
	bool flagRequestReset;
	bool flagRequestZero;
  

};

//********************************************
cprMover4HW::cprMover4HW()
{

	nrOfJoints = 4;
	jointIDs[0] = 16;		// depending on the robot hardware, these 
					// are the CAN message IDs of the joint modules
					// When changing IDs also change read-loop test 
					// in cpr_RS232CAN.cpp
	jointIDs[1] = 32;
	jointIDs[2] = 48;
	jointIDs[3] = 64;

	itf.keys = &keyboard;
	itf.Connect();	
}


//********************************************
void cprMover4HW::DoComm(){

	int id = 16;
	int l = 5;				// length for position commands: 5 byte. for velocity commands 4 bytes
	char data[8] = {4, 125, 99, 0, 45};	// cmd, vel, posH, posL, counter


	double v[6];
	keyboard.GetMotionVec(v);					// Get the user input
	kin.SetMotionVec(v);						// forward user input to the kinematic
	kin.moveJoint();						// compute next set point position (could also be cartesian)
	keyboard.SetJoints(kin.setPointState.j, kin.currState.j);	// and hand back for visualization


	if(keyboard.flagReset){
		keyboard.flagReset = false;
		ResetError();
		EnableMotors();
	}

	if(keyboard.flagZero){
		keyboard.flagZero = false;
		ResetJointsToZero();
	}

	int tics = 32000;		// Write CAN messages with the desired encoder tics
	for(int i=0; i<nrOfJoints; i++){
		tics = kin.computeTics(kin.setPointState.j[i]);
		data[2] = tics / 256;
		data[3] = tics % 256;
		itf.WriteMsg(jointIDs[i], l, data);		// the CAN messages must have the correct length, otherwise they will be ignored
		Wait(3);

	}

	double p[4];
	for(int i=0; i<nrOfJoints; i++){	// and read the current joint positions
		itf.GetMsg(jointIDs[i]+1, &l, data);
		tics = (256 * ((int)((unsigned char)data[2]))) + ((unsigned int)((unsigned char)data[3]));
		kin.currState.j[i] = kin.computeJointPos(tics);
		kin.currState.errorCode[i] = (int)data[0];
	}


	string s = boost::lexical_cast<string>( kin.currState.errorCode[0] );
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[1] );
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[2] );
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[3] );
	keyboard.SetStatus(s);


	//std::cout << "joints: "<< p[0] <<" "<< p[1] <<" "<< p[2] <<" "<< p[3]<< std::endl;

	return;
}



//********************************************
void cprMover4HW::ResetJointsToZero(){
	int id = 16;
	int l = 4;
	char data[8] = {1, 8, 125, 0, 0, 0, 0};	// CAN message to set the joint positions to zero

	DisableMotors();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
	keyboard.SetMessage("Joints set to zero");
}

//***********************************************************
void cprMover4HW::ResetError(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 6, 0, 0, 0, 0, 0};		// CAN message to reset the errors
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
	keyboard.SetMessage("Error reset");
}

//********************************************
void cprMover4HW::EnableMotors(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 9, 0, 0, 0, 0, 0};		// CAN message to enable the motors
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
	keyboard.SetMessage("Motors enabled");
}

//********************************************
void cprMover4HW::DisableMotors(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 10, 0, 0, 0, 0, 0};		// CAN message to disable the motors
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data);
		Wait(3);
	}
	keyboard.SetMessage("Motors disabled");
}


//********************************************
void cprMover4HW::Wait(int ms){
	using namespace boost::posix_time;
	ptime start, now;
	time_duration passed;

	start = microsec_clock::universal_time();
	now = microsec_clock::universal_time();
	passed = now - start;
	while( passed.total_milliseconds() < ms){
		now = microsec_clock::universal_time();
		passed = now - start;
	}

}




//********************************************
int main( int argc, char** argv )
{
		
	using namespace boost::posix_time;
	ptime start, now;
	time_duration passed;
	char KeyStroke;

	cprMover4HW myMover;	
  	
	//myMover.ResetError();		// now has to be done manually
	//myMover.EnableMotors();

	while (true)			// this is the main loop
	{				// but all interesting things are done in DoComm()
		start = microsec_clock::universal_time();
		myMover.DoComm();
		now = microsec_clock::universal_time();
		passed = now - start;


		while( passed.total_milliseconds() < 50){	// set the cycle time here
			now = microsec_clock::universal_time();
			passed = now - start;

		}

	}



}



