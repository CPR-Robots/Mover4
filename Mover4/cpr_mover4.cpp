/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-14, Commonplace Robotics GmbH
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
 * Version 0.6	   March 3rd, 2014    info@cpr-robots.com
 * Please see the readme file
 * ToDo: This program is only a communication demo, not a usable robot interface. E.g. thread safety is completely missing up till now.
 */



#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/gregorian/gregorian.hpp"
#include <boost/lexical_cast.hpp>
#include <iostream>


#include "cpr_PCAN.h"
//#include "cpr_RS232CAN.h"
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
	cpr_PCAN itf;		// provides the hardware interface to the robots CAN bus via a virtual com port
	//cpr_RS232CAN itf;		// provides the hardware interface to the robots CAN bus via a virtual com port
	cpr_InputKeyboard keyboard;	// reads keys to move the joints; a minimal interface

	robotState state;
	int jointIDs[4];		// CAN IDs of the robots joints
	int nrOfJoints;

	bool flagDoComm;		// flag to interrupt the standard comm loop, e.g. to set joints to zero

	int InvKin(void);		// the kinematics module
	
  

};

//********************************************
cprMover4HW::cprMover4HW()
{

	nrOfJoints = 4;
	jointIDs[0] = 2;		// depending on the robot hardware, these 
					// are the CAN message IDs of the joint modules
					// When changing IDs also change read-loop test 
					// in cpr_RS232CAN.cpp
	jointIDs[1] = 4;
	jointIDs[2] = 6;
	jointIDs[3] = 8;

	flagDoComm = true;

	itf.keys = &keyboard;
	itf.Connect();	
}


//********************************************
// Main communication loop. 
// Reads the keyboard, generates new joint values with the help of the kinematic module and then writes on the CAN bus.
void cprMover4HW::DoComm(){

	// storage necessary for the CAN communication
	int id = 16;				
	int l = 5;				// length for position commands: 5 byte. for velocity commands 4 bytes
	char data[8] = {4, 125, 99, 0, 45};	// cmd, vel, posH, posL, counter


	double v[6];
	keyboard.GetMotionVec(v);			// Get the user input, joint or cartesian velocites
	kin.SetMotionVec(v);				// forward user input to the kinematic
	
	if(keyboard.motionType == 0)			// did the user want to move in joint or cart mode?
		kin.moveJoint();			// compute next set point position in Joint mode
	else
		kin.moveCart();				// compute next set point position in Cartesian mode
	
	keyboard.SetJoints(kin.setPointState.j, kin.currState.j);	// and hand back for visualization
	keyboard.SetPosition(kin.setPointState.p, kin.currState.p);	


	if(keyboard.flagReset){				// do a motor reset if requested
		keyboard.flagReset = false;
		ResetError();
	}

	if(keyboard.flagEnable){			// enable the motors if requested
		keyboard.flagEnable = false;
		EnableMotors();
	}
	
	if(keyboard.flagZero){				// set the joints to zero if requested (zeroes the EEPROM in the robots joint controllers)
		keyboard.flagZero = false;
		ResetJointsToZero();
	}


	// Write the desired set point position to the joints
	int tics = 32000;	// send the desired encoder tics from 0 to 64000, 32000 is zero
	if(flagDoComm){
		for(int i=0; i<nrOfJoints; i++){				// go through all four joints

			tics = kin.computeTics(i, kin.setPointState.j[i]);	// get the tics out of the joint position in degree
			data[2] = tics / 256;					// Joint Position High Byte
			data[3] = tics % 256;					// Joint Position Low Byte
			itf.WriteMsg(jointIDs[i], l, data, true);			// write the message to the joint controller
			Wait(5);						// wait a short time to avoid a crowded bus
		}
	}


	// read the answers of the robot joint controllers
	double p[4];
	for(int i=0; i<nrOfJoints; i++){					// jor all four joints
		itf.GetMsg(jointIDs[i]+1, &l, data);				// get the last message that has been received
		tics = (256 * ((int)((unsigned char)data[2]))) + ((unsigned int)((unsigned char)data[3]));	// combine the two bytes to the position in encoder tics	
		kin.currState.j[i] = kin.computeJointPos(i, tics);		// compute the Joint position in degree
		kin.currState.errorCode[i] = (int)data[0];			// store the joints error code (see protocol description)
	}


	string s = boost::lexical_cast<string>( kin.currState.errorCode[0] );	// generate a string from the four error codes
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[1] );
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[2] );
	s += " " + boost::lexical_cast<string>( kin.currState.errorCode[3] );
	keyboard.SetStatus(s);

	return;
}



//********************************************
// Resets the position EEPROM on the four joint controller
void cprMover4HW::ResetJointsToZero(){
	int id = 16;
	int l = 4;
	char data[8] = {1, 8, 125, 0, 0, 0, 0};	// CAN message to set the joint positions to zero

	flagDoComm = false;

	DisableMotors();			// otherwise the robot will make a jump afterwards (until the lag error stops him)

	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data, false);	// first reset command.. but thats not sufficient
		Wait(5);
		itf.WriteMsg(jointIDs[i], l, data, false);	// the command has to be sent twice in the time of two seconds to take effect
		Wait(5);
	}
	keyboard.SetMessage("Joints set to zero");

	flagDoComm = true;
}


//***********************************************************
// This function does two things: 
// 1. Copy the current position into the setPoint position
// 2. Send the robot the resetError messages
void cprMover4HW::ResetError(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 6, 0, 0, 0, 0, 0};		// CAN message to reset the errors

	flagDoComm = false;

	// copy current to setPoint
	for(int i=0; i<4; i++)
		kin.setPointState.j[i] = kin.currState.j[i];	// joint angles
	for(int i=0; i<6; i++)
		kin.setPointState.p[i] = kin.currState.p[i];	// cartesian position
	
	// then reset the errors
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data, false);
		Wait(3);
	}
	keyboard.SetMessage("Error reset");

	flagDoComm = true;
}

//********************************************
// enable the motors, necessary to move them
void cprMover4HW::EnableMotors(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 9, 0, 0, 0, 0, 0};		// CAN message to enable the motors
	flagDoComm = false;
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data, false);
		Wait(3);
	}
	keyboard.SetMessage("Motors enabled");
	flagDoComm = true;
}

//********************************************
void cprMover4HW::DisableMotors(){
	int id = 16;
	int l = 2;
	char data[8] = {1, 10, 0, 0, 0, 0, 0};		// CAN message to disable the motors
	flagDoComm = false;
	for(int i=0; i<nrOfJoints; i++){
		itf.WriteMsg(jointIDs[i], l, data, false);
		Wait(3);
	}
	keyboard.SetMessage("Motors disabled");
	flagDoComm = true;
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



