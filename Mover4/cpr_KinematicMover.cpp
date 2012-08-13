/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Commonplace Robotics GmbH
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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>


using namespace std;


#include "cpr_KinematicMover.h"





//***************************************************************
cpr_KinematicMover::cpr_KinematicMover()
{

	for(int i=0; i<4; i++){
		setPointState.j[i] = 0.0;
		currState.j[i] = 0.0;
		v[i] = 0.0;
	}
}


//***************************************************************
void cpr_KinematicMover::SetMotionVec(double *vec){
	for(int i=0; i<6; i++)
		motionVec[i] = vec[i];
}


//***************************************************************
int cpr_KinematicMover::moveJoint(){

	for(int i=0; i<4; i++){
		setPointState.j[i] += 0.1 * motionVec[i];
	}
	return 0;
}


//***************************************************************
int cpr_KinematicMover::moveCart(double *vel){
	return 0;

}


//***************************************************************
int cpr_KinematicMover::forwardKin(robotState s){
	return 0;
}


//***************************************************************
int cpr_KinematicMover::inverseKin(robotState s){
	return 0;
}

//***************************************************************
// int ticks:		joint encoder tics
// return value: 	joint position in degree
double cpr_KinematicMover::computeJointPos(int ticks){
	double gearScale = 65.0;
	double gearZero = 32000.0;

	double p = (ticks - gearZero)/gearScale;

	return p;

}


//***************************************************************
// double pos:		joint position in degree
// return value: 	joint encoder ticks
int cpr_KinematicMover::computeTics(double pos){
	double gearScale = 65.0;
	double gearZero = 32000.0;

	int t =  ((int)(pos*gearScale)) + gearZero;

	return t;

}




