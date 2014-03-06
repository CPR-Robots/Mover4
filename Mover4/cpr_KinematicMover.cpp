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

// Last change: March 3rd, 2014

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <stdexcept>


using namespace std;


#include "cpr_KinematicMover.h"
#include "cpr_Matrix4.h"





//***************************************************************
// init all values
cpr_KinematicMover::cpr_KinematicMover()
{
	for(int i=0; i<4; i++){
		setPointState.j[i] = 0.0;
		currState.j[i] = 0.0;
		v[i] = 0.0;
	}

	setPointState.j[0] = 0.0;	// Start Position (only relevant when not connected)
	setPointState.j[1] = 0.0;
	setPointState.j[2] = 90.0;
	setPointState.j[3] = 0.0;

	gearScale[0] = 65.0;		// reduction ratio of the joints: 1° = 65 encoder tics
	gearScale[1] = 65.0;
	gearScale[2] = -65.0;
	gearScale[3] = 65.0;

}


//***************************************************************
// This motion vector will be used to update the joint or cart position
void cpr_KinematicMover::SetMotionVec(double *vec){
	for(int i=0; i<6; i++)
		motionVec[i] = vec[i];
}


//***************************************************************
// uses the motionVec to generate a new joint position,
// then does the forward kinematics to get the current cartesian position.
// Stores the symmetry flag.
int cpr_KinematicMover::moveJoint(){

	for(int i=0; i<4; i++){
		setPointState.j[i] += 0.1 * motionVec[i];
	}
	forwardKin(setPointState);

 	// Store the symmetries in the position
        if (setPointState.j[2] >= 0.0)                    
        	flagSymmetryEllbowUp = true;
        else
                flagSymmetryEllbowUp = false;

	return 0;
}


//***************************************************************
// Update the cartesian position based on the motionVec
// Then generates the joint positions with the inverse kinematics
int cpr_KinematicMover::moveCart(){
	
	for(int i=0; i<4; i++){
		setPointState.p[i] += 1.0 * motionVec[i];
	}
	inverseKin(setPointState);

	return 0;
}


//***************************************************************
// generate the cartesian position based on the current robots joint values
// Sets up five 4x4 matrices with the joint length and rotations and multiplies them.
// Afterwards stores the resulting matrix and generates the B angle
int cpr_KinematicMover::forwardKin(robotState s){

	cpr_Matrix4 mat0, mat1, mat2, mat3, mat4;

	mat0.mat[11] = lz0;
	mat0.SetRotationDeg(2, s.j[0]);
	mat1.mat[11] = lz1;
	mat1.SetRotationDeg(1, s.j[1]);
	mat2.mat[11] = lz2;
	mat2.SetRotationDeg(1, s.j[2]);
	mat3.mat[11] = lz3;
	mat3.SetRotationDeg(1, s.j[3]);
	mat4.mat[11] = lz4;
	
	cpr_Matrix4 res;
	res = mat1 * mat0;
	res = mat2 * res;
	res = mat3 * res;
	res = mat4 * res;

	// compute the ABC angles
	double a,b,c;
	res.MatrixToABC(&a, &b, &c);

	setPointState.p[0] = res.mat[3];	
	setPointState.p[1] = res.mat[7];	
	setPointState.p[2] = res.mat[11];	
	setPointState.p[3] = b;

	// store in the member variable
	for(int i=0; i<16; i++)
		lastPosition.mat[i] = res.mat[i];
	
	return 0;
}



//*************************************************************** 
// Computes the joint values for a given cartesian position
// Direct geometrical method
// The ellbow-up / down symmetry is choosen by the according flag
// this implementation does not allow a definition of the B angle
int cpr_KinematicMover::inverseKin(robotState s){

	// Create a Matrix out of the position
	cpr_Matrix4 targetMat;

	for(int i=0; i<16; i++)			// copy last orientation
		targetMat.mat[i] = lastPosition.mat[i];
	// to also rotate with B: generate this matrix by the ABC Euler angles

	targetMat.mat[3] = s.p[0];
	targetMat.mat[7] = s.p[1];
	targetMat.mat[11] = s.p[2];


	// get the wrist point by subtracting the last element
	double x = targetMat.mat[3] - targetMat.mat[2] * lz4;
        double y = targetMat.mat[7] - targetMat.mat[6] * lz4;
        double z = targetMat.mat[11]- targetMat.mat[10]* lz4;

	// subtract the height till the first y joint
	z = z - (lz0 + lz1);

	double e1, e2, e3, e4;

        //Center Axis
        //Check the proximity singularity
    	if (fabs(x) < EpsilonCenter && fabs(y) < EpsilonCenter)
    	{
        	return 3;    // too close to the center axis
    	}
    	e1 = atan2(y, x);

        // A2 und A3
        // Hypothenuse from 0|0 to x|y, view from above
        double HEbene = sqrt(x * x + y * y);

        // Hypothenuse in arm level
        double HVert = sqrt(HEbene * HEbene + z * z);

	// Singularity        
	if (HVert > (lz2 + lz3))
        {
            return 4;       // cannot reach the position
        }

        // alpha1 is the first angle A2 from the horizontal up
        double alpha1 = asin(z / HVert);

        // Theorem of Heron 
        double UArm = lz3;
        double alpha2 = acos((-(UArm * UArm) + HVert * HVert + lz2 * lz2) / (2.0 * HVert * lz2));
        double alpha3 = acos((-(HVert * HVert) + UArm * UArm + lz2 * lz2) / (2.0 * UArm * lz2));

	setPointState.j[0] = rad2deg * e1;
            
	// Choose the symmetry         
	if (flagSymmetryEllbowUp == true)
        {
            setPointState.j[1] = 90.0 - (rad2deg * (alpha1 + alpha2));
            setPointState.j[2] = 180.0 - (rad2deg * alpha3);
        }
        else
        {
            setPointState.j[1] = 90.0 + (rad2deg * (alpha2 - alpha1));
            setPointState.j[2] = -180.0 + (rad2deg * alpha3);
        }

	// Joint 4: Simple computation using B and the joints 2 and 3
	double tmp = 180.0 - (setPointState.j[1] + setPointState.j[2]);
	
	e4 = tmp - setPointState.p[3];
	setPointState.j[3] = e4;

	return 0;
}





//***************************************************************
// int ticks:		joint encoder tics
// return value: 	joint position in degree
double cpr_KinematicMover::computeJointPos(int joint, int ticks){
	double gearZero = 32000.0;
	double p = (ticks - gearZero)/gearScale[joint];
	return p;
}


//***************************************************************
// double pos:		joint position in degree
// return value: 	joint encoder ticks
int cpr_KinematicMover::computeTics(int joint, double pos){
	double gearZero = 32000.0;
	int t =  ((int)(pos*gearScale[joint])) + gearZero;
	return t;
}




