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

// Last change: May 16th, 2013



#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "cpr_Matrix4.h"

#define PI 3.14159265

using namespace std;




//********************************************
// Inits the matrix
cpr_Matrix4::cpr_Matrix4()
{
	for(int i=0; i<16; i++)
            mat[i] = 0.0f;

    	mat[0] = 1.0f;
    	mat[5] = 1.0f;
    	mat[10] = 1.0f;
    	mat[15] = 1.0f;
}


	

//********************************************
void cpr_Matrix4::SetRotationDeg(int axis, double ang)
{
    SetRotationRad(axis, ang * PI / 180.0);

}

//********************************************
// Set a 3D rotation to the matrix 
// axis: 0=x, 1=y, 2=z defines the rotation axis
// ang is in radian
void cpr_Matrix4::SetRotationRad(int axis, double ang)
{
    float cosa = (float)cos(ang);
    float sina = (float)sin(ang);

    if (axis == 0) //x
    {
        mat[0] = 1.0f; mat[1] = 0.0f; mat[2] = 0.0f;
        mat[4] = 0.0f; mat[5] = cosa; mat[6] = -sina;
        mat[8] = 0.0f; mat[9] = sina; mat[10] = cosa;
    }
    else if (axis == 1) //y
    {
        mat[0] = cosa; mat[1] = 0.0f; mat[2] = sina;
        mat[4] = 0.0f; mat[5] = 1.0f; mat[6] = 0.0f;
        mat[8] = -sina; mat[9] = 0.0f; mat[10] = cosa;
    }
    else if (axis == 2)	//Z
    {
        mat[0] = cosa; mat[1] = -sina; mat[2] = 0.0f;
        mat[4] = sina; mat[5] = cosa; mat[6] = 0.0f;
        mat[8] = 0.0f; mat[9] = 0.0f; mat[10] = 1.0f;
    }
    
}



//***************************************************************************************************
/// <summary>
/// computes the ABC Euler angel from the rotation matrix
/// ZYX-Convention (YawPichRoll) at http://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel
/// or Craig S47ff
/// angels in degree
/// At beta = +/- Pi/2 there are Gimbal Locks, multiple solutions
/// </summary>
void cpr_Matrix4::MatrixToABC(double * A, double * B, double * C)
{
   
    double cb;
    double eps = 0.001;
    double a, b, c;
    b = atan2(-mat[4*2+0], sqrt(mat[4*0+0] * mat[4*0+0] + mat[4*1+0] * mat[4*1+0]));

    
    if( fabs(b - PI/2.0) < eps){           // Singularity b = Pi/2
        a = 0.0;
        c = atan2(mat[4*0+1], mat[4*1+1]);

    }else if( fabs(b + PI/2.0) < eps){     // Singularity b = - PI/2
        a = 0.0;
        c = -atan2(mat[4*0+1], mat[4*1+1]);
    } else{                                             // normal case

        cb = cos(b);
        a = atan2(mat[4*1+0] / cb, mat[4*0+0] / cb);
        c = atan2(mat[4*2+1] / cb, mat[4*2+ 2] / cb);
    }
    
    a *= rad2deg;
    b *= rad2deg;
    c *= rad2deg;

    A[0] = a;
    B[0] = b;
    C[0] = c;
   
}


	


