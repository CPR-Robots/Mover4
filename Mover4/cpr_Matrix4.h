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


#ifndef def_cpr_Matrix4
#define def_cpr_Matrix4


#include <math.h>
#include <stdio.h>
#include <stdlib.h>


using namespace std;


// Basic 4x4 homogenous matrix
// the upper left area is the 3D rotation matrix, the last column is xyz
// indexing: 0 to 4: upper row
class cpr_Matrix4
{
	public:
		float mat[16]; 	// the matrix contents

	        const static double deg2rad = 3.14159265 / 180.0;
	        const static double rad2deg = 180.0 / 3.14159265;

	  	cpr_Matrix4();
		void SetRotationDeg(int axis, double ang);
		void SetRotationRad(int axis, double ang);
		void MatrixToABC(double * A, double * B, double * C);

	private:

	  

};

	//*************************************************
	/// <summary>
	/// Multiply two 4x4 Matrices
	/// </summary>
	static cpr_Matrix4 operator *(cpr_Matrix4 a, cpr_Matrix4 b)
	{
            cpr_Matrix4 res; 
            int i, j, k;
            float ab;

            for (j = 0; j < 4; j++)
            {
                for (i = 0; i < 4; i++)
                {
                    ab = 0.0f;
                    for (k = 0; k < 4; k++)
                        ab += a.mat[k*4+i] * b.mat[j*4+k];
                    res.mat[j*4+i] = ab;
                }
            }
            return res;
        }


	

#endif

