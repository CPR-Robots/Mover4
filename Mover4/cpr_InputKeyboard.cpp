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
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>


#include <ncurses.h>		// terminal lib to get keyboard input
//http://tldp.org/HOWTO/NCURSES-Programming-HOWTO/misc.html

#include "cpr_InputKeyboard.h"

using namespace std;




//***************************************************************
void keyLoop(void * context )
{
	cpr_InputKeyboard *ctx;
	ctx = (cpr_InputKeyboard*)context;


	(void) initscr();      // init the curses library
	keypad(stdscr, TRUE);  // enable keyboard mapping
	(void) nonl();         // tell curses not to do NL->CR/NL on output
	(void) cbreak();       // take input chars one at a time, no wait for \n ; necessary for moving the robot
	(void) noecho();        // do not echo input

    char c;

    while (true)
    {

    	// Output on the terminal screen
    	mvprintw(2,0, "  Mover4 test program");
    	mvprintw(3,0, "  Use the following keys to jog the joints");
    	mvprintw(5,0, "    q           w           e           r");
    	mvprintw(6,0, "  Joint 1     Joint 2     Joint 3     Joint 4");
    	mvprintw(7,0, "    a           s           d           f");
    	mvprintw(9,2,  "%.1lf", ctx->setPointJoints[0]);
    	mvprintw(9,14,  "%.1lf", ctx->setPointJoints[1]);
    	mvprintw(9,26,  "%.1lf", ctx->setPointJoints[2]);
    	mvprintw(9,38,  "%.1lf", ctx->setPointJoints[3]);

    	mvprintw(10,2, "%.1lf", ctx->currJoints[0]);
    	mvprintw(10,14, "%.1lf", ctx->currJoints[1]);
    	mvprintw(10,26, "%.1lf", ctx->currJoints[2]);
    	mvprintw(10,38, "%.1lf", ctx->currJoints[3]);

    	mvprintw(12, 2, "Status: %s", ctx->statusString);

    	mvprintw(14, 2, "Reset errors and enable:    p");
    	mvprintw(15, 2, "Set joint position to zero: z");


    	c = getch();

    	switch(c){
			case 'q': ctx->motionVec[0] =  1.0; break;
			case 'a': ctx->motionVec[0] = -1.0; break;
			case 'w': ctx->motionVec[1] =  1.0; break;
			case 's': ctx->motionVec[1] = -1.0; break;
			case 'e': ctx->motionVec[2] =  1.0; break;
			case 'd': ctx->motionVec[2] = -1.0; break;
			case 'r': ctx->motionVec[3] =  1.0; break;
			case 'f': ctx->motionVec[3] = -1.0; break;
			case 'p': ctx->flagReset = true; break;
			case 'z': ctx->flagZero = true; break;
    	}

    	refresh();			/* Print it on to the real screen */

    }
    endwin();				/* End curses mode		  */
    return;
}


//***************************************************************
void cpr_InputKeyboard::GetMotionVec(double * v, bool reset, bool zero){

	for(int i=0; i<6; i++){
		v[i] = motionVec[i];

		if(fabs(motionVec[i]) > 0.1){
			if(motionVec[i] > 0.0)
				motionVec[i] -= 0.05;
			else if(motionVec[i] < 0.0)
				motionVec[i] += 0.05;
		}else{
			motionVec[i] = 0.0;
		}
	}

	reset = flagReset;
	zero = flagZero;
	flagReset = false;
	flagZero = false;

	return;
}



//***************************************************************
void cpr_InputKeyboard::SetJoints(double * sj, double * cj){

	for (int i=0; i<4; i++ ){
		setPointJoints[i] = sj[i];
		currJoints[i] = cj[i];
	}

}



//***************************************************************
cpr_InputKeyboard::cpr_InputKeyboard()
{
	boost::thread keyThread(keyLoop, (void*)this);

	for (int i=0; i<6; i++ )
		motionVec[i] = 0.0;

	statusString[0] = 'n';
	statusString[1] = '/';
	statusString[2] = 'a';

}








