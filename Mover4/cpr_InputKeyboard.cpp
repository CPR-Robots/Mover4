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

// Last change: May 16th, 2013


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



    char c;

    while (true)
    {
	erase();

    	// Output on the terminal screen
    	mvprintw(2,0, "  Mover4 test program");
    	mvprintw(3,0, "  Use the following keys to jog the joints");
    	mvprintw(5,0, "    q         w        e       r          z       u       i         ");
    	mvprintw(6,0, "  Joint1   Joint2   Joint3   Joint4     CartX   CartY   CartZ   RotB");
    	mvprintw(7,0, "    a         s        d       f          h       j       k         ");
    	mvprintw(9,2,  "%.1lf", ctx->setPointJoints[0]);
    	mvprintw(9,12,  "%.1lf", ctx->setPointJoints[1]);
    	mvprintw(9,22,  "%.1lf", ctx->setPointJoints[2]);
    	mvprintw(9,31,  "%.1lf", ctx->setPointJoints[3]);

    	mvprintw(9,42,  "%.1lf", ctx->setPointPosition[0]);
    	mvprintw(9,50,  "%.1lf", ctx->setPointPosition[1]);
    	mvprintw(9,58,  "%.1lf", ctx->setPointPosition[2]);
    	mvprintw(9,66,  "%.1lf", ctx->setPointPosition[3]);


      	mvprintw(10,2, "%.1lf", ctx->currJoints[0]);
    	mvprintw(10,12, "%.1lf", ctx->currJoints[1]);
    	mvprintw(10,22, "%.1lf", ctx->currJoints[2]);
    	mvprintw(10,31, "%.1lf", ctx->currJoints[3]);

	mvprintw(10,42,  "%.1lf", ctx->currPosition[0]);
    	mvprintw(10,50,  "%.1lf", ctx->currPosition[1]);
    	mvprintw(10,58,  "%.1lf", ctx->currPosition[2]);
    	mvprintw(10,66,  "%.1lf", ctx->currPosition[3]);


    	mvprintw(14, 2, "Reset errors: p - Enable Motors: o - Set joint positions to zero: x");

	ctx->UpdateMessages();



    	c = getch();

	double vel = 1.0;

    	switch(c){
			// joint keys
			case 'q': ctx->motionType = 0; ctx->motionVec[0] = vel; break;
			case 'a': ctx->motionType = 0; ctx->motionVec[0] = -vel; break;
			case 'w': ctx->motionType = 0; ctx->motionVec[1] = vel; break;
			case 's': ctx->motionType = 0; ctx->motionVec[1] = -vel; break;
			case 'e': ctx->motionType = 0; ctx->motionVec[2] = vel; break;
			case 'd': ctx->motionType = 0; ctx->motionVec[2] = -vel; break;
			case 'r': ctx->motionType = 0; ctx->motionVec[3] = vel; break;
			case 'f': ctx->motionType = 0; ctx->motionVec[3] = -vel; break;
			
			// cartesian keys
			case 'z': ctx->motionType = 1; ctx->motionVec[0] = vel; break;
			case 'h': ctx->motionType = 1; ctx->motionVec[0] = -vel; break;
			case 'u': ctx->motionType = 1; ctx->motionVec[1] = vel; break;
			case 'j': ctx->motionType = 1; ctx->motionVec[1] = -vel; break;
			case 'i': ctx->motionType = 1; ctx->motionVec[2] = vel; break;
			case 'k': ctx->motionType = 1; ctx->motionVec[2] = -vel; break;

			// reset, enable and zero
			case 'p': ctx->flagReset = true; break;
			case 'o': ctx->flagEnable = true; break;
			case 'x': ctx->flagZero = true; break;
    	}

    	refresh();			/* Print it on to the real screen */

    }
    endwin();				/* End curses mode		  */
    return;
}


//***************************************************************
// Provides the motion vector the user has generated with the keys
void cpr_InputKeyboard::GetMotionVec(double * v){

	for(int i=0; i<6; i++){
		v[i] = motionVec[i];			// copy the current values generated in the readLoop

		if(fabs(motionVec[i]) > 0.1){		// and do a slow deceleration
			if(motionVec[i] > 0.0)
				motionVec[i] -= 0.1;
			else if(motionVec[i] < 0.0)
				motionVec[i] += 0.1;
		}else{
			motionVec[i] = 0.0;
		}
	}


	return;
}



//***************************************************************
// stores the joint values for printing on the screen
void cpr_InputKeyboard::SetJoints(double * sj, double * cj){

	for (int i=0; i<4; i++ ){
		setPointJoints[i] = sj[i];
		currJoints[i] = cj[i];
	}

}


//***************************************************************
// stores the position values for printing on the screen
void cpr_InputKeyboard::SetPosition(double *sp, double *cp){
	for (int i=0; i<4; i++ ){
		setPointPosition[i] = sp[i];
		currPosition[i] = cp[i];
	}
}


//***************************************************************
// adds a message to be shown on the screen
void cpr_InputKeyboard::SetMessage(string msg){

	messages[indexMsg].clear();
	messages[indexMsg].append(msg);
	indexMsg++;
	if(indexMsg > 4)
		indexMsg = 0;

	return;
}


//***************************************************************
// prints the messages on the screen
void cpr_InputKeyboard::UpdateMessages(){

	mvprintw(12, 2, "Status: %s", statusString.c_str());
	mvprintw(16, 2, "Messages:");
	for(int i=0; i<5; i++){
		int nr = indexMsg-1-i;
		if(nr<0) nr+=5;
		mvprintw(17+i, 5, "                                                 ");
		mvprintw(17+i, 5, messages[nr].c_str());
	}
	refresh();
	return;
}

//***************************************************************
// prints the joint controller status on the screen
void cpr_InputKeyboard::SetStatus(string msg){
	statusString.clear();
	statusString.append(msg);
	mvprintw(12, 2, "                                                ");
	mvprintw(12, 2, "Status: %s", statusString.c_str());
	refresh();
	return;
}




//***************************************************************
// constructor
cpr_InputKeyboard::cpr_InputKeyboard()
{
	flagEnable = false;	// initialize the flags
	flagReset = false;
	flagZero = false;

	motionType = 0;		// Joint motion first

	(void) initscr();      // init the curses library
	keypad(stdscr, TRUE);  // enable keyboard mapping
	(void) nonl();         // tell curses not to do NL->CR/NL on output
	(void) cbreak();       // take input chars one at a time, no wait for \n ; necessary for moving the robot
	(void) noecho();        // do not echo input

	boost::thread keyThread(keyLoop, (void*)this);		// main loop to read/write the keyboard/screen

	for (int i=0; i<6; i++ )
		motionVec[i] = 0.0;

	statusString[0] = 'n';
	statusString[1] = '/';
	statusString[2] = 'a';

	for(int i=0; i<5; i++)
		messages[i] = ".";

	indexMsg = 0;
}








