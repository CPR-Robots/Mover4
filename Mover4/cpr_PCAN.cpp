/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Commonplace Robotics GmbH
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
#include <string>
#include <iostream>
#include <stdexcept>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/lexical_cast.hpp>

#include "cpr_PCAN.h"

using namespace boost::posix_time;
using namespace std;





//***************************************************************
void readLoopPCAN(void * context )
{
	ptime start, now;
	time_duration passed;

	TPCANRdMsg msg;					// PEAK message structure
	msg.Msg.LEN = 0;
	int ret = 0;
	int i=0;


	cpr_PCAN *ctx;
	ctx = (cpr_PCAN*)context;
    	std::cout << "readLoop: running"  << std::endl;
    
    	int tmpid = 0;
    	while (true)
    	{

    		/*
    	 	* Main Read loop
    	 	*/

    		if(ctx->active){

			ret = LINUX_CAN_Read_Timeout(ctx->h, &msg, 1000); /* wait 1 ms */

			//if(ret == CAN_ERR_OK){
			if(msg.Msg.LEN > 2){
				int id = msg.Msg.ID;
				ctx->msgBuffer[id].length = msg.Msg.LEN;
				ctx->msgBuffer[id].id = id;
				for(i=0; i<8; i++)
					ctx->msgBuffer[id].data[i] = msg.Msg.DATA[i];
			}else{
				;
			}
		}

    	}



    std::cout << "readLoop: finished" << std::endl;
}

//***************************************************************
cpr_PCAN::cpr_PCAN()
{
	// init the message buffer
	for(int i=0; i< 256; i++){
		msgBuffer[i].length = 0;
		msgBuffer[i].id = i;
		for(int j=0; j<8; j++)
			msgBuffer[i].data[j] = 0x80;
	}

}


//***************************************************************
bool cpr_PCAN::Connect(void )
{
	active = false;
	string s;
	try{
		h = LINUX_CAN_Open("/dev/pcan32", O_RDWR);			// pcan32 for PCAN-USB adapter

		int status = CAN_Status(h);
		string sts = "";
		if(status == 0x0000 || status == 0x0020)			// ok oder RcBufferEmpty
			sts = "no error";
		else if( status == 0x0004 || status == 0x0008 || status == 0x0010)
			sts = "bus error";
		else if( status ==0x0100  || status == 0x2000 || status == 0x1C00 || status == 0x4000 || status == 0x8000)		
			sts = "device error";
		else
			sts = "general error";

		if(sts == "no error"){
			active = true;
			s = "PCANUSB opened successfully: ";
			s += sts;	
		}else{
			active = false;
			s = "could not connect to PCANUSB: ";
			s += sts;
		}	


	}catch (boost::system::system_error &e){
		boost::system::error_code ec = e.code();
		s = "Could not connect to PCANUSB! "; 
	}catch(std::exception e){
		s = "Could not connect to PCANUSB! "; // + string(e.what());

	}

	keys->SetMessage(s);
	boost::thread readThread(readLoopPCAN, (void*)this);

	return true;
}

//***************************************************************
bool cpr_PCAN::Disconnect()
{
	active = false;
	CAN_Close(h);
	std::cout << "PCAN disconnected";
	return active;
}



//***************************************************************
void cpr_PCAN::WriteMsg(int id, int length, char* data, bool waitForAnswer)
{
	TPCANMsg msg;					// PEAK message structure
	int ret = 0;

	if(!active)
		return;

	msg.ID = id;
	msg.LEN = (unsigned char)length;
	for (int i = 0; i < length; i++)
		msg.DATA[i] = data[i];
	msg.MSGTYPE = 0x00;				// only standard messages

  	ret = CAN_Write(h, &msg);

        if (ret != CAN_ERR_OK){
		string s = "error in writing";
		keys->SetMessage(s);
//        }else{
//		string s = "success in writing";
//		keys->SetMessage(s);
	}

	return;
}




//***************************************************************
// Forwards a received message from the buffer
int cpr_PCAN::GetMsg(int id, int *length, char* data)
{


	if(id>255)
		throw std::string("invalid message id!");


	length[0] = msgBuffer[id].length;
	for(int i=0; i<8; i++)
		data[i] = msgBuffer[id].data[i];
	//m.time = msgBuffer[id].time;

	return 0;

	
}








