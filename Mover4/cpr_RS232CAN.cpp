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

#include "cpr_RS232CAN.h"

using namespace::boost::asio;
using namespace std;


#define COMPORT "/dev/ttyUSB0"




//***************************************************************
void readLoop(void * context )
{
	cpr_RS232CAN *ctx;
	ctx = (cpr_RS232CAN*)context;
    //std::cout << "readLoop: running"  << std::endl;
    char bu[10];
    char buffer[10];
    int bufferCnt = 0;

    int tmpid = 0;
    while (true)
    {

    	/*
    	 * Main Read loop: read one byte in a time
    	 * When a known sender is found the complete CAN message of 10 bytes is read
    	 * Still missing: resync when too much time has passed between the single bytes
    	 */

    	if(ctx->active)
    			boost::asio::read(*(ctx->port), boost::asio::buffer(bu, 1));


    	if(bufferCnt == 0){
    		tmpid = (int)bu[0];
    		if(tmpid == 17 || tmpid == 33 || tmpid == 49 || tmpid == 65){
    			buffer[0] = bu[0];
    			bufferCnt++;
    		}
    	}else{
    		buffer[bufferCnt] = bu[0];
    		bufferCnt++;

    		if(bufferCnt == 10){
    			ctx->EvaluateBuffer(buffer);
    			bufferCnt = 0;
    		}


    	}

/*
    		std::cout << "read: " << (int)buffer[0];
    		std::cout << " " << (int)buffer[1];
    		std::cout << " " << (int)buffer[2];
    		std::cout << " " << (int)buffer[3];
    		std::cout << " " << (int)buffer[4];
    		std::cout << " " << (int)buffer[5];
    		std::cout << " " << (int)buffer[6];
    		std::cout << " " << (int)buffer[7];
    		std::cout << " " << (int)buffer[8];
    		std::cout << " " << (int)buffer[9] << std::endl;
*/


    }

    std::cout << "readLoop: finished" << std::endl;
}


//***************************************************************
cpr_RS232CAN::cpr_RS232CAN()
{

}


//***************************************************************
bool cpr_RS232CAN::Connect(void )
{
	io_service io;
	const char *PORT = COMPORT;
	serial_port_base::baud_rate baud_option(115200);
	active = false;

	try{
		port = new serial_port(io);
		port->open(PORT);
		port->set_option(baud_option); // set the baud rate

		active = true;
		std::cerr << "Port " << PORT << " opened\n";

	}catch (boost::system::system_error &e){
		boost::system::error_code ec = e.code();
		std::cerr << "Cannot open port " << PORT << " - error code: " << ec.category().name() << std::endl;
	}catch(std::exception e){
		std::cerr << "Cannot open port " << PORT << " - error code: " << e.what() << endl;
	}

	boost::thread readThread(readLoop, (void*)this);

	return true;
}

//***************************************************************
bool cpr_RS232CAN::Disconnect()
{
	if(port->is_open()){
		active = false;
		port->close();
		std::cout << "Port closed";
	}
}


//***************************************************************
/*
 * Checks for validity of the messages and stores them in a buffer for later access
 * Validity check here is poor; For performing applications further checks should be incorporated.
 */
int cpr_RS232CAN::EvaluateBuffer(char* buf){

	int i = 0;
	int mid = (int)buf[0];
	int length = (int)buf[1];

	if(length <= 8){
		msgBuffer[mid].length = length;
		msgBuffer[mid].id = mid;
		for(i=0; i<8; i++)
			msgBuffer[mid].data[i] = buf[i+2];
		//std::cout << "found good msg\n";
	}else{
		std::cerr << "found bad msg\n";

	}

	return 0;
}


//***************************************************************
void cpr_RS232CAN::WriteMsg(int id, int length, char* data)
{
	unsigned char commands[10] = {16, 4, 4, 125, 125, 0,0,0,0,0};

	commands[0] = id;
	commands[1] = length;
	for(int i=0; i<8; i++)
		commands[2+i] = data[i];

	if(active)
		boost::asio::write(*port, boost::asio::buffer(commands, 10));

	return;
}


//***************************************************************
// Forwards a received message from the buffer
int cpr_RS232CAN::GetMsg(int id, int *length, char* data)
{

	if(id>255)
		throw std::string("invalid message id!");


	length[0] = msgBuffer[id].length;
	for(int i=0; i<8; i++)
		data[i] = msgBuffer[id].data[i];
	//m.time = msgBuffer[id].time;

	return 0;

	
}






