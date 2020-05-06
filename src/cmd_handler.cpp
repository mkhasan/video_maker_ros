/*
 * cmd_handler.cpp
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */


#include "ros/ros.h"


#include "video_maker/cmd_handler.h"
#include "client_interface/client_interface.h"

#include <boost/algorithm/string.hpp>


#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>



using namespace std;





video_maker::CmdHandler::CmdHandler(info_t & _info, const char *id, const char *host, int port)
	: mosquittopp(id)
	, info(_info)
{
	int keepalive = 60;

	/* Connect immediately. This could also be done by calling
	 * mqtt_tempconv->connect(). */

	connect(host, port, keepalive);
};


video_maker::CmdHandler::~CmdHandler()
{
	disconnect();
}

void video_maker::CmdHandler::on_connect(int rc)
{
	cout << "Connected with code " << rc << endl;
	if(rc == 0){


		subscribe(NULL, DB_AUTO_VIDEO_START);
		subscribe(NULL, DB_AUTO_STOP);
		subscribe(NULL, DB_AUTO_VIDEO_STOP);


	}
}

void video_maker::CmdHandler::on_message(const struct mosquitto_message *message)
{
	if (!strcmp(message->topic, DB_AUTO_VIDEO_START)) {
		ROS_DEBUG("starting");
		info.mode = info_t::AUTO;
	}

	else if (!strcmp(message->topic, DB_AUTO_STOP) || !strcmp(message->topic, DB_AUTO_VIDEO_STOP)) {
		ROS_DEBUG("stopping");
		info.mode = info_t::NONE;
	}
	/*
	else if (!strcmp(message->topic, "TEST")) {
		ROS_DEBUG("testing");
		info.mode = info_t::SNAP_AROUND;
		sleep(1);
//		info.mode = info_t::NONE;
	}
	*/



}


