/*
 * video_maker_node.cpp
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "spectator/info.h"
#include "spectator/get_str_list.h"

#include "video_maker/video_maker.h"
#include "video_maker/cmd_handler.h"
#include "client_interface/client_interface.h"

#include <sstream>
#include <signal.h>
#include <string>


using namespace std;
using namespace video_maker;

static const int FREQUENCY = 20.0;

boost::shared_ptr<CmdHandler> cmdHandler;

pthread_t video_maker_thread;
info_t info;

void mySigintHandler(int sig)
{

	ROS_INFO("going to terminate");

	info.quit = true;
	pthread_join( video_maker_thread, NULL);
	ros::shutdown();

}


void SpectatorCallback(const spectator::info::ConstPtr & msg) {

	//ROS_DEBUG("current snapping mode is %d", msg->snapperWaitingMode);

	if (msg->snapperWaitingMode == client_interface::SNAPPER_WAITING_FOR_VIDEO) {
		info.mode = info_t::SNAP_AROUND;
	}
	else {
		if (info.mode == info_t::SNAP_AROUND) {
			info.mode = info_t::NONE;
		}
	}


}





int main(int argc, char **argv) {
	ros::init(argc, argv, "video_maker_node", ros::init_options::NoSigintHandler);

	signal(SIGTERM, mySigintHandler);
	signal(SIGINT, mySigintHandler);          // caught in a different way fo$
	signal(SIGHUP, mySigintHandler);
	signal(SIGKILL, mySigintHandler);
	signal(SIGTSTP, mySigintHandler);



	ros::NodeHandle nh;


	ros::Rate loop_rate(FREQUENCY);


	ros::Subscriber cur_angle_sub = nh.subscribe("spectator/info", 1000, SpectatorCallback);

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	string mutexPrefix;
	nh.param("/ROBOT/MUTEX_PREFIX", mutexPrefix, string("kict_mp_camera00_"));

	ROS_DEBUG("MUTEX PREFIX IS %s", mutexPrefix.c_str());



	string outputDir;
	nh.getParam("/path/common", outputDir);
	if (outputDir.length() == 0)
		outputDir = "video_files";
	ROS_DEBUG("out put dir is %s len is %d", outputDir.c_str(), outputDir.length());

	int rc;


	ros::ServiceClient client = nh.serviceClient<spectator::get_str_list>("client_interface/get_camera_names");
	spectator::get_str_list camNames;

	int tryCount;

	for (tryCount=0; tryCount < 10; ++tryCount) {
		if(client.call(camNames)) {
			info.cameraNames = camNames.response.list;
			if (info.cameraNames.size() > 0)
				break;
			else {
				ROS_ERROR("no camera found ...");
			}


		}
		else {
			ROS_ERROR("Error in getting camera names ...");

		}

		ros::Duration(1.5).sleep();
	}

	if (tryCount == 10)
		return -1;

	info.mode = info_t::NONE;
	info.quit = false;

	string video_root = "/home/kict" + string("/");
	video_root += outputDir;
	info.video_root = video_root + string("/");
	info.mutexPrefix = mutexPrefix;

	int iret = pthread_create( &video_maker_thread, NULL, video_maker_func, (void *)&info);
	if(iret) {
		ROS_ERROR("Error in creating video_maker_func");
		return -1;
	}



	cmdHandler.reset(new CmdHandler(info, "video_maker_node", "127.0.0.1", 1883));

	while (ros::ok()) {


		rc = cmdHandler->loop();
		if(rc)
			cmdHandler->reconnect();

		//ROS_DEBUG("Inside the main loop");
		ros::spinOnce();
		loop_rate.sleep();


	}



	return 0;
}


