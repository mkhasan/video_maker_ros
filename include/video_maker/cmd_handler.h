/*
 * cmd_handler.h
 *
 *  Created on: Oct 22, 2019
 *      Author: kict
 */

#ifndef CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_CMD_HANDLER_H_
#define CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_CMD_HANDLER_H_

#include "video_maker/video_maker.h"

#include <mosquittopp.h>


namespace video_maker  {

class CmdHandler : public mosqpp::mosquittopp
{

private:
	info_t & info;
public:
	CmdHandler(info_t & info, const char *id, const char *host, int port);
	~CmdHandler();




	void on_connect(int rc);
	void on_message(const struct mosquitto_message *message);




};


}



#endif /* CATKIN_WS_SRC_VIDEO_MAKER_INCLUDE_VIDEO_MAKER_CMD_HANDLER_H_ */
