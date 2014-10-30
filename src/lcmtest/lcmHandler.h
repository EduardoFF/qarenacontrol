/*
 * LcmHandler.h
 *
 *  Created on: 29/10/2014
 *      Author: roberto
 */

#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"

class LCMHandler {
public:

	LCMHandler();


	/*
	 *
	 * */
	void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const poselcm::pose_list_t* msg);

	poselcm::pose_t getInfoByRobotId(const poselcm::pose_list_t * msg, const std::string robotId);

	inline void setRobotList(poselcm::pose_list_t * list) {
		robotList = list;
	}

	inline poselcm::pose_list_t * getRobotList(){
		return robotList;
	}

	virtual ~LCMHandler();

private:
	/* List of robots from LCM */
	poselcm::pose_list_t * robotList;

};

