/*
 * LcmHandler.cpp
 *
 *  Created on: 29/10/2014
 *      Author: roberto
 */

#include <stdio.h>

#include "lcmHandler.h"

LCMHandler::LCMHandler() {
	// TODO Auto-generated constructor stub
}

void LCMHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const poselcm::pose_list_t* msg) {
	int i;
	printf("Received message on channel \"%s\":\n", chan.c_str());
	printf("  timestamp   = %lld\n", (long long) msg->timestamp);
	printf("  poses:");
	for (i = 0; i < msg->n; i++) {
		const poselcm::pose_t &pose = msg->poses[i];
		printf("  robotid = %d\n", pose.robotid);
		printf("  position    = (%d, %d, %d)\n", pose.position[0], pose.position[1], pose.position[2]);
		printf("  orientation = (%d, %d, %d, %d)\n", pose.orientation[0], pose.orientation[1], pose.orientation[2], pose.orientation[3]);

	}
	printf("\n");

	robotList = msg;
}

poselcm::pose_t getInfoByRobotId(const poselcm::pose_list_t * msg, const std::string robotId) {

	poselcm::pose_t myPose;
	poselcm::pose_t pose;

	uint8_t robotIdInt;
	robotIdInt = atoi(robotId.substr(3).c_str());

	for (int i = 0; i < msg->n; i++) {
			 &pose = msg->poses[i];

			 // Is it mine?
			 if (robotIdInt == pose.robotid){
				 myPose = pose;
			 }
	}

	return myPose;

}

LCMHandler::~LCMHandler() {
	// TODO Auto-generated destructor stub
}

