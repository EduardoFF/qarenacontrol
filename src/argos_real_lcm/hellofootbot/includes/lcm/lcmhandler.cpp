/*
 * Copyright (C) 2014, IDSIA (Institute Dalle Molle for Artificial Intelligence), http://http://www.idsia.ch/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "lcmhandler.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

void LCMHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const poselcm::pose_list_t* msg) {

	// Node to insert or update
	Node * node;
	// Node positions
	CVector3 positions;
	// Node orientation
	CQuaternion orientations;
	// To set the timestamp
	struct timeval tp;

	for (int i = 0; i < msg->n; i++) {
		const poselcm::pose_t &pose = msg->poses[i];

		// New node
		node = new Node();

		// ID
		node->setId(pose.robotid);

		// Timestamp
		gettimeofday(&tp, NULL);
		UInt64 ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
		node->setTimestamp(ms);

		// Position
		positions.Set(pose.position[0], pose.position[1], pose.position[2]);
		node->setPosition(positions);

		// Orientation
		orientations = CQuaternion(pose.orientation[0]/UNIT_CONV_FACTOR, pose.orientation[1]/UNIT_CONV_FACTOR, pose.orientation[2]/UNIT_CONV_FACTOR, pose.orientation[3]/UNIT_CONV_FACTOR);
		node->setOrientation(orientations);

		// Add or Update to the list
		addOrUpdateNode(node);

	}
	printf("\n");

	// Print new list status
	printNodeListElements();

}

bool LCMHandler::existNode(const UInt8 id) {
	return listAllNodes.end() != listAllNodes.find(id);
}

void LCMHandler::addOrUpdateNode(Node * node) {

	// If does not exist, add it!
	if (!existNode(node->getId())) {
		listAllNodes.insert(pair<UInt8, Node*>(node->getId(), node));
	} else {
		//Remove
		listAllNodes.erase(node->getId());
		//Insert
		listAllNodes.insert(pair<UInt8, Node*>(node->getId(), node));
	}

}

void LCMHandler::printNodeListElements() {

	printf("Node list with %d elements.\n", listAllNodes.size());

	for (map<UInt8, Node*>::iterator it = listAllNodes.begin(); it != listAllNodes.end(); it++) {

		printf("ID %d - timestamp %lu\n", it->first, (it->second)->getTimestamp());
		printf("ID %d - location (%f,%f,%f)\n", it->first, (it->second)->getPosition().GetX(), (it->second)->getPosition().GetY(), (it->second)->getPosition().GetZ());
		printf("ID %d - orientation (%f,%f,%f,%f)\n", it->first, (it->second)->getOrientation().GetW(), (it->second)->getOrientation().GetX(), (it->second)->getOrientation().GetY(),
				(it->second)->getOrientation().GetZ());

	}

}

Node * LCMHandler::getNodeById(const UInt8 id) {
	return listAllNodes.at(id);
}
