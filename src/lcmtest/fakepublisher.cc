#include <stdlib.h>
#include <time.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"

int main(int argc, char ** argv) {

	lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
	//lcm::LCM lcm;
	if (!lcm.good())
		return 1;

	/* initialize random seed: */
	srand (time(NULL));

	/* generate secret number between 1 and 10: */
int 	randomNumber = rand() % 1000 + 1;

	poselcm::pose_list_t mymsg;
	mymsg.n = 2;
	mymsg.poses.resize(mymsg.n);

	//ID 2
	poselcm::pose_t my_data;
	my_data.robotid = 0;
	my_data.position[0] = rand() % 5000 + 1;
	my_data.position[1] = rand() % 5000 + 1;
	my_data.position[2] = rand() % 5000 + 1;

	my_data.orientation[0] = rand() % 5000 + 1;
	my_data.orientation[1] = rand() % 5000 + 1;
	my_data.orientation[2] = rand() % 5000 + 1;
	my_data.orientation[3] = rand() % 5000 + 1;
	mymsg.poses[0] = my_data;

	//ID 1
	poselcm::pose_t my_data_1;
	my_data_1.robotid = 1;
	my_data_1.position[0] = rand() % 5000 + 1;
	my_data_1.position[1] = rand() % 5000 + 1;
	my_data_1.position[2] = rand() % 5000 + 1;

	my_data_1.orientation[0] = rand() % 5000 + 1;
	my_data_1.orientation[1] = rand() % 5000 + 1;
	my_data_1.orientation[2] = rand() % 5000 + 1;
	my_data_1.orientation[3] = rand() % 5000 + 1;
	mymsg.poses[1] = my_data_1;

	printf("Message sent with %d poses\n", mymsg.n);

	for (int i = 0; i < mymsg.n; i++) {
		printf("ID %d - position (%d,%d,%d)\n", mymsg.poses[i].robotid, mymsg.poses[i].position[0], mymsg.poses[i].position[1], mymsg.poses[i].position[2]);
		printf("ID %d - orientation (%d,%d,%d,%d)\n", mymsg.poses[i].robotid, mymsg.poses[i].orientation[0], mymsg.poses[i].orientation[1], mymsg.poses[i].orientation[2], mymsg.poses[i].orientation[3]);
	}

	mymsg.timestamp = time(NULL);

	lcm.publish("EXAMPLE", &mymsg);

	return 0;
}
