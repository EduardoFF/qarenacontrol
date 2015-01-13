#include <stdlib.h>
#include <time.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"

int main(int argc, char ** argv) {

	if (argc < 4) {
		printf("\nUse : ./fakelistener <robot id> <X> <Y> <LCM channel>\n\n");
	} else {

		lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");

		if (!lcm.good()) {
			printf("Error initializing LCM engine ...");
			return 1;
		}

		poselcm::pose_list_t mymsg;
		mymsg.n = 1;
		mymsg.poses.resize(mymsg.n);

		poselcm::pose_t my_data;
		my_data.robotid = atoi(argv[1]);

		my_data.position[0] = atoi(argv[2]);
		my_data.position[1] = atoi(argv[3]);
		my_data.position[2] = 0;

		my_data.orientation[0] = 1;
		my_data.orientation[1] = 0;
		my_data.orientation[2] = 0;
		my_data.orientation[3] = 0;

		my_data.velocity = 10;

		mymsg.poses[0] = my_data;

		mymsg.timestamp = time(NULL);

		lcm.publish(argv[4], &mymsg);

		return 0;
	}
}
