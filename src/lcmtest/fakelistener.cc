#include <stdio.h>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const poselcm::pose_list_t* msg)
        {
            int i;
            printf("Received message on channel \"%s\":\n", chan.c_str());
            printf("  timestamp   = %lld\n", (long long)msg->timestamp);
            printf("  poses:\n");
            for(i = 0; i < msg->n; i++)
				{
				  const poselcm::pose_t &pose = msg->poses[i];
				  printf("  robotid = %d\n", pose.robotid);
				  printf("  position    = (%d, %d, %d)\n",
					 pose.position[0], pose.position[1], pose.position[2]);
				  printf("  orientation = (%d, %d, %d, %d)\n",
					 pose.orientation[0], pose.orientation[1],
					 pose.orientation[2], pose.orientation[3]);
				  printf(" velocity = %d\n", pose.velocity);

				}
					printf("\n");
				}
};

int main(int argc, char** argv)
{
    lcm::LCM lcm ("udpm://239.255.76.67:7667?ttl=1");
	//lcm::LCM lcm;

    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
