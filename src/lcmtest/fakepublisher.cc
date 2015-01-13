#include <stdlib.h>
#include <time.h>
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/lcmmsgs.hpp"

int main(int argc, char ** argv)
{
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
    if(!lcm.good())
        return 1;

    poselcm::pose_list_t mymsg;
    poselcm::pose_t my_data;
    mymsg.n = 1;
    mymsg.poses.resize(mymsg.n);
    my_data.robotid = atoi(argv[1]);
    my_data.position[0] = atoi(argv[2]);
    my_data.position[1] = atoi(argv[3]);
    my_data.position[2] = 0;

    my_data.orientation[0] = 0;
    my_data.orientation[1] = 0;
    my_data.orientation[2] = 0;
    my_data.orientation[3] = 0;
    mymsg.poses[0] = my_data;
    mymsg.timestamp = time( NULL );

    lcm.publish(argv[4], &mymsg);

    return 0;
}
