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
    mymsg.n = 5;
    mymsg.poses.resize(mymsg.n);
    for( int i=0; i < mymsg.n; i++)
    {
      poselcm::pose_t my_data;
      my_data.robotid = i;
      my_data.position[0] = 1;
      my_data.position[1] = 2;
      my_data.position[2] = 3;

      my_data.orientation[0] = 1;
      my_data.orientation[1] = 0;
      my_data.orientation[2] = 0;
      my_data.orientation[3] = 0;
      mymsg.poses[i] = my_data;
    }
    printf("Filled message with 5 poses\n");
    mymsg.timestamp = time( NULL );

    lcm.publish("EXAMPLE", &mymsg);

    return 0;
}
