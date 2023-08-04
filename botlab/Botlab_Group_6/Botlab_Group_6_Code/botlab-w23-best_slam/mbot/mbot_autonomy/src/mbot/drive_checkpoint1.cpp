#include <common_utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>

int main(int argc, char** argv)
{
    int numTimes = 1;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    mbot_lcm_msgs::robot_path_t path;
    path.path.resize(numTimes * 10);
    
    mbot_lcm_msgs::pose_xyt_t nextPose;
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n] = nextPose;
    }
    

    nextPose.x = 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n+1] = nextPose;
    }
    
    nextPose.x = 0.61f;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 2] = nextPose;
    }
    
    nextPose.x = 2 * 0.61f;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 3] = nextPose;
    }
    
    nextPose.x = 2 * 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 4] = nextPose;
    }
    
    nextPose.x = 3 * 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 5] = nextPose;
    }

    nextPose.x = 3 * 0.61f;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 6] = nextPose;
    }

    nextPose.x = 4 * 0.61f;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 7] = nextPose;
    }

    nextPose.x = 4 * 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 8] = nextPose;
    }

    nextPose.x = 5 * 0.61f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 9] = nextPose;
    }

    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
