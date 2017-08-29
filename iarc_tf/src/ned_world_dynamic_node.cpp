#include "iarc_tf/ned_world_dynamic.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ned_world_dynamic_node");
    NedWorldDynamic nedworlddynamic;
    //ROS_INFO("fuck...");
    ros::spin();
    return 0;
}