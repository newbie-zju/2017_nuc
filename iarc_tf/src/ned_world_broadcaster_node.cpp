#include <iarc_tf/ned_world_broadcaster.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "ned_world_broadcaster_node");
    ros::NodeHandle nh;
    NedWorldBroadcaster nedworldbroadcaster(nh);
    ros::spin();
    return 0;
}
    