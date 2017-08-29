#include "ros/ros.h"
#include "obstacleAvoidance.h"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::NodeHandle nh;
    iarcRplidar::ObstacleAvoidance irobot(nh);
    ros::spin();
    return 0;
}