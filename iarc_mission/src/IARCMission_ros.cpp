#include <ros/ros.h>
#include "IARCMission.h"
using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "iarc_mission_node");
	ros::NodeHandle nh;
	mission::IARCMission CIARCMission(nh);
	ros::spin();
	return 0;
}