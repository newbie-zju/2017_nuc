#include <ros/ros.h>
#include "irobotDetect.h"
using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "irobot_detect_node");
	ros::NodeHandle nh;
	irobot::IrobotDetect irobot(nh);
	ros::spin();
	return 0;
}