#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include "Dptouching.h"
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include "iarc_mission/TG.h"
#include <iarc_tf/Velocity.h>
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Dptouching_node");
	ros::NodeHandle nh_;
	DpTouching CDpTouching(nh_);
	ros::spin();
	return 0;
}
