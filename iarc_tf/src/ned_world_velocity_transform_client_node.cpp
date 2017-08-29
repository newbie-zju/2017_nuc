#include "iarc_tf/ned_world_velocity_transform_client.h"

using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "velocity_transform_client_node");
	ros::NodeHandle nh;
	ned_world_velocity_transform_client::NedWorldVelocityTransformClient nedworldvelocitytransformclient;
	return 0;
}