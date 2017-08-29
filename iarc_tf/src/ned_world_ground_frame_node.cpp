#include "iarc_tf/ned_world_ground_frame.h"

using namespace std;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ground_position_node");
	ros::NodeHandle nh;
	ned_world_ground::NedWorldGroundFrame nedworldgroundframe;
	return 0;
}