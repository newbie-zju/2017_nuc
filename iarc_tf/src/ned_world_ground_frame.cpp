#include "iarc_tf/ned_world_ground_frame.h"
using namespace std;

ned_world_ground::NedWorldGroundFrame::NedWorldGroundFrame():nh("~")
{
    local_position_sub = nh.subscribe("/dji_sdk/local_position",10,&NedWorldGroundFrame::positionCallback,this);
    ground_position_pub = nh.advertise<dji_sdk::LocalPosition>("/ground_position",10);
}

ned_world_ground::NedWorldGroundFrame::~NedWorldGroundFrame()
{
    ROS_INFO("destroying ground_position_node ...");
}

void ned_world_ground::NedWorldGroundFrame::positionCallback(const dji_sdk::LocalPositionConstPtr& msg)
{
    ground_local_position.x = msg->x;
    ground_local_position.y = msg->y;
    ground_local_position.z = msg->z;
    ground_local_position.header = msg->header;
    ground_local_position.ts = msg->ts;
    
    tf::TransformListener listener;
    //ros::Rate rate(20);
    
    transformState =listener.waitForTransform("/ground","/body",ros::Time(0),ros::Duration(10.0));
    while(transformState){
	tf::StampedTransform transform;
	try{
	    listener.lookupTransform("/ground","/body",ros::Time(0),transform);
	    ground_position.x = transform.getOrigin().x();
	    ground_position.y = transform.getOrigin().y();
	    ground_position.z = ground_local_position.z;
	    ground_position.header = ground_local_position.header;
	    ground_position.ts = ground_local_position.ts;
	    
	    ground_position_pub.publish(ground_position);
	}
	catch(tf::TransformException &ex){
	    ROS_ERROR("%s",ex.what());
	}
    }
    
}




