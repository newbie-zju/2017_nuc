#include "iarc_tf/ned_world_velocity_transform_client.h"

ned_world_velocity_transform_client::NedWorldVelocityTransformClient::NedWorldVelocityTransformClient():nh("~")
{
    
    nh.param("velocity_topic",velocity_topic,string("/velocity"));
    velocity_sub = nh.subscribe(velocity_topic.c_str(),10,&NedWorldVelocityTransformClient::velocityCallback,this);
    velocity_pub = nh.advertise<geometry_msgs::Vector3>("/cruise_velocity/transformed",10);
    
    while(!ros::service::waitForService("velocity_transform",ros::Duration(3.0)))
    {
	ROS_INFO("Waiting for service velocity_transform to become available");
    }
    ROS_INFO("Requseting the transform ...");
    
}

ned_world_velocity_transform_client::NedWorldVelocityTransformClient::~NedWorldVelocityTransformClient()
{
    ROS_INFO("Destroying the velocity_transform_client");
}

void ned_world_velocity_transform_client::NedWorldVelocityTransformClient::velocityCallback(const geometry_msgs::Vector3ConstPtr& msg)
{
    velocity_origin.x = msg->x;
    velocity_origin.y = msg->y;
    velocity_origin.z = msg->z;
    
    ros::ServiceClient client = nh.serviceClient<iarc_tf::Velocity>("ned_world_velocity_transform_client");
    iarc_tf::Velocity srv;
    
    //TODO:velocity_origin.z only be 0.0 or 1.0, 0:ned frame; 1:ground frame
    if(velocity_origin.z < 0.5)
    {
	srv.request.velocityFrame = NED;
	srv.request.velocityX = velocity_origin.x;
	srv.request.velocityY = velocity_origin.y;
	if(!client.call(srv))
	    ROS_INFO("velocity transform from NED to ground call failed...");
	else
	{
	    velocity_target.x = srv.response.velocityXRes;
	    velocity_target.y = srv.response.velocityYRes;
	    velocity_target.z = srv.response.velocityFrameRes;
	}
    }
    else
    {
	srv.request.velocityFrame = GROUND;
	srv.request.velocityX = velocity_origin.x;
	srv.request.velocityY = velocity_origin.y;
	if(!client.call(srv))
	    ROS_INFO("velcocity transform from ground to NED failed...");
	else
	{
	    velocity_target.x = srv.response.velocityXRes;
	    velocity_target.y = srv.response.velocityYRes;
	    velocity_target.z = srv.response.velocityFrameRes;
	}
    }
    
    velocity_pub.publish(velocity_target);
}

