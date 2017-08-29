#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sys/socket.h>
#include <iarc_tf/Velocity.h>
#include <math.h>

/*****velocity converion between NED and ground *******/
/*
 *  
   |Vxw|   |cos(yaw)  sin(yaw)| |Vxn|
   |   | = |		      | |   |
   |Vyw|   |-sin(yaw) cos(yaw)|	|Vyn|
   
   |Vxn|   |cos(yaw) -sin(yaw)| |Vxw|
   |   | = |		      | |   |
   |Vyn|   |sin(yaw)  cos(yaw)|	|Vyw|
**/
enum VelState{NED,GROUND};
//geometry_msgs::Vector3 vel_origin;
geometry_msgs::Vector3 vel_target;
double ResState;
double offset_yaw;
bool velocitytransformCallback(iarc_tf::Velocity::Request &req, iarc_tf::Velocity::Response &res)
{
    //0, velocity in Ned frame
    switch(req.velocityFrame)
    {
	case NED: // velocity in NED frame
	    vel_target.x = cos(offset_yaw)*req.velocityX+sin(offset_yaw)*req.velocityY;
	    vel_target.y = -1.0*sin(offset_yaw)*req.velocityX+cos(offset_yaw)*req.velocityY;
	    ResState = 1.0;
	    vel_target.z = 0;
	    break;
	case GROUND: //velocity in ground frame
	    vel_target.x = cos(offset_yaw)*req.velocityX-sin(offset_yaw)*req.velocityY;
	    vel_target.y = sin(offset_yaw)*req.velocityX+cos(offset_yaw)*req.velocityY;
	    vel_target.z = 0;
	    ResState = 0.0;
	    break;	    
    }
    
    res.velocityFrameRes = ResState;
    res.velocityXRes = vel_target.x;
    res.velocityYRes = vel_target.y;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ned_world_velocity_service");
    ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
    if(!nh_param.getParam("yaw_origin",offset_yaw))offset_yaw = 0.0;
    //string service_name = "ned_world_velocity";    
    ros::ServiceServer service = nh.advertiseService("ned_world_velocity_transform_srvice",velocitytransformCallback);
    ROS_INFO("Ready to transform velocity between NED frame and ground frame");
    //ros::Subscriber sub = nh.subscribe("/dji_sdk/local_position",10,&poseCallback);
    
    ros::spin();
    return 0;
  
}
