#include "iarc_tf/ned_world_dynamic.h"
#include "math.h"
using namespace std;
enum VelState{NED,GROUND};
NedWorldDynamic::NedWorldDynamic():nh("~")
{
    sta_yaw = (-M_PI/2);
    //sta_yaw = -0.136;
    sta_x = 0.0;
    sta_y = 0.0;
	dyn_yaw = sta_yaw;
	dyn_x = sta_x;
	dyn_y = sta_y;
    dyn_local_position_sub = nh.subscribe("/dji_sdk/local_position",10,&NedWorldDynamic::localpositionCallback,this);
    dyn_boundary_output_sub = nh.subscribe("/boundary_output",10,&NedWorldDynamic::boundarydetectCallback,this);
    dyn_local_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion",10,&NedWorldDynamic::localquaternionCallback,this);
    
    //service = nh.advertiseService("ned_world_velocity_transform_srvice",&NedWorldDynamic::velocitytransformCallback,this);
    //ROS_INFO("Ready to transform velocity between NED frame and ground frame");
}

NedWorldDynamic::~NedWorldDynamic()
{
    ROS_INFO("destroying the ned_world_dynamic node ...");
}
void NedWorldDynamic::localquaternionCallback(const dji_sdk::AttitudeQuaternionConstPtr &msg)
{
	dyn_local_quaternion.q0 = msg->q0;
	dyn_local_quaternion.q1 = msg->q1;
	dyn_local_quaternion.q2 = msg->q2;
	dyn_local_quaternion.q3 = msg->q3;
	
// 	tf_quaternion.w()=dyn_local_quaternion.q0;
// 	tf_quaternion.x()=dyn_local_quaternion.q1;
// 	tf_quaternion.y()=dyn_local_quaternion.q2;
// 	tf_quaternion.z()=dyn_local_quaternion.q3;
}
void NedWorldDynamic::localpositionCallback(const dji_sdk::LocalPositionConstPtr& msg)
{
    //ROS_INFO("TEST..");
    dyn_local_position.x = msg->x;
    dyn_local_position.y = msg->y;
    dyn_local_position.z = msg->z;
	//cout << "dyn_local_position.y:" << dyn_local_position.y<<endl;
	
	static tf::TransformBroadcaster br;
    tf::Transform transformw2b;
	tf::Vector3 t_g_n = tf::Vector3(-dyn_local_position.x, -dyn_local_position.y,-dyn_local_position.z);
	transformw2b.setRotation(tf::Quaternion(dyn_local_quaternion.q1,dyn_local_quaternion.q2,dyn_local_quaternion.q3,dyn_local_quaternion.q0));
	tf::Vector3 t_n_g = transformw2b.getBasis().transpose() * t_g_n;
    transformw2b.setOrigin(t_n_g);
    tf::Quaternion quaternion;
    br.sendTransform(tf::StampedTransform(transformw2b,ros::Time::now(),"world","body"));
    
    //ROS_INFO_STREAM("dyn_local_position_x: "<<dyn_local_position.x);
    
}

void NedWorldDynamic::boundarydetectCallback(const geometry_msgs::PointConstPtr& msg)
{
    dyn_boundary_output.x = msg->x;
    dyn_boundary_output.y = msg->y;
    dyn_boundary_output.z = msg->z;
    //ROS_INFO_STREAM("dyn_boundary_x: "<<dyn_boundary_output.x);

    switch(int(dyn_boundary_output.z))
    {
	case 0:
	    //dyn_yaw = sta_yaw;
	    //dyn_x = sta_x;
	    //dyn_y = sta_y;
	    //ROS_INFO("NOEDGE");
	    break;
	case 1:
	    dyn_yaw = sta_yaw;
	    dyn_x = sta_x;
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*dyn_local_position.x-cos(dyn_yaw)*dyn_local_position.y;
	    //ROS_INFO("XEDGE");
	    break;
	case 2:
	    dyn_yaw = sta_yaw;
	    dyn_y = sta_y;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*dyn_local_position.y-cos(dyn_yaw)*dyn_local_position.x;
		cout << "dyn_x:" << dyn_x << endl;
	    ROS_INFO("YEDGE");
	    break;
	case 3:
	    dyn_yaw = sta_yaw;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*dyn_local_position.y-cos(dyn_yaw)*dyn_local_position.x;
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*dyn_local_position.x-cos(dyn_yaw)*dyn_local_position.y;
	    //ROS_INFO("XYEDGE");
	    break;
    }
    
    static tf::TransformBroadcaster br;
    tf::Transform transformn2g;
	tf::Vector3 t_n_g = tf::Vector3(-dyn_x, -dyn_y,0);
	transformn2g.setRotation(tf::createQuaternionFromYaw(dyn_yaw));
	tf::Vector3 t_g_n = transformn2g.getBasis().transpose() * t_n_g;
    transformn2g.setOrigin(t_g_n);
	/****
    static tf::TransformBroadcaster br;
    tf::Transform transformw2g;
    transformw2g.setOrigin(tf::Vector3(dyn_x, dyn_y,0.0));
    tf::Quaternion quaternion;
    transformw2g.setRotation(tf::createQuaternionFromYaw(dyn_yaw));
    **/
    br.sendTransform(tf::StampedTransform(transformn2g,ros::Time::now(),"world","ground"));
}
/*
bool NedWorldDynamic::velocitytransformCallback(iarc_tf::Velocity::Request& req, iarc_tf::Velocity::Response& res)
{
     //0, velocity in Ned frame
    switch(req.velocityFrame)
    {
	case NED: // velocity in NED frame
	    vel_target.x = cos(sta_yaw)*req.velocityX+sin(sta_yaw)*req.velocityX;
	    vel_target.y=-1.0*sin(sta_yaw)*req.velocityX+cos(sta_yaw)*req.velocityY;
	    ResState = 1.0;
	    vel_target.z = 0;
	    break;
	case GROUND: //velocity in ground frame
	    vel_target.x = cos(sta_yaw)*req.velocityX-sin(sta_yaw)*req.velocityX;
	    vel_target.y = sin(sta_yaw)*req.velocityX+cos(sta_yaw)*req.velocityY;
	    vel_target.z = 0;
	    ResState = 0.0;
	    break;	    
    }
    
    res.velocityFrameRes = ResState;
    res.velocityXRes = vel_target.x;
    res.velocityYRes = vel_target.y;
    return true;
}
*/


