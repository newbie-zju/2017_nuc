#ifndef __NED_WORLD_BROADCASTER_H_
#define __NED_WORLD_BROADCASTER_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"
#include "dji_sdk/LocalPosition.h"
#include "dji_sdk/AttitudeQuaternion.h"
#include "iarc_tf/Boundary.h"
#include "iarc_tf/Velocity.h"
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
using namespace std;
class NedWorldDynamic
{
public:
    ros::NodeHandle nh;
    ros::Subscriber dyn_local_position_sub;
    ros::Subscriber dyn_boundary_output_sub;
    ros::Subscriber dyn_local_quaternion_sub;
	ros::Publisher ground_position_pub;
	geometry_msgs::PointStamped ground_position;
    ros::ServiceServer service;
    
    string frame_name;
    geometry_msgs::Point dyn_boundary_output;
    geometry_msgs::Vector3 vel_target;
    double ResState;
    dji_sdk::LocalPosition dyn_local_position;
	dji_sdk::AttitudeQuaternion dyn_local_quaternion;
	//tf::Quaternion dyn_local_quaternion;
    double sta_yaw;
    double sta_x, sta_y;
    double dyn_yaw;
    double dyn_x, dyn_y;
	Quaterniond q;
	Matrix4d c_b2n_, c_n2g_;
	Matrix3d r_b2n, r_n2g;
	Vector3d t_b2n, t_n2g;
	Vector3d t_n2g_init;
    
    NedWorldDynamic();
    ~NedWorldDynamic();
    void boundarydetectCallback(const geometry_msgs::PointStampedConstPtr &msg);
    void localpositionCallback(const dji_sdk::LocalPositionConstPtr &msg);
	void localquaternionCallback(const dji_sdk::AttitudeQuaternionConstPtr &msg);
    bool velocitytransformCallback(iarc_tf::Velocity::Request &req, iarc_tf::Velocity::Response &res);
	Matrix3d quaternion2mat(Quaterniond q);
	Matrix3d yaw2mat(double yaw);
	Matrix4d c_n2g(double yaw, Vector3d t_n2g);
	Matrix4d c_b2n(Quaterniond q, Vector3d t_b2n);

	
};



#endif
