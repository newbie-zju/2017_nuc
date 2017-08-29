#ifndef __NED_WORLD_BROADCASTER_H_
#define __NED_WORLD_BROADCASTER_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"
#include "dji_sdk/LocalPosition.h"

class NedWorldBroadcaster{
public:
    ros::NodeHandle nh_;
    ros::ServiceClient client;
    double origin_yaw; //-0.136
    double origin_x; //1
    double origin_y; //1
    boundary_detect::Boundary boundary;
    geometry_msgs::Point boundary_output;
    dji_sdk::LocalPosition localposition_rectified;
    ros::Subscriber boundary_sub;
    ros::Publisher localposition_rectified_pub;
    
    NedWorldBroadcaster(ros::NodeHandle nh);
    ~NedWorldBroadcaster();
    
    void boundaryoutputCallback(const geometry_msgs::PointConstPtr &msg);

};
#endif