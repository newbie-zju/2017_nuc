#ifndef __NED_WORLD_VELOCITY_TRANSFORM_CLIENT_H_
#define __NED_WORLD_VELOCITY_TRANSFORM_CLIENT_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include "iarc_tf/Velocity.h"

#include <string>
#include <vector>
using namespace std;
enum VelState{NED,GROUND};

namespace ned_world_velocity_transform_client{
class NedWorldVelocityTransformClient{
public:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    ros::Subscriber velocity_sub;
    ros::Publisher velocity_pub;
    string velocity_topic;
    geometry_msgs::Vector3 velocity_origin;
    geometry_msgs::Vector3 velocity_target;
    
    NedWorldVelocityTransformClient();
    ~NedWorldVelocityTransformClient();
    
    void velocityCallback(const geometry_msgs::Vector3ConstPtr &msg);

};
};
#endif