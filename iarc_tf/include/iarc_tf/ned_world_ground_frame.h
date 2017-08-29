#ifndef __NED_WORLD_GROUND_FRAME_H_
#define __NED_WORLD_GROUND_FRAME_H_
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include "boundary_detect/Boundary.h"
#include "iarc_tf/NedWorldTransform.h"
#include "dji_sdk/LocalPosition.h"

namespace ned_world_ground{
class NedWorldGroundFrame{
public:
    ros::NodeHandle nh;
    ros::Subscriber local_position_sub;
    ros::Publisher ground_position_pub;
    dji_sdk::LocalPosition ground_local_position,ground_position;
    bool transformState;
    
    NedWorldGroundFrame();
    ~NedWorldGroundFrame();
    
    void positionCallback(const dji_sdk::LocalPositionConstPtr &msg);

};
};
#endif