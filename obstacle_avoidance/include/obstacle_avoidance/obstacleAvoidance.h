// obstacleAvoidance.h
#ifndef OBSTACLEAVOIDANCE_H_
#define OBSTACLEAVOIDANCE_H_

#include "LIDAR.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <obstacle_avoidance/velocityPosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <dji_sdk/dji_drone.h>
#include <dji_sdk/LocalPosition.h>

namespace iarcRplidar
{
class ObstacleAvoidance 
{
public:
    ros::NodeHandle nh_param;
    ros::NodeHandle nh_;

    ros::Publisher OA_vel_visualize_pub;
    ros::Publisher OA_tar_velocity_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber velocity_position_sub;
    ros::Subscriber ned_drone_position_sub;
    
    ObstacleAvoidance(ros::NodeHandle nh);
    ~ObstacleAvoidance();
    
    void initialize();
    void LaserScanCallback(const sensor_msgs::LaserScanConstPtr& msg);
    void VelocityPositionCallback(const std_msgs::Float32MultiArrayPtr& msg);
    void droneLocalPositionCallback(const dji_sdk::LocalPositionConstPtr& msg);
    void Body_to_Ned(const float body_polar_v[], float ned_v[]);
    void Ned_to_Body(const float ned_v[], float body_polar_v[], const float ned_p[], float body_polar_p[]);
    obstacle_avoidance::velocityPosition velocityPosition_msg;

    LIDAR rplidara2;
    DJIDrone *CDJIDrone;
    float body_velocity[3] = {0};
    float ned_velocity[3] = {0};
    float body_tar_position[3] = {0};
    float ned_tar_position[3] = {0};
    float body_polar_velocity[3] = {0};
    dji_sdk::LocalPosition dronePostion;
    float dronePosition[3];
private:
    float mode_flag;
    float Quater_last[4] = {0};
    float quater[4];
    std::vector<float> curVelocity;
    std::vector<float> curPosition;
};
}
#endif   
    