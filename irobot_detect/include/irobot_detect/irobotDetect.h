//irobotDetect.h
#ifndef IROBOTDETECT_H_
#define IROBOTDETECT_H_

#include <iostream>
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include "shapeRecg.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <irobot_detect/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>

namespace irobot{
class IrobotDetect
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param;
    IrobotDetect(ros::NodeHandle nh);
    ~IrobotDetect();
    ros::Publisher goal_pose_pub;
    ros::Subscriber image_rect_sub;
    ros::Subscriber quadrotorPos_sub;
    ros::Subscriber quaternion_sub;
    
    void readIntrinsicParameter();
    void initialize();
    void Body_to_Global(double Body_arry[], float theta_angle);
    float irobot_angle(cv::Mat & img, std::vector<std::vector<cv::Point> > vertex, int irobot_num);
    void solveIrobotToDronePose(const std::vector<std::vector<cv::Point> > vertex,  double T_vec[], int irobot_num);
    void irobotFilter();
    void irobot_body_to_ned(double irobotstate[], const float quadrotorPos[]);
    void drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType);
    
    void quaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg);
    void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg);
    void image_rect_callback(const sensor_msgs::ImageConstPtr &msg);
    ShapeRecg irobotshape;

    std::string image_SubscribeName;
    
private:
    dji_sdk::LocalPosition quadrotorPos;
    int irobotNum;
    float quadrotorPosion[2];
    float Quater[4], Quater_last[4];
    double irobotState[4]; //irobot位置
    //Eigen::VectorXd irobotPoision(3); //存放目标的大地坐标系下的x,y增量和小车运动方向与x轴的变换
    float irobotImgAngle; //小车角度
    float Markerlength;
    float Markerwidth;  //小车尺寸
    float fx, fy, u0, v0;
};
}
#endif
