#include <ros/ros.h>
#include <goal_detected/Pose3D.h>
#include <dji_sdk/LocalPosition.h>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;
dji_sdk::LocalPosition localPosNED;
void dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr& msg)
{
	localPosNED.x = msg->x;
	localPosNED.y = msg->y;
	localPosNED.z = msg->z;
}
float getlength(float x, float y)
{
	return sqrt(x*x+y*y);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_goal_pose_node");
	ros::NodeHandle nh;
	ros::Subscriber dji_local_pos_sub = nh.subscribe("/dji_sdk/local_position", 10, dji_local_pos_callback);
	ros::Publisher goal_pose_pub = nh.advertise<goal_detected::Pose3D>("/goal_detected/goal_pose",10);
	goal_detected::Pose3D pose;
	ros::Time time_now = ros::Time::now();
	ros::Time time_prev = time_now;
	ros::Duration delta_time;
	pose.flag = 1;
	pose.theta = 0;
	pose.x = 0.0;
	pose.y = 4.0;
	pose.z = 0.0;
	float vIrobot = 0.333;
	localPosNED.x = 0.0;
	localPosNED.y = 0.0;
	localPosNED.z = 0.0;
	cv::Mat src(30, 30, CV_8UC1);
	cv::namedWindow("keyboardReader");
	
	while(ros::ok())
	{/*
		while(ros::ok() && pose.theta < 1.5)
		{
			ros::spinOnce();
			cv::imshow("keyboardReader",src);
			time_now = ros::Time::now();
			delta_time = time_now - time_prev;
			time_prev = time_now;
			pose.x = pose.x + vIrobot * delta_time.toSec();
			if((pose.x > 5.0)||(27 == (char)cv::waitKey(30))){pose.theta = M_PI;vIrobot = -vIrobot;}
			usleep(20000);
			if(getlength(pose.x-localPosNED.x,pose.y-localPosNED.y)<3.0)pose.flag = 1;
			else pose.flag = 0;
			goal_pose_pub.publish(pose);
			ROS_INFO("flag=%d, theta=%3.1lf, x=%3.1lf",pose.flag, pose.theta, pose.x);
			//if(27 == c)pose.theta = -pose.theta;
		}
		while(ros::ok() && pose.theta > 1.5)
		{
			ros::spinOnce();
			cv::imshow("keyboardReader",src);
			time_now = ros::Time::now();
			delta_time = time_now - time_prev;
			time_prev = time_now;
			pose.x = pose.x + vIrobot * delta_time.toSec();
			if((pose.x < 0.0)||(27 == (char)cv::waitKey(30))){pose.theta = 0;vIrobot = -vIrobot;}
			usleep(20000);
			if(getlength(pose.x-localPosNED.x,pose.y-localPosNED.y)<3.0)pose.flag = 1;
			else pose.flag = 0;
			goal_pose_pub.publish(pose);
			ROS_INFO("flag=%d, theta=%3.1lf, x=%3.1lf",pose.flag, pose.theta, pose.x);
			//if(27 == c)pose.theta = -pose.theta;
		}
*/
		pose.flag = 1;
		pose.theta = M_PI/2.0;
		pose.x = 2;
		pose.y = 4;
		pose.z = 0;
		goal_pose_pub.publish(pose);
	}
	return 0;
}
