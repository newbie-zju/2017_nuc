#ifndef IARCMISSION_H
#define IARCMISSION_H
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <irobot_detect/Pose3D.h>
#include <iostream>
#include <dji_sdk/LocalPosition.h>
//#include <goal_detected/Pose3D.h>
#include <multirobot_detect_iarc/RobotCamPos.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/dji_drone.h>
#include <iarc_mission/TG.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include "unistd.h"
#include <vector>
#include <iarc_tf/Velocity.h>
#include <geometry_msgs/PointStamped.h>
#include <eigen3/Eigen/Dense>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp> 
#include <boost/regex.hpp>
#include <boost/thread/mutex.hpp>

#include <stdio.h>                 
#include <unistd.h>               //C 和 C++ 程序设计语言中提供对 POSIX 操作系统 API 的访问功能的头文件的名称 
#include <fcntl.h>                //fcntl.h定义了很多宏和open,fcntl函数原型；unistd.h定义了更多的函数原型 
#include <termios.h>              //这是Linux 下串口驱动头文件;一般只能在Linux下。
#include <string>
#include <stdlib.h> 


enum MissionState_{CRUISE_P1,CRUISE_P2,CRUISE_P3,CRUISE_TARGET,CRUISE_GREENBDR,SEARCH_P1,SEARCH_P2,SEARCH_P3,WANDER_SEARCH,WANDER_PREDICT,TRACK_TARGET,TRACK_APPROACH,APPROACH_HEAD,APPROACH_TOP,TEST_OBS};//
enum TGState_{TG_CRUISE,TG_TRACK,TG_TRACK_APP,TG_WANDER,TG_APP_HEAD,TG_APP_TOP,TG_TEST};
enum VelState_{NED,GROUND};
enum Action_{A_HEADCOLLISION, A_TOPTOUCH, A_HOLD};

#define PI 3.1415926
using namespace std;
namespace mission{
class IARCMission
{
public:
	ros::NodeHandle nh_;
	ros::NodeHandle nh_param;
	ros::Subscriber irobot_pos_sub;	//target position from computer vision (package = goal_detected)
	ros::Subscriber dji_local_pos_sub;	//local position of DJI in NED fram (package = dji_sdk)
	ros::Subscriber flight_ctrl_dst_sub;
	ros::Subscriber obstacleAvoidance_sub;
	ros::Subscriber boundaryDetect_sub;
	ros::Subscriber quadrotorPosGround_sub;
	ros::Subscriber rc_channel_sub;
	ros::Subscriber glbvision_sub;
	ros::Subscriber quater_sub;
	ros::Subscriber boundary_sub;
	ros::ServiceClient TG_client;
	ros::ServiceClient tf_vel_client;
	struct irobotPosNED_
	{
		double x;
		double y;
		double z;
		double theta;
		int8_t flag;
	};
	struct irobotsPosNEDWithReward_
	{
		vector<double> x;
		vector<double> y;
		vector<double> z;
		vector<double> theta;
		int8_t flag;
		vector<double> reward;
	};
	irobotPosNED_ irobotPosNED;
	irobotsPosNEDWithReward_ irobotsPosNEDWithReward;
	dji_sdk::LocalPosition localPosNED;
	geometry_msgs::Point32 flight_ctrl_dst;
	geometry_msgs::Point quadrotorGroundPos;
	std_msgs::Int8 mission_state_msg;
	ros::Time free_time;
	ros::Time free_time_prev;
	ros::Duration freeTimer;
	
	DJIDrone *CDJIDrone;
	
	bool obstacleEmergency;
	bool boundaryEmergency;	
	bool boundaryClose;
	double yaw_origin,yaw_origin_rad,sta_x,sta_y,xMax,yMax;
	float yaw;
	int quadState;
	struct irobotPose_
	{
		float x;
		float y;
		float theta;
	};	
	IARCMission(ros::NodeHandle nh);
	~IARCMission();
	void initialize();
	void irobot_pos_callback(const irobot_detect::Pose3DConstPtr &msg);
	void dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr &msg);
	void obstacleAvoidance_callback(const std_msgs::Bool msg);
	void boundaryDetect_callback(const geometry_msgs::PointStampedConstPtr &msg);
	void quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg);
	void glb_vision_callback(const multirobot_detect_iarc::RobotCamPosConstPtr& msg);
	void dji_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg);
	void rcChannelCallback(const dji_sdk::RCChannelsConstPtr& msg);
	bool mission_takeoff();
	bool mission_land();
	void mission_armact();
	void mission_armreset();
	int stateMachine();
	bool irobotSafe(double theta);
	void missionFree();
	void missionCruise_P1();
	void missionCruise_P2();
	void missionCruise_P3();
	void missionCruise_TARGET();
	void missionCruise_GREEN();
	void missionSearch_P1();
	void missionSearch_P2();
	void missionSearch_P3();
	void missionWander();
	void missionWander_PRE();
	void missionTrack();
	void missionTrack_Approach();
	void missionApproach_HEAD();
	void missionApproach_TOP();
	void missionTest();
	bool gotoFree();
	bool gotoCruise();
	bool gotoTrack();
	bool gotoApproach();
	float getLength2f(float x, float y);
	irobotPose_ predictIrobotPose(irobotPose_ irobotpose, float TPred, float TInLoop);
	inline float limitAng(float theta);
	void irobotReward();
	
	
	vector<double> GlbVision_P1; 
	vector<double> GlbVision_P2;
	vector<double> GlbVision_P3;
	vector<double> GlbVision_Target;
	double maxWaitTime;
	bool ifGlbVisionOK;
	bool testInteractionFlag;
	bool testObsFlag;
	bool inInteraction;
	ros::Time time_now;
	ros::Time time_last;
	ros::Time time_topCount;
	ros::Time time_missionStart;
	ros::Duration duration;
	ros::Duration duration_mission;
	ros::Duration duration_topCount;
	float time_in_20s;
	
	bool glbVisTargetFlag;
	double glbChosenTargetPos[2];
	int glbTargetDetectedNum;
	int glbObsDetectedNum;
	double glbVisTargetX[5]; 
	double glbVisTargetY[5]; 
	double glbVisTargetZ[5];
	double glbVisTargetX_NED[5]; 
	double glbVisTargetY_NED[5]; 
	double glbVisTargetZ_NED[5];
	double glbVisTargetX_Gr[5]; 
	double glbVisTargetY_Gr[5]; 
	double glbVisTargetZ_Gr[5];
	double glbVisObsX[4]; 
	double glbVisObsY[4]; 
	double glbVisObsZ[4];
	vector<double> cruiseGreenSetPX;
	vector<double> cruiseGreenSetPY;
	int ActionType;
	
	bool readytoHeadCollision();
	bool readytoTopTouch();
	bool readytoTrackApproach();
	float time_reactAftHead;
	float time_reactAftTop;
	irobotPose_ posPre_forReact;
	irobotPose_ pos_Now;
	void glb_Body2Ground();
	float Quater[4];
	void Cruise_Ground2NED(double groundPosX, double groundPosY);
	
	//float NED2GrTheta, NED2GrXOrigin, NED2GrYOrigin;
	float cruiseSetPX_NED, cruiseSetPY_NED;
	int greenSetPointID;
	float pilotMode;
	
	float CruiseHeight;
	float ApproachHeight;
	int topTouchCount;
	
		//-----------------------------------
	
	//dji_sdk::Velocity velocity_body_msg;
	void dji_velocity_callback(const dji_sdk::Velocity::ConstPtr &msg);
	void tar_velocity_callback(const  std_msgs::Float32MultiArrayConstPtr &msg);
	ros::Subscriber velocity_sub;
	ros::Subscriber tar_velocity_sub;
	float dji_velocity_NED[3];
	std_msgs::Float32MultiArray obs_msg;
	ros::Publisher obstacle_pub;
	
 	float obtsacle_velocity[2];
	bool obstacle_emergency = false;
	float obstacle_dist;
	
};

};
#endif
