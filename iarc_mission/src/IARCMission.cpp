#include <IARCMission.h>
#include <math.h>
#include "../include/iarc_mission/IARCMission.h"
#include <ros/init.h>
using namespace std;
namespace mission{
/*
VERTICAL_VELOCITY = 0x00,
VERTICAL_POSITION = 0x10,
VERTICAL_THRUST = 0x20,
HORIZONTAL_ANGLE = 0x00,
HORIZONTAL_VELOCITY = 0x40,
HORIZONTAL_POSITION = 0X80
*/

IARCMission::IARCMission(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
	initialize();
	CDJIDrone->request_sdk_permission_control();
	sleep(2);
	mission_armreset();
	mission_takeoff();
	
	
	ros::Publisher missionState_pub = nh.advertise<std_msgs::Int8>("missionState", 10);
	std_msgs::Int8 stateMsg;
	
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		//CDJIDrone->request_sdk_permission_control();
		switch(quadState)
		{
			case CRUISE_P1: 
			{
				missionCruise_P1();
				break;				
			}
			case CRUISE_P2: 
			{
				missionCruise_P2();
				break;				
			}
			case CRUISE_P3: 
			{
				missionCruise_P3();
				break;				
			}
			case CRUISE_TARGET: 
			{
				missionCruise_TARGET();
				break;				
			}
			case SEARCH_P1: 
			{
				missionSearch_P1();
				break;				
			}	
			case SEARCH_P2: 
			{
				missionSearch_P2();
				break;				
			}
			case SEARCH_P3: 
			{
				missionSearch_P3();
				break;				
			}
			case CRUISE_GREENBDR:
			{
				missionCruise_GREEN();
				break;
			}
			case WANDER_SEARCH:
			{
				missionWander();
			}
			case TRACK_TARGET:
			{
				missionTrack();
				break;
			}
			case TRACK_APPROACH:
			{
				missionTrack_Approach();
				break;
			}
			case APPROACH_HEAD:
			{
				missionApproach_HEAD();
				break;
			}
			case APPROACH_TOP:
			{
				missionApproach_TOP();
				break;
			}
			case WANDER_PREDICT:
			{
				missionWander_PRE();
				break;
			}
			case TEST_OBS:
			{
				missionTest();
				break;
			}
		}
		stateMsg.data = quadState;
		missionState_pub.publish(stateMsg);
		loop_rate.sleep();
	}

}

IARCMission::~IARCMission()
{
	ROS_INFO("Destroying IARCMission......");
}


void IARCMission::initialize()
{
	/*
	try {
		ifstream is("/home/zmart/2017/src/iarc_mission/parameter.xml");
		// read parameters from xml file
		using boost::property_tree::ptree;
		ptree pt;
		if (!is.is_open()) 
		{
			ROS_FATAL("xml config not found"); 
			return;
		}
		read_xml(is, pt);

		// get vector of desiifGlbVisionOKred heights	
		
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("glbvision_p1"))
			GlbVision_P1.push_back(v.second.get<double>(""));
			
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("glbvision_p2"))
			GlbVision_P2.push_back(v.second.get<double>(""));	
		
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("glbvision_p3"))
			GlbVision_P3.push_back(v.second.get<double>(""));	
		
		// initialize Joint State variable
		maxWaitTime = pt.get<double>("maxwaitTime");
		testInteractionFlag = pt.get<bool>("testInteractionOnly");
		testObsFlag = pt.get<bool>("testObsFlag");
		time_reactAftHead = pt.get<float>("timeReactAftHead");
		time_reactAftTop = pt.get<float>("timeReactAftTop");
		NED2GrTheta = pt.get<float>("NED2GrTheta");
		NED2GrXOrigin = pt.get<float>("NED2GrXOrigin");
		NED2GrYOrigin = pt.get<float>("NED2GrYOrigin");
		
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("cruiseGreenSetPX"))
			cruiseGreenSetPX.push_back(v.second.get<double>(""));
			
		BOOST_FOREACH(ptree::value_type &v,	pt.get_child("cruiseGreenSetPY"))
			cruiseGreenSetPY.push_back(v.second.get<double>(""));
	} 
	catch (int e) 
	{
		cout << "reading xml file failed" << e << endl;
	}
	*/
	
	CruiseHeight = 1.6;
	ApproachHeight = 0.8;
	
	GlbVision_P1 = {10,10,CruiseHeight};
	GlbVision_P2 = {8,7,CruiseHeight};
	GlbVision_P3 = {8,13,CruiseHeight};

	maxWaitTime = 10;
 	testInteractionFlag = true;
	testObsFlag = false;
	time_reactAftHead = 10;
	time_reactAftTop = 3;
	//NED2GrTheta = M_PI;
	//NED2GrXOrigin = 0;
	//NED2GrYOrigin = 0;
	
	cruiseGreenSetPX = {5,5,5,5,5,13,13,13,13,13};
	cruiseGreenSetPY = {10,7,13,4,16,10,7,13,4,16};
  
	double yaw_origin_;
	if(!nh_param.getParam("yaw_origin",yaw_origin_))yaw_origin_ = 0.0;
	if(!nh_param.getParam("sta_x",sta_x))sta_x = 0.0;
	if(!nh_param.getParam("sta_y",sta_y))sta_y = 0.0;
	if(!nh_param.getParam("xMax",xMax))xMax = 0.0;
	if(!nh_param.getParam("yMax",yMax))yMax = 0.0;
 	
	ROS_INFO("initialize yaw = %3.1f, sta_x = %3.1f,sta_y = %3.1f,xMax = %3.1f,yMax = %3.1f",yaw_origin_,sta_x,sta_y,xMax,yMax);
	yaw_origin_rad = yaw_origin_;
	yaw_origin = (float)yaw_origin_*180.0/M_PI;
	boundaryEmergency = false;
	boundaryClose = false;
	obstacleEmergency = false;
	free_time = ros::Time::now();
	free_time_prev = free_time;
	irobotPosNED.x = 0.0;
	irobotPosNED.y = 0.0;
	irobotPosNED.z = 0.0;
	irobotPosNED.theta = 0.0;
	irobotPosNED.flag = 0;
	irobotsPosNEDWithReward.x.clear();
	irobotsPosNEDWithReward.y.clear();
	irobotsPosNEDWithReward.z.clear();
	irobotsPosNEDWithReward.theta.clear();
	irobotsPosNEDWithReward.flag = 0;
	irobotsPosNEDWithReward.reward.clear();
	irobot_pos_sub = nh_.subscribe("/goal_detected/goal_pose", 10, &IARCMission::irobot_pos_callback, this);
	dji_local_pos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IARCMission::dji_local_pos_callback, this);
	quadrotorPosGround_sub = nh_.subscribe("/ground_position",10, &IARCMission::quadrotorPosGroundCallback,this);
	glbvision_sub = nh_.subscribe("/robot_cam_position",10, &IARCMission::glb_vision_callback,this);
	rc_channel_sub = nh_.subscribe("/dji_sdk/rc_channels",10, &IARCMission::rcChannelCallback,this);
	quater_sub = nh_.subscribe("/dji_sdk/attitude_quaternion",10, &IARCMission::dji_quaternion_callback,this);
	boundary_sub = nh_.subscribe("/boundary_output",10, &IARCMission::boundaryDetect_callback,this);
	velocity_sub = nh_.subscribe("/dji_sdk/velocity", 10, &IARCMission::dji_velocity_callback, this);
	obstacle_pub = nh_.advertise<std_msgs::Float32MultiArray>("/obstacle/velocityPosition", 1);
	tar_velocity_sub = nh_.subscribe("/obstacleAvoidance/target_velocity", 1, &IARCMission::tar_velocity_callback, this);
	TG_client = nh_.serviceClient<iarc_mission::TG>("/TG/TG_service");
	tf_vel_client = nh_.serviceClient<iarc_tf::Velocity>("ned_world_velocity_transform_srvice");
	CDJIDrone = new DJIDrone(nh_);
	
	ROS_ERROR("Start global timer, please WAIT for irobot turn and then press any key to continue...");
	getchar();
	time_missionStart = ros::Time::now();
	time_last = ros::Time::now();
	time_now = ros::Time::now();
	duration = time_now - time_last;
	duration_mission = time_now - time_missionStart;
	
	if (testInteractionFlag == false)
	{
		quadState = CRUISE_P1;
		ifGlbVisionOK = true;
	}
	else
	{
		quadState = WANDER_SEARCH;
		ifGlbVisionOK = false;
	}
	
	if (testObsFlag)
	{
		quadState = TEST_OBS;
		ifGlbVisionOK = false;
	}
		
	inInteraction = false;
	ActionType = A_HOLD;
	glbVisTargetFlag = false;
	greenSetPointID = 0;
	topTouchCount = 0;

}

//===========================callbacks==================================================
void IARCMission::irobot_pos_callback(const irobot_detect::Pose3DConstPtr& msg)
{
	irobotsPosNEDWithReward.x.clear();
	irobotsPosNEDWithReward.y.clear();
	irobotsPosNEDWithReward.z.clear();
	irobotsPosNEDWithReward.theta.clear();
	irobotsPosNEDWithReward.flag = 0;
	irobotsPosNEDWithReward.reward.clear();
	irobotPosNED.x = 0.0;
	irobotPosNED.y = 0.0;
	irobotPosNED.z = 0.0;
	irobotPosNED.flag = 0;
	irobotPosNED.theta = 0.0;
	
	float distance;
	int NumCrowded = 0;
	bool CrowdFlag = false;
	
	if(msg->flag > 0)
	{
		irobotsPosNEDWithReward.flag = msg->flag;
		irobotPosNED.flag = msg->flag;
		for(int i = 0;i != msg->flag;i++)
		{
			irobotsPosNEDWithReward.x.push_back(msg->x[i]);
			irobotsPosNEDWithReward.y.push_back(msg->y[i]);
			irobotsPosNEDWithReward.z.push_back(msg->z[i]);
			irobotsPosNEDWithReward.theta.push_back(msg->theta[i]);
			irobotsPosNEDWithReward.reward.push_back(999.9);
			
			distance = getLength2f(irobotsPosNEDWithReward.x[i]-localPosNED.x,irobotsPosNEDWithReward.y[i]-localPosNED.y);
			if (distance < 2)
				NumCrowded++;
		}
		irobotReward();		//calculate reward of each irobot
		float minReward = 999.9;
		int minRewardIndex = 999;
		for(int i = 0;i != irobotsPosNEDWithReward.flag;i++)
		{
			if(irobotsPosNEDWithReward.reward[i] < minReward)	//find the irobot with the smallest reward, which meas forward to red line, and should to be track & approach
			{
				minReward = irobotsPosNEDWithReward.reward[i];
				minRewardIndex = i;
			}
		}
		irobotPosNED.x = irobotsPosNEDWithReward.x[minRewardIndex];
		irobotPosNED.y = irobotsPosNEDWithReward.y[minRewardIndex];
		irobotPosNED.z = irobotsPosNEDWithReward.z[minRewardIndex];
		irobotPosNED.theta = irobotsPosNEDWithReward.theta[minRewardIndex];
		
		if (NumCrowded > 1)
		{
			CrowdFlag = true;
			ROS_ERROR("TOO crowded!!!!!");
		}
		
		
		ROS_ERROR_THROTTLE(0.2,"selected irobotPosNED=(%4.2f,%4.2f,%4.2f),flag=%d",irobotPosNED.x,irobotPosNED.y,irobotPosNED.theta,irobotPosNED.flag);
		
		
	}
}

void IARCMission::rcChannelCallback(const dji_sdk::RCChannelsConstPtr& msg)
{
	pilotMode = msg->mode;
}

void IARCMission::glb_vision_callback(const multirobot_detect_iarc::RobotCamPosConstPtr& msg)
{
	/*
	if (ifGlbVisionOK == false)
		return;
	*/
	glbVisTargetFlag = false;
	glbTargetDetectedNum = 0;
	glbObsDetectedNum = 0;
	
	glbChosenTargetPos[0] = 0;
	glbChosenTargetPos[1] = 0;
	for(int i=0; i < 5; i++)
	{
		glbVisTargetX[i] = 0;
		glbVisTargetY[i] = 0;
		glbVisTargetX_NED[i] = 0;
		glbVisTargetY_NED[i] = 0;
		glbVisTargetZ_NED[i] = 0;
		glbVisTargetX_Gr[i] = 0;
		glbVisTargetY_Gr[i] = 0;
		glbVisTargetZ_Gr[i] = 0;
	}
	
	for(int i=0; i < 5; i++)
	{
		glbVisObsX[i] = 0;
		glbVisObsY[i] = 0;
	}
	
	if (msg->exist_rob_flag == true)
	{
		 glbVisTargetFlag = msg->exist_rob_flag;
		 glbTargetDetectedNum = msg->rob_num;
		 glbObsDetectedNum = msg->obs_num;
		 for(int i=0; i < glbTargetDetectedNum; i++)
		 {
			glbVisTargetX[i] = msg->rob_cam_pos_x[i];
			glbVisTargetY[i] = msg->rob_cam_pos_y[i];
			glbVisTargetZ[i] = CruiseHeight;
		 }
		 
		 //ROS_INFO("1. x=%3.1f,y=%3.1f",glbVisTargetX[1],glbVisTargetY[1]);
		 
		 /*
		 for(int i=0; i < glbObsDetectedNum; i++)
		 {
			glbVisObsX[i] = msg->obs_cam_pos_x[i];
			glbVisObsY[i] = msg->obs_cam_pos_y[i]; 
			glbVisObsZ[i] = CruiseHeight;
		 }
		 */
		 glb_Body2Ground();
		 //ROS_INFO("2. x=%3.1f,y=%3.1f",glbVisTargetX_NED[1],glbVisTargetY_NED[1]);
		 
		 int chosenTargetID = 0;
		 for (int i=0; i < glbTargetDetectedNum; i++)
		 {
			if (glbVisTargetX_Gr[i] < 0)
				glbVisTargetX_Gr[i] = xMax;
			if (glbVisTargetX_Gr[i] < glbVisTargetX_Gr[chosenTargetID])
				chosenTargetID = i;
		 }
		 glbChosenTargetPos[0] = glbVisTargetX_NED[chosenTargetID];
		 glbChosenTargetPos[1] = glbVisTargetY_NED[chosenTargetID];
		 
		 ROS_INFO_THROTTLE(1,"3. sx=%3.1f,y=%3.1f",glbChosenTargetPos[0],glbChosenTargetPos[1]);
		 
	}
}

void IARCMission::dji_local_pos_callback(const dji_sdk::LocalPositionConstPtr& msg)
{
	localPosNED.x = msg->x;
	localPosNED.y = msg->y;
	localPosNED.z = msg->z;
}

void IARCMission::boundaryDetect_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
	if(msg->point.z > 0)
	{
		boundaryEmergency = true;
		//ROS_ERROR("Vision boundary detected!!");
		
		if (quadrotorGroundPos.x > xMax-1.5 || quadrotorGroundPos.x < 1.5 || quadrotorGroundPos.y > yMax-1.5 || quadrotorGroundPos.y < 1.5)
		{
			boundaryClose = true;
			ROS_ERROR("DANGER!! BOUNDARY CLOSE!!");
		}	
		else
			boundaryClose = false;
	}
	else
	{
		boundaryEmergency = false;
		boundaryClose = false;
	}
}

void IARCMission::obstacleAvoidance_callback(const std_msgs::Bool msg)
{}

void IARCMission::quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
	quadrotorGroundPos.x = msg->point.x;
	quadrotorGroundPos.y = msg->point.y;
	
	ROS_INFO_THROTTLE(1,"Ground position x = %3.1f, y = %3.1f",quadrotorGroundPos.x,quadrotorGroundPos.y);
}

void IARCMission::dji_quaternion_callback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg)
{
	Quater[0] = msg->q0;
	Quater[1] = msg->q1;
	Quater[2] = msg->q2;
	Quater[3] = msg->q3;
	yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
	//ROS_INFO("yaw = %f",yaw);
}

void IARCMission::dji_velocity_callback(const dji_sdk::Velocity::ConstPtr &msg)
{
	dji_velocity_NED[0] = msg->vx;
	dji_velocity_NED[1] = msg->vy;
}


void IARCMission::tar_velocity_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  obtsacle_velocity[0] = msg->data[0];
  obtsacle_velocity[1] = msg->data[1];
  obstacle_emergency = (msg->data[2] > 0.5);
  obstacle_dist = msg->data[3];
}
//=========================mission function=================================


void IARCMission::missionCruise_P1()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	ROS_INFO("in CRUISE_P1!");
	ROS_INFO("quadGroundX = %3.1f, quadGroundY = %3.1f, setNEDX = %3.1f, setNEDY = %3.1f",quadrotorGroundPos.x,quadrotorGroundPos.y,cruiseSetPX_NED,cruiseSetPY_NED);
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	while(ros::ok() && quadState == CRUISE_P1)
	{
		ros::spinOnce();
		
		TG_srv.request.quadrotorState = TG_CRUISE;
		Cruise_Ground2NED(GlbVision_P1[0], GlbVision_P1[1]);
		TG_srv.request.targetPosNEDx = cruiseSetPX_NED;
		TG_srv.request.targetPosNEDy = cruiseSetPY_NED;
		TG_srv.request.targetPosNEDz = GlbVision_P1[2];
		TG_srv.request.theta = 0.0;
		TG_srv.request.cruiseStep = 0;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);

		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(getLength2f(localPosNED.x-cruiseSetPX_NED,localPosNED.y-cruiseSetPY_NED)<0.5)
		{
			quadState = SEARCH_P1;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_P1!");
		}
	}
}

void IARCMission::missionSearch_P1()
{
	ros::spinOnce();
	ROS_INFO("in SEARCH_P1!");
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	while (ros::ok() && quadState == SEARCH_P1)
	{
		ros::spinOnce();
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = 0;
	    delta_y = 0;
	    delta_z = 0;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(obstacle_emergency)
		{
			delta_vx = obtsacle_velocity[0];
			delta_vy = obtsacle_velocity[1];    
			CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
		}
		else
		{
			CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
		}
			
		if (glbVisTargetFlag)
		{
			quadState = CRUISE_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("global vision target detected, out SEARCH_P1!");
		}	
		else if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_GREEN!");
		}
		else if (duration.toSec() > 10)
		{
			quadState = CRUISE_P2;
			time_last = ros::Time::now();
			ROS_INFO("out SEARCH_P1!");
		}	
	}
}

void IARCMission::missionCruise_P2()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	ROS_INFO("in CRUISE_P2!");
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	while(ros::ok() && quadState == CRUISE_P2)
	{
		ros::spinOnce();
		
		TG_srv.request.quadrotorState = TG_CRUISE;
		Cruise_Ground2NED(GlbVision_P2[0], GlbVision_P2[1]);
		TG_srv.request.targetPosNEDx = cruiseSetPX_NED;
		TG_srv.request.targetPosNEDy = cruiseSetPY_NED;
		TG_srv.request.targetPosNEDz = GlbVision_P2[2];
		TG_srv.request.theta = 0.0;
		TG_srv.request.cruiseStep = 0;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(getLength2f(localPosNED.x-cruiseSetPX_NED,localPosNED.y-cruiseSetPY_NED)<0.5)
		{
			quadState = SEARCH_P2;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_P2!");
		}
	}
}

void IARCMission::missionSearch_P2()
{
	ros::spinOnce();
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	ROS_INFO("in SEARCH_P2!");
	
	while (ros::ok() && quadState == SEARCH_P2)
	{
		ros::spinOnce();
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = 0;
	    delta_y = 0;
	    delta_z = 0;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(obstacle_emergency)
		{
			delta_vx = obtsacle_velocity[0];
			delta_vy = obtsacle_velocity[1];    
			CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
		}
		else
		{
			CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
		}
		
		if (glbVisTargetFlag)
		{
			quadState = CRUISE_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("global vision target detected, out SEARCH_P2!");
		}
		else if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_GREEN!");
		}
		else if (duration.toSec() > 10)
		{
			quadState = CRUISE_P3;
			time_last = ros::Time::now();
			ROS_INFO("out SEARCH_P2!");
		}
	}
}

void IARCMission::missionCruise_P3()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TG_CRUISE;
	Cruise_Ground2NED(GlbVision_P3[0], GlbVision_P3[1]);
	TG_srv.request.targetPosNEDx = cruiseSetPX_NED;
	TG_srv.request.targetPosNEDy = cruiseSetPY_NED;
	TG_srv.request.targetPosNEDz = GlbVision_P3[2];
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	ROS_INFO("in CRUISE_P3!");
	
	while(ros::ok() && quadState == CRUISE_P3)
	{
		ros::spinOnce();
		
		TG_srv.request.quadrotorState = TG_CRUISE;
		Cruise_Ground2NED(GlbVision_P3[0], GlbVision_P3[1]);
		TG_srv.request.targetPosNEDx = cruiseSetPX_NED;
		TG_srv.request.targetPosNEDy = cruiseSetPY_NED;
		TG_srv.request.targetPosNEDz = GlbVision_P3[2];
		TG_srv.request.theta = 0.0;
		TG_srv.request.cruiseStep = 0;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(getLength2f(localPosNED.x-cruiseSetPX_NED,localPosNED.y-cruiseSetPY_NED)<0.5)
		{
			quadState = SEARCH_P3;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_P3!");
		}
	}
}

void IARCMission::missionSearch_P3()
{
	ros::spinOnce();
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	ROS_INFO("in SEARCH_P3!");
	
	while (ros::ok() && quadState == SEARCH_P3)
	{
		ros::spinOnce();
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = 0;
	    delta_y = 0;
	    delta_z = 0;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(obstacle_emergency)
		{
			delta_vx = obtsacle_velocity[0];
			delta_vy = obtsacle_velocity[1];    
			CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, 0, yaw_origin);
		}
		else
		{
			CDJIDrone->attitude_control(0x40, 0, 0, 0, yaw_origin);
		}
		
		if (glbVisTargetFlag)
		{
			quadState = CRUISE_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out SEARCH_P3!");
		}
		else if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_GREEN!");
		}
		else if (duration.toSec() > 10)
		{
			quadState = CRUISE_GREENBDR;
			ifGlbVisionOK = false;
			time_last = ros::Time::now();
			ROS_INFO("out SEARCH_P3!");
		}
	}
}


void IARCMission::missionCruise_TARGET()
{
	ros::spinOnce();
	
	float targetSetX, targetSetY;
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	iarc_mission::TG TG_srv;
	
	ROS_INFO("in CRUISE_TARGET!");
	
	while (ros::ok() && quadState == CRUISE_TARGET)
	{
		ros::spinOnce();
		
		//ROS_INFO("cruise_target chosen target x = %3.1f, y = %3.1f",targetSetX,targetSetY);
		
		if (glbTargetDetectedNum > 0)
		{
			targetSetX = glbChosenTargetPos[0];
			targetSetY = glbChosenTargetPos[1];
		}
	
		TG_srv.request.quadrotorState = TG_CRUISE;
		TG_srv.request.targetPosNEDx = targetSetX;
		TG_srv.request.targetPosNEDy = targetSetY;
		TG_srv.request.targetPosNEDz = CruiseHeight;
		TG_srv.request.theta = 0.0;
		TG_srv.request.cruiseStep = 0;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		
		if(getLength2f(localPosNED.x-targetSetX,localPosNED.y-targetSetY)<1 && irobotPosNED.flag>0)
		{
			ROS_INFO("Jump from cruise_target to track");
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			
		}
		else if(getLength2f(localPosNED.x-targetSetX,localPosNED.y-targetSetY)<1 && !(irobotPosNED.flag>0))
		{
			quadState = WANDER_SEARCH;
			time_last = ros::Time::now();
		}
		else if(boundaryClose)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}

void IARCMission::missionCruise_GREEN()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TG_CRUISE;
	Cruise_Ground2NED(cruiseGreenSetPX[greenSetPointID], cruiseGreenSetPY[greenSetPointID]);
	TG_srv.request.targetPosNEDx = cruiseSetPX_NED;
	TG_srv.request.targetPosNEDy = cruiseSetPY_NED;
	TG_srv.request.targetPosNEDz = CruiseHeight;
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	ROS_INFO("in CRUISE_GREEN!");
	
	while(ros::ok() && quadState == CRUISE_GREENBDR)
	{
		ros::spinOnce();
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(getLength2f(localPosNED.x-cruiseSetPX_NED,localPosNED.y-cruiseSetPY_NED)<0.5)
		{
			quadState = WANDER_SEARCH;
			greenSetPointID++;
			greenSetPointID = (int)fmod(greenSetPointID,10);
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_GREEN!");
		}
		else if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out CRUISE_GREEN!");
		}
		else if (glbVisTargetFlag)
		{
			quadState = CRUISE_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("JUMP from wander to cruise_target");
		}
		else if(boundaryClose)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}



void IARCMission::missionWander()   //TODO: add the WANDER case in the trajectory generation package
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TG_WANDER;
	TG_srv.request.targetPosNEDx = localPosNED.x;
	TG_srv.request.targetPosNEDy = localPosNED.y;
	TG_srv.request.targetPosNEDz = CruiseHeight;
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	
	ROS_INFO("in WANDER_SEARCH!");
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	while (ros::ok() && quadState == WANDER_SEARCH)
	{
		//ROS_INFO("WANDERING");
		ros::spinOnce();
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = 0;
	    delta_y = 0;
	    delta_z = 0;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("out WANDER_SEARCH!");
		}
		else if (duration.toSec() > 20 && ifGlbVisionOK)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
			ROS_INFO("out WANDER_SEARCH!");
		}
		else if (duration.toSec() > 20 && !ifGlbVisionOK && !testInteractionFlag)
		{
			quadState = CRUISE_GREENBDR;
			time_last = ros::Time::now();
			ROS_INFO("out WANDER_SEARCH!");
		}
		else if (glbVisTargetFlag)
		{
			quadState = CRUISE_TARGET;
			time_last = ros::Time::now();
			ROS_INFO("JUMP from wander to cruise_target");
		}
		else if(boundaryClose)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}

void IARCMission::missionWander_PRE()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TG_CRUISE;
	TG_srv.request.targetPosNEDx = posPre_forReact.x;
	TG_srv.request.targetPosNEDy = posPre_forReact.y;
	TG_srv.request.targetPosNEDz = CruiseHeight;
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	
	ROS_INFO("JUMP to prediction point x = %3.1f, y=%3.1f", posPre_forReact.x, posPre_forReact.y);
	
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	while (ros::ok() && quadState == WANDER_PREDICT)
	{
		ros::spinOnce();
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		if(irobotPosNED.flag > 0)
		{
			quadState = TRACK_TARGET;
			inInteraction = true;
			time_last = ros::Time::now();
		}
		/*
		else if (duration.toSec() > 10 && ifGlbVisionOK)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
		*/
		else if (duration.toSec() > 10)
		{
			quadState = WANDER_SEARCH;
			time_last = ros::Time::now();
			ROS_INFO("Jump to WANDER_SEARCH");
		}
		else if(boundaryClose)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}

void IARCMission::missionTrack()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	float tarX,tarY,tarTheta;
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;

	while(ros::ok() && quadState == TRACK_TARGET)
	{
		//ROS_INFO("TRACKING TARGET");
		ros::spinOnce();
		if (irobotPosNED.flag > 0)
		{
			tarX = irobotPosNED.x;
			tarY = irobotPosNED.y;	
			tarTheta = irobotPosNED.theta;
		}
		
		TG_srv.request.quadrotorState = TG_TRACK;
		TG_srv.request.targetPosNEDx = tarX;
		TG_srv.request.targetPosNEDy = tarY;
		TG_srv.request.targetPosNEDz = CruiseHeight;
		TG_srv.request.theta = tarTheta;
		TG_srv.request.cruiseStep = 0;
		
		obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);
		
		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if(obstacle_emergency)
			{
				delta_vx = obtsacle_velocity[0];
				delta_vy = obtsacle_velocity[1];    
				CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
		
		//ROS_INFO("x = %3.1f, y=%3.1f",irobotPosNED.x,irobotPosNED.y);
		
		if (irobotPosNED.flag > 0)
		{
			time_last = ros::Time::now();
		}
		time_now = ros::Time::now();
		duration = time_now - time_last;
		
		if (irobotPosNED.flag == 0 && duration.toSec()>1.5)
		{
			quadState = WANDER_SEARCH;
			time_last = ros::Time::now();
			ROS_INFO("Jump from track to wander");
		}
		else if (irobotPosNED.flag > 0 && readytoHeadCollision())
		{
			ActionType = A_HEADCOLLISION;
			quadState = APPROACH_HEAD;
			time_last = ros::Time::now();
		}
		
		else if (irobotPosNED.flag > 0 && readytoTrackApproach())
		{
			quadState = TRACK_APPROACH;
			time_last = ros::Time::now();
			ROS_INFO("Jump from track to track_approach");
		}
		else if(boundaryClose)
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}

void IARCMission::missionTrack_Approach()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	float tarX,tarY,tarTheta;
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;

	while(ros::ok() && quadState == TRACK_APPROACH)
	{
		//ROS_INFO("TRACKING TARGET");
		ros::spinOnce();
		
		if (localPosNED.z > ApproachHeight)
		{
			if (irobotPosNED.flag > 0)
			{
				tarX = irobotPosNED.x;
				tarY = irobotPosNED.y;
				tarTheta = irobotPosNED.theta;
			}
			//CDJIDrone->attitude_control(0x00, 0.33*cos(tarTheta), 0.33*sin(tarTheta), -0.4, yaw_origin);
			//ROS_INFO("quadrotor higher than 1.2m... landing... z = %3.1f",localPosNED.z);
			
			TG_srv.request.quadrotorState = TG_TRACK_APP;
			TG_srv.request.targetPosNEDx = tarX;
			TG_srv.request.targetPosNEDy = tarY;
			TG_srv.request.targetPosNEDz = 0;
			TG_srv.request.theta = tarTheta;
			TG_srv.request.cruiseStep = 0;
			
			obs_msg.data.clear();
			obs_msg.data.push_back(Quater[0]);
			obs_msg.data.push_back(Quater[1]);
			obs_msg.data.push_back(Quater[2]);
			obs_msg.data.push_back(Quater[3]);
			
			obs_msg.data.push_back(dji_velocity_NED[0]);
			obs_msg.data.push_back(dji_velocity_NED[1]);
			obs_msg.data.push_back(dji_velocity_NED[2]);
			
			delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
			delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
			delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
			obs_msg.data.push_back(delta_x);
			obs_msg.data.push_back(delta_y);
			obs_msg.data.push_back(delta_z);
			
			obstacle_pub.publish(obs_msg);
			
			if(!TG_client.call(TG_srv))
				ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
			else
			{
				if(obstacle_emergency)
				{
					delta_vx = obtsacle_velocity[0];
					delta_vy = obtsacle_velocity[1];    
					CDJIDrone->attitude_control(0x40, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
				}
				else
				{
					CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
				}
			}
		}
		else if (localPosNED.z < ApproachHeight)
		{
			ros::spinOnce();
			//ROS_INFO("lower than 1.1m, z = %3.1f, duration = %3.1f",localPosNED.z,duration.toSec());
			if (irobotPosNED.flag > 0)
			{
				tarX = irobotPosNED.x;
				tarY = irobotPosNED.y;
				tarTheta = irobotPosNED.theta;
			}
			
			TG_srv.request.quadrotorState = TG_TRACK;
			TG_srv.request.targetPosNEDx = tarX;
			TG_srv.request.targetPosNEDy = tarY;
			TG_srv.request.targetPosNEDz = ApproachHeight;
			TG_srv.request.theta = tarTheta;
			TG_srv.request.cruiseStep = 0;
			
			if(!TG_client.call(TG_srv))
				ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
			else
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
			
			if (irobotPosNED.flag > 0)
			{
				time_last = ros::Time::now();
			}
		
			time_now = ros::Time::now();
			duration = time_now - time_last;
			duration_topCount = time_now - time_topCount;
			if (duration.toSec()>1.2)
			{
				mission_takeoff();
				quadState = WANDER_SEARCH;
				time_last = ros::Time::now();
				ROS_INFO("LOSE target!! jump from track_approach to wander");
			}
			else if(obstacle_emergency && obstacle_dist < 1.5)  //wxd change
			{
				quadState = CRUISE_P1;
				time_last = ros::Time::now();
				ROS_ERROR("avoidance taking off");
			}
			else if(topTouchCount == 5)
			{
				quadState = CRUISE_P1;
				time_last = ros::Time::now();
				topTouchCount = 0;
			}
			else if (readytoTopTouch()&& duration_topCount.toSec()>3)
			{
				ActionType = A_TOPTOUCH;
				//quadState = APPROACH_TOP;
				mission_armact();
				//sleep(2);
				time_last = ros::Time::now();
				time_topCount = ros::Time::now();
				topTouchCount++;
				//mission_takeoff();
				ROS_ERROR("ARM ACTION!!!");
				
			}
			else if(irobotPosNED.flag>0 && fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0))) < M_PI/6.0 || fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0 + M_PI))) < M_PI/6.0)
			{
				mission_takeoff();
				quadState = TRACK_TARGET;
				time_last = ros::Time::now();
			}
			else if(boundaryClose)
			{
				quadState = CRUISE_P1;
				time_last = ros::Time::now();
			}
		}
	}
}


void IARCMission::missionApproach_HEAD()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	float tarX,tarY,tarTheta;

	while(ros::ok() && quadState == APPROACH_HEAD)
	{
		//ROS_INFO("TRACKING TARGET");
		ros::spinOnce();
		
		if (irobotPosNED.flag > 0)
		{
			tarX = irobotPosNED.x;
			tarY = irobotPosNED.y;	
			tarTheta = irobotPosNED.theta;
		}
		
		TG_srv.request.quadrotorState = TG_APP_HEAD;
		TG_srv.request.targetPosNEDx = tarX;
		TG_srv.request.targetPosNEDy = tarY;
		TG_srv.request.targetPosNEDz = 0;
		TG_srv.request.theta = tarTheta;
		TG_srv.request.cruiseStep = 0;
	
		ROS_INFO_THROTTLE(1,"APPROACH HEAD COLLISION");

		if(!TG_client.call(TG_srv))
			ROS_INFO("IARCMission TG_client.call failed");
		else
		{
			ROS_INFO_THROTTLE(0.3, "mission_APP:%d,%3.1f,%3.1f,%3.1f",TG_srv.response.flightFlag,TG_srv.response.flightCtrlDstx,TG_srv.response.flightCtrlDsty,TG_srv.response.flightCtrlDstz);
			if((uint8_t)0x80 == TG_srv.response.flightFlag)
				CDJIDrone->attitude_control(0x80, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
		if(localPosNED.z < 0.3)	//TODO: Z!!??  accumulated error in z axis!!
		{
			ROS_INFO("going to land...");
			mission_land();
			//CDJIDrone->request_sdk_permission_control();
			//sleep(10);
			mission_takeoff();
			
			pos_Now.x = tarX;
			pos_Now.y = tarY;
			pos_Now.theta = tarTheta + M_PI;
			time_now = ros::Time::now();
			duration_mission = time_now - time_missionStart;
			time_in_20s = fmod(duration_mission.toSec(),20);
			
			posPre_forReact.x = pos_Now.x + 5*cos(pos_Now.theta);
			posPre_forReact.y = pos_Now.y + 5*sin(pos_Now.theta);
			
			//posPre_forReact = predictIrobotPose(pos_Now, time_reactAftHead, time_in_20s);
			
			quadState = WANDER_PREDICT;
			ActionType = A_HOLD;
			time_last = ros::Time::now();
		}
		
		if(obstacle_emergency && obstacle_dist < 1.5)  //wxd change
		{
			quadState = CRUISE_P1;
			time_last = ros::Time::now();
		}
	}
}

void IARCMission::missionApproach_TOP()
{
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	
	float tarX,tarY,tarTheta;
	
	ROS_INFO("APPROACH TOP TOUCH");
	
	mission_armact();

	while(ros::ok() && quadState == APPROACH_TOP)
	{
		//ROS_INFO("APPROACH TOP TOUCH");
		ros::spinOnce();
		
		if (irobotPosNED.flag > 0)
		{
			tarX = irobotPosNED.x;
			tarY = irobotPosNED.y;
			tarTheta = irobotPosNED.theta;
		}
			
		TG_srv.request.quadrotorState = TG_TRACK;
		TG_srv.request.targetPosNEDx = tarX;
		TG_srv.request.targetPosNEDy = tarY;
		TG_srv.request.targetPosNEDz = 1.1;
		TG_srv.request.theta = tarTheta;
		TG_srv.request.cruiseStep = 0;
			
		

		if(!TG_client.call(TG_srv))
			ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
		else
		{
			if((uint8_t)0x40 == TG_srv.response.flightFlag)
			{
				CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			}
		}
			
		/*
		if(!TG_client.call(TG_srv))
			ROS_INFO("IARCMission TG_client.call failled......");
		else
		{
			ROS_INFO_THROTTLE(0.3, "mission_APP:%d,%3.1f,%3.1f,%3.1f",TG_srv.response.flightFlag,TG_srv.response.flightCtrlDstx,TG_srv.response.flightCtrlDsty,TG_srv.response.flightCtrlDstz);
			if((uint8_t)0x80 == TG_srv.response.flightFlag)
				CDJIDrone->attitude_control(0x80, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
			if((uint8_t)0x50 == TG_srv.response.flightFlag)
			{
				ros::spinOnce();
				while((ros::ok()) && (localPosNED.z<1.4))
				{
					ros::spinOnce();
					CDJIDrone->attitude_control(0x40, 0, 0, 0.8, yaw_origin);
					ROS_INFO_THROTTLE(1,"avoidance taking off...");
					usleep(20000);
				}
				break;
			}
		}
		*/
		if(localPosNED.z < 1.2)	//TODO: Z!!??  accumulated error in z axis!!
		{
			ROS_INFO("going to toptouch...");
			//mission_armact();
			
			/*
			pos_Now.x = irobotPosNED.x;
			pos_Now.y = irobotPosNED.y;
			pos_Now.theta = irobotPosNED.theta + M_PI/4;
			time_now = ros::Time::now();
			duration_mission = time_now - time_missionStart;
			time_in_20s = fmod(duration_mission.toSec(),20);
			
			posPre_forReact = predictIrobotPose(pos_Now, time_reactAftHead, time_in_20s);
			*/
			
			quadState = TRACK_APPROACH;
			ActionType = A_TOPTOUCH;
			time_last = ros::Time::now();
			time_topCount = ros::Time::now();
		}
	}
}


void IARCMission::missionTest()
{
	ROS_INFO("TEST mode");
	//mission_armact();
	//-------------------cruise mode--------------------------
	/*
	iarc_mission::TG TG_srv;
	TG_srv.request.quadrotorState = TG_CRUISE;
	TG_srv.request.targetPosNEDx = 12;
	TG_srv.request.targetPosNEDy = 0;
	TG_srv.request.targetPosNEDz = 0.8;
	TG_srv.request.theta = 0.0;
	TG_srv.request.cruiseStep = 0;
	float P_ = 0.5;
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	
	ROS_INFO("in TEST_OBS_CRUISE!");
	
	while(ros::ok() && quadState == TEST_OBS)
	{
	    ros::spinOnce();
	    obs_msg.data.clear();
	    obs_msg.data.push_back(Quater[0]);
	    obs_msg.data.push_back(Quater[1]);
	    obs_msg.data.push_back(Quater[2]);
	    obs_msg.data.push_back(Quater[3]);
	    
	    obs_msg.data.push_back(dji_velocity_NED[0]);
	    obs_msg.data.push_back(dji_velocity_NED[1]);
	    obs_msg.data.push_back(dji_velocity_NED[2]);
	    
	    delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
	    delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
	    delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
	    obs_msg.data.push_back(delta_x);
	    obs_msg.data.push_back(delta_y);
	    obs_msg.data.push_back(delta_z);
	    
	    obstacle_pub.publish(obs_msg);

	    if(!TG_client.call(TG_srv))
	        ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
	    else
	    {

		if(obstacle_emergency)
		{
		    delta_vx = obtsacle_velocity[0];
		    delta_vy = obtsacle_velocity[1];
                      
		    CDJIDrone->attitude_control(TG_srv.response.flightFlag, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
		else
		{
		    CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
		}
	    }
	}
	*/
	//------------------track mode---------------------
	
	ros::spinOnce();
	iarc_mission::TG TG_srv;
	float delta_vx, delta_vy, delta_x, delta_y, delta_z;
	ROS_INFO("in TEST_OBS_CRUISE!");

	while(ros::ok() && quadState == TEST_OBS)
	{
		//ROS_INFO("TRACKING TARGET");
		ros::spinOnce();
		
		if (irobotPosNED.flag > 0)
		{
			TG_srv.request.quadrotorState = TG_TRACK;
			TG_srv.request.targetPosNEDx = irobotPosNED.x;
			TG_srv.request.targetPosNEDy = irobotPosNED.y;
			TG_srv.request.targetPosNEDz = 1.6;
			TG_srv.request.theta = irobotPosNED.theta;
			TG_srv.request.cruiseStep = 0;
			
			obs_msg.data.clear();
			obs_msg.data.push_back(Quater[0]);
			obs_msg.data.push_back(Quater[1]);
			obs_msg.data.push_back(Quater[2]);
			obs_msg.data.push_back(Quater[3]);
	    
			obs_msg.data.push_back(dji_velocity_NED[0]);
			obs_msg.data.push_back(dji_velocity_NED[1]);
			obs_msg.data.push_back(dji_velocity_NED[2]);
	    
			delta_x = (float)TG_srv.request.targetPosNEDx - localPosNED.x;
			delta_y = (float)TG_srv.request.targetPosNEDy - localPosNED.y;
			delta_z = (float)TG_srv.request.targetPosNEDz - localPosNED.z;
			obs_msg.data.push_back(delta_x);
			obs_msg.data.push_back(delta_y);
			obs_msg.data.push_back(delta_z);
	    
			obstacle_pub.publish(obs_msg);
			
			if(!TG_client.call(TG_srv))
				ROS_INFO_THROTTLE(0.2,"IARCMission TG_client.call failled......");
			else
			{

				if(obstacle_emergency)
				{
					delta_vx = obtsacle_velocity[0];
					delta_vy = obtsacle_velocity[1];
					CDJIDrone->attitude_control(TG_srv.response.flightFlag, delta_vx, delta_vy, TG_srv.response.flightCtrlDstz, yaw_origin);
				}
				else
				{
					CDJIDrone->attitude_control(TG_srv.response.flightFlag, TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, yaw_origin);
				}
			}
		}
		else
		{
			CDJIDrone->attitude_control(0x40, 0.0, 0.0, 0.0, yaw_origin);
		}
	}
}



//===============================other function============================

void IARCMission::irobotReward()
{
	
	time_now = ros::Time::now();
	duration_mission = time_now - time_missionStart;
	time_in_20s = fmod(duration_mission.toSec(),20);
	//if theta is forward to green line, reward is small, should goto track and approach, should be selected
	for(int i = 0;i != irobotsPosNEDWithReward.flag;i++)
	{
		float reward;
		//reward = fabs(limitAng(irobotsPosNEDWithReward.theta[i]) - limitAng(yaw_origin*M_PI/180.0 + 0.5*M_PI));
		if (time_in_20s>19.5 || time_in_20s < 2.5)
			reward = getLength2f(irobotsPosNEDWithReward.x[i]-localPosNED.x,irobotsPosNEDWithReward.y[i]-localPosNED.y);
		else
		{
			reward  =min(fabs(limitAng(irobotsPosNEDWithReward.theta[i] - limitAng(yaw_origin*M_PI/180.0))),fabs(limitAng(irobotsPosNEDWithReward.theta[i] - limitAng(yaw_origin*M_PI/180.0 + M_PI))));
			if (reward < M_PI/9)
				reward = -1;
		}
		//ROS_INFO_THROTTLE(0.2,"irobotPosNED=(%4.2f,%4.2f),reward=%4.2f",irobotsPosNEDWithReward.x[i],irobotsPosNEDWithReward.y[i],reward);
		//irobotsPosNEDWithReward.reward.push_back(reward);
		irobotsPosNEDWithReward.reward[i] = reward;
	}
}



bool IARCMission::mission_takeoff()
{
	ros::spinOnce();
	CDJIDrone->drone_arm();
	while((ros::ok()) && (localPosNED.z<1.5))
	{
		ros::spinOnce();
		//ROS_INFO_THROTTLE(0.3, "PosNED.z=%4.2f",localPosNED.z);
		CDJIDrone->attitude_control(0x00, 0, 0, 0.8, yaw_origin);
		ROS_INFO_THROTTLE(1,"taking off...");
		usleep(20000);
	}
	return true;
}

bool IARCMission::mission_land()
{

	for(int i = 0; i < 75; i ++) 
	{
		CDJIDrone->attitude_control(0x00, 0, 0, -0.6, yaw_origin);
		ROS_INFO_THROTTLE(1,"landing...");
		usleep(20000);
	}
	for(int i = 0; i < 25; i ++) 
	{
		CDJIDrone->drone_arm();
		ROS_INFO_THROTTLE(1,"awaiting takeoff...");
		usleep(20000);
	}
	return true;
}


float IARCMission::getLength2f(float x, float y)
{
	return sqrt(x*x+y*y);
}

IARCMission::irobotPose_ IARCMission::predictIrobotPose(IARCMission::irobotPose_ irobotpose, float TPred, float TInLoop)
{
	float dT = 0.1;
	IARCMission::irobotPose_ ret;
	for(int i = 1;i < floor(TPred/dT);i++)
	{
		if(fmod(dT*i + TInLoop, 20)<2.0)
		{
			ret.x = irobotpose.x;
			ret.y = irobotpose.y;
			ret.theta = M_PI/2.0*dT + irobotpose.theta;
		}
		else
		{
			ret.x = irobotpose.x + 0.3333333*cos(irobotpose.theta)*dT;
			ret.y = irobotpose.y + 0.3333333*sin(irobotpose.theta)*dT;
			ret.theta = 0;			
		}
	}
	return ret;
}

float IARCMission::limitAng(float theta)
{
	if(theta < -M_PI)theta += 2*M_PI;
	if(theta > M_PI)theta -= 2*M_PI;
	return theta;
}

bool IARCMission::readytoHeadCollision()
{
	time_now = ros::Time::now();
	duration_mission = time_now - time_missionStart;
	time_in_20s = fmod(duration_mission.toSec(),20);
	
	ROS_INFO_THROTTLE(1,"global timer is %2.1f", time_in_20s);
	
	bool inActionRange;
	inActionRange = (getLength2f(irobotPosNED.x+0.5*cos(irobotPosNED.theta)-localPosNED.x,irobotPosNED.y+0.5*sin(irobotPosNED.theta)-localPosNED.y) < 0.6);

	if (inActionRange && fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0))) < M_PI/4.0 && time_in_20s > 2.5 && time_in_20s < 6)
		return true;
	else if (inActionRange && fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0 + M_PI))) < M_PI/4.0 && time_in_20s < 15 && time_in_20s >11)
		return true;
	else
		return false;
}

bool IARCMission::readytoTrackApproach()
{
	time_now = ros::Time::now();
	duration_mission = time_now - time_missionStart;
	time_in_20s = fmod(duration_mission.toSec(),20);
	
	float dist;
	bool inActionRange;
	bool inTopActionAngle;
	bool inTime;
	
	dist = getLength2f(irobotPosNED.x+0.2*cos(irobotPosNED.theta+M_PI/2)-localPosNED.x,irobotPosNED.y+0.2*sin(irobotPosNED.theta+M_PI/2)-localPosNED.y); 
	ROS_INFO_THROTTLE(1,"dist = %3.1f",dist);
	
	if (localPosNED.z > 1.2)
		inActionRange = (getLength2f(irobotPosNED.x+0.5*cos(irobotPosNED.theta)-localPosNED.x,irobotPosNED.y+0.5*sin(irobotPosNED.theta)-localPosNED.y) < 1.0);
	else
		//inActionRange = (getLength2f(irobotPosNED.x+0.6*cos(irobotPosNED.theta)+0.2*cos(irobotPosNED.theta+M_PI/2)-localPosNED.x,irobotPosNED.y+0.6*sin(irobotPosNED.theta)+0.2*sin(irobotPosNED.theta+M_PI/2)-localPosNED.y) < 0.6);
		inActionRange = (getLength2f(irobotPosNED.x+0.1*cos(irobotPosNED.theta)+0.2-localPosNED.x,irobotPosNED.y+0.1*sin(irobotPosNED.theta)-localPosNED.y) < 0.2);
	inTopActionAngle = !(fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0))) < M_PI/4.0 || fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0 + M_PI))) < M_PI/4.0);
	inTime = ((time_in_20s>2.5) && (time_in_20s<18));
	
	if (inActionRange && inTopActionAngle && inTime)
		return true;
	else
		return false;
}

bool IARCMission::readytoTopTouch()
{
	time_now = ros::Time::now();
	duration_mission = time_now - time_missionStart;
	time_in_20s = fmod(duration_mission.toSec(),20);
	
	float dist;
	bool inActionRange;
	bool inTopActionAngle;
	bool inTime;
	
	dist = getLength2f(irobotPosNED.x+0.2*cos(irobotPosNED.theta+M_PI/2)-localPosNED.x,irobotPosNED.y+0.2*sin(irobotPosNED.theta+M_PI/2)-localPosNED.y); 
	ROS_INFO_THROTTLE(1,"dist = %3.1f",dist);
	
	if (localPosNED.z > 1.2)
		inActionRange = (getLength2f(irobotPosNED.x+0.5*cos(irobotPosNED.theta)-localPosNED.x,irobotPosNED.y+0.5*sin(irobotPosNED.theta)-localPosNED.y) < 1.0);
	else
		//inActionRange = (getLength2f(irobotPosNED.x+0.6*cos(irobotPosNED.theta)+0.2*cos(irobotPosNED.theta+M_PI/2)-localPosNED.x,irobotPosNED.y+0.6*sin(irobotPosNED.theta)+0.2*sin(irobotPosNED.theta+M_PI/2)-localPosNED.y) < 0.6);
		inActionRange = (getLength2f(irobotPosNED.x+0.05*cos(irobotPosNED.theta)+0.2-localPosNED.x,irobotPosNED.y+0.05*sin(irobotPosNED.theta)-localPosNED.y) < 0.15);
	//inTopActionAngle = !(fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0))) < M_PI/4.0 || fabs(limitAng(irobotPosNED.theta - limitAng(yaw_origin*M_PI/180.0 + M_PI))) < M_PI/4.0);
	inTime = ((time_in_20s>2.5) && (time_in_20s<18));
	
	if (inActionRange && inTime)
		return true;
	else
		return false;
}

void IARCMission::mission_armreset()
{
	int fd=-1;
  	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd==-1)
	{
		perror("Open Serial Port Error!\n");
		return;
	} 
	ROS_INFO("ARM!!");
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME]=0;
	options.c_cc[VMIN]=1;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW, &options); 
	  
	unsigned char buff[30]={0xFF,0x09,0x00,0x01,0x00};
	 
//	waitKey(0);
	write (fd,buff,5);
	   
	close (fd);
}

void IARCMission::mission_armact()
{
	ros::spinOnce();
	if (pilotMode < -7000)
	{
		ROS_ERROR("Pilot manual control, no arm action");
		return;
	}
		
	int fd=-1;
  	fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd==-1)
	{
		perror("Open Serial Port Error!\n");
		return;
	} 
	ROS_INFO("ARM!!");
	struct termios options;
	tcgetattr(fd, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VTIME]=0;
	options.c_cc[VMIN]=1;
	tcflush(fd,TCIFLUSH);
	tcsetattr(fd,TCSANOW, &options); 
	  
	unsigned char buff[30]={0xFF,0x09,0x00,0x00,0x00};
	 
//	waitKey(0);
	write (fd,buff,5);
	   
	close (fd);
}


void IARCMission::glb_Body2Ground()
{	
	ros::spinOnce();
	float RotationMat[3][3];
	
	RotationMat[0][0] = Quater[0] * Quater[0] + Quater[1] * Quater[1] - Quater[2] * Quater[2] - Quater[3] * Quater[3];
	RotationMat[0][1] = 2 * ( Quater[1] * Quater[2] - Quater[0] * Quater[3]);
	RotationMat[0][2] = 2 * ( Quater[1] * Quater[3] + Quater[0] * Quater[2]);
	RotationMat[1][0] = 2 * ( Quater[1] * Quater[2] + Quater[0] * Quater[3]);
	RotationMat[1][1] = Quater[0] * Quater[0] - Quater[1] * Quater[1] + Quater[2] * Quater[2] - Quater[3] * Quater[3];
	RotationMat[1][2] = 2 * ( Quater[2] * Quater[3] - Quater[0] * Quater[1]);
	RotationMat[2][0] = 2 * ( Quater[1] * Quater[3] - Quater[0] * Quater[2]);
	RotationMat[2][1] = 2 * ( Quater[2] * Quater[3] + Quater[0] * Quater[1]);
	RotationMat[2][2] = Quater[0] * Quater[0] - Quater[1] * Quater[1] - Quater[2] * Quater[2] + Quater[3] * Quater[3];
	

	
	for (int i = 0; i < glbTargetDetectedNum; i++)
	{
		glbVisTargetX_NED[i] = RotationMat[0][0]*glbVisTargetX[i] + RotationMat[0][1]*glbVisTargetY[i] + RotationMat[0][2]*glbVisTargetZ[i] + localPosNED.x;
		glbVisTargetY_NED[i] = RotationMat[1][0]*glbVisTargetX[i] + RotationMat[1][1]*glbVisTargetY[i] + RotationMat[1][2]*glbVisTargetZ[i] + localPosNED.y;
		glbVisTargetZ_NED[i] = 0;
		glbVisTargetX_Gr[i] = cos(yaw_origin_rad)*glbVisTargetX_NED[i] - sin(yaw_origin_rad)*glbVisTargetY_NED[i] + sta_x;
		glbVisTargetY_Gr[i] = sin(yaw_origin_rad)*glbVisTargetX_NED[i] + cos(yaw_origin_rad)*glbVisTargetY_NED[i] + sta_x;
		glbVisTargetZ_Gr[i] = 0;
	}
	
	//ROS_INFO("4. x=%3.1f,y=%3.1f",glbVisTargetX_NED[1],glbVisTargetY_NED[1]);

	/*
	for (int i = 0; i < glbTargetDetectedNum; i++)
	{
		ObsXTemp = glbVisObsX[i];
		ObsYTemp = glbVisObsY[i];
		ObsZTemp = glbVisObsZ[i];
		glbVisObsX[i] = RotationMat[0][0]*ObsXTemp + RotationMat[0][1]*ObsYTemp + RotationMat[0][2]*ObsZTemp + localPosNED.x;
		glbVisObsY[i] = RotationMat[1][0]*ObsXTemp + RotationMat[1][1]*ObsYTemp + RotationMat[1][2]*ObsZTemp + localPosNED.y;
		ObsXTemp = glbVisObsX[i];
		ObsYTemp = glbVisObsY[i];
		glbVisObsX[i] = cos(NED2GrTheta)*ObsXTemp - sin(NED2GrTheta)*ObsYTemp + NED2GrXOrigin;
		glbVisObsY[i] = sin(NED2GrTheta)*ObsXTemp + cos(NED2GrTheta)*ObsYTemp + NED2GrYOrigin;
		glbVisObsZ[i] = 0;
	}
	ROS_INFO("glb 10");
	*/
}

void IARCMission::Cruise_Ground2NED(double groundPosX, double groundPosY)
{
	float RelDistX, RelDistY;
	cruiseSetPX_NED = cos(-yaw_origin_rad)*(groundPosX-quadrotorGroundPos.x) - sin(-yaw_origin_rad)*(groundPosY-quadrotorGroundPos.y) + localPosNED.x;
	cruiseSetPY_NED = sin(-yaw_origin_rad)*(groundPosX-quadrotorGroundPos.x) + cos(-yaw_origin_rad)*(groundPosY-quadrotorGroundPos.y) + localPosNED.y;
}


};


