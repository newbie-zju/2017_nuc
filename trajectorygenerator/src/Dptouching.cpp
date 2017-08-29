#include "Dptouching.h"
#include <math.h>
// #define max((a),(b)) ((a)>(b)?(a):(b))
// #define min((a),(b)) ((a)<(b)?(a):(b))
using namespace std;

DpTouching::DpTouching(ros::NodeHandle nh_):nh(nh_),nh_param("~")
{
	initialize();
}

bool DpTouching::insideRec(float tx,float ty,float x1,float y1,float x2,float y2)

{
	if ((tx-x1)*(tx-x2)<0&&(ty-y1)*(ty-y2)<0)
	{
		return true;
	} 
	else
	{
		return false;
	}
}

void DpTouching::initialize()
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
		
		wanderR = pt.get<float>("wanderR");
		dxy = pt.get<float>("dxy");
	} 
	catch (int e) 
	{
		cout << "reading xml file failed" << e << endl;
	}
	*/
	
	wanderR = 1.5;
	dxy = 0.5;
	
	
	dT = 0.1;
	TG_server = nh.advertiseService("/TG/TG_service", &DpTouching::calculateTrajectoryCallback, this);
	quadrotorPosNED_sub = nh.subscribe("/dji_sdk/local_position", 10, &DpTouching::quadrotorPosNEDCallback, this);
	quadrotorPosGround_sub = nh.subscribe("ground_position",10, &DpTouching::quadrotorPosGroundCallback,this);
	hokuyoBody_sub = nh.subscribe("/hokuyo/pillar_data",10, &DpTouching::hokuyo_dataCallback, this);
	tf_client = nh.serviceClient<iarc_tf::Velocity>("ned_world_velocity_transform_srvice");
	guidance_distance_sub = nh.subscribe("/guidance/ultrasonic", 1, &DpTouching::guidanceObstacleCallback, this);
	guidance_emergency = false;
	hokuyo_emergency =false;
	if(!nh_param.getParam("xMax", xMax))xMax = 5.0;
	if(!nh_param.getParam("yMax", yMax))yMax = 5.0;
	if(!nh_param.getParam("avoidanceV", avoidanceV))avoidanceV = 0.6;
	if(!nh_param.getParam("cruiseVel", tarV))tarV = 0.5;
	if(!nh_param.getParam("Kr", Kr))Kr = 3.25;
	if(!nh_param.getParam("fattractive", fattractive))fattractive = 1.0;
	number_obstacle = 0;
}

void DpTouching::quadrotorPosNEDCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPosNED.x = msg->x;
	quadrotorPosNED.y = msg->y;
	quadrotorPosNED.z = msg->z;
}

void DpTouching::quadrotorPosGroundCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
	quadrotorPos.x = msg->point.x;
	quadrotorPos.y = msg->point.y;
}

void DpTouching::hokuyo_dataCallback(const obstacle_avoidance::Hokuyo::ConstPtr &msg)
{
	number_obstacle = msg->number;
	//ROS_INFO_THROTTLE(0.3,"DP: %3.1f,%3.1f",msg->ranges[0],msg->angles[0]);
	for(int i = 0; i < number_obstacle; i++)
	{
		obstacle_ranges[i] = msg->ranges[i];
		obstacle_angles[i] = msg->angles[i];
	}
}
void DpTouching::guidanceObstacleCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	if ((msg->intensities[1] > 0.0) && (msg->ranges[1] < 2.5) && (msg->ranges[1] > 0.5))
	{
		guidancerang = msg->ranges[1];
		guidance_emergency = true;
		
	}
	else
	{
		guidance_emergency = false;
		guidancerang =999;
	}
}
bool DpTouching::calculateTrajectoryCallback(iarc_mission::TG::Request &req, iarc_mission::TG::Response &res)
{
	ros::spinOnce();
	float k = 0.7;
	float tarVz = k * (1.6 - quadrotorPosNED.z);
	switch(req.quadrotorState)
	{
		
		case TG_CRUISE:
		{
			//ROS_INFO("Cruising to P1");
			tarX = req.targetPosNEDx;
			tarY = req.targetPosNEDy;
			tarZ = 1.6;

			ROS_INFO_THROTTLE(1, "TG_CRUISE:quadPosNED=%4.2f,%4.2f,%4.2f,irobotPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,req.targetPosNEDx,req.targetPosNEDy,req.theta);
// 			runMethod();
			cruise_dist = getLength2f(tarX-quadrotorPosNED.x,tarY-quadrotorPosNED.y);
			if(cruise_dist > 1)
			{
				res.flightCtrlDstx = 0.9*(tarX-quadrotorPosNED.x)/cruise_dist;
				res.flightCtrlDsty = 0.9*(tarY-quadrotorPosNED.y)/cruise_dist;
			}
			else
			{
				res.flightCtrlDstx = 0.7*(tarX-quadrotorPosNED.x);
				res.flightCtrlDsty = 0.7*(tarY-quadrotorPosNED.y);
			}
			
			//res.flightCtrlDstx = 0.6*(tarX-quadrotorPosNED.x);
			//res.flightCtrlDsty = 0.6*(tarY-quadrotorPosNED.y);

			//res.flightCtrlDstz = 1.0*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = tarVz;
			//ROS_INFO_THROTTLE(0.2,"TRACK: dp_x=%4.2f,dp_y=%4.2f",res.flightCtrlDstx,res.flightCtrlDsty);
			res.flightFlag = 0x80;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0)) || guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
		
		
		case TG_WANDER:
		{
			
			wanderCenterX = req.targetPosNEDx;
			wanderCenterY = req.targetPosNEDy;
			tarZ = 1.6;
			ROS_INFO_THROTTLE(1, "TG_WANDER:quadNED:%4.2f, %4.2f",quadrotorPosNED.x,quadrotorPosNED.y);
			

			float theta_center2quad = atan2((quadrotorPosNED.y-wanderCenterY),(quadrotorPosNED.x-wanderCenterX));
			float dtheta = dxy/wanderR;		

			tarX = cos(theta_center2quad - dtheta) * wanderR + wanderCenterX;
			tarY = sin(theta_center2quad - dtheta) * wanderR + wanderCenterY;

			float theta_quad2tar = atan2((tarY-quadrotorPosNED.y),(tarX-quadrotorPosNED.x));
			tarVx = cos(theta_quad2tar) * tarV;
			tarVy = sin(theta_quad2tar) * tarV;

			res.flightCtrlDstx = tarVx;
			res.flightCtrlDsty = tarVy;
			res.flightCtrlDstz = tarVz;
			res.flightFlag = 0x40;


			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0))||guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(res.flightCtrlDstx, res.flightCtrlDsty));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}		
			break;
		}
			
		case TG_TRACK:
		{
			//ROS_INFO("DpTouching: TRACK");
			
			float vx,vy,v_horizon,rel_theta;
			
			if (req.targetPosNEDz > 1.2)
			{
				tarX = req.targetPosNEDx + 0.5 * cos(req.theta);
				tarY = req.targetPosNEDy + 0.5 * sin(req.theta);
				tarZ = req.targetPosNEDz;
				tarVz = k * (tarZ - quadrotorPosNED.z);
				res.flightCtrlDstx = min(0.63*(tarX-quadrotorPosNED.x),0.8);
				res.flightCtrlDsty = min(0.63*(tarY-quadrotorPosNED.y),0.8);
				res.flightCtrlDstz = tarVz;
			}
			else
			{
				
				tarX = req.targetPosNEDx + 0.7 * cos(req.theta) + 0.2;
				tarY = req.targetPosNEDy + 0.7 * sin(req.theta);
				tarZ = req.targetPosNEDz;
				tarVz = k * (tarZ - quadrotorPosNED.z);
				vx = min(0.55*(tarX-quadrotorPosNED.x),0.6);
				vy = min(0.55*(tarY-quadrotorPosNED.y),0.6);
		
				
				/*
				tarX = req.targetPosNEDx + 0.4 * cos(req.theta) - 0.23;
				tarY = req.targetPosNEDy + 0.4 * sin(req.theta);
				tarZ = req.targetPosNEDz;
				tarVz = k * (tarZ - quadrotorPosNED.z);
				rel_theta = atan2(tarY-quadrotorPosNED.y,tarX-quadrotorPosNED.x);
 				vx = 0.42*cos(rel_theta);
				vy = 0.42*sin(rel_theta);
				*/
				res.flightCtrlDstx = vx;
				res.flightCtrlDsty = vy;
				res.flightCtrlDstz = tarVz;
			}
				
			
			

			ROS_INFO_THROTTLE(0.2, "TG_TRACK:quadPosNED=%4.2f,%4.2f,%4.2f,irobotPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,req.targetPosNEDx,req.targetPosNEDy,req.theta);
			
			ROS_INFO_THROTTLE(0.2,"TRACK: dp_x=%4.2f,dp_y=%4.2f",res.flightCtrlDstx,res.flightCtrlDsty);
			res.flightFlag = 0x40;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0)) || guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
		case TG_TRACK_APP:
		{
			ROS_INFO_THROTTLE(1, "TG_TRACK_APP:quadPosNED=%4.2f,%4.2f,%4.2f,irobotPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,req.targetPosNEDx,req.targetPosNEDy,req.theta);
			tarX = req.targetPosNEDx + 0.7 * cos(req.theta);
			tarY = req.targetPosNEDy + 0.7 * sin(req.theta);
			tarZ = req.targetPosNEDz;
			
			res.flightCtrlDstx = min(0.4*(tarX-quadrotorPosNED.x),0.7);//+quadrotorPosNED.x;
			res.flightCtrlDsty = min(0.4*(tarY-quadrotorPosNED.y),0.7);//+quadrotorPosNED.y;
			res.flightCtrlDstz = -0.5;
			//ROS_INFO_THROTTLE(0.2,"TRACK: dp_x=%4.2f,dp_y=%4.2f",res.flightCtrlDstx,res.flightCtrlDsty);
			res.flightFlag = 0x40;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0)) || guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
		case TG_APP_HEAD:
		{
			//ROS_INFO("DpTouching: APPROACH");
			tarX = req.targetPosNEDx + 1.3 * cos(req.theta);
			tarY = req.targetPosNEDy + 1.3 * sin(req.theta);
			tarZ = -0.5;

			ROS_INFO_THROTTLE(0.5, "TG_APPROACH_HEAD: quadPosNED=%4.2f,%4.2f,%4.2f,tarPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,tarX,tarY,tarZ);

			res.flightCtrlDstx = 0.55*(tarX-quadrotorPosNED.x);//+quadrotorPosNED.x;
			res.flightCtrlDsty = 0.55*(tarY-quadrotorPosNED.y);//+quadrotorPosNED.y;
			//res.flightCtrlDstz = 1.2*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = -0.7;
			res.flightFlag = 0x80; 
		
			
			if(number_obstacle > 0)
			{
				for(int i = 0;i != number_obstacle; i++)
				{
					if(obstacle_ranges[i] < 3.0)
					{
						iarc_tf::Velocity srv;
						srv.request.velocityFrame = GROUND;
						repulsiveVec(0) = obstacle_ranges[i];
						repulsiveVec(1) = obstacle_angles[i];
						srv.request.velocityX = -obstacle_ranges[i] * cos(-repulsiveVec(1));           
						srv.request.velocityY = -obstacle_ranges[i] * sin(-repulsiveVec(1));
						if(tf_client.call(srv))
						{
							//repulsiveForce(0) += srv.response.velocityXRes;
							//repulsiveForce(1) += srv.response.velocityYRes;
							obstcleNEDx = srv.response.velocityXRes + quadrotorPosNED.x;
							obstcleNEDy = srv.response.velocityYRes + quadrotorPosNED.y;
							float dis_obstacle_tar = sqrt((obstcleNEDx-tarX)*(obstcleNEDx-tarX)+(obstcleNEDy-tarY)*(obstcleNEDy-tarY));
							if(dis_obstacle_tar<2.5)
							{
								hokuyo_emergency = true;
								break;
							}
							else
								hokuyo_emergency = false;
						}
					}
				}
			}
			else
				hokuyo_emergency =false;
			if(guidance_emergency == true || hokuyo_emergency == true)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = quadrotorPosNED.z;
				res.flightFlag = 0x50;
			}
			
			break;
			
		}
		case TG_APP_TOP:
		{
			//ROS_INFO("DpTouching: APPROACH");
			tarX = req.targetPosNEDx + 0.85 * cos(req.theta);
			tarY = req.targetPosNEDy + 0.85 * sin(req.theta);
			tarZ = -0.5;

			ROS_INFO_THROTTLE(0.5, "TG_APPROACH_TOPP: quadPosNED=%4.2f,%4.2f,%4.2f,tarPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,tarX,tarY,tarZ);

			res.flightCtrlDstx = 0.9*(tarX-quadrotorPosNED.x);//+quadrotorPosNED.x;
			res.flightCtrlDsty = 0.9*(tarY-quadrotorPosNED.y);//+quadrotorPosNED.y;
			//res.flightCtrlDstz = 1.2*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = -0.6;
			res.flightFlag = 0x80; 
		
			
			if(number_obstacle > 0)
			{
				for(int i = 0;i != number_obstacle; i++)
				{
					if(obstacle_ranges[i] < 3.0)
					{
						iarc_tf::Velocity srv;
						srv.request.velocityFrame = GROUND;
						repulsiveVec(0) = obstacle_ranges[i];
						repulsiveVec(1) = obstacle_angles[i];
						srv.request.velocityX = -obstacle_ranges[i] * cos(-repulsiveVec(1));           
						srv.request.velocityY = -obstacle_ranges[i] * sin(-repulsiveVec(1));
						if(tf_client.call(srv))
						{
							//repulsiveForce(0) += srv.response.velocityXRes;
							//repulsiveForce(1) += srv.response.velocityYRes;
							obstcleNEDx = srv.response.velocityXRes + quadrotorPosNED.x;
							obstcleNEDy = srv.response.velocityYRes + quadrotorPosNED.y;
							float dis_obstacle_tar = sqrt((obstcleNEDx-tarX)*(obstcleNEDx-tarX)+(obstcleNEDy-tarY)*(obstcleNEDy-tarY));
							if(dis_obstacle_tar<2.5)
							{
								hokuyo_emergency = true;
								break;
							}
							else
								hokuyo_emergency = false;
						}
					}
				}
			}
			else
				hokuyo_emergency =false;
			if(guidance_emergency == true || hokuyo_emergency == true)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = quadrotorPosNED.z;
				res.flightFlag = 0x50;
			}
			
			break;
			
		}
		case TG_TEST:
		{
			//ROS_INFO("DpTouching: TRACK");
			tarX = req.targetPosNEDx + 0.5 * cos(req.theta);
			tarY = req.targetPosNEDy + 0.5 * sin(req.theta);
			tarZ = 1.6;

			ROS_INFO_THROTTLE(1, "TG_TEST:quadPosNED=%4.2f,%4.2f,%4.2f,irobotPosNED=%4.2f,%4.2f,%4.2f",quadrotorPosNED.x,quadrotorPosNED.y,quadrotorPosNED.z,req.targetPosNEDx,req.targetPosNEDy,req.theta);
			res.flightCtrlDstx = 0.6*(tarX-quadrotorPosNED.x);
			res.flightCtrlDsty = 0.6*(tarY-quadrotorPosNED.y);

			//res.flightCtrlDstz = 1.0*(tarZ-quadrotorPosNED.z)+quadrotorPosNED.z;
			res.flightCtrlDstz = tarVz;
			//ROS_INFO_THROTTLE(0.2,"TRACK: dp_x=%4.2f,dp_y=%4.2f",res.flightCtrlDstx,res.flightCtrlDsty);
			res.flightFlag = 0x80;
			if(((number_obstacle > 0) && (obstacle_ranges[0] < 3.0)) || guidance_emergency)
			{
				doAvoidance(Eigen::Vector2f(tarX - quadrotorPosNED.x, tarY - quadrotorPosNED.y));
				res.flightCtrlDstx = tarVx;
				res.flightCtrlDsty = tarVy;
				res.flightCtrlDstz = tarVz;
				res.flightFlag = 0x40;
			}
			
			break;
		}
	}
	return true;
}


void DpTouching::doAvoidance(Eigen::Vector2f attVec)
{
	setAttVec(attVec);		//set attractiveVec
	calcAttractiveForce();	//calc attractive force
	repulsiveForce.setZero();
	if(number_obstacle > 0 || guidance_emergency)
	{
		calcRepulsiceForce();
	}
	ROS_INFO_THROTTLE(0.2,"doAvoidance: range=%4.2f,ang=%4.2f,repulsiveForce=(%4.2f,%4.2f), attractiveForce=(%4.2f,%4.2f)",obstacle_ranges[0],obstacle_angles[0],repulsiveForce(0),repulsiveForce(1),attractiveForce(0),attractiveForce(1));
	//joinForce(0) = attractiveForce(0) + repulsiveForce(0);
	//joinForce(1) = attractiveForce(1) + repulsiveForce(1);
	joinForce = attractiveForce + repulsiveForce;
	float theta_tar = atan2(joinForce(1), joinForce(0));
	tarVx = cos(theta_tar) * avoidanceV;	//NED
	tarVy = sin(theta_tar) * avoidanceV;
}


void DpTouching::setAttVec(Eigen::Vector2f attVec)
{
	attractiveVec(0) = attVec(0);
	attractiveVec(1) = attVec(1);
}

void DpTouching::setRepVec(Eigen::Vector2f repVec)
{
	repulsiveVec(0) = repVec(0);
	repulsiveVec(1) = repVec(1);
}


void DpTouching::getForce(Eigen::Vector2f &attForce, Eigen::Vector2f &repForce)
{
	attForce(0) = attractiveForce(0);
	attForce(1) = attractiveForce(1);
	repForce(0) = repulsiveForce(0);
	repForce(1) = repulsiveForce(1);
}

void DpTouching::calcAttractiveForce()
{
	float theta_goal = atan2(attractiveVec(1),attractiveVec(0));
	attractiveForce(0) = fattractive * cos(theta_goal);
	attractiveForce(1) = fattractive * sin(theta_goal);
	//ROS_ERROR("%4.2f,%4.2f,%4.2f,%4.2f,%4.2f,%4.2f",attractiveVec(0),attractiveVec(1),theta_goal,fattractive,attractiveForce(0),attractiveForce(1));
}

void DpTouching::calcRepulsiceForce()
{
	repulsiveForce.setZero();
	if(number_obstacle > 0)
	{
		for(int i = 0;i != number_obstacle;i++)
		{
			if(obstacle_ranges[i] < 3.0)
			{
				iarc_tf::Velocity srv;
				srv.request.velocityFrame = GROUND;
				repulsiveVec(0) = obstacle_ranges[i];
				repulsiveVec(1) = obstacle_angles[i];
				srv.request.velocityX = Kr/(repulsiveVec(0)*repulsiveVec(0)) * cos(-repulsiveVec(1));           
				srv.request.velocityY = Kr/(repulsiveVec(0)*repulsiveVec(0)) * sin(-repulsiveVec(1));
				if(tf_client.call(srv))
				{
					repulsiveForce(0) += srv.response.velocityXRes;
					repulsiveForce(1) += srv.response.velocityYRes;
				}
				//ROS_INFO_THROTTLE(0.3,"repulsiveVec=%3.1f,%3.1f,%3.1f,%3.1f,%3.1f",repulsiveVec(0),repulsiveVec(1),Kr/(repulsiveVec(0)*repulsiveVec(0)),repulsiveForce(0),repulsiveForce(1));
				
			}
		}
	}
	if(guidance_emergency)
	{
		iarc_tf::Velocity srv;
		srv.request.velocityFrame = GROUND;
		srv.request.velocityX = -6.5/guidancerang*guidancerang;          
		srv.request.velocityY = 0;
		if(tf_client.call(srv))
		{
			repulsiveForce(0) += srv.response.velocityXRes;
			repulsiveForce(1) += srv.response.velocityYRes;
		}
	}
}

float DpTouching::getLength2f(float x, float y)
{
	return sqrt(x*x+y*y);
}




