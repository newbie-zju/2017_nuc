#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <boost/concept_check.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point32.h>
#include <dji_sdk/LocalPosition.h>
#include <iarc_mission/TG.h>
#include <goal_detected/Pose3D.h>
using namespace DJI::onboardSDK;
geometry_msgs::Point32 flight_goal;
dji_sdk::LocalPosition quadrotorPos;
goal_detected::Pose3D irobotPosNED;

float test_z = 1.17;
	
static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > ------------------------+\n");
	printf("| [a] SDK Version Query         | [s] Virtual RC Test             |\n");
	printf("| [b] Request Control           | [t] Set Sync Flag Test          |\n");	
	printf("| [c] Release Control           | [u] Set Msg Frequency Test      |\n");	
	printf("| [d] Takeoff                   | [v] Waypoint Mission Upload     |\n");	
	printf("| [e] Landing                   | [w] Hotpoint Mission Upload     |\n");	
	printf("| [f] Go Home                   | [x] Followme Mission Upload     |\n");	
	printf("| [g] Gimbal Control Sample     | [y] Mission Start               |\n");	
	printf("| [h] Attitude Control Sample   | [z] Mission Pause               |\n");	
	printf("| [i] Draw Circle Sample        | [1] Mission Resume              |\n");	
	printf("| [j] Draw Square Sample        | [2] Mission Cancel              |\n");	
	printf("| [k] Take a Picture            | [3] Mission Waypoint Download   |\n");	
	printf("| [l] Cruise       | [4] Mission Waypoint Set Speed  |\n");	 
	printf("| [m] IARC Test         | [5] Mission Waypoint Get Speed  |\n");	
	printf("| [n] Draw Square Test     | [6] Mission Hotpoint Set Speed  |\n");	
	printf("| [o] Global Navigation Test    | [7] Mission Hotpoint Set Radius |\n");	
	printf("| [p] Waypoint Navigation Test  | [8] Mission Hotpoint Reset Yaw  |\n");	
	printf("| [q] Arm the Drone             | [9] Mission Followme Set Target |\n");	
	printf("| [r] Disarm the Drone          | [0] Mission Hotpoint Download   |\n");
    printf("+-----------------------------------------------------------------+\n");
    printf("input a/b/c etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
    printf("input: ");
}
void flight_ctrl_dst_callback(const geometry_msgs::Point32ConstPtr &msg)
{
	flight_goal.x = msg->x;
	flight_goal.y = msg->y;
	flight_goal.z = msg->z;
}

void quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadrotorPos.x = msg->x;
	quadrotorPos.y = msg->y;
	quadrotorPos.z = msg->z;
}
void irobot_pos_callback(const goal_detected::Pose3DConstPtr& msg)
{
	irobotPosNED.x = msg->x;
	irobotPosNED.y = msg->y;
	irobotPosNED.z = msg->z;
	irobotPosNED.theta = msg->theta;
	irobotPosNED.flag = msg->flag;
}
int main(int argc, char **argv)
{
    int main_operate_code = 0;
    int temp32;
    bool valid_flag = false;
    bool err_flag = false;
	bool takeoff_flag = 0;
    ros::init(argc, argv, "sdk_client");
    ROS_INFO("sdk_service_client_test");
    ros::NodeHandle nh;
    ros::Subscriber flight_ctrl_dst_sub = nh.subscribe("/flight_ctrl_dst", 10, flight_ctrl_dst_callback);
    ros::Subscriber quadrotorPos_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorPosCallback);
    ros::Subscriber irobot_pos_sub = nh.subscribe("/goal_detected/goal_pose", 10, irobot_pos_callback);
    ros::ServiceClient TG_client = nh.serviceClient<iarc_mission::TG>("/TG/TG_service");
    DJIDrone* drone = new DJIDrone(nh);

	//virtual RC test data
	uint32_t virtual_rc_data[16];
	//set frequency test data
	uint8_t msg_frequency_data[16] = {1,2,3,4,3,2,1,2,3,4,3,2,1,2,3,4};
	//waypoint action test data
    dji_sdk::WaypointList newWaypointList;
    dji_sdk::Waypoint waypoint0;
    dji_sdk::Waypoint waypoint1;
    dji_sdk::Waypoint waypoint2;
    dji_sdk::Waypoint waypoint3;
    dji_sdk::Waypoint waypoint4;

	//groundstation test data
	dji_sdk::MissionWaypointTask waypoint_task;
	dji_sdk::MissionWaypoint 	 waypoint;
	dji_sdk::MissionHotpointTask hotpoint_task;
	dji_sdk::MissionFollowmeTask followme_task;
	dji_sdk::MissionFollowmeTarget followme_target;
	
    Display_Main_Menu();
    while(ros::ok())
    {
	ros::spinOnce();
        temp32 = getchar();
        if(temp32 != 10)
        {
            if(valid_flag == false)
            {
                main_operate_code = temp32;
                valid_flag = true;
            }
            else
            {
                err_flag = true;
            }
            continue;
        }
        else
        {
            if(err_flag == true)
            {
                printf("input: ERROR\n");
                Display_Main_Menu();
                err_flag = valid_flag = false;
                continue;
            }
        }
        switch(main_operate_code)
        {
		case 'a':
			/* SDK version query*/
			drone->check_version();
			break;
            case 'b':
                /* request control ability*/
                drone->request_sdk_permission_control();
                break;
            case 'c':
                /* release control ability*/
                drone->release_sdk_permission_control();
                break;
            case 'd':
                /* take off */
				ros::spinOnce();
				drone->takeoff();
				
// 				  for(int i = 0; i < 300; i ++)
// 				{
// 					usleep(20000);
// 				}			
				for(int i = 0; i < 300; i ++) 
				{
					drone->local_position_control(quadrotorPos.x, quadrotorPos.y, 1.8, 0 );
					usleep(20000);
				}
				
                break;
            case 'e':
                /* landing*/
                if(drone->landing())ROS_INFO("LANDING FINISHED......");
                break;
            case 'f':
                /* go home*/
                drone->gohome();
                break;
            case 'g':
                /*gimbal test*/

                drone->gimbal_angle_control(0, 0, 1800, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 0, -1800, 20);
                sleep(2);
                drone->gimbal_angle_control(300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(-300, 0, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, 300, 0, 20);
                sleep(2);
                drone->gimbal_angle_control(0, -300, 0, 20);
                sleep(2);
                drone->gimbal_speed_control(100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(-100, 0, 0);
                sleep(2);
                drone->gimbal_speed_control(0, 0, 200);
                sleep(2);
                drone->gimbal_speed_control(0, 0, -200);
                sleep(2);
                drone->gimbal_speed_control(0, 200, 0);
                sleep(2);
                drone->gimbal_speed_control(0, -200, 0);
                sleep(2);
                drone->gimbal_angle_control(0, 0, 0, 20);
                break;

            case 'h':
                /* attitude control sample*/
                drone->takeoff();
                sleep(8);


                for(int i = 0; i < 100; i ++)
                {
                    if(i < 90)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, -2, 0, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, -2, 0, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, 0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0x40, 0, 0, -0.5, 0);
                    else
                        drone->attitude_control(0x40, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, 90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                for(int i = 0; i < 200; i ++)
                {
                    if(i < 180)
                        drone->attitude_control(0xA, 0, 0, 0, -90);
                    else
                        drone->attitude_control(0xA, 0, 0, 0, 0);
                    usleep(20000);
                }
                sleep(1);

                drone->landing();

                break;

            case 'i':
                /*draw circle sample*/
                static float time = 0;
                static float R = 2;
                static float V = 2;
                static float vx;
                static float vy;
                /* start to draw circle */
                for(int i = 0; i < 300; i ++)
                {
                    vx = V * sin((V/R)*time/50.0f);
                    vy = V * cos((V/R)*time/50.0f);
        
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            vx, vy, 0, 0 );
                    usleep(20000);
                    time++;
                }
                break;

            case 'j':
                /*draw square sample*/
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0, 3, 0, 0 );
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 3, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            3, 0, 0, 0);
                    usleep(20000);
                }
                for(int i = 0;i < 60;i++)
                {
                    drone->attitude_control( Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE,
                            0, 0, 0, 0);
                    usleep(20000);
                }
                break;
            case 'k':
                /*take a picture*/
                drone->take_picture();
                break;
            case 'l':
                /*start video*/
               // drone->start_video();
               while(ros::ok())
		    {
			    ros::spinOnce();
			    

					while(ros::ok()) 
					{
						ros::spinOnce();
						iarc_mission::TG TG_srv;
						TG_srv.request.quadrotorState = 1;
						TG_srv.request.irobotPosNEDx = 0;
						TG_srv.request.irobotPosNEDy = 0;
						TG_srv.request.irobotPosNEDz = 0;
						TG_srv.request.theta = 0;
						if(!TG_client.call(TG_srv))
							ROS_INFO("IARCMission TG_client.call failled......");
						else
						{
							
							drone->takeOffandLandTest(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, TG_srv.response.flightCtrlDstz, 0);	//TODO: YAW = 0?
							ROS_INFO("vx = %f vy = %f z = %f ",TG_srv.response.flightCtrlDstx,TG_srv.response.flightCtrlDsty,TG_srv.response.flightCtrlDstz );
							usleep(20000);
						}
					}
					break;
					     
		    } 
                
                
                break;
            case 'm': 
	    {
		    while(ros::ok())
		    {
			    ros::spinOnce();
			    if(irobotPosNED.flag > 0)
			    {

					while(ros::ok()) 
					{
						ros::spinOnce();
						iarc_mission::TG TG_srv;
						TG_srv.request.quadrotorState = 2;
						TG_srv.request.irobotPosNEDx = irobotPosNED.x;
						TG_srv.request.irobotPosNEDy = irobotPosNED.y;
						TG_srv.request.irobotPosNEDz = irobotPosNED.z;
						TG_srv.request.theta = irobotPosNED.theta;
						if(!TG_client.call(TG_srv))
							ROS_INFO("IARCMission TG_client.call failled......");
						else
						{
							//ROS_INFO("%6.3f, %6.3f, %6.3f",TG_srv.response.flightCtrlDstx,TG_srv.response.flightCtrlDsty,TG_srv.response.flightCtrlDstz);
							drone->local_position_control(TG_srv.response.flightCtrlDstx, TG_srv.response.flightCtrlDsty, 1.6, 0);	//TODO: YAW = 0?
							if ( quadrotorPos.z < 0.3 )
							{
								ROS_INFO("client: begin landing...");
								drone->landing();
								break;
							}
							usleep(20000);
						}
					}
					break;
				}	     
		    }
		    break;
 	    }
		
            case 'n':  //action
                /* Local Navi Test */
                //drone->local_position_navigation_send_request(flight_goal.x, flight_goal.y, flight_goal.z, 0);
				while(ros::ok())
				{
					ros::spinOnce();
					while(1)
					{
						ros::spinOnce();
						drone->local_position_control(1.0,0.0,1.6,0);
						usleep(20000);
						
						if((fabs(quadrotorPos.x-1.0)<0.1)&&(fabs(quadrotorPos.y-0.0)<0.1)&&(fabs(quadrotorPos.z-1.6)<0.1))
						{ROS_INFO("JUMP1");
							test_z = fabs(quadrotorPos.x-1.0);
							ROS_INFO("test_z = %6.3f",test_z);
							sleep(2);
							break;
						}
						ROS_INFO("stage1");
					}
					
					while(1)
					{
						ros::spinOnce();
						drone->local_position_control(1.0,-1.0,1.6,0);
						usleep(20000);
						
						if((fabs(quadrotorPos.x-1.0)<0.1)&&(fabs(quadrotorPos.y+1.0)<0.1)&&(fabs(quadrotorPos.z-1.6)<0.1))
						{ROS_INFO("JUMP2");
							sleep(2);
							break;
						}
						ROS_INFO("stage2");
					}
					
					while(1)
					{
						ros::spinOnce();
						drone->local_position_control(0.0,-1.0,1.6,0);
						usleep(20000);
						
						if((fabs(quadrotorPos.x-0.0)<0.1)&&(fabs(quadrotorPos.y+1.0)<0.1)&&(fabs(quadrotorPos.z-1.6)<0.1))
						{ROS_INFO("JUMP3");
							sleep(2);
							break;
							
						}
						ROS_INFO("stage3");
					}
					
					while(1)
					{
						ros::spinOnce();
						drone->local_position_control(0.0,0.0,1.6,0);
						usleep(20000);
						
						if((fabs(quadrotorPos.x-0.0)<0.1)&&(fabs(quadrotorPos.y-0.0)<0.1)&&(fabs(quadrotorPos.z-1.6)<0.1))
						{ROS_INFO("JUMP4");
							sleep(2);
							break;
						}
						ROS_INFO("stage4");
					}
				
					ROS_INFO("JUMP");
					
				}			
                break;
            case 'o':
                /* GPS Navi Test */
                //drone->global_position_navigation_send_request(22.535, 113.95, 100);
                while(ros::ok())
				{   
				 ros::spinOnce();
					if(takeoff_flag == 0)
					{	
						drone->drone_arm();
						while((ros::ok()) && (quadrotorPos.z<0.8))
						{
							ros::spinOnce();
							drone->takeOffandLandTest(0,0,0.8,0);
							usleep(20000);
							ROS_INFO("takeoff stage1, position.z = %6.3f",quadrotorPos.z);
						}
						for(int i = 0; i < 300; i ++) 
						{
							drone->local_position_control(quadrotorPos.x, quadrotorPos.y, 1.7, 0 );
							usleep(20000);
							ROS_INFO("takeoff stage2");
						}
					
						takeoff_flag = 1;
					}
					if(takeoff_flag ==1)
					{
						while((ros::ok()) && (quadrotorPos.z>0.2))
						{
						 ros::spinOnce();
 					     drone->takeOffandLandTest(0,0,-0.5,0);
						 usleep(20000);
						 ROS_INFO("landing");
						}
						for(int i = 0; i < 100; i ++) 
						{
							drone->takeOffandLandTest(0, 0, -0.3, 0 );
							usleep(20000);
							ROS_INFO("landing sleep");
						}
						//sleep(3);
						
						takeoff_flag = 0;
					}
				}
                
				
				break;
            case 'p':
                /* Waypoint List Navi Test */
                {
                    waypoint0.latitude = 22.535;
                    waypoint0.longitude = 113.95;
                    waypoint0.altitude = 100;
                    waypoint0.staytime = 5;
                    waypoint0.heading = 0;
                }
                newWaypointList.waypoint_list.push_back(waypoint0);

                {
                    waypoint1.latitude = 22.535;
                    waypoint1.longitude = 113.96;
                    waypoint1.altitude = 100;
                    waypoint1.staytime = 0;
                    waypoint1.heading = 90;
                }
                newWaypointList.waypoint_list.push_back(waypoint1);

                {
                    waypoint2.latitude = 22.545;
                    waypoint2.longitude = 113.96;
                    waypoint2.altitude = 100;
                    waypoint2.staytime = 4;
                    waypoint2.heading = -90;
                }
                newWaypointList.waypoint_list.push_back(waypoint2);

                {
                    waypoint3.latitude = 22.545;
                    waypoint3.longitude = 113.96;
                    waypoint3.altitude = 10;
                    waypoint3.staytime = 2;
                    waypoint3.heading = 180;
                }
                newWaypointList.waypoint_list.push_back(waypoint3);

                {
                    waypoint4.latitude = 22.525;
                    waypoint4.longitude = 113.93;
                    waypoint4.altitude = 50;
                    waypoint4.staytime = 0;
                    waypoint4.heading = -180;
                }
                newWaypointList.waypoint_list.push_back(waypoint4);

                drone->waypoint_navigation_send_request(newWaypointList);
                break;
			case 'q':
				//drone arm
				drone->drone_arm();
				/*while(ros::ok() && quadrotorPos.z < 1.5)
				{
					ros::spinOnce();
					drone->local_position_control(quadrotorPos.x, quadrotorPos.y, 1.8, 0);
					usleep(20000);
					ROS_INFO("fuck");
				}*/
                break;
			case 'r':
				//drone disarm
				drone->drone_disarm();
                break;
			case 's':
				//virtual rc test 1: arm & disarm
				drone->virtual_rc_enable();
				usleep(20000);

				virtual_rc_data[0] = 1024-660;	//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024-660;	//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024-660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024+660;	//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1684;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for (int i = 0; i < 100; i++){
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}

				//virtual rc test 2: yaw 
				drone->virtual_rc_enable();
				virtual_rc_data[0] = 1024;		//0-> roll     	[1024-660,1024+660] 
				virtual_rc_data[1] = 1024;		//1-> pitch    	[1024-660,1024+660]
				virtual_rc_data[2] = 1024+660;	//2-> throttle 	[1024-660,1024+660]
				virtual_rc_data[3] = 1024;		//3-> yaw      	[1024-660,1024+660]
				virtual_rc_data[4] = 1324;	 	//4-> gear		{1684(UP), 1324(DOWN)}
				virtual_rc_data[6] = 1552;    	//6-> mode     	{1552(P), 1024(A), 496(F)}

				for(int i = 0; i < 100; i++) {
					drone->virtual_rc_control(virtual_rc_data);
					usleep(20000);
				}
				drone->virtual_rc_disable();
				break;
			case 't':
				//sync flag
				drone->sync_flag_control(1);
				break;
			case 'u':
				//set msg frequency
				drone->set_message_frequency(msg_frequency_data);
				break;

			case 'v':
				//mission waypoint upload
				waypoint_task.velocity_range = 10;
				waypoint_task.idle_velocity = 3;
				waypoint_task.action_on_finish = 0;
				waypoint_task.mission_exec_times = 1;
				waypoint_task.yaw_mode = 4;
				waypoint_task.trace_mode = 0;
				waypoint_task.action_on_rc_lost = 0;
				waypoint_task.gimbal_pitch_mode = 0;

				waypoint.latitude = 22.540091;
				waypoint.longitude = 113.946593;
				waypoint.altitude = 100;
				waypoint.damping_distance = 0;
				waypoint.target_yaw = 0;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				/*
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				*/

				waypoint_task.mission_waypoint.push_back(waypoint);

				waypoint.latitude = 22.540015;
				waypoint.longitude = 113.94659;
				waypoint.altitude = 120;
				waypoint.damping_distance = 2;
				waypoint.target_yaw = 180;
				waypoint.target_gimbal_pitch = 0;
				waypoint.turn_mode = 0;
				waypoint.has_action = 0;
				/*
				waypoint.action_time_limit = 10;
				waypoint.waypoint_action.action_repeat = 1;
				waypoint.waypoint_action.command_list[0] = 1;
				waypoint.waypoint_action.command_list[1] = 1;
				waypoint.waypoint_action.command_parameter[0] = 1;
				waypoint.waypoint_action.command_parameter[1] = 1;
				*/

				waypoint_task.mission_waypoint.push_back(waypoint);

				drone->mission_waypoint_upload(waypoint_task);
				break;
			case 'w':
				//mission hotpoint upload
				hotpoint_task.latitude = 22.540091;
				hotpoint_task.longitude = 113.946593;
				hotpoint_task.altitude = 20;
				hotpoint_task.radius = 10;
				hotpoint_task.angular_speed = 10;
				hotpoint_task.is_clockwise = 0;
				hotpoint_task.start_point = 0;
				hotpoint_task.yaw_mode = 0;

				drone->mission_hotpoint_upload(hotpoint_task);
				break;
			case 'x':
				//mission followme upload
				followme_task.mode = 0;
				followme_task.yaw_mode = 0;
				followme_task.initial_latitude = 23.540091;
				followme_task.initial_longitude = 113.946593;
				followme_task.initial_altitude = 10;
				followme_task.sensitivity = 1;

				drone->mission_followme_upload(followme_task);
				break;
			case 'y':
				//mission start
				drone->mission_start();
				break;
			case 'z':
				//mission pause
				drone->mission_pause();
				break;
			case '1':
				//mission resume
				drone->mission_resume();
				break;
			case '2':
				//mission cancel
				drone->mission_cancel();
				break;
			case '3':
				//waypoint mission download
				waypoint_task = drone->mission_waypoint_download();
				break;
			case '4':
				//mission waypoint set speed
				drone->mission_waypoint_set_speed((float)5);
				break;
			case '5':
				//mission waypoint get speed
				printf("%f", drone->mission_waypoint_get_speed());
				break;
			case '6':
				//mission hotpoint set speed
				drone->mission_hotpoint_set_speed((float)5,(uint8_t)1);
				break;
			case '7':
				//mission hotpoint set radius
				drone->mission_hotpoint_set_radius((float)5);
				break;
			case '8':
				//mission hotpoint reset yaw
				drone->mission_hotpoint_reset_yaw();
				break;
			case '9':
				//mission followme update target
				for (int i = 0; i < 20; i++)
				{
					followme_target.latitude = 22.540091 + i*0.000001;
					followme_target.longitude = 113.946593 + i*0.000001;
					followme_target.altitude = 100;
					drone->mission_followme_update_target(followme_target);
					usleep(20000);
				}
				break;
			case '0':
				hotpoint_task = drone->mission_hotpoint_download();

            default:
                break;
        }
        main_operate_code = -1;
        err_flag = valid_flag = false;
        Display_Main_Menu();
    }
    return 0;
}
