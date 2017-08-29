#include "iarc_tf/ned_world_dynamic.h"
#include "math.h"
using namespace std;
enum VelState{NED,GROUND};

//FrameConversion frameconversion;
NedWorldDynamic::NedWorldDynamic():nh("~")
{
    if(!nh.getParam("yaw_origin",sta_yaw))sta_yaw = 0.0;
    //sta_yaw = -0.136;
    //sta_x = 2.5;
    //sta_y = 2.5;
	if(!nh.getParam("sta_x",sta_x))sta_x = 0.0;
	if(!nh.getParam("sta_y",sta_y))sta_y = 0.0;
    dyn_yaw = sta_yaw;
    dyn_x = sta_x;
    dyn_y = sta_y;
	t_n2g_init << dyn_x, dyn_y, 0;
   
    //initialize
    t_b2n.setZero(); 
    t_n2g << sta_x, sta_y,0;
	
    c_b2n_ << cos(-sta_yaw),sin(-sta_yaw),0,0,
			  -sin(-sta_yaw),cos(sta_yaw),0,0,
			  0,     0,        1,           0,
			  0,     0,        0,           1;
    c_n2g(sta_yaw,t_n2g_init);
    
    dyn_local_position_sub = nh.subscribe("/dji_sdk/local_position",10,&NedWorldDynamic::localpositionCallback,this);
    dyn_boundary_output_sub = nh.subscribe("/boundary_output",10,&NedWorldDynamic::boundarydetectCallback,this);
    dyn_local_quaternion_sub = nh.subscribe("/dji_sdk/attitude_quaternion",10,&NedWorldDynamic::localquaternionCallback,this);
    
    ground_position_pub = nh.advertise<geometry_msgs::PointStamped>("/ground_position",10);
}

NedWorldDynamic::~NedWorldDynamic()
{
    ROS_INFO("destroying the ned_world_dynamic node ...");
}
void NedWorldDynamic::localquaternionCallback(const dji_sdk::AttitudeQuaternionConstPtr &msg)
{
	q.w() = msg->q0;
	q.x() = msg->q1;
	q.y() = msg->q2;
	q.z() = msg->q3;
	
	quaternion2mat(q);
	
}
void NedWorldDynamic::localpositionCallback(const dji_sdk::LocalPositionConstPtr& msg)
{

	t_b2n(0) = msg->x;
	t_b2n(1) = msg->y;
	t_b2n(2) = -1*msg->z;
	c_b2n(q,t_b2n);  
	
	Matrix4d c_b2g = c_n2g_*c_b2n_;
	
	ground_position.point.x = c_b2g(0,3);
	ground_position.point.y = c_b2g(1,3);
	ground_position.point.z = -1*c_b2g(2,3);
	ground_position.header.stamp = ros::Time::now();
	
	ground_position_pub.publish(ground_position);
	
    
    //ROS_INFO_STREAM("dyn_local_position_x: "<<dyn_local_position.x);
    
}

void NedWorldDynamic::boundarydetectCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
    dyn_boundary_output.x = msg->point.x;
    dyn_boundary_output.y = msg->point.y;
    dyn_boundary_output.z = msg->point.z;
    //ROS_INFO_STREAM("dyn_boundary_x: "<<dyn_boundary_output.x);
    
    if(dyn_boundary_output.z < 0.5)
    {
		//ROS_INFO("0");
    }
    else if((dyn_boundary_output.z >0.5) && (dyn_boundary_output.z < 1.5) )
    {
	dyn_yaw = sta_yaw;
	dyn_x = sta_x;
	dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*t_b2n(0)-cos(dyn_yaw)*t_b2n(1);
	t_n2g(0)=dyn_x;
	t_n2g(1)=dyn_y;
	t_n2g(2)=0.0;
	c_n2g(dyn_yaw,t_n2g);
	sta_y = dyn_y;
		//ROS_INFO("1");
    }
    else if((dyn_boundary_output.z > 1.5) && (dyn_boundary_output.z < 2.5))
    {
	dyn_yaw = sta_yaw;
	dyn_y = sta_y;
	dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*t_b2n(1)-cos(dyn_yaw)*t_b2n(0);
	t_n2g(0)=dyn_x;
	t_n2g(1)=dyn_y;
	t_n2g(2)=0.0;
	c_n2g(dyn_yaw,t_n2g);
	sta_x = dyn_x;
	//ROS_INFO("2");   
    }
    else{
	dyn_yaw = sta_yaw;
	dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*t_b2n(1)-cos(dyn_yaw)*t_b2n(0);
	dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*t_b2n(0)-cos(dyn_yaw)*t_b2n(1);
	t_n2g(0)=dyn_x;
	t_n2g(1)=dyn_y;
	t_n2g(2)=0.0;
	c_n2g(dyn_yaw,t_n2g);
	sta_x = dyn_x;
	sta_y = dyn_y;
	//ROS_INFO("3");
    }
/**
    switch(int(dyn_boundary_output.z))
    {
	case 0:
	   
	    ROS_ERROR("0");
	    break;
	case 1:
	    dyn_yaw = sta_yaw;
	    dyn_x = sta_x;
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*t_b2n(0)-cos(dyn_yaw)*t_b2n(1);
		t_n2g(0)=dyn_x;
		t_n2g(1)=dyn_y;
		t_n2g(2)=0.0;
		c_n2g(dyn_yaw,t_n2g);
		sta_y = dyn_y;
	    ROS_INFO("1");
	    break;
	case 2:
	    dyn_yaw = sta_yaw;
	    dyn_y = sta_y;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*t_b2n(1)-cos(dyn_yaw)*t_b2n(0);
		    t_n2g(0)=dyn_x;
		t_n2g(1)=dyn_y;
		t_n2g(2)=0.0;
		c_n2g(dyn_yaw,t_n2g);
		sta_x =  dyn_x;
	    ROS_INFO("2");
	    break;
	case 3:
	    dyn_yaw = sta_yaw;
	    dyn_x = dyn_boundary_output.x-sin(dyn_yaw)*t_b2n(1)-cos(dyn_yaw)*t_b2n(0);
	    dyn_y = dyn_boundary_output.y+sin(dyn_yaw)*t_b2n(0)-cos(dyn_yaw)*t_b2n(1);
		t_n2g(0)=dyn_x;
		t_n2g(1)=dyn_y;
		t_n2g(2)=0.0;
		c_n2g(dyn_yaw,t_n2g);
		sta_x = dyn_x;
		sta_y = dyn_y;
	    ROS_INFO("3");
	    break;
    }
**/

}



/*****************************
 *  |xw|   |cos(yaw)  sin(yaw)  tx | |xn|
 *  |yw| = |-sin(yaw) cos(yaw)  ty | |yn|
 *  |1 |   |	0	 0      1  | |1 |
 * 
 *  yaw 为 n系相对于w系转动的角度，顺时针为正
 *  (tx,ty)为n系原点在w系下的坐标点
 * 
****************************/


//TODO : Rotation matrix: R_body2ned
Matrix3d NedWorldDynamic::quaternion2mat(Quaterniond q)
{
    double q0=q.w(),q1=q.x(),q2=q.y(),q3=q.z();
    r_b2n<< q0*q0+q1*q1-q2*q2-q3*q3,2*(q1*q2-q0*q3),2*(q1*q3+q0*q2),
	    2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3,2*(q2*q3-q0*q1),
	    2*(q1*q3-q0*q2),2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3;
    return r_b2n;
}
//TODO:R_n_g; framen相对于frameg转动的角度，顺时针为正
Matrix3d NedWorldDynamic::yaw2mat(double yaw)
{
    
    double cy = cos(yaw);
    double sy = sin(yaw);
    
    r_n2g<< cy,  sy, 0,
	    -sy, cy,  0,
	    0,   0,  1;
	
    return r_n2g;
}
Matrix4d NedWorldDynamic::c_n2g(double yaw, Vector3d t_n2g)
{
    Matrix3d r;
    r = yaw2mat(yaw);
    c_n2g_ << r(0,0),r(0,1),r(0,2),t_n2g(0),
	      r(1,0),r(1,1),r(1,2),t_n2g(1),
	      r(2,0),r(2,1),r(2,2),t_n2g(2),
	         0  ,    0 ,    0 ,    1 ;
    return c_n2g_;
    
}

Matrix4d NedWorldDynamic::c_b2n(Quaterniond q, Vector3d t_b2n)
{
    
    Matrix3d r;
    r = quaternion2mat(q);
    c_b2n_ << r(0,0),r(0,1),r(0,2),t_b2n(0),
	    r(1,0),r(1,1),r(1,2),t_b2n(1),
	    r(2,0),r(2,1),r(2,2),t_b2n(2),
	       0  ,   0  ,   0  ,   1  ;
    return c_b2n_;
}



