#include "irobotDetect.h"
namespace irobot{

IrobotDetect::IrobotDetect(ros::NodeHandle nh):nh_(nh),nh_param("~")
{

	ros::param::get("~Convexlength", Markerlength);
    ros::param::get("~Convexwidth", Markerwidth);
	ros::param::get("~image_subscribeName", image_SubscribeName);
    fx = 240.355225;
	fy = 240.430267;
	u0 = 328.620270;
	v0 = 233.413681;
    Quater_last[0] = 0;
    Quater_last[1] = 0;
    Quater_last[2] = 0;
    Quater_last[3] = 0;
    
	initialize();
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
IrobotDetect::~IrobotDetect()
{
    ROS_INFO("Destroying irobotDetect......");
}

void IrobotDetect::initialize()
{
    image_rect_sub = nh_.subscribe(IrobotDetect::image_SubscribeName,1,&IrobotDetect::image_rect_callback,this); 
    goal_pose_pub = nh_.advertise<irobot_detect::Pose3D>("/goal_detected/goal_pose", 1);
    quadrotorPos_sub = nh_.subscribe("/dji_sdk/local_position", 10, &IrobotDetect::quadrotorPosCallback, this);
    quaternion_sub = nh_.subscribe("dji_sdk/attitude_quaternion", 10, &IrobotDetect::quaternionCallback, this);
}

void IrobotDetect::image_rect_callback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat src_img;
	//std::cout << "fuck weng" << std::endl;
    double T_vec[3] = {0};
    double t=(double)cvGetTickCount(); 
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(src_img);
	
	//cv::namedWindow("irobot detect src img");
    //cv::imshow("irobot detect src img", src_img);
    //cv::waitKey(1);
	
    //std::cout << "in image_call_back" << std::endl;
    float Quaternion[4];
    irobot_detect::Pose3D pose;
    
	 static int nulltime=0;
    static std::vector<std::vector<cv::Point> > Resultstp;
    
    std::vector<std::vector<cv::Point> > Results = irobotshape.shaperecg(src_img, true);
    
    if(Results.size()==0)
    {
	nulltime++;
	if(nulltime < 7)
        {
	  Results=Resultstp;
        }
        else
	{
	    nulltime=0;
	    Resultstp.clear();
	}
    }
    else
    {
	Resultstp=Results;
	nulltime=0;
    }
    
    //std::vector<std::vector<cv::Point> > Results = irobotshape.shaperecg(src_img, true);
    std::vector<std::vector<cv::Point2f> > float_Results, anglept;
    
    irobotNum = Results.size();
    std::cout << irobotNum << std::endl;
    if(Results.size() != 0)
    {
        for(int goal_ID=0;goal_ID < Results.size();goal_ID++)
		{
			irobotImgAngle = irobot_angle(src_img, Results, goal_ID);
			solveIrobotToDronePose(Results, T_vec, goal_ID);
			
			quadrotorPosion[0] = quadrotorPos.x;
			quadrotorPosion[1] = quadrotorPos.y;
			
			irobotState[0] = -T_vec[1];
			irobotState[1] = T_vec[0];
			irobotState[2] = T_vec[2];
			irobotState[3] = irobotImgAngle;
			
			irobot_body_to_ned(irobotState, quadrotorPosion); //最终返回的为小车在local坐标下的坐标
			
			//irobotState[3] = irobotState[3] * 180 / 3.1415927; //
			pose.x.push_back(irobotState[0]);
			pose.y.push_back(irobotState[1]);
			pose.z.push_back(irobotState[2]);
			pose.theta.push_back(irobotState[3]);
		}
    }
    else
    {
		T_vec[0]=0.0;
		T_vec[1]=0.0;
		T_vec[2]=0.0;
		irobotImgAngle = 0.0;
		irobotNum = 0; 
		ROS_INFO("no irobot");
	
		irobotState[0] = -T_vec[1];
		irobotState[1] = T_vec[0];
		irobotState[2] = T_vec[2];
		irobotState[3] = irobotImgAngle;
			
		pose.x.push_back(irobotState[0]);
		pose.y.push_back(irobotState[1]);
		pose.z.push_back(irobotState[2]);
		pose.theta.push_back(irobotState[3]);
    }
	cv::namedWindow("irobot detect",CV_WINDOW_AUTOSIZE);
	cv::imshow("irobot detect", src_img);
	cv::waitKey(1);
    pose.flag = irobotNum;
    goal_pose_pub.publish(pose);
}

void IrobotDetect::quadrotorPosCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
    quadrotorPos.x = msg->x;
    quadrotorPos.y = msg->y;
    quadrotorPos.z = msg->z;
}

void IrobotDetect::quaternionCallback(const dji_sdk::AttitudeQuaternion::ConstPtr &msg)
{
    float yaw;
    Quater[0] = msg->q0;
    Quater[1] = msg->q1;
    Quater[2] = msg->q2;
    Quater[3] = msg->q3;
	if(fabs(Quater[0]*Quater[0]+Quater[1]*Quater[1]+Quater[2]*Quater[2]+Quater[3]*Quater[3] - 1)  > 0.2)
    {
        Quater[0] = Quater_last[0];
	    Quater[1] = Quater_last[1];
	    Quater[2] = Quater_last[2];
	    Quater[3] = Quater_last[3];
	//count_quater = count_quater+1;
	//ROS_INFO("count= %d", count_quater);
    }
  //  std::cout << "I am in ned_to " << std::endl;
    //ROS_INFO("Quater=%4.2f,%4.2f,%4.2f,%4.2f",Quater[0],Quater[1],Quater[2],Quater[3]);
    Quater_last[0] = Quater[0];
    Quater_last[1] = Quater[1];
    Quater_last[2] = Quater[2];
    Quater_last[3] = Quater[3];
    yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
    //ROS_INFO("yaw from irobot detect = %f",yaw);
}
void IrobotDetect::solveIrobotToDronePose(const std::vector< std::vector<cv::Point> > vertex, double T_vec[], int irobot_num)
{
    int goal_ID = irobot_num;
    
    //小车凸字形世界坐标系下的坐标
    cv::Mat Objpoints(4,3,CV_32FC1); 
    Objpoints.at<float>(0,0)=-Markerlength/2;
    Objpoints.at<float>(0,1)=0;
    Objpoints.at<float>(0,2)=0;
    Objpoints.at<float>(1,0)=-Markerlength/2;
    Objpoints.at<float>(1,1)=Markerwidth;
    Objpoints.at<float>(1,2)=0;
    Objpoints.at<float>(2,0)=Markerlength/2;
    Objpoints.at<float>(2,1)=Markerwidth;
    Objpoints.at<float>(2,2)=0;
    Objpoints.at<float>(3,0)=Markerlength/2;
    Objpoints.at<float>(3,1)=0;
    Objpoints.at<float>(3,2)=0;
    
    //相机内参
    cv::Mat CameraMatrix(3,3,CV_32FC1);
    CameraMatrix.at<float>(0,0)=fx;//fx
    CameraMatrix.at<float>(0,1)=0;
    CameraMatrix.at<float>(0,2)=u0;//U0
    CameraMatrix.at<float>(1,0)=0;
    CameraMatrix.at<float>(1,1)=fy;//fy
    CameraMatrix.at<float>(1,2)=v0;//V0
    CameraMatrix.at<float>(2,0)=0;
    CameraMatrix.at<float>(2,1)=0;
    CameraMatrix.at<float>(2,2)=1;
    
    //小车凸字在图像坐标系中的位置
    cv::Mat Imgpoints(4,2,CV_32FC1);

    Imgpoints.at<float>(0,0)=vertex[goal_ID][2].x;
    Imgpoints.at<float>(0,1)=vertex[goal_ID][2].y;
    Imgpoints.at<float>(1,0)=vertex[goal_ID][3].x;
    Imgpoints.at<float>(1,1)=vertex[goal_ID][3].y;
    Imgpoints.at<float>(2,0)=vertex[goal_ID][4].x;
    Imgpoints.at<float>(2,1)=vertex[goal_ID][4].y;
    Imgpoints.at<float>(3,0)=vertex[goal_ID][5].x;
    Imgpoints.at<float>(3,1)=vertex[goal_ID][5].y;
   /* std::cout << "imagepint0 " << Imgpoints.at<float>(0,0) <<std::endl;
    std::cout << "imagepint1 " << Imgpoints.at<float>(0,1) <<std::endl;
    std::cout << "imagepint2 " << Imgpoints.at<float>(1,0) <<std::endl;
    std::cout << "imagepint3 " << Imgpoints.at<float>(1,1) <<std::endl;
    std::cout << "imagepint4 " << Imgpoints.at<float>(2,0) <<std::endl;*/
    /*
RefcornerPointsIdx中每个点代表的位置，Results每一子向量里地面机器人中的八个点也依次如下给出
               7----------0
               |          |
               |          |
   5-----------6          1-----------2
   |                                  |
   |                                  |
   |                                  |
   |                                  |
   4----------------------------------3

*/
    //相机畸变参数，已校正相机，畸变设为0
    cv::Mat distCoeffs(4,1,CV_32FC1);
    for(int i=0;i<4;i++) distCoeffs.at<float>(i,0)=0;
    
    //解pnp
    cv::Mat rvec,tvec;
    cv::Mat Rvec,Tvec;
    cv::solvePnP(Objpoints,Imgpoints,CameraMatrix,distCoeffs,rvec,tvec);
    tvec.convertTo(Tvec,CV_32F);
    rvec.convertTo(Rvec,CV_32F);
    for(int z=0;z<3;z++)
    {
        T_vec[z]=Tvec.at<float>(z,0);
    }
    //std::cout << "T" << T_vec[0] << std::endl;
}

float IrobotDetect::irobot_angle(cv::Mat & img, std::vector<std::vector<cv::Point> > vertex, int irobot_num)
{
    //根据提取的点求解凸字形的中心距来确定主轴方向
    int goal_ID = irobot_num;
    float rect_center = 0;
    float beta,k_angle, goal_angle;   //凸字最小矩形框中心点的y值
    cv::Point2f tempver[4];   //用于保存四个求解凸字上下边的中心点
    //std::vector<std::vector<cv::Point2f> > anglept;
 
    cv::Mat Imgpoints(8,2,CV_32FC1);
    for(int i=0;i < 8;i++)
    {
        Imgpoints.at<float>(i,0)=vertex[goal_ID][i].x;
        Imgpoints.at<float>(i,1)=vertex[goal_ID][i].y;    
    }
    //凸字的中心点y坐标（0号点和3号点的y值求平均）
    rect_center = (Imgpoints.at<float>(0,1) + Imgpoints.at<float>(3,1))/2;
    
/* RefcornerPointsIdx中每个点代表的位置，Results每一子向量里地面机器人中的八个点也依次如下给出
               7----------0
               |          |
               |          |
   5-----------6          1-----------2
   |                                  |
   |                                  |
   |                                  |
   |                                  |
   4----------------------------------3

*/
    tempver[0] = vertex[goal_ID][2];
    tempver[1] = vertex[goal_ID][3];
    tempver[2] = vertex[goal_ID][4];
    tempver[3] = vertex[goal_ID][5];
    
    cv::Moments moment = moments(Imgpoints); //求解特征矩
    cv::Point2f moment_center = cv::Point2f(moment.m10/moment.m00,moment.m01/moment.m00);
    
    //确定坐标系点顺序
    if(moment_center.y < rect_center) //<canonicalbox.rows-moment_center.y
    {
        cv::Point temp0,temp1;
	temp0=tempver[0];
	temp1=tempver[1];
	tempver[0]=tempver[2];
	tempver[1]=tempver[3];
	tempver[2]=temp0;
	tempver[3]=temp1;
    }
    //保存计算方向的点组
    cv::Point p0((tempver[0].x+tempver[3].x)/2,(tempver[0].y+tempver[3].y)/2);
    cv::Point p1((tempver[1].x+tempver[2].x)/2,(tempver[1].y+tempver[2].y)/2);
    
    std::vector<cv::Point2f> angpt;
    angpt.push_back(p0);
    angpt.push_back(p1);

    if((angpt[0].x-angpt[1].x)!=0)
    {
        k_angle=(angpt[0].y-angpt[1].y)/(angpt[0].x-angpt[1].x);
	beta=(atan(k_angle))*360/(2*3.1415926);
    }
    else
    {
        beta=0;
    }
    
    if(angpt[0].x < angpt[1].x)
    {
        goal_angle=beta+90;
    }
    else if(angpt[0].x > angpt[1].x)
    {
        goal_angle = beta-90;
    }
    else
    {
        if(angpt[0].y < angpt[1].y)
	{
	    goal_angle=180;
	}
	else 
	{
	    goal_angle=0;
	}
    }
    tempver[0] = vertex[goal_ID][0];
    tempver[1] = vertex[goal_ID][3];
    tempver[2] = vertex[goal_ID][4];
    tempver[3] = vertex[goal_ID][7];
        if(moment_center.y < rect_center) //<canonicalbox.rows-moment_center.y
    {
        cv::Point temp0,temp1;
	    temp0=tempver[0];
		temp1=tempver[1];
		tempver[0]=tempver[2];
		tempver[1]=tempver[3];
		tempver[2]=temp0;
		tempver[3]=temp1;
    }
    cv::Point p2((tempver[0].x+tempver[3].x)/2,(tempver[0].y+tempver[3].y)/2);
    cv::Point p3((tempver[1].x+tempver[2].x)/2,(tempver[1].y+tempver[2].y)/2);
    
    drawArrow(img,p2, p3,20,15, cv::Scalar(0,255,255),5,8);
    //drawArrow(img,cv::Point((tempver[0].x+tempver[3].x)/2,(tempver[0].y+tempver[3].y)/2), cv::Point((tempver[1].x+tempver[2].x)/2,(tempver[1].y+tempver[2].y)/2),20,15, cv::Scalar(0,255,255),5);		
    return goal_angle;
}

void IrobotDetect::drawArrow(cv::Mat& img, cv::Point pStart, cv::Point pEnd, int len, int alpha,cv::Scalar color, int thickness, int lineType)
 { 
     const double PI = 3.1415926;
     cv::Point arrow;    
     double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
     cv::line(img, pStart, pEnd, color, thickness, lineType);
     arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
     cv::line(img, pEnd, arrow, color, thickness, lineType);
     arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
     arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
     cv::line(img, pEnd, arrow, color, thickness, lineType);
	 printf("drawArrow...\n");
}

void IrobotDetect::Body_to_Global(double Body_arry[], float theta_angle)
{
	double Gtheta_angle, theta_yaw;
	Eigen::VectorXd Global_vector(3),  Result_vector(3), Body_vector(3);
	Body_vector(0) = Body_arry[0];
	Body_vector(1) = Body_arry[1];
	Body_vector(2) = Body_arry[2];
	Eigen::MatrixXd Rotate(3, 3); 
	Rotate(0,0) = Quater[0] * Quater[0] + Quater[1] * Quater[1] - Quater[2] * Quater[2] - Quater[3] * Quater[3];
	Rotate(0,1) = 2 * ( Quater[1] * Quater[2] - Quater[0] * Quater[3]);
	Rotate(0,2) = 2 * ( Quater[1] * Quater[3] + Quater[0] * Quater[2]);
	Rotate(1,0) = 2 * ( Quater[1] * Quater[2] + Quater[0] * Quater[3]);
	Rotate(1,1) = Quater[0] * Quater[0] - Quater[1] * Quater[1] + Quater[2] * Quater[2] - Quater[3] * Quater[3];
	Rotate(1,2) = 2 * ( Quater[2] * Quater[3] - Quater[0] * Quater[1]);
	Rotate(2,0) = 2 * ( Quater[1] * Quater[3] - Quater[0] * Quater[2]);
	Rotate(2,1) = 2 * ( Quater[2] * Quater[3] + Quater[0] * Quater[1]);
	Rotate(2,2) = Quater[0] * Quater[0] - Quater[1] * Quater[1] - Quater[2] * Quater[2] + Quater[3] * Quater[3];
	
	//Rotate = Rotate.inverse();
	Global_vector = Rotate * Body_vector; //转换成NED坐标系下机体相对于小车的x y坐标的增量
	theta_yaw = atan2(2.0 * (Quater[3] * Quater[0] + Quater[1] * Quater[2]) , - 1.0 + 2.0 * (Quater[0] * Quater[0] + Quater[1] * Quater[1]));
	//ROS_INFO_THROTTLE(1,"theta_yaw: %f",theta_yaw);
	Result_vector(0) = Global_vector(0);
	Result_vector(1) = Global_vector(1);
	Gtheta_angle = theta_yaw + (float) theta_angle;
	if(Gtheta_angle<-M_PI) Gtheta_angle += 2*M_PI;
	if(Gtheta_angle>M_PI) Gtheta_angle -= 2*M_PI;
	Result_vector(2) = Gtheta_angle;
	Body_arry[0] = Result_vector(0);
	Body_arry[1] = Result_vector(1);
	Body_arry[2] = 0;
	Body_arry[3] = Result_vector(2);
	//结果向量包含了机体相对于小车的x y坐标的增量和大地坐标系下的theta角
}

void IrobotDetect::irobot_body_to_ned(double irobotstate[], const float quadrotorPos[])
{
    float theta_angle = irobotstate[3]*3.1415926/180.0;
    IrobotDetect::Body_to_Global( irobotstate, theta_angle);
    irobotstate[0] = irobotstate[0] + quadrotorPos[0];
    irobotstate[1] = irobotstate[1] + quadrotorPos[1];
}

}
