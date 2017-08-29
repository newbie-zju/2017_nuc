#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/objdetect/objdetect.hpp>  
#include <opencv2/ml/ml.hpp>  
#include <opencv2/gpu/gpu.hpp>  
#include "some_method.h"
#include "parameter.h"

//#include <dji_sdk/LocalPosition.h> //dji_sdk
#include <multirobot_detect_iarc/RobotCamPos.h>

using namespace cv;
using namespace std;


class MultirobotDetect
{
public:
  //node
  ros::NodeHandle nh_;
  ros::NodeHandle nh_image_param;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  string subscribed_topic;
  ros::Publisher msg_pub;
  //StateEstimate
  PositionEstimate pe;
  VelocityEstimate ve;
  multirobot_detect_iarc::RobotCamPos rcp;
  int rob_pub_num, obs_pub_num;
  //svm
  MySVM svm_detect;
  MySVM svm_classify;
  //dimension of HOG descriptor: 
    //[(window_width-block_width)/block_stride_width+1]*[(window_height-block_height)/block_stride_height+1]*bin_number*(block_width/cell_width)*(block_height/cell_height)
  int descriptor_dim_detect;
  int descriptor_dim_classify;
  int support_vector_num_detect;//number of vector, not the dimention of vector
  int support_vector_num_classify;
  Mat alpha_mat_detect;
  Mat support_vector_mat_detect;
  Mat result_mat_detect;
  //HOG descriptor
  //gpu::HOGDescriptor HOG_descriptor_detect;//gpu
  HOGDescriptor HOG_descriptor_detect;
  HOGDescriptor HOG_descriptor_classify;
  //video
  //string INPUT_VIDEO_WINDOW_NAME;
  string RESULT_VIDEO_WINDOW_NAME;
  bool show_video_flag;
  bool save_result_video_flag;
  double video_rate;
  double image_hight;
  double image_width;
  double video_delay;
  VideoWriter result_video;
  string result_video_file_name;
  //frame
  int frame_num;
  double this_time, last_time, dt;
  Mat src_3,src_4,dst_3;
  gpu::GpuMat src_GPU;//gpu
  vector<Rect> location_detect;
  vector<RobotMessage> detect_message, detect_rob_message, detect_obs_message, detect_rob_message_last, detect_obs_message_last;
  vector<float> scores;
  FilterAdd fa4detect;
  vector<float> result_classify;
  
  MultirobotDetect(PositionEstimate pe0):
  it_(nh_),//intial it_
  nh_image_param("~")
  {
    //node
    if(!nh_image_param.getParam("subscribed_topic", subscribed_topic))subscribed_topic = "/dji_sdk/image_raw";
    // Subscrive to input video feed from "/dji_sdk/image_raw" topic, imageCb is the callback function
    image_sub_ = it_.subscribe(subscribed_topic, 1, &MultirobotDetect::imageCb, this);
    msg_pub  = nh_.advertise<multirobot_detect_iarc::RobotCamPos>("/robot_cam_position", 10);
    //StateEstimate
    pe = pe0;
    ve = VelocityEstimate(pe);
    rob_pub_num = sizeof(rcp.rob_cam_pos_x) / sizeof(rcp.rob_cam_pos_x[0]);
    obs_pub_num = sizeof(rcp.obs_cam_pos_x) / sizeof(rcp.obs_cam_pos_x[0]);
    //svm
    svm_detect.load(DetectSvmName);
    svm_classify.load(ClassifySvmName);
    descriptor_dim_detect = svm_detect.get_var_count();
    descriptor_dim_classify = svm_classify.get_var_count();
    support_vector_num_detect = svm_detect.get_support_vector_count();
    support_vector_num_classify = svm_classify.get_support_vector_count();
    alpha_mat_detect = Mat::zeros(1, support_vector_num_detect, CV_32FC1);
    support_vector_mat_detect = Mat::zeros(support_vector_num_detect, descriptor_dim_detect, CV_32FC1);
    result_mat_detect = Mat::zeros(1, descriptor_dim_detect, CV_32FC1);
    //HOG descriptor
    //HOG_descriptor_detect = gpu::HOGDescriptor(WinSizeDetect,BlockSizeDetect,BlockStrideDetect,CellSizeDetect,NbinsDetect,1,-1,0,0.2,false,10);//gpu
    HOG_descriptor_detect = HOGDescriptor(WinSizeDetect,BlockSizeDetect,BlockStrideDetect,CellSizeDetect,NbinsDetect,1,-1,0,0.2,false,12);
    HOG_descriptor_classify = HOGDescriptor(WinSizeClassify,BlockSizeClassify,BlockStrideClassify,CellSizeClassify,NbinsClassify);
    for(int i=0; i<support_vector_num_detect; i++) 
    {
      const float * support_vector_detect = svm_detect.get_support_vector(i);
      for(int j=0; j<descriptor_dim_detect; j++)  
        support_vector_mat_detect.at<float>(i,j) = support_vector_detect[j];  
    }
    double * alpha_detect = svm_detect.get_alpha_vector();
    for(int i=0; i<support_vector_num_detect; i++)
      alpha_mat_detect.at<float>(0,i) = alpha_detect[i];  
    result_mat_detect = -1 * alpha_mat_detect * support_vector_mat_detect;
    vector<float> detector_detect;
    for(int i=0; i<descriptor_dim_detect; i++)
      detector_detect.push_back(result_mat_detect.at<float>(0,i)); 
    detector_detect.push_back(svm_detect.get_rho());//add rho
    
    cout<<"dimension of svm detector for HOG detect(w+b):"<<detector_detect.size()<<endl;
    HOG_descriptor_detect.setSVMDetector(detector_detect);
    //video
    if(!nh_image_param.getParam("show_video_flag", show_video_flag))show_video_flag = true;
    if(show_video_flag)
    {
    //INPUT_VIDEO_WINDOW_NAME="input video";
    RESULT_VIDEO_WINDOW_NAME="detect result";
    //namedWindow(INPUT_VIDEO_WINDOW_NAME);
    namedWindow(RESULT_VIDEO_WINDOW_NAME);
    }
    if(!nh_image_param.getParam("save_result_video_flag", save_result_video_flag))save_result_video_flag = false;
    if(!nh_image_param.getParam("rate", video_rate))video_rate = 5.0;
    video_delay = 1000/video_rate;
    if(!nh_image_param.getParam("result_video_file_name", result_video_file_name))result_video_file_name = "/home/ubuntu/ros_my_workspace/src/multirobot_detect/result/a544.avi";
    //frame
    frame_num = 1;
    this_time = 0;
    last_time = 0;
    dt = 1;
  }
  
  ~MultirobotDetect()
  {
    destroyAllWindows();
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv_ptr->image.copyTo(src_3);
    //cout<<"rows"<<src_3.rows<<endl;
    //cout<<"cols"<<src_3.cols<<endl;
    
    if(frame_num == 1)
    {
      image_hight = src_3.rows;
      image_width = src_3.cols;
      result_video = VideoWriter(result_video_file_name, CV_FOURCC('M', 'J', 'P', 'G'), video_rate, Size(image_width, image_hight));
    }
    
    //frame
    //cout<<"frame_num: "<<frame_num<<endl;
    frame_num++;
    
    src_3.copyTo(dst_3);
    //cvtColor(src_3,src_4,CV_BGR2BGRA);//gpu
    //src_GPU.upload(src_4);//gpu
    
    this_time = ros::Time::now().toSec();
    dt = this_time - last_time;
    //cout<<"dt:"<<dt<<endl;
    
    //reset
    resetState();
    
    //detect
    //HOG_descriptor_detect.detectMultiScale(src_GPU, location_detect, HitThreshold, WinStride, Size(), DetScale, 0.2, true);//gpu
    HOG_descriptor_detect.detectMultiScale(src_3, location_detect, HitThreshold, WinStride, Size(0,0), DetScale, 2.0, false);
    location_detect = resize_boxes(location_detect, src_3, detect_resize_rate);
    
    //Non-maximum suppression
    scores = get_scores(src_3, location_detect, svm_detect, descriptor_dim_detect, WinSizeDetect, HOG_descriptor_detect);
    location_detect = non_maximum_suppression(location_detect, scores, SuppressionRate);
    
    //filter and estimate
    detect_message = fa4detect.run(src_3, location_detect, BBOverlapRate);
    
    //classfy
    for(int i=0; i<detect_message.size(); i++)  
    {
      //cout<<"width:"<<detect_message[i].location_image.width<<"  height:"<<detect_message[i].location_image.height<<endl;
      //cout<<"label:"<<detect_message[i].label<<endl;
      vector<float> descriptor_classify;
      Mat descriptor_mat_classify(1, descriptor_dim_classify, CV_32FC1);
      Mat src_classify;
      
      resize(src_3(detect_message[i].location_image),src_classify,WinSizeClassify);
      HOG_descriptor_classify.compute(src_classify,descriptor_classify);
      for(int j=0; j<descriptor_dim_classify; j++)  
	descriptor_mat_classify.at<float>(0,j) = descriptor_classify[j];
      float temp_result_classify = svm_classify.predict(descriptor_mat_classify);
      result_classify.push_back(temp_result_classify);
      
      //label the robot for (save video, show video, save set)
      if(save_result_video_flag | show_video_flag)
      {
	if (temp_result_classify == 1)//irobot
	{
	  detect_rob_message.push_back(detect_message[i]);
	  rectangle(dst_3, detect_message[i].location_image, CV_RGB(0,0,255), 3);
	} 
	else if (temp_result_classify == 2)//obstacle
	{
	  detect_obs_message.push_back(detect_message[i]);
	  rectangle(dst_3, detect_message[i].location_image, CV_RGB(0,255,0), 3);
	}
	else if (temp_result_classify ==3)//background
	{
	  //rectangle(dst_3, detect_message[i].location_image, Scalar(0,0,255), 3);
	}
	else//other
	{
	  rectangle(dst_3, detect_message[i].location_image, Scalar(255,255,255), 3);
	}
      }
    }
    
    //set RobotCamPos
    for(int i = 0; i < detect_rob_message.size(); i++)//irobot
    {
      if(rcp.rob_num < rob_pub_num)
      {
	for(int j = 0; j < detect_rob_message_last.size(); j++)
	{
	  if(rcp.rob_num < rob_pub_num && detect_rob_message[i].label == detect_rob_message_last[j].label)
	  {
	    rcp.exist_rob_flag = true;
	    //estimate position
	    pe.getEstimate(detect_rob_message[i].location_image.x, detect_rob_message[i].location_image.y, detect_rob_message[i].location_image.width, detect_rob_message[i].location_image.height, double(src_3.cols), double(src_3.rows));
	    rcp.rob_cam_pos_x[rcp.rob_num] = pe.robot_mycam_x;
	    rcp.rob_cam_pos_y[rcp.rob_num] = pe.robot_mycam_y;
	    //estimate velocity
	    ve.computeVelocity(detect_rob_message[i], detect_rob_message_last[j], src_3, dt);
	    rcp.rob_cam_vel_x[rcp.rob_num] = ve.velocity_mycam_x;
	    rcp.rob_cam_vel_y[rcp.rob_num] = ve.velocity_mycam_y;
	    //drawArrow
	    if(show_video_flag)
	    {
	      Point pStart(pe.robot_center_x - ve.velocity_img_cx, pe.robot_center_y - ve.velocity_img_cy);
	      Point pEnd(pe.robot_center_x, pe.robot_center_y);
	      Scalar color(0, 255, 255);
	      drawArrow(dst_3, pStart, pEnd, 10, 30, color, 1, 4);
	      //cout<<"rob_dx"<<ve.velocity_img_cx<<endl;
	      //cout<<"rob_dy"<<ve.velocity_img_cy<<endl;
	    }
	    rcp.rob_num++;
	  }
	}
      }
    }
    
    for(int i = 0; i < detect_obs_message.size(); i++)//obstacle
    {
      if(rcp.obs_num < obs_pub_num)
      {
	for(int j = 0; j < detect_obs_message_last.size(); j++)
	{
	  if(rcp.obs_num < obs_pub_num && detect_obs_message[i].label == detect_obs_message_last[j].label)
	  {
	    rcp.exist_obs_flag = true;
	    //estimate position
	    pe.getEstimate(detect_obs_message[i].location_image.x, detect_obs_message[i].location_image.y, detect_obs_message[i].location_image.width, detect_obs_message[i].location_image.height, double(src_3.cols), double(src_3.rows));
	    rcp.obs_cam_pos_x[rcp.obs_num] = pe.robot_mycam_x;
	    rcp.obs_cam_pos_y[rcp.obs_num] = pe.robot_mycam_y;
	    //estimate velocity
	    ve.computeVelocity(detect_obs_message[i], detect_obs_message_last[j], src_3, dt);
	    rcp.obs_cam_vel_x[rcp.obs_num] = ve.velocity_mycam_x;
	    rcp.obs_cam_vel_y[rcp.obs_num] = ve.velocity_mycam_y;
	    //drawArrow
	    if(show_video_flag)
	    {
	      Point pStart(pe.robot_center_x - ve.velocity_img_cx, pe.robot_center_y - ve.velocity_img_cy);
	      Point pEnd(pe.robot_center_x, pe.robot_center_y);
	      Scalar color(0, 255, 255);
	      drawArrow(dst_3, pStart, pEnd, 10, 30, color, 1, 4);
	      //cout<<"rob_dx"<<ve.velocity_img_cx<<endl;
	      //cout<<"rob_dy"<<ve.velocity_img_cy<<endl;
	    }
	    rcp.obs_num++;
	  }
	}
      }
    }
    
    //publish
    msg_pub.publish(rcp);
    
    //save and show video
    if(save_result_video_flag)
    {
      result_video<<dst_3;
    }
    if(show_video_flag)
    {
      //imshow(INPUT_VIDEO_WINDOW_NAME, src_3);
      imshow(RESULT_VIDEO_WINDOW_NAME, dst_3);
      waitKey(1);
    }
    
    //set last
    setLast();
  }
  
  void resetState()
  {
    //PositionEstimate
    rcp.exist_rob_flag = false;
    rcp.exist_obs_flag = false;
    rcp.rob_num = 0;
    rcp.obs_num = 0;
    for(int i = 0; i < rob_pub_num; i++)
    {
      rcp.rob_cam_pos_x[i] = 0;
      rcp.rob_cam_pos_y[i] = 0;
      rcp.rob_cam_vel_x[i] = 0;
      rcp.rob_cam_vel_y[i] = 0;
    }
    for(int i = 0; i < obs_pub_num; i++)
    {
      rcp.obs_cam_pos_x[i] = 0;
      rcp.obs_cam_pos_y[i] = 0;
      rcp.obs_cam_vel_x[i] = 0;
      rcp.obs_cam_vel_y[i] = 0;
    }
    
    //detect
    location_detect.clear();
    detect_message.clear();
    detect_rob_message.clear();
    detect_obs_message.clear();
    result_classify.clear();
  }
  
  void setLast()
  {
    //frame
    last_time = this_time;
    //StateEstimate
    detect_rob_message_last.assign(detect_rob_message.begin(), detect_rob_message.end());
    detect_obs_message_last.assign(detect_obs_message.begin(), detect_obs_message.end());
  }
};

int main(int argc, char** argv)
{
  cout << "opencv version: "<<CV_VERSION << endl;
  ros::init(argc, argv, "multirobot_detect_iarc_node");//node name
  double loop_rate;
  PositionEstimate pe;//class initializing
  MultirobotDetect md(pe);
  ros::NodeHandle nh_loop_param("~");
  if(!nh_loop_param.getParam("rate", loop_rate))loop_rate = 5;//video
  ros::Rate loop_rate_class(loop_rate);//frequency: n Hz

  while(ros::ok())
    {
      ros::spinOnce();
      loop_rate_class.sleep();
    }
    ros::spin();
    return 0;
}

