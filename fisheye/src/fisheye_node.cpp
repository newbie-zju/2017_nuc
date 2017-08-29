#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <affine.hpp>
//#include </home/alex/opencv-3.1.0/modules/core/include/opencv2/core/affine.hpp>

class FisheyeModel
{
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param;
    FisheyeModel(ros::NodeHandle nh);
    
    ros::Subscriber sub_img;
    ros::Publisher pub_undistortionImg;
    //ros::Publisher pub_fisheyeIntrinsic;
    
    void initialize();
    void undistortImage(cv::Mat & distorted, cv::Mat & undistorted, cv::Mat & K, cv::Mat & D, cv::Mat & Knew, const cv::Size& new_size);
    void initUndistortRectifyMap( cv::Mat & K, cv::Mat & D, cv::Mat & R, cv::Mat & P,
    const cv::Size& size, int m1type, cv::Mat & map1, cv::Mat & map2 );
    void FishEyeImgUndistort(cv::Mat &src_img, cv::Mat &UndistortImg, cv::Mat &intrinsic_mat, cv::Mat &distortion_coeffs, cv::Mat &new_intrinsic_mat);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	//void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
   // float focalMultiple, centralPointDeviation_u, centralPointDeviation_v;
private:
    cv::Mat intrinsicMatrix, distortionCoeffs;
    float focalMultiple, centralPointDeviation_u, centralPointDeviation_v;  
};
FisheyeModel::FisheyeModel(ros::NodeHandle nh):nh_(nh),nh_param("~")
{
    cv::FileStorage fs("/home/zmart/2017/src/fisheye/data/camera.yml", cv::FileStorage::READ);
    fs["intrinsicMatrix"] >> intrinsicMatrix;
    fs[ "distortionCoeffs"] >> distortionCoeffs;
    if(!nh_param.getParam("focal_multiple",focalMultiple)) focalMultiple = 1.0;
	if(!nh_param.getParam("central_point_deviation_u",centralPointDeviation_u)) centralPointDeviation_u = 0.0;
	if(!nh_param.getParam("central_point_deviation_v",centralPointDeviation_v)) centralPointDeviation_v = 0.0;
   /* ros::param::get("~focal_multiple", focalMultiple);
    ros::param::get("~central_point_deviation_u", centralPointDeviation_u);
    ros::param::get("~central_point_deviation_v", centralPointDeviation_v);*/
 
    //std::cout << focalMultiple << std::endl;
    initialize();
    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//void FisheyeModel::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
void FisheyeModel::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat src_img, undistort_img, new_intrinsicMatrix;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv_ptr->image.copyTo(src_img);
    
    FishEyeImgUndistort(src_img, undistort_img, intrinsicMatrix, distortionCoeffs, new_intrinsicMatrix);

    //cv::namedWindow("src_fisheye_img");
    //cv::namedWindow("undistortion_fisheye_img");
   // cv::imshow("src_fisheye_img", src_img);
    //cv::imshow("undistortion_fisheye_img", undistort_img);
    //cv::waitKey(10);
}
void FisheyeModel::initialize()
{
    sub_img = nh_.subscribe("/usb_cam/image_raw",1,&FisheyeModel::imageCallback,this); 
    pub_undistortionImg = nh_.advertise<sensor_msgs::Image>("/fisheye/image", 1);
  //  pub_fisheyeIntrinsic = nh_.advertise<std_msgs::Float32MultiArray>("/fisheye/intrinsic", 1);
}

void FisheyeModel::initUndistortRectifyMap( cv::Mat & K, cv::Mat & D, cv::Mat & R, cv::Mat & P,
    const cv::Size& size, int m1type, cv::Mat & map1, cv::Mat & map2 )
{
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
    CV_Assert((P.depth() == CV_32F || P.depth() == CV_64F) && (R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(K.size() == cv::Size(3, 3) && (D.empty() || D.total() == 4));
    CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(P.empty() || P.size() == cv::Size(3, 3) || P.size() == cv::Size(4, 3));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F)
    {
        cv::Matx33f camMat = K;
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
    }
    else
    {
        cv::Matx33d camMat = K;
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
    }

    cv::Vec4d k = cv::Vec4d::all(0);
    if (!D.empty())
        k = D.depth() == CV_32F ? (cv::Vec4d)*D.ptr<cv::Vec4f>(): *D.ptr<cv::Vec4d>();

    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.convertTo(rvec, CV_64F);
        //RR = rvec;
	RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.colRange(0, 3).convertTo(PP, CV_64F);

    cv::Matx33d iR = (PP * RR).inv(cv::DECOMP_SVD);

    for( int i = 0; i < size.height; ++i)
    {
        float* m1f = map1.ptr<float>(i);
        float* m2f = map2.ptr<float>(i);
        short*  m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;

        double _x = i*iR(0, 1) + iR(0, 2),
               _y = i*iR(1, 1) + iR(1, 2),
               _w = i*iR(2, 1) + iR(2, 2);

        for( int j = 0; j < size.width; ++j)
        {
            double x = _x/_w, y = _y/_w;

            double r = sqrt(x*x + y*y);
            double theta = atan(r);

            double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
            double theta_d = theta * (1 + k[0]*theta2 + k[1]*theta4 + k[2]*theta6 + k[3]*theta8);

            double scale = (r == 0) ? 1.0 : theta_d / r;
            double u = f[0]*x*scale + c[0];
            double v = f[1]*y*scale + c[1];

            if( m1type == CV_16SC2 )
            {
                int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }

            _x += iR(0, 0);
            _y += iR(1, 0);
            _w += iR(2, 0);
        }
    }
}

void FisheyeModel::undistortImage(cv::Mat & distorted, cv::Mat & undistorted, cv::Mat & K, cv::Mat & D, cv::Mat & Knew, const cv::Size& new_size)
{
    cv::Size size = new_size.area() != 0 ? new_size : distorted.size();

    cv::Mat map1, map2;
    cv::Mat R = cv::Mat::eye(3,3,CV_32F);
    FisheyeModel::initUndistortRectifyMap(K, D, R, Knew, size, CV_16SC2, map1, map2 );
    cv::remap(distorted, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void FisheyeModel::FishEyeImgUndistort(cv::Mat &src_img, cv::Mat &UndistortImg, cv::Mat &intrinsic_mat, cv::Mat &distortion_coeffs, cv::Mat &new_intrinsic_mat)
{
    std_msgs::Float32MultiArray intrinsic_msg;
    sensor_msgs::ImagePtr image_msg;
    intrinsic_mat.copyTo(new_intrinsic_mat);
    
    new_intrinsic_mat.at<float>(0, 0) *= focalMultiple;
    new_intrinsic_mat.at<float>(1, 1) *= focalMultiple;
   // std::cout << focalMultiple << std::endl;
    
    new_intrinsic_mat.at<float>(0, 2) += centralPointDeviation_u;
    new_intrinsic_mat.at<float>(1, 2) += centralPointDeviation_v;
    
   // intrinsic_msg.data.push_back(new_intrinsic_mat.at<float>(0, 0));
    //intrinsic_msg.data.push_back(new_intrinsic_mat.at<float>(1, 1));  //新定义的摄像头焦距fx和fy
   // intrinsic_msg.data.push_back(new_intrinsic_mat.at<float>(0, 2));
   // intrinsic_msg.data.push_back(new_intrinsic_mat.at<float>(1, 2));   //图像中心点坐标u和v
    
    //pub_fisheyeIntrinsic.publish(intrinsic_msg);

    undistortImage(
	src_img,
	UndistortImg,
	intrinsic_mat,
	distortion_coeffs,
	new_intrinsic_mat,
	cv::Size(640, 480));
    
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", UndistortImg).toImageMsg();
    pub_undistortionImg.publish(image_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fisheye_node");
    ros::NodeHandle nh;
    FisheyeModel fisheye(nh);
    ros::spin();
    return 0;
}
