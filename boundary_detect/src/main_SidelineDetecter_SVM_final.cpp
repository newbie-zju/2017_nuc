#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <strstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/ml/ml.hpp>


//-whd-
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <dji_sdk/LocalPosition.h>
#include <boundary_detect/Boundary.h>

ros::Publisher Boundary_pub;
ros::Publisher pub;

using namespace std; 
using namespace cv;
//-----------------------宏定义----------------------------
#define VideoOutputHight 128.0		//输出视频图像的高度
#define BIAS -0.5					//超平面偏移
//#define TestVideo "../Data/TestVideo/output.avi"	//用于检测的测试视频
//#define ResultVideo "../Data/Result/output.avi"		//测试视频的检测结果
#define LoadSvmName "/home/zmart/2017/src/boundary_detect/src/SVM_HOG.xml"	//载入已有的模型文件名称
#define xMin 0						//场地范围#8.27
//#define xMax 5						//场地范围#8.27
#define yMin 0						//场地范围#8.27
//#define yMax 5						//场地范围#8.27
//#define sideWidth 0.5//#9.22
double xMax;
double yMax;
//-----------------------继承类----------------------------
class MySVM : public CvSVM  
{  
public:  
	//获得SVM的决策函数中的alpha数组  
	double * get_alpha_vector()  
	{  
		return this->decision_func->alpha;  
	}  

	//获得SVM的决策函数中的rho参数,即偏移量  
	float get_rho()  
	{  
		return this->decision_func->rho;  
	}  
};  

enum xSideType{noxSide,leftSide,rightSide};
enum ySideType{noySide,topSide,bottomSide};

//计算点P0指向P1和P2所在直线的向量像素长度
double computeDistance(Point2d P0,Point2d P1,Point2d P2);
//将像素长度转换为实际长度
double imgDis2realDis(double h, double f,double imgDis);
//计算点到点的像素长度//#9.15
double p2pDis(Point2d P0,Point2d P1);

//*******************************输入输出********************************
//输入
Mat src;
double quadHeight = 2.0;//四旋翼高度
double camF = 224;//焦距的像素长度#8.27
double angleBias = 0;//线角度矫正，即竖线在图像中应有的角度(逆时针为正)
//输出
xSideType xSide;//竖线结果
ySideType ySide;//横线结果
Point2d Px1,Px2;//竖线段端点（没找到则返回-1,-1）
Point2d Py1,Py2;//横线段端点（没找到则返回-1,-1）
double xDis;//竖线与四旋翼距离（竖线在图像左边时为负）（没找到则返回10000）
double yDis;//横线与四旋翼距离（竖线在图像下边时为负）（没找到则返回10000）
Point3d outputResult = Point3d(999,999,0);//输出坐标：x，y，标志位#8.27
//x：向上为正方向，y：向右为正方向(没有找到则为999)，标志位：(0-无边界，2-x更新，1-y更新，3-xy更新)#8.27

double width = 640;
double height = 480;
float rho;//b偏移的结果
Mat src32FC3;
Mat threshold_img;
Mat edges;//定义帧：转换数据类型，灰度图，二值图，边缘图
Mat srcChannels[3];//RBG
Mat R,G,B;//RGB单通道
vector<Vec4i> lines,vlines,hlines;//lines线段矢量集合(列，行，列，行)，vlines：竖线集合，hlines：横线集合
Point2d Pcenter = Point2d(VideoOutputHight*width/(2*height),VideoOutputHight/2);//图像中心点坐标
Mat resultMat = Mat::zeros(1, 3, CV_32FC1);//alpha向量乘以支持向量矩阵的结果


//-whd-
//dji_sdk::LocalPosition quadrotorPos;

//-whd-
void quadrotorHightCallback(const dji_sdk::LocalPosition::ConstPtr &msg)
{
	quadHeight = msg->z;
}

//xk
/*
void fisheyeCamFCallBack(const )
{
	Camf = msg
}
*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
       ros::spinOnce();
       

	boundary_detect::Boundary msg1;
	geometry_msgs::PointStamped output_msgs;
	cv_bridge::CvImagePtr cvPtr;
	try
	{
		cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		cvPtr->image.copyTo(src);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//###########################
	resize(src,src,Size(VideoOutputHight*width/height,VideoOutputHight));//调整大小
	//GaussianBlur(src,src,Size(7,7),1.5);//高斯滤波
	src.convertTo(src32FC3,CV_32FC3);//转换图像数据类型
	split(src32FC3,srcChannels);//分离RGB通道
	B = srcChannels[0];G = srcChannels[1];R = srcChannels[2];

	//进行图像分割（色彩空间SVM方法）（矩阵操作方法，速度快）
	threshold_img = (resultMat.at<float>(0)*R + resultMat.at<float>(1)*G + resultMat.at<float>(2)*B + rho) > BIAS;
	//threshold = - 0.273370087*B - (-0.377990603)*G - 0.350543886*R + 32.809016951026422 > 0;//0.273370087等为xml中的数据

	//形态学滤波
			//---just for 9.16 test---!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			morphologyEx(threshold_img,threshold_img,MORPH_OPEN,getStructuringElement(MORPH_ELLIPSE, Size(11,11)));//形态学滤波：场内去噪//#9.16
			morphologyEx(threshold_img,threshold_img,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE, Size(9,9)));//形态学滤波：场外去噪//#9.16

			//morphologyEx(threshold_img,threshold_img,MORPH_CLOSE,getStructuringElement(MORPH_ELLIPSE, Size(9,9)));//形态学滤波：场外去噪//#9.15
			//morphologyEx(threshold_img,threshold_img,MORPH_OPEN,getStructuringElement(MORPH_ELLIPSE, Size(11,11)));//形态学滤波：场内去噪//#9.15


	//形态学滤波：边缘检测
	morphologyEx(threshold_img,edges,MORPH_GRADIENT ,getStructuringElement(MORPH_RECT, Size(7,7)));//#9.15
	
	//namedWindow("threshold_img",CV_WINDOW_NORMAL);
	//imshow("threshold_img",threshold_img);
	waitKey(1);
	//进行霍夫变换
	HoughLinesP(edges,lines,1,CV_PI/360,48,48,0);
	/*	第五个参数，Accumulator threshold parameter. Only those lines are returned that get enough votes (  ).
		第六个参数，double类型的minLineLength，有默认值0，表示最低线段的长度，比这个设定参数短的线段就不能被显现出来。
		第七个参数，double类型的maxLineGap，有默认值0，允许将同一行点与点之间连接起来的最大的距离*/

	//找出竖线集合

	vlines.clear();
	vector<double> anglex;//竖线集合与场地竖边界的夹角
	vector<double> lengthx;//竖线长度//#9.15
	for( size_t i = 0; i < lines.size(); i++ )//找出竖线集合
	{
		Vec4i l = lines[i];
		double angle = atan(double(l[2]-l[0])/double(l[3]-l[1]))/CV_PI*180;//此线与y轴所呈角度
		if (abs(angle-angleBias)<15)
		{
			vlines.push_back(l);
			anglex.push_back(abs(angle-angleBias));
			lengthx.push_back(p2pDis(Point2d(l[0],l[1]),Point2d(l[2],l[3])));//#9.15
			//line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 1, CV_AA);//画线，point:(列，行)
		}
	}

	//找出竖线
	if (vlines.size()>0)
	{
		vector<double> anglexS(anglex);//角度从大到小排序
		sort(anglexS.begin(),anglexS.end());
		
		vector<double> lengthxS(lengthx);//长度从大到小排序//#9.15
		sort(lengthxS.begin(),lengthxS.end());//#9.15

		for (size_t i = 0;i < anglex.size();i++)
		{
			//cout<<anglexS[i]<<" ";
			//if (anglex[i] == anglexS[anglexS.size()-1])//确定竖线
			if (lengthx[i] == lengthxS[lengthxS.size()-1])//确定竖线//#9.15
			{
				//cout<<"minx"<<anglex[i];
				Px1 = Point2d(vlines[i][0],vlines[i][1]);
				Px2 = Point2d(vlines[i][2],vlines[i][3]);

				//分类函数系数
				int f2a = (Px1.y-Px2.y)*(Px1.x-Px2.x);
				int f2b = -(Px1.y-Px2.y)*(Px1.y-Px2.y);
				int f2c = (Px1.y-Px2.y)*(Px1.y*Px2.x-Px2.y*Px1.x);

				//计算长度
				double xImgDis = computeDistance(Pcenter,Px1,Px2);//计算中心点到竖线的像素长度
				if (f2a*Pcenter.y+f2b*Pcenter.x+f2c<0)//距离赋符号：点在线右边
					xImgDis = -xImgDis;
				xDis = imgDis2realDis(quadHeight,camF,xImgDis * height/VideoOutputHight);//从像素距离转换到实际距离
				//cout<<"xImgDis:"<<xImgDis<<endl;
				//cout<<"xDis:"<<xDis<<endl;

				//判断竖线类型
				int countLeft = 0,countRight = 0;//积分值
				int areaLeft = 0,areaRight = 0;//面积
				for (int xi = 0;xi<threshold_img.cols;xi++)
				{
					for (int yi = 0;yi<threshold_img.rows;yi++)
					{
						if (f2a*yi+f2b*xi+f2c>0)//左
						{
							areaLeft++;
							if (threshold_img.at<uchar>(yi,xi)!=0)
							{
								countLeft++;
							} 
						} 
						else//右
						{
							areaRight++;
							if (threshold_img.at<uchar>(yi,xi)!=0)
							{
								countRight++;
							}
						}

					}
				}
				if (1.0*countLeft/areaLeft>0.6)//判断类型
				{
					//cout<<"left"<<1.0*countLeft/areaLeft<<endl;
					xSide = leftSide;
					line( src, Px1, Px2, Scalar(255,0,0), 1, CV_AA);//画线，point:(列，行)

				} 
				else if (1.0*countRight/areaRight>0.6)
				{
					//cout<<"right"<<1.0*countRight/areaRight<<endl;
					xSide = rightSide;
					line( src, Px1, Px2, Scalar(0,255,0), 1, CV_AA);//画线，point:(列，行)

				}
				else
				{
					//cout<<"xfalse"<<endl;
					xSide = noxSide;
				}
				break;
			}
		}
	}
	else
	{
		xSide = noxSide;
		Px1 = Point2d(-1,-1);
		Px2 = Point2d(-1,-1);
		xDis = 10000;
	}

	//找出横线集合
	hlines.clear();
	vector<double> angley;//横线集合的角度
	vector<double> lengthy;//竖线长度//#9.15
	for( size_t i = 0; i < lines.size(); i++ )//找出横线集合
	{
		Vec4i l = lines[i];
		double angle = atan(double(l[3]-l[1])/double(l[2]-l[0]))/CV_PI*180;//此线与x轴所呈角度
		if (abs(angle+angleBias)<15)
		{
			hlines.push_back(l);
			angley.push_back(abs(angle-90-angleBias));
			lengthy.push_back(p2pDis(Point2d(l[0],l[1]),Point2d(l[2],l[3])));//#9.15
			//line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 1, CV_AA);//画线，point:(列，行)
		}
	}

	//找出横线
	if (hlines.size()>0)
	{
		vector<double> angleyS(angley);//角度从大到小排序
		sort(angleyS.begin(),angleyS.end());
		
		vector<double> lengthyS(lengthy);//长度从大到小排序//#9.15
		sort(lengthyS.begin(),lengthyS.end());//#9.15
		for (size_t i = 0;i < angley.size();i++)
		{
			//cout<<angleyS[i]<<" ";
			//if (angley[i] == angleyS[angleyS.size()-1])//确定横线
			if (lengthy[i] == lengthyS[lengthyS.size()-1])//确定竖线//#9.15
			{
				//cout<<"miny"<<angley[i];
				Py1 = Point2d(hlines[i][0],hlines[i][1]);
				Py2 = Point2d(hlines[i][2],hlines[i][3]);

				//分类函数系数
				int f1a = (Py1.x-Py2.x)*(Py1.y-Py2.y);
				int f1b = -(Py1.x-Py2.x)*(Py1.x-Py2.x);
				int f1c = (Py1.x-Py2.x)*(Py1.x*Py2.y-Py2.x*Py1.y);

				//计算长度
				double yImgDis = computeDistance(Pcenter,Py1,Py2);//计算中心点到横线的像素长度
				if (f1a*Pcenter.x+f1b*Pcenter.y+f1c>0)//距离赋符号：点在线上边
					yImgDis = -yImgDis;
				yDis = imgDis2realDis(quadHeight,camF,yImgDis * height/VideoOutputHight);//从像素距离转换到实际距离
				//cout<<"yImgDis:"<<yImgDis<<endl;
				//cout<<"yDis:"<<yDis;

				//判断竖线类型
				int countBottom = 0,countTop = 0;//积分值
				int areaBottom = 0,areaTop = 0;//面积
				for (int xi = 0;xi<threshold_img.cols;xi++)
				{
					for (int yi = 0;yi<threshold_img.rows;yi++)
					{
						if (f1a*xi+f1b*yi+f1c<0)//下
						{
							areaBottom++;
							if (threshold_img.at<uchar>(yi,xi)!=0)
							{
								countBottom++;
							} 
						} 
						else//上
						{
							areaTop++;
							if (threshold_img.at<uchar>(yi,xi)!=0)
							{
								countTop++;
							}
						}

					}
				}
				if (1.0*countBottom/areaBottom>0.6)//判断类型
				{
					//cout<<"Bottom"<<1.0*countBottom/areaBottom<<endl;
					ySide = bottomSide;
					line( src, Py1, Py2, Scalar(0,0,255), 1, CV_AA);//画线，point:(列，行)

				} 
				else if (1.0*countTop/areaTop>0.6)
				{
					//cout<<"Top"<<1.0*countTop/areaTop<<endl;
					ySide = topSide;
					line( src, Py1, Py2, Scalar(255,255,0), 1, CV_AA);//画线，point:(列，行)

				}
				else
				{
					cout<<"yfalse"<<endl;
					ySide = noySide;
				}
				break;
			}
		}
	}
	else
	{
		ySide = noySide;
		Py1 = Point2d(-1,-1);
		Py2 = Point2d(-1,-1);
		yDis = 10000;
	}


	//计算四旋翼坐标#8.27
	Point3d detectResult = Point3d(999,999,0);//每帧初始化为无边界
	//x：图像上向右为正方向，y：图像上向上为正方向(没有找到则为999)，标志位：(0-无边界，1-x更新，2-y更新，3-xy更新)#8.27

	if (xSide != noxSide)//x更新#8.27
	{
		if (xSide == leftSide)
		{
			detectResult.x = xMin - xDis;
		} 
		else
		{
			detectResult.x = xMax - xDis;
		}
		detectResult.z = 1;
	}
	if (ySide != noySide)//y更新#8.27
	{
		if (ySide == bottomSide)
		{
			detectResult.y = yMin - yDis;
		} 
		else
		{
			detectResult.y = yMax - yDis;
		}
		detectResult.z = 2;
	}
	if(xSide != noxSide && ySide != noySide)//xy更新#8.27
	{
		detectResult.z = 3;
	}
	if(quadHeight<1.0)
	{
		detectResult.z = 0;
		detectResult.x = 0;
		detectResult.y = 0;
	}

	outputResult = Point3d(detectResult.y,detectResult.x,detectResult.z);//输出坐标赋值

	/*
	//------test---------
	static int i = 0;
	static int dx = -1;
	static int dy = 0;
	static int myflag = 2;
	if(i%100 == 0)
	{
		myflag = 2;
		dx++;
	}
	else if(i%50 == 0)
	{
		myflag = 3;
		dy++;
	}
	else
	{
		myflag = 0;
	}
	outputResult = Point3d(dx,dy,myflag);
	i++;
	//------test end---------
	*/

	//cout<<"flag"<<outputResult.z<<"  x"<<outputResult.x<<"  y"<<outputResult.y<<endl;//#8.27

	//ROS_INFO_THROTTLE(0.2,"DisOutput: xDis=\t%4.2f, yDis=\t%4.2f",xDis,yDis);
	ROS_INFO_THROTTLE(0.2,"boudary output: flag=\t%d,x=\t%4.2f,y=\t%4.2f",(int)outputResult.z,outputResult.x,outputResult.y);
	//namedWindow("src",CV_WINDOW_NORMAL);
	//imshow("src",src);
	waitKey(1);

	msg1.xSide = xSide;
	msg1.ySide = ySide;
	msg1.xDis = xDis;
	msg1.yDis = yDis;

	output_msgs.point.x = outputResult.x;
	output_msgs.point.y = outputResult.y;
	output_msgs.point.z = outputResult.z;
	output_msgs.header.stamp = ros::Time::now();

	Boundary_pub.publish(msg1);
	pub.publish(output_msgs);
	
}

//*********************************end***********************************
int main(int argc ,char** argv)
{
	//----------------读取模型数据生成w+b---------------------
	//变量定义
	MySVM svm;//SVM分类器
	svm.load(LoadSvmName);//从XML文件读取训练好的SVM模型
	int supportVectorNum = svm.get_support_vector_count();//支持向量的个数  
	Mat alphaMat = Mat::zeros(1, supportVectorNum, CV_32FC1);//alpha向量，长度等于支持向量个数  
	Mat supportVectorMat = Mat::zeros(supportVectorNum, 3, CV_32FC1);//支持向量矩阵  
	//将支持向量的数据复制到supportVectorMat矩阵中  
	for(int i=0; i<supportVectorNum; i++)  
	{  
		const float * pSVData = svm.get_support_vector(i);//返回第i个支持向量的数据指针  
		for(int j=0; j<3; j++)  
		{
			supportVectorMat.at<float>(i,j) = pSVData[j];  
		}  
	}  
	//将alpha向量的数据复制到alphaMat中  
	double * pAlphaData = svm.get_alpha_vector();//返回SVM的决策函数中的alpha向量  
	for(int i=0; i<supportVectorNum; i++)  
	{  
		alphaMat.at<float>(0,i) = pAlphaData[i];  
	}  
	//计算-(alphaMat * supportVectorMat),结果放到resultMat中  
	resultMat = -1 * alphaMat * supportVectorMat;  
	//b偏移结果
	rho = svm.get_rho();
	ros::init(argc, argv, "boundaryDetect_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");
	if(!nh_param.getParam("xMax", xMax))xMax = 5.0;
	if(!nh_param.getParam("yMax", yMax))yMax = 5.0;
	ROS_ERROR("boudary detect: xMax=%4.2f",(float)xMax);
	image_transport::ImageTransport it(nh);
	//image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw",1, imageCallback);
	image_transport::Subscriber image_sub = it.subscribe("/fisheye/image",1, imageCallback);
	//-whd-
	ros::Subscriber quadrotorHight_sub = nh.subscribe("/dji_sdk/local_position", 10, quadrotorHightCallback);
	Boundary_pub = nh.advertise<boundary_detect::Boundary>("boundary", 10);
	pub = nh.advertise<geometry_msgs::PointStamped>("boundary_output",10);
	ros::Rate loop_rate(20);
      while(ros::ok())
	{
	 ros::spinOnce();
         loop_rate.sleep();
	}
	ros::spin();
	return 0;
}

double computeDistance(Point2d P0,Point2d P1,Point2d P2)
{
	double c_2 = (P0.x - P1.x)*(P0.x - P1.x) + (P0.y - P1.y)*(P0.y - P1.y);//斜边平方
	double a = ((P2.x - P1.x)*(P0.x - P1.x) + (P2.y - P1.y)*(P0.y - P1.y))/sqrt((P2.x - P1.x)*(P2.x - P1.x)+(P2.y - P1.y)*(P2.y - P1.y));//直角边a

	return sqrt(c_2-a*a);
}
double imgDis2realDis(double h, double f,double imgDis)
{
	return h*imgDis/f;
}
double p2pDis(Point2d P0,Point2d P1)//#9.15
{
	return sqrt((P1.x - P0.x)*(P1.x - P0.x) + (P1.y - P0.y)*(P1.y - P0.y));
}
