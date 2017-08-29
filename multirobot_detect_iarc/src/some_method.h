#include <iostream>  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/objdetect/objdetect.hpp>  
#include <opencv2/ml/ml.hpp>  
#include <time.h>
#include <algorithm>
#include <ros/ros.h>

using namespace cv;
using namespace std;

//SVM: decision_func is "protected", to get alpha and rho, you have to create a class to inherit from CvSVM
class MySVM : public CvSVM  
{  
public:  
  double * get_alpha_vector()  
  {
    return this->decision_func->alpha;  
  }
  float get_rho()  
  {
    return this->decision_func->rho;  
  }
};

//HOG: set detectHOG from detectSvm
void setHOG(MySVM &detectSvm, HOGDescriptor &detectHOG)
{

  //dimension of HOG descriptor: 
  //[(window_width-block_width)/block_stride_width+1]*[(window_height-block_height)/block_stride_height+1]*bin_number*(block_width/cell_width)*(block_height/cell_height)
  int descriptorDimDetect;
  //int descriptorDimClassify;
  descriptorDimDetect = detectSvm.get_var_count();
  //descriptorDimClassify = classifySvm.get_var_count();
  int supportVectorDetectNum = detectSvm.get_support_vector_count();
  //int supportVectorCassifyNum = classifySvm.get_support_vector_count();
  cout<<"number of Detect SVM: "<<supportVectorDetectNum<<endl;  
  //cout<<"number of Classify SVM: "<<supportVectorCassifyNum<<endl;  
  Mat alphaDetectMat = Mat::zeros(1, supportVectorDetectNum, CV_32FC1);
  //Mat alphaClassifyMat = Mat::zeros(1, supportVectorCassifyNum, CV_32FC1);
  Mat supportVectorDetectMat = Mat::zeros(supportVectorDetectNum, descriptorDimDetect, CV_32FC1); 
  //Mat supportVectorClassifyMat = Mat::zeros(supportVectorCassifyNum, descriptorDimClassify, CV_32FC1);
  Mat resultDetectMat = Mat::zeros(1, descriptorDimDetect, CV_32FC1); 
  //Mat resultClassifyMat = Mat::zeros(1, descriptorDimClassify, CV_32FC1); 

  //compute w array
  for(int i=0; i<supportVectorDetectNum; i++)
  {
    const float * pSVData = detectSvm.get_support_vector(i);
    for(int j=0; j<descriptorDimDetect; j++)  
      supportVectorDetectMat.at<float>(i,j) = pSVData[j];  
  }
  //for(int i=0; i<supportVectorCassifyNum; i++)
  //{
  //  const float * pSVData = classifySvm.get_support_vector(i);
  //  for(int j=0; j<descriptorDimClassify; j++)  
  //    supportVectorClassifyMat.at<float>(i,j) = pSVData[j];  
  //}
  double * pAlphaDetectData = detectSvm.get_alpha_vector();
  //double * pAlphaClassifyData = classifySvm.get_alpha_vector();
  for(int i=0; i<supportVectorDetectNum; i++)
    alphaDetectMat.at<float>(0,i) = pAlphaDetectData[i];  
  //for(int i=0; i<supportVectorCassifyNum; i++)
  //  alphaClassifyMat.at<float>(0,i) = pAlphaClassifyData[i];  
  resultDetectMat = -1 * alphaDetectMat * supportVectorDetectMat;//resultMat = -(alphaMat * supportVectorMat)
  //resultClassifyMat = -1 * alphaClassifyMat * supportVectorClassifyMat;//resultMat = -(alphaMat * supportVectorMat)

  //get detector for setSVMDetector(const vector<float>& detector)
  vector<float> myDetector;
  for(int i=0; i<descriptorDimDetect; i++)//add resultMat
    myDetector.push_back(resultDetectMat.at<float>(0,i));  
  myDetector.push_back(detectSvm.get_rho());//add rho  
  cout<<"dimension of detect SVM Detector (w+b): "<<myDetector.size()<<endl;

  //set SVMDetector
  detectHOG.setSVMDetector(myDetector);  

  return;
}


//Label: struct for robot message
struct RobotMessage
{
  Rect location_image;//robot location on image
  Point2i center;//center of robot location on image
  int label;
  float distance_min;//minimum distance for point from this frame to last frame

  RobotMessage(){}

  RobotMessage(Rect location_image0)
  {
    location_image = location_image0;
    computerCenter();
    label = 0;
    distance_min = 10000;
  }

  ~RobotMessage(){}

  void computerCenter()
  {
    center.x = location_image.x + location_image.width / 2;
    center.y = location_image.y + location_image.height / 2;
  }
};

//Label: class for labeling robot
class LabelRobot
{
private:
  vector<RobotMessage> robots;
  vector<RobotMessage> robots_last;
  int label_max;//maximum used label
  int distance_max;//max distance from last position to this position
  int number_limit;//maximum robot number

public:
  LabelRobot()
  {
    label_max = 0;
    distance_max = 100000;
    number_limit = 10;
  }

  LabelRobot(int max_distance0, int number_limit0)
  {
    label_max = 0;
    distance_max = max_distance0;
    number_limit = number_limit0;
  }

  ~LabelRobot(){}

  vector<RobotMessage> getLabel(const vector<RobotMessage> &input_robots)
  {
    robots.assign(input_robots.begin(), input_robots.end());
    labelRobot();
    robots_last.assign(robots.begin(), robots.end());
    return robots;
  }

private:
  void labelRobot()
  {
    vector<RobotMessage> robots_temp;
    robots_temp.insert(robots_temp.end(), robots.begin(), robots.end());
    vector<RobotMessage> robots_last_temp;
    robots_last_temp.insert(robots_last_temp.end(), robots_last.begin(), robots_last.end());
    vector<RobotMessage> robots_labeled;

    while(1)
    {
      if (robots_temp.size() == 0 || robots_labeled.size() >= number_limit)
      {
        break;
      }

      if (robots_last_temp.size() == 0)
      {
        for (int i = 0; i<robots_temp.size();i++)
        {
          label_max++;
          robots_temp[i].label = label_max;
        }
        robots_labeled.insert(robots_labeled.end(), robots_temp.begin(), robots_temp.end());
        break;
      }

      float distance_min = 10000;
      int robot_number_min, pair_number_i, pair_number_min;
      for (int i = 0; i < robots_temp.size(); i++)
      {
        calculateMinDistance(robots_temp[i], robots_last_temp, pair_number_i);
        if (robots_temp[i].distance_min < distance_min)
        {
          robot_number_min = i;
          pair_number_min = pair_number_i;
          distance_min = robots_temp[i].distance_min;
        }
      }

      if (distance_min > distance_max)
      {
        for (int i = 0; i<robots_temp.size();i++)
        {
          label_max++;
          robots_temp[i].label = label_max;
        }
        robots_labeled.insert(robots_labeled.end(), robots_temp.begin(), robots_temp.end());
        break;
      } 
      else
      {
        robots_temp[robot_number_min].label = robots_last_temp[pair_number_min].label;
        robots_labeled.insert(robots_labeled.end(), robots_temp[robot_number_min]);
        robots_temp.erase(robots_temp.begin() + robot_number_min);
        robots_last_temp.erase(robots_last_temp.begin() + pair_number_min);
      }
    }

    robots.clear();
    robots.insert(robots.end(), robots_labeled.begin(), robots_labeled.end());
  }

  void calculateMinDistance(RobotMessage &robots_temp_i, vector<RobotMessage> &robots_last_temp, int &pair_number_i)
  {
	robots_temp_i.distance_min = 10000.0;
    float dx, dy, distance;
    for (int i = 0; i<robots_last_temp.size(); i++)
    {
      dx = robots_temp_i.center.x - robots_last_temp[i].center.x;
      dy = robots_temp_i.center.y - robots_last_temp[i].center.y;
      distance = sqrt(dx*dx+dy*dy);
      if (distance <= robots_temp_i.distance_min)
      {
        robots_temp_i.distance_min = distance;
        pair_number_i = i;
      }
    }
  }
};

class PositionEstimate
{
public:
  //node
  ros::NodeHandle nh;
  ros::NodeHandle nh_param;
  //ros::Subscriber sub_loc;//dji_sdk
  //robot and image parameter
  int robot_x, robot_y, robot_width, robot_height, robot_center_x, robot_center_y;
  double image_width, image_height;
  //LocalPosition
  bool listen_h_flag;
  double loc_h;
  //camera
  double camera_pitch;
  double fu,fv;
  //estimate
  float c2r_x;//camera 2 robot, camera frame
  float c2r_y;
  float gu, gv, pu, pv, lu, lv, alpha, yo, h;
  double robot_mycam_x, robot_mycam_y;
  
  PositionEstimate():
  nh_param("~")
  {
    //node
    //sub_loc = nh.subscribe("/dji_sdk/local_position", 10, &PositionEstimate::localPositionCallback,this);//dji_sdk
    //camera
    if(!nh_param.getParam("camera_pitch", camera_pitch))camera_pitch = 36.0;
    if(!nh_param.getParam("fu", fu))fu = 376.629954;
    if(!nh_param.getParam("fv", fv))fv = 494.151786;
    //LocalPosition
    if(!nh_param.getParam("listen_h_flag", listen_h_flag))listen_h_flag = true;
    if(!nh_param.getParam("loc_h", loc_h))loc_h = 1.5;
  }
  
  ~PositionEstimate(){}
  
  void getEstimate(int robot_x0, int robot_y0, int robot_width0, int robot_height0, double image_width0, double image_height0)
  {
    //set robot and image parameter
    robot_x = robot_x0;
    robot_y = robot_y0;
    robot_width = robot_width0;
    robot_height = robot_height0;
    image_width = image_width0;
    image_height = image_height0;
    robot_center_x = robot_x + robot_width / 2;
    robot_center_y = robot_y + robot_height / 2;
    
    //runEstimate
    if(camera_pitch>-90.0 && camera_pitch<30.0 && loc_h>0.5 && loc_h<4)
    {
      this->runEstimate(false);
    }
    else
    {
      cout<<"camera_pitch or loc_h is error"<<endl;
      cout<<"camera_pitch: "<<camera_pitch<<endl;
      cout<<"loc_h: "<<loc_h<<endl;
    }
  }
  /*
  void localPositionCallback(const dji_sdk::LocalPosition::ConstPtr& msg)//dji_sdk
  {
    if(listen_h_flag)
      loc_h = msg->z;
  }
  */
  
private:
  void runEstimate(bool atHead)
  {
    //c2r_x
    gv = image_height / 2.0;
    pv = robot_center_y;
    yo = camera_pitch / 360.0 * (2.0*M_PI);
    h = loc_h;
    
    alpha = atan((gv - pv) /fv);
    c2r_x = tan(alpha + yo + M_PI/2.0) * h;
    
    //c2r_y
    lu = robot_center_x;
    pu = image_width / 2.0;
    
    c2r_y = (lu-pu)*cos(alpha)*h / (fu*cos(alpha+yo+M_PI/2.0));
    /*
    cout<<"gv: "<<gv<<endl;
    cout<<"pv: "<<pv<<endl;
    cout<<"yo: "<<yo<<endl;
    cout<<"h: "<<h<<endl;
    cout<<"(gv - pv) /fv: "<<(gv - pv) /fv<<endl;
    cout<<"alpha: "<<alpha<<endl;
    cout<<"c2r_x: "<<c2r_x<<endl;
    cout<<"lu: "<<lu<<endl;
    cout<<"pu: "<<pu<<endl;
    cout<<"(lu-pu)*cos(alpha)*h: "<<(lu-pu)*cos(alpha)*h<<endl;
    cout<<"(fu*cos(alpha+yo)): "<<(fu*cos(alpha+yo))<<endl;
    cout<<"c2r_y: "<<c2r_y<<endl;
    */
    //publish
    if(atHead)
	{
		//camera at head
		robot_mycam_x = c2r_x;
		robot_mycam_y = c2r_y;
	}
	else
	{
		//camera at tail
		robot_mycam_x = -c2r_x;
		robot_mycam_y = -c2r_y;
	}
  }
};

class VelocityEstimate
{
public:
  PositionEstimate pe;
  double postition_mycam_x, postition_mycam_y;
  double postition_mycam_x_last, postition_mycam_y_last;
  double velocity_mycam_x, velocity_mycam_y;
  int postition_img_cx, postition_img_cy;
  int postition_img_cx_last, postition_img_cy_last;
  int velocity_img_cx, velocity_img_cy;
  
  VelocityEstimate(){}
  VelocityEstimate(const PositionEstimate &pe0)
  {
    pe = pe0;
  }
  ~VelocityEstimate(){}
  
  void computeVelocity(const RobotMessage &message, const RobotMessage &message_last, const Mat &src, double dt)
  {
    //this camPos
    pe.getEstimate(message.location_image.x, message.location_image.y, message.location_image.width, message.location_image.height, double(src.cols), double(src.rows));
    postition_mycam_x = pe.robot_mycam_x;
    postition_mycam_y = pe.robot_mycam_y;
    
    //this img_center_pos
    postition_img_cx = pe.robot_center_x;
    postition_img_cy = pe.robot_center_y;
    
    //last camPos
    pe.getEstimate(message_last.location_image.x, message_last.location_image.y, message_last.location_image.width, message_last.location_image.height, double(src.cols), double(src.rows));
    postition_mycam_x_last = pe.robot_mycam_x;
    postition_mycam_y_last = pe.robot_mycam_y;
    
    //last img_center_pos
    postition_img_cx_last = pe.robot_center_x;
    postition_img_cy_last = pe.robot_center_y;
    
    //camVel
    velocity_mycam_x = (postition_mycam_x - postition_mycam_x_last) / dt;
    velocity_mycam_y = (postition_mycam_y - postition_mycam_y_last) / dt;
    
    //img_center_velocity
    velocity_img_cx = postition_img_cx - postition_img_cx_last;
    velocity_img_cy = postition_img_cy - postition_img_cy_last;
  }
};

//Train: create a disorder array (elements are integer from zero to n-1)
void random(int a[], int n)
{
  for (int nu = 0;nu<n;nu++)
  {
    a[nu] = nu;
  }
  int index, tmp, i;
  srand(time(NULL));
  for (i = 0; i <n; i++)
  {
    index = rand() % (n - i) + i;
    if (index != i)
    {
      tmp = a[i];
      a[i] = a[index];
      a[index] = tmp;
    }
  }
}

//Train: initial type array (0-train,1-vaild,2-test)
void typeHandle(int arr[],int setNo,int trainNo,int vaildNo)
{
  for (int i = 0;i<setNo;i++)
  {
    if (arr[i]<trainNo)
    {
      arr[i] = 0;
    } 
    else if(arr[i]<trainNo + vaildNo)
    {
      arr[i] = 1;
    }
    else
    {
      arr[i] = 2;
    }
  }
}

//get scores
vector<float> get_scores(Mat &src, vector<Rect> &boxes, MySVM &svm, int descriptor_dim, Size WinSize, HOGDescriptor &HOG_descriptor)
{
  vector<float> scores;
  for(int i=0; i<boxes.size(); i++)  
  {
    vector<float> descriptor;
    Mat descriptor_mat(1, descriptor_dim, CV_32FC1);
    Mat src_resize;
    
    resize(src(boxes[i]), src_resize, WinSize);
    HOG_descriptor.compute(src_resize, descriptor);
    for(int j=0; j<descriptor_dim; j++)  
      descriptor_mat.at<float>(0,j) = descriptor[j];
    float score = svm.predict(descriptor_mat, true);
    score = -score;
    //cout<<score<<endl;
    scores.push_back(score);
  }
  return scores;
}

//Non-maximum suppression
struct BoxWithScore
{
  Rect box;
  float score;
  
  BoxWithScore(){}
  BoxWithScore(Rect box0, float score0)
  {
    box = box0;
    score = score0;
  }
  ~BoxWithScore(){}
};

bool less_second(const BoxWithScore & m1, const BoxWithScore & m2) {
        return m1.score < m2.score;
}

vector<Rect> sort_boxes(const vector<Rect> &boxes, const vector<float> &scores)
{
  vector<BoxWithScore> boxes_with_scores;
  vector<Rect> boxes_sort;
  for(int i = 0; i < boxes.size(); i++)
  {
    boxes_with_scores.push_back(BoxWithScore(boxes[i], scores[i]));
  }
  sort(boxes_with_scores.begin(), boxes_with_scores.end(), less_second);
  
  for(int i = 0; i < boxes.size(); i++)
  {
    boxes_sort.push_back(boxes_with_scores[i].box);
    //cout<<boxes_with_scores[i].score<<endl;
  }
  return boxes_sort;
}

float bbOverlap(const Rect& box1, const Rect& box2)  
{  
  if (box1.x > box2.x+box2.width) { return 0.0; }  
  if (box1.y > box2.y+box2.height) { return 0.0; }  
  if (box1.x+box1.width < box2.x) { return 0.0; }  
  if (box1.y+box1.height < box2.y) { return 0.0; }  
  float colInt =  min(box1.x+box1.width,box2.x+box2.width) - max(box1.x, box2.x);  
  float rowInt =  min(box1.y+box1.height,box2.y+box2.height) - max(box1.y,box2.y);  
  float intersection = colInt * rowInt;  
  float area1 = box1.width*box1.height;  
  float area2 = box2.width*box2.height;  
  return intersection / (area1 + area2 - intersection);  
}

vector<Rect> non_maximum_suppression(const vector<Rect> &boxes, const vector<float> &scores, float suppression_rate)
{
  if(boxes.size() < 2)
    return boxes;
  
  vector<Rect> boxes_sort, boxes_nms;
  boxes_sort = sort_boxes(boxes, scores);
  while(boxes_sort.size() > 0)
  {
    for(int i = boxes_sort.size() - 2; i > -1; i--)
    {
      if(bbOverlap(boxes_sort[boxes_sort.size() - 1], boxes_sort[i]) > suppression_rate)
	boxes_sort.erase(boxes_sort.begin() + i);
    }
    boxes_nms.push_back(boxes_sort[boxes_sort.size() - 1]);
    boxes_sort.pop_back();
  }
  return boxes_nms;
}

// filter noise boxes, add omission boxes
class FilterAdd
{
public:
  vector<Rect> boxes_no_filter, boxes_no_filter_last;
  vector<Rect> boxes_filter;
  vector<RobotMessage> boxes_filter_messages, boxes_filter_stable_messages, boxes_filter_messages_last, boxes_filter_messages_last2;
  LabelRobot label_boxes;
  
  FilterAdd(void)
  {
    label_boxes = LabelRobot(50, 10);
  }
  /*
  FilterAndEstimate(const vector<Rect> &boxes0)
  {
    boxes.assign(boxes0.begin(), boxes0.end());
    boxes_last.assign(boxes0.begin(), boxes0.end());
  }
  */
  ~FilterAdd(){}
  
  vector<RobotMessage> run(const Mat & src, const vector<Rect> &boxes0, float bbOverlap_rate)
  {
    boxes_filter.assign(boxes0.begin(), boxes0.end());
    boxes_no_filter.assign(boxes0.begin(), boxes0.end());
    
    //filter noise boxes
    for(int i = boxes_filter.size() - 1; i > -1; i--)
    {
      bool delete_flag = true;
      for(int j = 0; j < boxes_no_filter_last.size(); j++)
      {
	if(bbOverlap(boxes_filter[i], boxes_no_filter_last[j]) > bbOverlap_rate)
	{
	  delete_flag = false;
	  break;
	}
      }
      if(delete_flag)
	boxes_filter.erase(boxes_filter.begin() + i);
    }
    
    //label boxes
    boxes_filter_messages.clear();
    for(int i = 0; i < boxes_filter.size(); i++)
      boxes_filter_messages.insert(boxes_filter_messages.end(), RobotMessage(boxes_filter[i]));
    boxes_filter_messages = label_boxes.getLabel(boxes_filter_messages);
    
    //get stable boxes (label exist in last frame)
    boxes_filter_stable_messages.assign(boxes_filter_messages.begin(), boxes_filter_messages.end());
    
    for(int i = boxes_filter_stable_messages.size() - 1; i > -1; i--)
    {
      bool delete_flag = true;
      for(int j = 0; j < boxes_filter_messages_last.size(); j++)
      {
	if(boxes_filter_stable_messages[i].label == boxes_filter_messages_last[j].label)
	{
	  delete_flag = false;
	  break;
	}
      }
      if(delete_flag)
      {
	boxes_filter_stable_messages.erase(boxes_filter_stable_messages.begin() + i);
      }
    }
    
    //add omission boxes
    for(int i = boxes_filter_messages_last.size() - 1; i > -1; i--)
    {
      bool exist_in_this = false;
      for(int j = boxes_filter_stable_messages.size() - 1; j > -1; j--)
      {
	if(boxes_filter_messages_last[i].label == boxes_filter_stable_messages[j].label)
	{
	  exist_in_this = true;
	  break;
	}
      }
      if(!exist_in_this)
      {
	for(int k = boxes_filter_messages_last2.size() - 1; k > -1; k--)
	{
	  if(boxes_filter_messages_last[i].label == boxes_filter_messages_last2[k].label)
	  {
	    int box_x = 2 * boxes_filter_messages_last[i].location_image.x - boxes_filter_messages_last2[k].location_image.x;
	    int box_y = 2 * boxes_filter_messages_last[i].location_image.y - boxes_filter_messages_last2[k].location_image.y;
	    int box_width = boxes_filter_messages_last[i].location_image.width;
	    int box_height = boxes_filter_messages_last[i].location_image.height;
	    if(box_x > 0 & box_x + box_width < src.cols & box_y > 0 & box_y + box_height < src.rows)
	    {
	      boxes_filter_stable_messages.push_back(RobotMessage(Rect(box_x, box_y, box_width, box_height)));
	    }
	    break;
	  }
	}
      }
    }
    
    //handle last
    boxes_no_filter_last.assign(boxes_no_filter.begin(), boxes_no_filter.end());
    boxes_filter_messages_last2.assign(boxes_filter_messages_last.begin(), boxes_filter_messages_last.end());//last_last
    boxes_filter_messages_last.assign(boxes_filter_messages.begin(), boxes_filter_messages.end());
    
    //return boxes_filter_stable;
    return boxes_filter_stable_messages;
  }
};


void drawArrow(Mat& img, Point pStart, Point pEnd, int len, int alpha, Scalar& color, int thickness, int lineType)
{
  const double PI = 3.1415926;
  Point arrow;
  double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
  line(img, pStart, pEnd, color, thickness, lineType);
  arrow.x = pEnd.x + len * cos(angle + PI * alpha / 180);
  arrow.y = pEnd.y + len * sin(angle + PI * alpha / 180);
  line(img, pEnd, arrow, color, thickness, lineType);
  arrow.x = pEnd.x + len * cos(angle - PI * alpha / 180);
  arrow.y = pEnd.y + len * sin(angle - PI * alpha / 180);
  line(img, pEnd, arrow, color, thickness, lineType);
}

//resize boxes
vector<Rect> resize_boxes(const vector<Rect> boxes_org, const Mat src, const double rate)
{
  vector<Rect> boxes_resize;
  Rect box_tmp;
  double x, y, width, height;
  for(int i = 0; i < boxes_org.size(); i++)
  {
    width = boxes_org[i].width * rate;
    height = boxes_org[i].height * rate;
    x = boxes_org[i].x - (rate - 1.0) * boxes_org[i].width / 2.0;
    y = boxes_org[i].y - (rate - 1.0) * boxes_org[i].height / 2.0;
    if(x > 0.0 & y > 0.0 & x + width < src.cols & y + height < src.rows)
    {
      box_tmp = Rect(int(x), int(y), int(width), int(height));
    }
    else
    {
      box_tmp = boxes_org[i];
    }
    boxes_resize.push_back(box_tmp);
  }
  return boxes_resize;
}
