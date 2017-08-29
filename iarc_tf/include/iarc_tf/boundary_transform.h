#ifndef __BOUNDARY_TRANSFORM_H_
#define __BOUNDARY_TRANSFORM_H_

#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class BoundaryTransform
{
public:
    float yaw;
    float deltax,deltay;
    Mat p_world;
    Mat p_ned;
    Mat p_temp_w;
    Mat p_temp_n;
    BoundaryTransform();
    ~BoundaryTransform();
    
    void getBoundaryX(float x);
    void getBoundaryY(float y);
    void getBoundaryXY(float x, float y);
    void getNedXY(float x, float y);
    void getWorld2NedTransform(void);
    void getNed2WorldTransform(void);
    
};


#endif 