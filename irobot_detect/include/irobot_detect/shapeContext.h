//shapRecg.h
#ifndef SHAPECONTEXT_H_
#define SHAPECONTEXT_H_

#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>


#define BIG 100000
//#define M_PI 3.14159265358979

namespace irobot{
class  ShapeContext 
{
public:
  
    ShapeContext();
    ~ShapeContext();
    typedef double(*Hist)[60];
    typedef double cost;
    
    double dist(cv::Point& p1, cv::Point& p2);
    double angle(cv::Point& p1, cv::Point& p2, cv::Point2d &CoG);
    Hist getHistogramFromContourPts(const std::vector<cv::Point>& contourPts);
    void getChiStatistic(double **stats, Hist histogram1, Hist histogram2, int size);
    std::vector<cv::Point> getSampledPoints(std::vector<cv::Point>& v, int sp);
    std::pair<double, int> minMatchingCost(int size, cost **chiStatistics, int *Ref2Shape);
    
private:
    cv::Scalar RED;
    cv::Scalar PINK;
    cv::Scalar BLUE;
    cv::Scalar LIGHTBLUE;
    cv::Scalar GREEN;
    cv::Scalar WHITE;
};
}

#endif