//shapRecg.h
#ifndef SHAPERECG_H_
#define SHAPERECG_H_

#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <sstream>
#include <string>
#include <algorithm>
#include "shapeContext.h"

namespace irobot{
class ShapeRecg 
{
public:
    ShapeRecg();
    ~ShapeRecg();
    std::string int2str(int &i);
    void showImg(const cv::Mat &Img, const std::string &WindowName, bool BGRornot = true, int ColorcvtCode = cv::COLOR_HSV2BGR);
    void preprocess(const cv::Mat &Img, cv::Mat &HSVImg);
    void recgColor(const cv::Mat &SrcImg, const cv::Mat &HSVImg, cv::Mat &TargImg);
    void getContours(const cv::Mat &TargImg, std::vector<std::vector<cv::Point> > &contours);
    void getRefShape(std::vector<std::vector<cv::Point> > &RefContours, int NumofPoints, int NumofRefs);
    void filterShape(std::vector<std::vector<cv::Point> > &Contours, const std::vector<std::vector<cv::Point> > &RefContours, std::vector<int> &d, int minNumofPoints, double DisSimilarityThresh);
    std::vector<std::vector<cv::Point> > shaperecg(cv::Mat SrcImg, bool debug = false);
    ShapeContext shapecontext_obj;
    
private:
        //以下均为可调参数
    int iLowHr1;
    int iHighHr1;
    int iLowHr2;
    int iHighHr2;
    int iLowSr;
    int iHighSr;
    int iLowVr;
    int iHighVr;

    int iLowHg;
    int iHighHg;
    int iLowSg;
    int iHighSg;
    int iLowVg;
    int iHighVg;
    
    int NumofPoints;//ShapeContext距离度量精细程度，可调参数,但对于算PnP对应点而言目前标准参考形状下27的倍数最佳
    float shapeSim_thresha;
    int refShapeNum;
};
}
#endif