#include "shapeContext.h"

namespace irobot{

ShapeContext::ShapeContext()
{
    RED = cv::Scalar(0, 0, 255);
    PINK = cv::Scalar(230, 130, 255);
    BLUE = cv::Scalar(255, 0, 0);
    LIGHTBLUE = cv::Scalar(255, 255, 160);
    GREEN = cv::Scalar(0, 255, 0);
    WHITE = cv::Scalar(255, 255, 255);
}
ShapeContext::~ShapeContext()
{
    ROS_INFO("Destroying ShapeContext......");
}
double ShapeContext::dist(cv::Point& p1, cv::Point& p2)
{
    return sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

double ShapeContext::angle(cv::Point& p1, cv::Point& p2, cv::Point2d &CoG)
{
    int ydif = p2.y - p1.y;
    int xdif = p2.x - p1.x;
    double yodif = p1.y - CoG.y;
    double xodif = p1.x - CoG.x;
    double delta = atan2(ydif, xdif) - atan2(yodif, xodif);
    
    if (delta < 0) delta += 2 * M_PI;
        return delta;
}

ShapeContext::Hist ShapeContext::getHistogramFromContourPts(const std::vector<cv::Point>& contourPts)
{
    double avgDistance = 0;
    int numPairs = 0;
    cv::Point All = cv::Point(0, 0);

    for (int i = 0; i < contourPts.size(); i++)
    {
        cv::Point p1 = contourPts[i];
	All += p1;
	for (int j = i + 1; j < contourPts.size(); j++)
	{
	    cv::Point p2 = contourPts[j];

	    double distance = dist(p1, p2);
	    avgDistance += distance;
	    numPairs++;  
	}
    }
    cv::Point2d CoG;
    CoG.x = All.x*1.0 / contourPts.size();
    CoG.y = All.y*1.0 / contourPts.size();

    avgDistance = avgDistance / numPairs;

    double maxLogDistance = -HUGE_VAL;
    double minLogDistance = HUGE_VAL;
    for (int i = 0; i < contourPts.size(); i++)
    {
        cv::Point p1 = contourPts[i];
	for (int j = i + 1; j < contourPts.size(); j++)
	{
	    cv::Point p2 = contourPts[j];
	    double distance = dist(p1, p2);
	    distance /= avgDistance;
	    
	    distance = log(distance);
	    if (distance > maxLogDistance) maxLogDistance = distance;
	    if (distance < minLogDistance) minLogDistance = std::max(0.0, distance);
	}
    }
    double radialBound = maxLogDistance + (maxLogDistance - minLogDistance) * 0.01;
    double intervalSize = radialBound / 5.0;
    double angleSize = M_PI / 6.0;
    
    Hist histogram = new double[contourPts.size()][60];
    for (int i = 0; i < contourPts.size(); i++)
    {
        for (int j = 0; j < 60; j++)
	{
	    histogram[i][j] = 0;
	}
    }
    for (int i = 0; i < contourPts.size(); i++)
    {
        cv::Point p1 = contourPts[i];
	
	for (int j = 0; j < contourPts.size(); j++)
	{
	    if (i == j) continue;
	    
	    cv::Point p2 = contourPts[j];
	    double ang = angle(p1, p2, CoG);
	    int angleBin = (int)floor(ang / angleSize);
	    double distance = dist(p1, p2);
	    distance /= avgDistance;
	    distance = std::max(0.0, log(distance));
	    int distanceBin = (int)floor(distance / intervalSize);
	    
	    histogram[i][distanceBin * 12 + angleBin] += 1.0 / contourPts.size();
	}
    }
    return histogram;
}

void ShapeContext::getChiStatistic(double **stats, Hist histogram1, Hist histogram2, int size)
{
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < size; j++)
	{
	    double summation = 0;
	    for (int k = 0; k < 60; k++)
	    {
	        double sum = histogram1[i][k] + histogram2[j][k];
		
		if (sum == 0) continue;
		double diff = histogram1[i][k] - histogram2[j][k];
		summation = summation + diff*diff / sum;
	    }
	    summation = summation / 2;
	    
	    stats[i][j] = summation;
	}
    }
    return;
}

std::vector<cv::Point> ShapeContext::getSampledPoints(std::vector<cv::Point>& v, int sp)
{
    std::vector<cv::Point> result;
    float sr = (float)(v.size()*1.0 / sp);
    float partial = 0;
    float r = 0;
    int q = 0;
    int p = 0;
    cv::Point tp;
    do
    {
        tp.x = (int)(partial*(v[q + 1].x - v[q].x) + v[q].x);
	tp.y = (int)(partial*(v[q + 1].y - v[q].y) + v[q].y);
	result.push_back(tp);
	p++;
	r = p*sr;
	q = (int)r;
	partial = r - q;
	} while ((q + 1) < v.size());
	
    return result;
}

std::pair<double, int> ShapeContext::minMatchingCost(int size, cost **chiStatistics, int *Ref2Shape)
{
    double minAvgCost = HUGE_VAL;
    double AvgCost = 0;
    int mind = 0;
    for (int d = 0; d < size; d++)
    {
        AvgCost = 0;
	for (int q = 0; q < size; q++)
	{
	    AvgCost += chiStatistics[q][(q + d) % size];
	}
	
	if (minAvgCost > AvgCost)
	{
	    minAvgCost = AvgCost;
	    mind = d;
	} 
    }
    minAvgCost /= size;
    return std::make_pair(minAvgCost, mind);
}

}