#include "LIDAR.h"

using namespace std;
using namespace nanoflann;

void LIDAR::grab_data(const std::vector<double> thetaAngle, const std::vector<double> distance, const int counts)
{
    ros::param::get("~mindistance", _mindistance);
    ros::param::get("~maxdistance", _maxdistance);
    ros::param::get("~maxconnectdistance", _maxconnectdistance);
    ros::param::get("~obstaclemaxsize", _obstaclemaxsize);
    ros::param::get("~doavoidancedist", _doavoidancedist);
    ros::param::get("~saferadius", _saferadius);
    ros::param::get("~avoidancevelocitymag", _avoidancevelocitymag);
 
    count  = counts;
    points.pts.clear();
    for(int i = 0; i < counts; i++)
    {
	//std::cout << thetaAngle[i] << std::endl;
	theta[i] = 180 - thetaAngle[i] * 180 / PI;  //exchange angle 0 - 360
	//std::cout << "theta" << thetaAngle[i] << "dist" << (distance[i] < 0.25 && distance[i] > 0.1) << std::endl;
	dist[i] = distance[i] * 1000;
	PointCloud::Point pt;
	pt.x=(float)(dist[i]*sin(theta[i]/180*PI));
	pt.y=(float)(dist[i]*cos(theta[i]/180*PI));
	points.pts.push_back(pt);
	//std::cout << theta[i] << std::endl;
    }
}

vector<int> LIDAR::RadiusSearch(float x, float y, float radius, my_kd_tree_t &Myindex)
{
	const float query_pt[2] = { x, y };

	//半径内搜索
	const float search_radius = radius;
	std::vector<std::pair<size_t, float> > ret_matches;
	nanoflann::SearchParams params;
	const size_t nMatches = Myindex.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
	vector<int> indexsearched;
	for (size_t i = 0; i < nMatches; i++)
	{
		indexsearched.push_back((int)(ret_matches[i].first));
	}

	return indexsearched;
}


vector<vector<int> > LIDAR::calcObstacles(void)
{
	vector<vector<int> > obstacles;
	my_kd_tree_t Myindex(2, points, nanoflann::KDTreeSingleIndexAdaptorParams(720 /* 最大叶子数量 */));
	Myindex.buildIndex();
	vector<int> predecessor;
	int obstacleNum = 0;
	for(int i = 0;i < count; i++)
	{
	    predecessor.push_back(-1);
	}
	//std:: cout << theta[0] << std::endl;
	//利用距离信息计算点云连通性并以前驱数组的形式给出障碍物点云树构成的森林
	for (size_t p = 0; p < count; p++)
	{
		if (dist[p] > _maxdistance || dist[p] < _mindistance)
		{
			predecessor[p] = -1;
			continue;
		}
		if ((predecessor[p] < 0))
			predecessor[p] = (int)p;
		vector<int> tp = RadiusSearch(points.pts[p].x, points.pts[p].y, _maxconnectdistance, Myindex);
		for (size_t q = 0; q < tp.size(); q++)
		{
			if ((tp[q] > (int)p) && (predecessor[tp[q]] < 0))
				predecessor[tp[q]] = (int)p;
			else if (predecessor[tp[q]] >= 0)
			{
				int temp = (int)p;
				int pretemp = predecessor[temp];
				while (pretemp != temp)
				{
					temp = pretemp;
					pretemp = predecessor[temp];
				}
				predecessor[tp[q]] = temp;
			}
		}
	}
	
	//计算每个障碍物的点云并分别储存在不同vector中
	for (int p = (int)predecessor.size() - 1; p >= 0; p--)
	{
		int tp = p;
		int pretp = predecessor[tp];
		if (pretp < 0) continue;
		vector<int> vectp;
		while (pretp != tp)
		{
			vectp.push_back(tp);
			tp = pretp;
			pretp = predecessor[tp];
		}
		for (size_t q = 0; q < vectp.size(); q++)
		{
			predecessor[vectp[q]] = tp;
		}
	}
	for (int p = (int)predecessor.size() - 1; p >= 0; p--)
	{
		int prep = predecessor[p];
		if (prep < 0) continue;
		bool root = true;
		size_t q;
		for (q = 0; q < obstacles.size(); q++)
		{
			if (prep == obstacles[q][0])
			{
				root = false;
				break;
			}
		}
		if (root)
		{
			std::vector<int> newtree;
			newtree.push_back(prep);
			newtree.push_back(p);
			obstacles.push_back(newtree);
		}
		else
		{
			obstacles[q].push_back(p);
		}
	}
	
	for (size_t p = 0; p < obstacles.size(); p++)
		obstacles[p].erase(obstacles[p].end() - 1);
	//计算符合尺度要求的障碍物以过滤去场景信息
	for (int p = (int)obstacles.size() - 1; p >= 0; p--)
	{
		vector<int> tp = RadiusSearch(points.pts[obstacles[p][0]].x, points.pts[obstacles[p][0]].y, _obstaclemaxsize, Myindex);
		if (tp.size() < obstacles[p].size())
		{
			obstacles.erase(obstacles.begin() + p);
		}
	}
	Myindex.freeIndex();

	return obstacles;
}

std::vector<std::pair<double, double> > LIDAR::obstaclesPos(void)
{
	vector<pair<double, double> > obstaclespos;
	vector<vector<int> > obstaclesIdx = calcObstacles();
	
	for (int p = 0; p < (int)obstaclesIdx.size(); p++)
	{
		double xtp = 0;
		double ytp = 0;
                
		for (int q = 0; q < (int)obstaclesIdx[p].size(); q++)
		{
			xtp += (dist[obstaclesIdx[p][q]] * sin(theta[obstaclesIdx[p][q]] / 180 * PI));
			ytp += (dist[obstaclesIdx[p][q]] * cos(theta[obstaclesIdx[p][q]] / 180 * PI));
		}
		xtp /= obstaclesIdx[p].size();
		ytp /= obstaclesIdx[p].size();
		double tp = atan2(xtp, ytp) / PI * 180;
		if (tp < 0) tp += 360;
		obstaclespos.push_back(make_pair(tp, sqrt(xtp*xtp + ytp*ytp)));
	}
	return obstaclespos;
}

vector<pair<double, double> > LIDAR::checkdoAvoidance(std::vector<std::pair<double, double> > &obstacles)
{
    //std::cout << "I am checkdoAvoidance" << std::endl;
    if(doAvoidance==true)
	{
	    doAvoidance=false;
	}

	for (int p = (int)obstacles.size() - 1; p >= 0; p--)
	{
	    if((_doavoidancedist > obstacles[p].second) && (doAvoidance==false))
	    {
		doAvoidance=true;
		break;
	    }
	}
	
    return obstacles;
}


pair<double, double> LIDAR::velocityPlanner(pair<double, double> &curV, pair<double, double> &tarP, vector<pair<double, double> > &obstacles)
//tarP<theta,dist>为目标当前位置,tarV<theta,mag>为飞机当前需要跟踪的速度，以上均为飞机机体坐标系下的值
//其中RPLIDAR A2的安装方式是尾部有线头的朝后，激光雷达的theta是俯视雷达上方时顺时针为正[0,360) degree
//tarV.mag=0时候表面被包围但是无危险,其余时候tarV.mag>0
{
  //std::cout << " tarP " << tarP.first << std::endl;
	pair<double, double> tarV;
	vector<pair<double, double> > emergentobstacles;
	for (int p = (int)obstacles.size() - 1; p >= 0; p--)
	{
	    //std::cout<<_saferadius<<" "<<obstacles[p].second<<std::endl;
		if (_saferadius > obstacles[p].second)
		{
			emergentobstacles.push_back(obstacles[p]);
			obstacles.erase(obstacles.begin() + p);
		}
	}
	

	for (int p = (int)obstacles.size() - 1; p >= 0; p--)
	{
		if (obstacles[p].second > tarP.second)
			obstacles.erase(obstacles.begin() + p);
	}

	vector<pair<double/*lower bound*/, double/*upper bound*/> > thetablocked;
	for (int p = 0; p < (int)obstacles.size(); p++)
	{
		double tp = _saferadius / obstacles[p].second;

		double deltatheta = asin(tp) / PI * 180;

		thetablocked.push_back(make_pair(obstacles[p].first - deltatheta, obstacles[p].first + deltatheta));
	}
	
	thetablockedMerge(thetablocked);

	bool surrounded = false;
	if ((thetablocked.size() == 1) && (thetablocked[0].first < 0) && (thetablocked[0].second > 360))
		surrounded = true;

	//case 0 与障碍物距离小于安全距离，立即规避
	if (emergentobstacles.size() > 0)
	{
	   if(emergentobstacles.size() == 1)
	    {
		double deltatheta = fabs(emergentobstacles[0].first - tarP.first);
		deltatheta = min(deltatheta, fabs(360 - deltatheta));
		if(deltatheta > 90)
		{
		tarV.first = tarP.first;
		tarV.second = _avoidancevelocitymag;
 		return tarV;
		}
		double theta1 = emergentobstacles[0].first - 90;
		double theta2 = emergentobstacles[0].first + 90;
		theta1 = (theta1 < 0) ? (theta1 + 360) : theta1;
		theta2 = (theta2 > 360) ? (theta2 - 360) : theta2;
		
		double a = fabs(theta1 - tarP.first);
		a = min(a, fabs(360 - a));
		
		double b = fabs(theta2 - tarP.first);
		b = min(b, fabs(360 - b));
		
		tarV.first = (a < b) ? theta1 : theta2;
		tarV.second = _avoidancevelocitymag;
		//std::cout << " tarV " << tarV.first << " theta1 " << theta1 << " theta2 " << theta2 << " tarP " << tarP.first << std::endl;
 		return tarV;
	    }
		double xtp = 0;
		double ytp = 0;
		double weightsum = 0;
		//std::cout << emergentobstacles[0].first << "........" << emergentobstacles[0].second << std::endl;
		for (int p = 0; p < (int)emergentobstacles.size(); p++)
		{
			double tp = emergentobstacles[p].first + 180;
			if (tp > 360) tp -= 360;
			double weighttp = 1 / emergentobstacles[p].second;
			xtp += sin(tp * PI / 180)*weighttp;
			ytp += cos(tp * PI / 180)*weighttp;
			weightsum += weighttp;
		}
		
		tarV.first = atan2(xtp, ytp) / PI * 180;
		if (tarV.first < 0) tarV.first += 360;
		tarV.second = _avoidancevelocitymag;
		return tarV;
	}

	//无紧急情况
	bool head = false;
	bool tail = false;
	if (thetablocked.size() > 0)
	{
		head = thetablocked.begin()->first < 0;//头出现奇异情况则为true,以下同理
		tail = (thetablocked.end() - 1)->second>360;
		if (head) thetablocked.begin()->first += 360;
		if (tail) (thetablocked.end() - 1)->second -= 360;
	}

	for (int p = (int)thetablocked.size() - 1; p >= 0; p--)
	{
		if (thetablocked[p].second > thetablocked[p].first)
		{
			if ((thetablocked[p].second < tarP.first) || ((thetablocked[p].first > tarP.first)))
			{
				thetablocked.erase(thetablocked.begin() + p);
			}
		}
		else
		{
			if ((thetablocked[p].second < tarP.first) && (thetablocked[p].first > tarP.first))
			{
				thetablocked.erase(thetablocked.begin() + p);
			}
		}
	}

	//case 1 无障碍，直奔目标
	if (thetablocked.size() == 0)
	{
		tarV.first = tarP.first;
		if(tarP.second < _mindistance / 2)
		  tarV.second = 0;
		else
		  tarV.second = _avoidancevelocitymag;
		return tarV;
	}


	//case 2 被包围，水平速度为0
	if (surrounded)
	{
		tarV.first = 0;
		tarV.second = 0;
		return tarV;
	}

	//case 3 目标被挡住，绕过障碍物
	if ((head&&tail) || (head || tail) && (thetablocked.begin()->first < (thetablocked.end() - 1)->second))//处理2pi周期处的奇异情况
	{
		thetablocked.begin()->first = (thetablocked.end() - 1)->first;
		thetablocked.pop_back();
	}
	
	//std::cout << thetablocked[0].first << " " << thetablocked[0].second << " " << tarP.first << std::endl;

	double tarVtheta = thetablocked[0].first;
	for (int p = 0; p < (int)thetablocked.size(); p++)//最小机动
	{
		double a1 = fabs(thetablocked[p].first - tarP.first);
		a1 = min(a1, fabs(360 - a1));
		double a2 = fabs(thetablocked[p].first - curV.first);
		a2 = min(a2, fabs(360 - a2));
		double a = a1 + a2;
		
		double b1 = fabs(thetablocked[p].second - tarP.first);
		b1 = min(b1, fabs(360 - b1));
		double b2 = fabs(thetablocked[p].second - curV.first);
		b2 = min(b2, fabs(360 - b2));
		double b = b1 + b2;
		
		double c1 = fabs(tarVtheta - tarP.first);
		c1 = min(c1, fabs(360 - c1));
		double c2 = fabs(tarVtheta - curV.first);
		c2 = min(c2, fabs(360 - c2));
		double c = c1 + c2;
		
		if (c > a)
			tarVtheta = thetablocked[p].first;
		else if (c > b)
			tarVtheta = (b < a) ? thetablocked[p].second : tarVtheta;
	}
	tarV = make_pair(tarVtheta, _avoidancevelocitymag);
	return tarV;
}

bool intervalless(const pair<double, double> &p1, const pair<double, double> &p2)
{
	return p1.first < p2.first;
}

void LIDAR::thetablockedMerge(vector<pair<double, double> > &thetablocked)
{
	if (thetablocked.size() == 0) return;
	sort(thetablocked.begin(), thetablocked.end(), intervalless);
	vector<pair<double, double> > thetamerged;
	thetamerged.push_back(thetablocked[0]);
	for (int p = 1; p < (int)thetablocked.size(); p++)
	{
		if (thetablocked[p].first < (*(thetamerged.end() - 1)).second)
			if ((*(thetamerged.end() - 1)).second < thetablocked[p].second)
				(*(thetamerged.end() - 1)).second = thetablocked[p].second;
			else
				continue;
		else
			thetamerged.push_back(thetablocked[p]);
	}
	thetablocked = thetamerged;
}