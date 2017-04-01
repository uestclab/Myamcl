#ifndef CSMMAP_H
#define CSMMAP_H

#include "Eigen/Cholesky"
#include "Eigen/Core" 

#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"







enum MapMode {TRINARY, SCALE, RAW};

namespace amcl{
	//typedef Eigen::MatrixXd GaussianCell; 
    double normpdf(double dist, double mu, double sigma);

class LoadMap {
public:

	LoadMap(const nav_msgs::OccupancyGrid& map_msg);
	LoadMap();
	virtual ~LoadMap();
	//void SearchBestAlign(scan,)

	Eigen::MatrixXd hit_;
	Eigen::MatrixXd loadmap_;
	double origin_x,origin_y;
private:
	double res_;
	int width_, height_;


};

}
#endif
