#ifndef GLOBALLOCALIZATION_H
#define GLOBALLOCALIZATION_H

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/LU"
#include "CSMmap.h"
//#include "GlobalHelper.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace Eigen;
using namespace std;

namespace amcl
{

double calcProbHit(double laserRange, double laserExpectRange, float maxRange, float std_dev_hit)
{
    if (laserExpectRange > maxRange)
    {
        cout << "error in calcProbHit() !" << endl;
        //cout << "laserExpectRange > maxRange ---" << laserExpectRange << "---" << maxRange << endl;
        return 0;
    }
    double proHit = normpdf(laserRange, laserExpectRange, std_dev_hit);
    if (laserRange < 0 || laserRange > maxRange)
        proHit = 0;
    return proHit;
}

double calcProbShort(double laserRange, double laserExpectRange, float maxRange, float lambda_short)
{
    double normalizor = 1 / (1 - exp(-lambda_short * laserExpectRange));
    double probShort = lambda_short * exp(-lambda_short * laserRange) * normalizor;
    if (laserRange < 0 || laserRange > laserExpectRange)
        probShort = 0;
    return probShort;
}

double calcProbMax(double laserRange, double laserExpectRange, float maxRange)
{
    if (laserRange < maxRange)
        return 0;
    else
        return 1;
}

double calcProbRand(double laserRange, double laserExpectRange, float maxRange)
{
    double probRand = 1 / maxRange;
    if (laserRange < 0 || laserRange >= maxRange)
        probRand = 0;
    return probRand;
}

double sampleNormal()
{
    double u = ((double)rand() / (RAND_MAX)) * 2 - 1;
    double v = ((double)rand() / (RAND_MAX)) * 2 - 1;
    double r = u * u + v * v;
    if (r == 0 || r > 1)
        return sampleNormal();
    double c = sqrt(-2 * log(r) / r);
    return u * c;
}

bool judge(const pair<Vector3d, double> a, const pair<Vector3d, double> b)
{
    return a.second > b.second;
}

class GlobalLocalization
{
  public:
    GlobalLocalization(Vector3d &init, double res, MatrixXd &map, double ox, double oy);
    virtual ~GlobalLocalization();

    void predict_move(MatrixXd &map, Vector3d &Pre_odom, Vector3d &Cur_odom);                                      // check map value == 0 for unknow space
    void observe_ROSLikelihood(double *laserRange, double *theta, int range_count, double maxRange, int interval); // all sample call , return weight of each sample
    double observe_ROSBeamModel(double *laserRange, double *theta, int range_count, MatrixXd &map, int interval = 10);
    void ROSFindExpectedRange(Vector3d &laserPose, double *theta, int range_count, MatrixXd &map, double maxRange, int interval, vector<Vector3d> &expect);

    void GiveWholeMapRandom(MatrixXd &map, int valueNum = 200); //map -> world coordinate ,
    void GenerateRandomPose(float range, float angle);          //all sample call
	void InitPoseLocalization(MatrixXd& map, double *initPose, int poseDimension = 3, double radius = 10, int number = 10000);

    void cov(double *cvx, double *cvy, double *cvtheta);
    double SearchBestAlign(Eigen::Vector3d *Pose1, double *laserRange, double *theta, int range_count, float xRange, float yRange, float del_Range, float ThRange, float del_Th, bool cal_cov, Eigen::Matrix3d *cov, double maxRange);

    double test_ROSBeamModel(double *laserRange, double *theta, int range_count, MatrixXd &map, int interval, Vector3d pose);

    vector<pair<Vector3d, double>> particles; // pair<pose,weight>
    vector<Vector3d> tempparticels;
    vector<pair<Vector3d, double>> swap;
    Vector3d initPose;
    double origin_x, origin_y;
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

  private:
    MatrixXd GaussianMap_;
    double resolution_;
    int numParticels; // 10000
    float odom_params[4];
    float zParam[4];
};
}
#endif
