#include "GlobalLocalization.h"
#include "CSMmap.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <time.h>
#include <cstdlib>

using namespace Eigen;
using namespace std;

namespace amcl
{

GlobalLocalization::GlobalLocalization(Vector3d &init, double res,
                                       MatrixXd &GaussianMap, double ox, double oy)
{
    initPose = init;
    GaussianMap_ = GaussianMap;
    numParticels = 10000;
    //odom_params = [0.001 0.001 0.0001 0.0001 ]
    odom_params[0] = 0.001;
    odom_params[1] = 0.001;
    odom_params[2] = 0.0001;
    odom_params[3] = 0.0001;
    // = {0.7,0.2,0.1,0.1};
    zParam[0] = 0.7;
    zParam[1] = 0.2;
    zParam[2] = 0.1;
    zParam[3] = 0.1;
    origin_x = ox;
    origin_y = oy;

    cout << "CSMmap_ != NULL... origin_x = " << origin_x << "---origin_y = " << origin_y << endl;

    resolution_ = res;
    cout << "size of map sizeX = " << GaussianMap_.rows() << "----- sizeY = " << GaussianMap_.cols() << endl;
}

GlobalLocalization::~GlobalLocalization()
{
}

void GlobalLocalization::predict_move(MatrixXd &map, Vector3d &Pre_odom, Vector3d &Cur_odom) // check map value == 0 for unknow space
{
    double delta_x = Cur_odom(0) - Pre_odom(0);
    double delta_y = Cur_odom(1) - Pre_odom(1);
    double rot1 = atan2(delta_y, delta_x) - Pre_odom(2);
    double trans = sqrt(delta_x * delta_x + delta_y * delta_y);
    double rot2 = Cur_odom(2) - Pre_odom(2) - rot1;
    int num = particles.size();

    double rot1temp = sqrt(odom_params[0] * rot1 * rot1 + odom_params[1] * trans * trans);
    double transtemp = sqrt(odom_params[2] * trans * trans + odom_params[3] * rot2 * rot2 + odom_params[3] * rot1 * rot1);
    double rot2temp = sqrt(odom_params[0] * rot2 * rot2 + odom_params[1] * trans * trans);
    Vector3d predict;
    Vector3d current;
    double rot1Noise, transNoise, rot2Noise;
    float scale = 1 / resolution_;
    int size_x = GaussianMap_.rows();
    int size_y = GaussianMap_.cols();

    swap.clear();
    for (int i = 1; i < num; i++)
    {
        rot1Noise = rot1 - sampleNormal() * rot1temp;
        transNoise = trans - sampleNormal() * transtemp;
        rot2Noise = rot2 - sampleNormal() * rot2temp;
        current = particles[i].first;
        predict(0) = current(0) + transNoise * cos(current(2) + rot1Noise);
        predict(1) = current(1) + transNoise * sin(current(2) + rot1Noise);
        predict(2) = current(2) + rot1Noise + rot2Noise;
        particles[i].first = predict;

        if (num > 100)
        {
            int map_x = round((predict(0) - origin_x) * scale);
            int map_y = round((predict(1) - origin_y) * scale);
            if (map_x > size_x || map_y > size_y || map_x < -1 || map_y < -1)
            {
                continue;
            }
            else
            {
                if (map(map_x, map_y) == -1)
                {
                    swap.push_back(make_pair(predict, particles[i].second));
                }
            }
        }
    }

    if (num > 100)
    {
        particles.clear();
        for (int i = 0; i < swap.size(); i++)
        {
            particles.push_back(make_pair(swap[i].first, swap[i].second));
        }
    }
}

void GlobalLocalization::observe_ROSLikelihood(double *laserRange, double *theta,
                                               int range_count, double maxRange, int interval) // each sample call , return weight of each sample
{
    int num = particles.size();
    Vector3d pose;
    Vector2d laserPosition;
    Vector2d end;
    double sum = 0;
    double weight;

    float scale = 1 / resolution_;
    int size_y = GaussianMap_.cols();
    int size_x = GaussianMap_.rows();
    for (int i = 0; i < num; i++)
    {
        pose = particles[i].first;
        laserPosition(0) = pose(0) + 0.19 * cos(pose(2)) - origin_x;
        laserPosition(1) = pose(1) + 0.19 * sin(pose(2)) - origin_y;
        weight = 0;
        for (int j = 0; j < range_count; j = j + interval)
        {
            if (laserRange[j] > maxRange)
                continue;
            end(0) = laserPosition(0) + laserRange[j] * cos(theta[j] + pose(2));
            end(1) = laserPosition(1) + laserRange[j] * sin(theta[j] + pose(2));
            int idx_x = round(end(0) * scale);
            int idx_y = round(end(1) * scale);
            if (idx_y < size_y && idx_x > -1 && idx_x < size_x && idx_y > -1)
            {
                weight = weight + GaussianMap_(idx_x, idx_y);
            }
        }
        particles[i].second = weight;
        sum = sum + weight;
    }

    cout << "sum = " << sum << endl;
    weight = 1 / sum;
    for (int i = 0; i < num; i++)
    {
        particles[i].second = particles[i].second * weight;
    }

    sort(particles.begin(), particles.end(), judge); /////////////// sort

    if (particles.size() > 200)
    {
        swap.clear();
        for (int i = 0; i < 100; i++)
        {
            swap.push_back(make_pair(particles[i].first, particles[i].second));
        }
        particles.clear();
        for (int i = 0; i < 100; i++)
        {
            particles.push_back(make_pair(swap[i].first, swap[i].second));
        }
    }
}

double GlobalLocalization::observe_ROSBeamModel(double *laserRange, double *theta,
                                                int range_count, MatrixXd &map, int interval)
{
    float std_dev_hit = 0.2;
    float lambda_short = 0.1;
    float laser_max_range = 10;
    double q = 0;
    float zHit = zParam[0], zShort = zParam[1], zMax = zParam[2], zRand = zParam[3];

    int num = particles.size();
    vector<Vector3d> expect;
    Vector3d laserWorld;
    double sum = 0;

    for (int i = 0; i < num; i++)
    {
        laserWorld(0) = particles[i].first(0) + 0.19 * cos(particles[i].first(2));
        laserWorld(1) = particles[i].first(1) + 0.19 * sin(particles[i].first(2));
        laserWorld(2) = particles[i].first(2);
        expect.clear();
        ROSFindExpectedRange(laserWorld, theta, range_count, map, laser_max_range, interval, expect);
        int expectSize = expect.size();

        q = 0;
        int count = 0;
        for (int j = 0; j < range_count; j = j + interval)
        {
            double laserExpect = expect[count](2);
            double pHit = calcProbHit(laserRange[j], laserExpect, laser_max_range, std_dev_hit);
            double pShort = calcProbShort(laserRange[j], laserExpect, laser_max_range, lambda_short);
            double pMax = calcProbMax(laserRange[j], laserExpect, laser_max_range);
            double pRand = calcProbRand(laserRange[j], laserExpect, laser_max_range);

            double p = zHit * pHit + zShort * pShort + zMax * pMax + zRand * pRand;
            q = q + p * p * p;
            count++;
        }

        particles[i].second = q;

        sum = sum + q;
    }

    q = 1 / sum;

    for (int i = 0; i < num; i++)
    {
        particles[i].second = particles[i].second * q;
    }
}

void GlobalLocalization::ROSFindExpectedRange(Vector3d &laserPose, double *theta,
                                              int range_count, MatrixXd &map, double maxRange,
                                              int interval, vector<Vector3d> &expect)
{
    vector<double> index_theta;
    int size_x = map.rows();
    int size_y = map.cols();
    index_theta.clear();
    for (int i = 0; i < range_count; i = i + interval)
    {
        double temp = theta[i] + laserPose(2);
        index_theta.push_back(temp);
    }
    int size = index_theta.size(); // 1081/10 = 109 rays
    double scale = 1 / resolution_;
    int size_r = int(maxRange * scale); // 200
    vector<double> r;
    for (int i = 0; i < size_r; i++)
    {
        r.push_back(i + 1);
    }

    for (int i = 0; i < size; i++)
    {
        Vector3d r_vec = Vector3d::Zero();
        r_vec(0) = cos(index_theta[i]);
        r_vec(1) = sin(index_theta[i]);
        for (int j = 0; j < size_r; j++)
        {
            Vector3d map_index;
            map_index(0) = r_vec(0) * r[j] * resolution_ + laserPose(0);
            map_index(1) = r_vec(1) * r[j] * resolution_ + laserPose(1);
            map_index(2) = r[j] * resolution_;
            int idx_x = round((map_index(0) - origin_x) * scale);
            int idx_y = round((map_index(1) - origin_y) * scale);
            if (idx_y < size_y && idx_x > -1 && idx_x < size_x && idx_y > -1)
            {
                if (map(idx_x, idx_y) == 1)
                {
                    expect.push_back(map_index);
                    break;
                }
                else if (map(idx_x, idx_y) == 0)
                {
                    map_index(2) = maxRange;
                    expect.push_back(map_index);
                    break;
                }
            }

            if (j == size_r - 1)
            {
                Vector3d temp = Vector3d::Zero();
                temp(2) = maxRange;
                expect.push_back(temp);
            }
        }
    }

    int count = expect.size();

    if (count != size)
        cout << "error in expect.push_back() !" << endl;
}

void GlobalLocalization::GiveWholeMapRandom(MatrixXd &map, int valueNum) //map -> map world coordinate ,  test done
{
    int sizeX = map.rows(); // vector<pair<Vector3d,double>> particles
    int sizeY = map.cols();
    int numCell = 0;
    vector<Vector3d> pose;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            if (map(i, j) == -1)
            {
                Vector3d p = Vector3d::Zero();
                p(0) = i * resolution_ + origin_x;
                p(1) = j * resolution_ + origin_y;
                pose.push_back(p);
                numCell++;
            }
        }
    }

    particles.clear();
    int numParticles = ceil(resolution_ * resolution_ * numCell * valueNum);
    double weight = 1.0 / numParticles;
    srand((unsigned)time(NULL)); // rand()/double(RAND_MAX) M_PI
    for (int i = 0; i < numParticles; i++)
    {
        int index = ceil(numCell * (rand() / double(RAND_MAX)));
        Vector3d p1 = Vector3d::Zero();
        p1(0) = pose[index](0);
        p1(1) = pose[index](1);
        p1(2) = 2 * M_PI * (rand() / double(RAND_MAX));
        particles.push_back(make_pair(p1, weight));
    }

    //cout << "numParticles = " << numParticles << "---- particles.size() = " << particles.size() << endl;
}

void GlobalLocalization::GenerateRandomPose(float range, float angle)
{
    int num = particles.size();
    srand((unsigned)time(NULL));
    Vector3d delta;
    Vector3d center;
    tempparticels.clear();
    for (int j = 0; j < num; j++)
    {
        center = particles[j].first;
        int number = ceil((particles[j].second) * numParticels);
        for (int i = 0; i < number; i++)
        {
            delta(0) = center(0) + (rand() / double(RAND_MAX) - 0.5) * range;
            delta(1) = center(1) + (rand() / double(RAND_MAX) - 0.5) * range;
            delta(2) = center(2) + (rand() / double(RAND_MAX) - 0.5) * angle;

            tempparticels.push_back(delta);
        }
    }

    int size = tempparticels.size();
    double scale = 1.0 / size;
    particles.clear();
    for (int i = 0; i < size; i++)
    {
        particles.push_back(make_pair(tempparticels[i], scale));
    }
    cout << "GenerateRandomPose particles.size() = " << particles.size() << endl;
}

void GlobalLocalization::InitPoseLocalization(MatrixXd& map, double *initPose, int poseDimension, double radius, int number)
{
    if (poseDimension != 3)
    {
        cout << "error! Pose is not 3 dimension " << endl;
        return;
    }

    double init_x = initPose[0];
    double init_y = initPose[1];
    double init_theta = initPose[2];
    int sizeX = map.rows();
    int sizeY = map.cols();
    int numCell = 0;
    double scale = 1.0 / resolution_;
    int map_x = (initPose[0] - origin_x) * scale;
    int map_y = (initPose[1] - origin_y) * scale;
    if(map_x < -1 || map_x > sizeX)
    {
        cout << " initPose X direction is not in map ! " << endl;
        init_x = (sizeX - 1) * resolution_ + origin_x;
    }

	if(map_y < -1 || map_y > sizeY)
	{
		cout << " initPose Y direction is not in map ! " << endl;
        init_y = (sizeY - 1) * resolution_ + origin_y;
	}
	

    double r_square = radius * radius;
    vector<Vector3d> pose;
	pose.clear();
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            if (map(i, j) == -1)
            {
                Vector3d p = Vector3d::Zero();
                p(0) = i * resolution_ + origin_x;
                p(1) = j * resolution_ + origin_y;
                double distance = (p(0)-init_x) * (p(0)-init_x) + (p(1)-init_y) * (p(1)-init_y);
                if(r_square >= distance)
                {
                    pose.push_back(p);
                    numCell++;
                }
            }
        }
    }
     
    particles.clear();
	if(pose.size() == 0)
		return;
    double weight = 1.0 / number;
    srand((unsigned)time(NULL)); // rand()/double(RAND_MAX) M_PI
    for (int i = 0; i < number; i++)
    {
        int index = ceil(numCell * (rand() / double(RAND_MAX)));
        Vector3d p1 = Vector3d::Zero();
        p1(0) = pose[index](0);
        p1(1) = pose[index](1);
        p1(2) = 2 * M_PI * (rand() / double(RAND_MAX));
        particles.push_back(make_pair(p1, weight));
    }
}

void GlobalLocalization::cov(double *cvx, double *cvy, double *cvtheta)
{
    if (particles.size() > 100)
    {
        cout << "particles.size() > 100 can not caculate cov !! " << endl;
        return;
    }
    int size = particles.size();
    double expect_x = 0, expect_y = 0, expect_theta = 0;
    double cov_x = 0, cov_y = 0, cov_theta = 0;
    for (int i = 0; i < size; i++)
    {
        expect_x = expect_x + particles[i].first(0);
        expect_y = expect_y + particles[i].first(1);
        expect_theta = expect_theta + particles[i].first(2);
    }
    expect_x = expect_x / size;
    expect_y = expect_y / size;
    expect_theta = expect_theta / size;

    for (int i = 0; i < size; i++)
    {
        cov_x = cov_x + (particles[i].first(0) - expect_x) * (particles[i].first(0) - expect_x);
        cov_y = cov_y + (particles[i].first(1) - expect_y) * (particles[i].first(1) - expect_y);
        cov_theta = cov_theta + (particles[i].first(2) - expect_theta) * (particles[i].first(2) - expect_theta);
    }
    *cvx = cov_x / (size - 1);
    *cvy = cov_y / (size - 1);
    *cvtheta = cov_theta / (size - 1);
}

double GlobalLocalization::SearchBestAlign(Eigen::Vector3d *Pose1, double *laserRange, double *theta,
                                           int range_count, float xRange, float yRange, float del_Range,
                                           float ThRange, float del_Th, bool cal_cov, Eigen::Matrix3d *cov,
                                           double maxRange)
{
    int range_y = GaussianMap_.cols();
    int range_x = GaussianMap_.rows();

    Eigen::Vector3d Pose = *Pose1;
    int size_theta = (2 * ThRange / del_Th) + 1;
    int size_x = (2 * xRange / del_Range) + 1;
    int size_y = (2 * yRange / del_Range) + 1;

    std::vector<double> thetas, x_search, y_search;
    for (int i = 0; i < size_theta; i++)
    {
        thetas.push_back(-ThRange + del_Th * i);
    }
    for (int i = 0; i < size_x; i++)
    {
        x_search.push_back(-xRange + del_Range * i);
    }
    for (int i = 0; i < size_y; i++)
    {
        y_search.push_back(-yRange + del_Range * i);
    }

    double best_score = -10000;
    double sum_score = 0;
    Vector3d best_align = Vector3d(0, 0, 0);

    std::vector<std::vector<std::vector<double>>> ScoreMap(size_theta, vector<vector<double>>(size_x, vector<double>(size_y, 0)));

    for (int z = 0; z < size_theta; z++)
    {
        double LaserAngle = Pose(2) + thetas[z];
        std::vector<Vector2d> EPinLaser;
        Vector2d ep;
        for (int i = 0; i < range_count; i++)
        {
            if (laserRange[i] > maxRange)
                continue;

            ep(0) = laserRange[i] * cos(theta[i] + LaserAngle);
            ep(1) = laserRange[i] * sin(theta[i] + LaserAngle);
            EPinLaser.push_back(ep);
        }

        for (int x = 0; x < size_x; x++)
        {
            for (int y = 0; y < size_y; y++)
            {
                double score = 0;
                Vector2d laserPose;
                laserPose(0) = Pose(0) + x_search[x] + 0.19 * cos(LaserAngle) - origin_x;
                laserPose(1) = Pose(1) + y_search[y] + 0.19 * sin(LaserAngle) - origin_y;
                float scale = 1 / resolution_;
                for (int i = 0; i < EPinLaser.size(); i++)
                {
                    int index_x = (laserPose(0) + EPinLaser[i](0)) * scale;
                    int index_y = (laserPose(1) + EPinLaser[i](1)) * scale;

                    if (index_y < range_y && index_x > -1 && index_x < range_x && index_y > -1)
                    {
                        score = score + GaussianMap_(index_x, index_y);
                    }
                }

                if (score > best_score)
                {
                    best_score = score;
                    best_align(0) = Pose(0) + x_search[x];
                    best_align(1) = Pose(1) + y_search[y];
                    best_align(2) = LaserAngle;
                }
                ScoreMap[z][x][y] = score;
                sum_score = sum_score + ScoreMap[z][x][y];
            }
        }
    }
    *Pose1 = best_align;
    if (cal_cov)
    {
        double accumulatedVarianceXX = 0;
        double accumulatedVarianceXY = 0;
        double accumulatedVarianceYY = 0;
        double accumulatedVarianceThTh = 0;
        double norm = 0;
        Matrix3d Covariance = Matrix3d::Zero();
        Covariance(0, 0) = 100;
        Covariance(1, 1) = 100;
        Covariance(2, 2) = 100;

        for (int z = 0; z < size_theta; z++)
        {
            for (int x = 0; x < size_x; x++)
            {
                for (int y = 0; y < size_y; y++)
                {
                    if ((ScoreMap[z][x][y]) >= (best_score * 0.8))
                    {
                        double response = ScoreMap[z][x][y] / sum_score;
                        norm = norm + response;
                        Vector3d select;
                        select(0) = Pose(0) + x_search[x];
                        select(1) = Pose(1) + y_search[y];
                        select(2) = Pose(2) + thetas[z];
                        accumulatedVarianceXX = accumulatedVarianceXX + (best_align(0) - select(0)) * (best_align(0) - select(0)) * response;
                        accumulatedVarianceXY = accumulatedVarianceXY + (best_align(0) - select(0)) * (best_align(1) - select(1)) * response;
                        accumulatedVarianceYY = accumulatedVarianceYY + (best_align(1) - select(1)) * (best_align(1) - select(1)) * response;
                        accumulatedVarianceThTh = (best_align(2) - select(2)) * (best_align(2) - select(2)) * response;
                    }
                }
            }
        }

        if (norm > 0)
        {
            double varianceXX = accumulatedVarianceXX / norm;
            double varianceXY = accumulatedVarianceXY / norm;
            double varianceYY = accumulatedVarianceYY / norm;
            double varianceThTh = accumulatedVarianceThTh / norm;

            double minVarianceXX = 0.1 * resolution_ * resolution_;
            double minVarianceYY = 0.1 * resolution_ * resolution_;
            double minVarianceThTh = (0.5 / 180 * M_PI) * (0.5 / 180 * M_PI);
            varianceXX = varianceXX > minVarianceXX ? varianceXX : minVarianceXX;
            varianceYY = varianceYY > minVarianceYY ? varianceYY : minVarianceYY;
            varianceThTh = varianceThTh > minVarianceThTh ? varianceThTh : minVarianceThTh;

            double multiplier = 1;
            Covariance(0, 0) = varianceXX * multiplier;
            Covariance(0, 1) = varianceXY * multiplier;
            Covariance(1, 0) = varianceXY * multiplier;
            Covariance(1, 1) = varianceYY * multiplier;
            Covariance(2, 2) = varianceThTh * multiplier;
        }
        if (Covariance(0, 0) * Covariance(0, 0) < 0.00000001)
            Covariance(0, 0) = 100;
        if (Covariance(1, 1) * Covariance(1, 1) < 0.00000001)
            Covariance(1, 1) = 100;
        *cov = Covariance;
    }

    return best_score;
}

double GlobalLocalization::test_ROSBeamModel(double *laserRange, double *theta, int range_count, MatrixXd &map, int interval, Vector3d pose)
{
    float std_dev_hit = 0.2;
    float lambda_short = 0.1;
    float laser_max_range = 10;
    double q = 0;
    float zHit = zParam[0], zShort = zParam[1], zMax = zParam[2], zRand = zParam[3];

    vector<Vector3d> expect;
    Vector3d laserWorld;
    double sum = 0;

    laserWorld(0) = pose(0) + 0.19 * cos(pose(2));
    laserWorld(1) = pose(1) + 0.19 * sin(pose(2));
    laserWorld(2) = pose(2);
    expect.clear();

    ROSFindExpectedRange(laserWorld, theta, range_count, map, laser_max_range, interval, expect);
    int expectSize = expect.size();

    q = 0;
    int count = 0;
    for (int j = 0; j < range_count; j = j + interval)
    {
        double laserExpect = expect[count](2);
        double pHit = calcProbHit(laserRange[j], laserExpect, laser_max_range, std_dev_hit);
        double pShort = calcProbShort(laserRange[j], laserExpect, laser_max_range, lambda_short);
        double pMax = calcProbMax(laserRange[j], laserExpect, laser_max_range);
        double pRand = calcProbRand(laserRange[j], laserExpect, laser_max_range);

        double p = zHit * pHit + zShort * pShort + zMax * pMax + zRand * pRand;
        q = q + p * p * p;
        count++;
    }

    //debug
    cloud.width = expect.size();
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].x = expect[i](0);
        cloud.points[i].y = expect[i](1);
        cloud.points[i].z = 0;

        cloud.points[i].r = 0;
        cloud.points[i].g = 255;
        cloud.points[i].b = 0;
    }

    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";

    //pcl_pub.publish(output);
}
}
