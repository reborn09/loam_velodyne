#ifndef BASICTRACKINGSHOW_H
#define BASICTRACKINGSHOW_H
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include "loam_velodyne/Twist.h"
using namespace std;

loam::Twist _transformSum;

//grid range, dimension m
int range_front_1 = 90;
int range_back_1 = 30;
int range_left_1 = 40;
int range_right_1 = 40;

//obs detect threshhold
float _obsHeightThreshhold_1 = 30.0; //cm
float _cartopHeightThreshhold_1 = 50.0; //cm
float _susHeightThreshhold_1 = 160.0; //cm
int ground_win_size_1 = 3; //estimate ground height during the window size
int exp_grid_size_1 = 1; //expand grid
int _grid_size_1 = 20;

enum GRID_ATTRIBUTE{
  UNKNOWN,  //no lidar point
  FLAT,     //traversable
  OBS,      //not traversable
  SUSPEND,   //suspeng obstacle
  CAR_TOP   //car top
};

class Grid{
public:
  bool valid;
  float min_height; //cm
  float max_height; //cm
  GRID_ATTRIBUTE attribute;
  std::vector<pcl::PointXYZI> cloud;
};

void pointToOrigin(const pcl::PointXYZI &pi, pcl::PointXYZI &po);
void projectPointToGrid(pcl::PointXYZI pi, Grid** _grid_attr);
void gridAttrToMat(Grid** _grid_attr, cv::Mat& img);
void cluster(Grid** _grid_attr, cv::Mat& img_gray, cv::Mat& img_color);

#endif // BASICTRACKINGSHOW_H
