#ifndef OBSTACLEDETECTION_H
#define OBSTACLEDETECTION_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "loam_velodyne/Twist.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "loam_velodyne/paremeterUse.h"

namespace loam {

enum GRID_ATTRIBUTE{
  UNKNOWN,  //no lidar point
  FLAT,     //traversable
  OBS       //not traversable
};

class Grid{
public:
  bool valid;
  float min_height; //cm
  float max_height; //cm
  GRID_ATTRIBUTE attribute;
};

class ObstacleDetection{

public:
  ObstacleDetection(int fusion_num = 5);
  bool setup(ros::NodeHandle& node);
  void spin();

private:
  void process();
  bool hasNewData();
  void reset();
  void pointToOrigin(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
  void projectPointToGridSingle(pcl:: PointXYZI pi);
  void projectPointToGridMulti(pcl:: PointXYZI pi);
  void gridAttrToMatSingle();
  void gridAttrToMatMulti();
  void saveResult();
  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg);

private:
  int _sequence = 0;
  int _fusion_num;
  int _cur_pos =0; //current positon in _laserCloudStack

  ros::Time _timeLaserCloud;
  ros::Time _timeOdometry;

  bool _newLaserCloud;
  bool _newOdometry;

  ros::Subscriber _subLaserCloud;
  ros::Subscriber _subOdometry;

  Twist _transformSum;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudStack;

  int _grid_size = 20;  //cm
  float _obsHeightThreshhold = 20.0; //cm

  Grid _grid_attr_single[325][200];  //front 50m, back 15m, left 20m, right 20m, every grid is 20cm
  cv::Mat obs_single;

  Grid _grid_attr_multi[325][200];
  cv::Mat obs_multi;

  cv::Mat img;

  std::string img_path;
  std::vector<std::string> file_lists;

};

}

#endif // OBSTACLEDETECTION_H
