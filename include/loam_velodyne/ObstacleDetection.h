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
#include <std_msgs/Int32.h>
#include "loam_velodyne/paremeterUse.h"
#include <sstream>

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
  ObstacleDetection();
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
  void imageSeqHandler(const std_msgs::Int32::ConstPtr& seqIn);

private:
  int _sequence = 0;
  int _cur_pos =0; //current positon in _laserCloudStack

  ros::Time _timeLaserCloud;
  ros::Time _timeOdometry;

  bool _newLaserCloud;
  bool _newOdometry;
  bool _newImageSeq;

  ros::Subscriber _subLaserCloud;
  ros::Subscriber _subOdometry;
  ros::Subscriber _subImageSeq;

  Twist _transformSum;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudStack;

  int _grid_size = 20;  //cm
  float _obsHeightThreshhold = 30.0; //cm

  Grid _grid_attr_single[600][400];  //front 90m, back 30m, left 40m, right 40m, every grid is 20cm
  cv::Mat obs_single;

  Grid _grid_attr_multi[600][400];
  cv::Mat obs_multi;

  cv::Mat img;

  std::string img_path;
  std::vector<std::string> frameNo;

};

}

#endif // OBSTACLEDETECTION_H
