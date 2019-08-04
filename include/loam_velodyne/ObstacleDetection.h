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
#include <opencv2/imgproc.hpp>
#include <std_msgs/Int32.h>
#include "loam_velodyne/paremeterUse.h"

namespace loam {

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

class ObstacleDetection{

public:
  ObstacleDetection();
  ~ObstacleDetection();
  bool setup(ros::NodeHandle& node);
  void spin();

private:
  void process();
  bool hasNewData();
  void reset();
  void pointToOrigin(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
  void projectPointToGrid(pcl::PointXYZI pi, Grid** _grid_attr);
  void gridAttrToMat(Grid** _grid_attr, cv::Mat& img);
  void griddiffer();
  void cluster(Grid** _grid_attr, cv::Mat& img_gray, cv::Mat& img_color);
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

  //600*400
  Grid** _grid_attr_single = NULL;  //current front 90m, back 30m, left 40m, right 40m, every grid is 20cm
  cv::Mat obs_single;
  cv::Mat obs_single_draw;

  Grid** _grid_attr_multi = NULL;
  cv::Mat obs_multi;

  Grid** _grid_attr_pre = NULL;
  cv::Mat obs_pre;

  cv::Mat obs_cluster;

  cv::Mat obs_differ;

  cv::Mat img;

  std::string img_path;
  std::vector<std::string> file_lists;

};

}

#endif // OBSTACLEDETECTION_H
