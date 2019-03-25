#include "loam_velodyne/ObstacleDetection.h"
#include "math_utils.h"


namespace loam {

ObstacleDetection::ObstacleDetection(int fusion_num) :
  _laserCloud(new pcl::PointCloud<pcl::PointXYZI>()),
  _fusion_num(fusion_num)
{
  for(int i=0;i<fusion_num;i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_laser(new pcl::PointCloud<pcl::PointXYZI>());
    tmp_laser->clear();
    _laserCloudStack.push_back(tmp_laser);
  }
  obs_single = cv::Mat::zeros(325, 200, CV_8UC3);
  obs_multi = cv::Mat::zeros(325, 200, CV_8UC3);
}

bool ObstacleDetection::setup(ros::NodeHandle &node){
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_registered", 2, &ObstacleDetection::laserCloudHandler, this);
  _subOdometry = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &ObstacleDetection::odometryHandler, this);
  return true;
}

void ObstacleDetection::spin(){
  ros::Rate rate(100);
  bool status = ros::ok();

  while(status){
    ros::spinOnce();
    process();
    status = ros::ok();
    rate.sleep();
  }
}

void ObstacleDetection::process(){
  if(!hasNewData()){
    return;
  }
  reset();

  //push laser cloud to stack
  _laserCloudStack[_cur_pos]->clear();
  for(auto const& pt : _laserCloud->points){
    _laserCloudStack[_cur_pos]->push_back(pt);
  }
  _cur_pos = (_cur_pos+1) % _fusion_num;

  pcl::PointXYZI pointsel;

  //process single frame
  for(auto const& pt : _laserCloud->points){
    pointToOrigin(pt, pointsel);
    projectPointToGridSingle(pointsel);
  }
  gridAttrToMatSingle();

  //process multi frame
  for(int index=0; index<_fusion_num; index++){
    for(auto const& pt : _laserCloudStack[index]->points){
      pointToOrigin(pt, pointsel);
      projectPointToGridMulti(pointsel);
    }
  }
  gridAttrToMatMulti();

  cv::imshow("single", obs_single);
  cv::imshow("multi", obs_multi);
  cv::waitKey(3);
}

bool ObstacleDetection::hasNewData(){
  return _newLaserCloud && _newOdometry && fabs((_timeLaserCloud-_timeOdometry).toSec())<0.005;
}

void ObstacleDetection::reset(){
  _newLaserCloud = false;
  _newOdometry = false;

  //reset mat
  for(int row = 0; row < 325; row++){
    for(int col = 0; col <200; col++){
      obs_single.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  //reset grid
  for(int row = 0; row < 325; row++){
    for(int col = 0; col <200; col++){
      _grid_attr_single[row][col].valid = false;
      _grid_attr_single[row][col].attribute = UNKNOWN;
    }
  }

  for(int row = 0; row < 325; row++){
    for(int col = 0; col <200; col++){
      obs_multi.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int row = 0; row < 325; row++){
    for(int col = 0; col <200; col++){
      _grid_attr_multi[row][col].valid = false;
      _grid_attr_multi[row][col].attribute = UNKNOWN;
    }
  }
}

void ObstacleDetection::pointToOrigin(const pcl::PointXYZI &pi, pcl::PointXYZI &po){
  pcl::PointXYZI pt_tmp;
  pt_tmp.x = pi.x - _transformSum.pos.x();
  pt_tmp.y = pi.y - _transformSum.pos.y();
  pt_tmp.z = pi.z - _transformSum.pos.z();
  pt_tmp.intensity = pi.intensity;

  rotateYXZ(pt_tmp, -_transformSum.rot_y, -_transformSum.rot_x, -_transformSum.rot_z);

  po.x = pt_tmp.z;
  po.y = pt_tmp.x;
  po.z = pt_tmp.y;
  po.intensity = pt_tmp.intensity;
}

void ObstacleDetection::projectPointToGridSingle(pcl::PointXYZI pi){
  //int use floor, int(-0.1) = -1
  int row = 249 - int(pi.x*100.0/_grid_size);
  int col = 99 - int(pi.y*100.0/_grid_size);

  if(0<=row && row<325 && 0<=col && col<200){
    if(_grid_attr_single[row][col].valid == false){
      _grid_attr_single[row][col].valid = true;
      _grid_attr_single[row][col].min_height = 100.0 * pi.z;
      _grid_attr_single[row][col].max_height = 100.0 * pi.z;
    }else{
      if(100.0*pi.z < _grid_attr_single[row][col].min_height){
        _grid_attr_single[row][col].min_height = 100.0 * pi.z;
      }
      if(100.0*pi.z > _grid_attr_single[row][col].max_height){
        _grid_attr_single[row][col].max_height = 100.0 * pi.z;
      }
    }
  }
}

void ObstacleDetection::projectPointToGridMulti(pcl::PointXYZI pi){
  //int use floor, int(-0.1) = -1
  int row = 249 - int(pi.x*100.0/_grid_size);
  int col = 99 - int(pi.y*100.0/_grid_size);

  if(0<=row && row<325 && 0<=col && col<200){
    if(_grid_attr_multi[row][col].valid == false){
      _grid_attr_multi[row][col].valid = true;
      _grid_attr_multi[row][col].min_height = 100.0 * pi.z;
      _grid_attr_multi[row][col].max_height = 100.0 * pi.z;
    }else{
      if(100.0*pi.z < _grid_attr_multi[row][col].min_height){
        _grid_attr_multi[row][col].min_height = 100.0 * pi.z;
      }
      if(100.0*pi.z > _grid_attr_multi[row][col].max_height){
        _grid_attr_multi[row][col].max_height = 100.0 * pi.z;
      }
    }
  }
}

void ObstacleDetection::gridAttrToMatSingle(){
  for(int row = 0; row < 325; row++){
    for(int col = 0; col<200; col++){
      if(_grid_attr_single[row][col].valid){
        float height_diff = _grid_attr_single[row][col].max_height - _grid_attr_single[row][col].min_height;
        if(height_diff > _obsHeightThreshhold){
          _grid_attr_single[row][col].attribute = OBS;
        }else{
          _grid_attr_single[row][col].attribute = FLAT;
        }
      }
    }
  }

  for(int row=0; row<325; row++){
    for(int col=0; col<200; col++){
      if(_grid_attr_single[row][col].attribute == UNKNOWN){
        obs_single.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);  //black
      }
      if(_grid_attr_single[row][col].attribute == FLAT){
        obs_single.at<cv::Vec3b>(row, col) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr_single[row][col].attribute == OBS){
        obs_single.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);  //black
      }
    }
  }
}

void ObstacleDetection::gridAttrToMatMulti(){
  for(int row = 0; row < 325; row++){
    for(int col = 0; col<200; col++){
      if(_grid_attr_multi[row][col].valid){
        float height_diff = _grid_attr_multi[row][col].max_height - _grid_attr_multi[row][col].min_height;
        if(height_diff > _obsHeightThreshhold){
          _grid_attr_multi[row][col].attribute = OBS;
        }else{
          _grid_attr_multi[row][col].attribute = FLAT;
        }
      }
    }
  }

  for(int row=0; row<325; row++){
    for(int col=0; col<200; col++){
      if(_grid_attr_multi[row][col].attribute == UNKNOWN){
        obs_multi.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);  //black
      }
      if(_grid_attr_multi[row][col].attribute == FLAT){
        obs_multi.at<cv::Vec3b>(row, col) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr_multi[row][col].attribute == OBS){
        obs_multi.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 255);  //black
      }
    }
  }
}

void ObstacleDetection::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  _timeLaserCloud = laserCloudMsg->header.stamp;
  _sequence = laserCloudMsg->header.seq;
  _newLaserCloud = true;
  _laserCloud->clear();
  pcl::fromROSMsg(*laserCloudMsg, *_laserCloud);
}

void ObstacleDetection::odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg){
  _timeOdometry = odometryMsg->header.stamp;
  _newOdometry = true;
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odometryMsg->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
  _transformSum.rot_x = -pitch;
  _transformSum.rot_y = -yaw;
  _transformSum.rot_z = roll;
  _transformSum.pos.x() = float(odometryMsg->pose.pose.position.x);
  _transformSum.pos.y() = float(odometryMsg->pose.pose.position.y);
  _transformSum.pos.z() = float(odometryMsg->pose.pose.position.z);
}


}
