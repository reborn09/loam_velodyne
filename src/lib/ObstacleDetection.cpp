#include "loam_velodyne/ObstacleDetection.h"
#include "math_utils.h"

namespace loam {

ObstacleDetection::ObstacleDetection() :
  _laserCloud(new pcl::PointCloud<pcl::PointXYZI>())
{
  for(int i=0;i<_fusion_num;i++){
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_laser(new pcl::PointCloud<pcl::PointXYZI>());
    tmp_laser->clear();
    _laserCloudStack.push_back(tmp_laser);
  }

  int row = 5*(range_front + range_back);
  int col = 5*(range_left + range_right);

  _grid_attr_single = new Grid *[row];
  for(int i=0; i<row; i++){
    _grid_attr_single[i] = new Grid[col];
  }

  _grid_attr_multi = new Grid *[row];
  for(int i=0; i<row; i++){
    _grid_attr_multi[i] = new Grid[col];
  }

  _grid_attr_pre = new Grid *[row];
  for(int i=0; i<row; i++){
    _grid_attr_pre[i] = new Grid[col];
  }

  obs_single = cv::Mat::zeros(row, col, CV_8UC3);
  obs_single_draw = cv::Mat::zeros(row, col, CV_8UC3);
  obs_multi = cv::Mat::zeros(row, col, CV_8UC3);
  obs_pre = cv::Mat::zeros(row, col, CV_8UC3);
  obs_differ = cv::Mat::zeros(row, col, CV_8UC3);

  obs_cluster = cv::Mat::zeros(row, col, CV_8UC1);

  std::stringstream ss;
  ss<<img_base_dir<<sequence<<"/image_2/";
  ss>>img_path;
  read_filelists(img_path, file_lists, "png");
  sort_filelists(file_lists, "png");
}

ObstacleDetection::~ObstacleDetection()
{
  int row = 5*(range_front + range_back);
  int col = 5*(range_left + range_right);

  for(int i = 0; i < row; i++){
    delete[] _grid_attr_single[i];
  }
  delete[] _grid_attr_single;

  for(int i = 0; i < row; i++){
    delete[] _grid_attr_multi[i];
  }
  delete[] _grid_attr_multi;

  for(int i = 0; i < row; i++){
    delete[] _grid_attr_pre[i];
  }
  delete[] _grid_attr_pre;
}

bool ObstacleDetection::setup(ros::NodeHandle &node){
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_registered", 2, &ObstacleDetection::laserCloudHandler, this);
  _subOdometry = node.subscribe<nav_msgs::Odometry>
      ("/aft_mapped_to_init", 5, &ObstacleDetection::odometryHandler, this);
  _subImageSeq = node.subscribe<std_msgs::Int32>
      ("/seq_mapping", 2, &ObstacleDetection::imageSeqHandler, this);
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
    double dist = (pt.x - _transformSum.pos.x()) * (pt.x - _transformSum.pos.x())
                + (pt.y - _transformSum.pos.y()) * (pt.y - _transformSum.pos.y())
                + (pt.z - _transformSum.pos.z()) * (pt.z - _transformSum.pos.z());
    dist = sqrt(dist);
    if(dist > 2.85){
      _laserCloudStack[_cur_pos]->push_back(pt);
    }
  }

  pcl::PointXYZI pointsel;

  //process single frame
  for(auto const& pt : _laserCloud->points){
    pointToOrigin(pt, pointsel);
    projectPointToGrid(pointsel, _grid_attr_single);
  }
  gridAttrToMat(_grid_attr_single, obs_single);
  gridAttrToMat(_grid_attr_single, obs_single_draw);
  cluster(_grid_attr_single, obs_cluster, obs_single_draw);

  int _pre_pos = _cur_pos - 1;
  if(_pre_pos < 0){
    _pre_pos = _fusion_num-1;
  }

  for(auto const& pt : _laserCloudStack[_pre_pos]->points){
    pointToOrigin(pt, pointsel);
    projectPointToGrid(pointsel, _grid_attr_pre);
  }
  gridAttrToMat(_grid_attr_pre, obs_pre);
  griddiffer();

  //process multi frame
  for(int index=0; index<_fusion_num; index++){
    for(auto const& pt : _laserCloudStack[index]->points){
      pointToOrigin(pt, pointsel);
      projectPointToGrid(pointsel, _grid_attr_multi);
    }
  }
  gridAttrToMat(_grid_attr_multi, obs_multi);

  std::string file_path = img_path + file_lists[_sequence];
  img = cv::imread(file_path);

#ifdef SAVE_RESULT
  saveResult();
#endif

  cv::imshow("image", img);
  cv::imshow("single", obs_single);
  cv::imshow("draw", obs_single_draw);
  cv::imshow("cluster", obs_cluster);
  cv::imshow("multi", obs_multi);
  //cv::imshow("pre", obs_pre);
  //cv::imshow("differ",obs_differ);
  cv::waitKey(3);

  _cur_pos = (_cur_pos+1) % _fusion_num;
}

bool ObstacleDetection::hasNewData(){
  return _newLaserCloud && _newOdometry && _newImageSeq
      && fabs((_timeLaserCloud-_timeOdometry).toSec())<0.005;
}

void ObstacleDetection::reset(){
  _newLaserCloud = false;
  _newOdometry = false;
  _newImageSeq = false;

  int row = 5*(range_front + range_back);
  int col = 5*(range_left + range_right);

  //reset mat
  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_single.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_single_draw.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  //reset grid
  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      _grid_attr_single[i][j].valid = false;
      _grid_attr_single[i][j].attribute = UNKNOWN;
      _grid_attr_single[i][j].cloud.clear();
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_multi.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      _grid_attr_multi[i][j].valid = false;
      _grid_attr_multi[i][j].attribute = UNKNOWN;
      _grid_attr_multi[i][j].cloud.clear();
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_pre.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      _grid_attr_pre[i][j].valid = false;
      _grid_attr_pre[i][j].attribute = UNKNOWN;
      _grid_attr_pre[i][j].cloud.clear();
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int i = 0; i<row; i++){
    for(int j = 0; j < col; j++){
      obs_cluster.at<uchar>(i, j) = 0; //black
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

void ObstacleDetection::projectPointToGrid(pcl::PointXYZI pi, Grid** _grid_attr){
  int row = 5*range_front - 1 - int(pi.x*100.0/_grid_size);
  int col = 5*range_left - 1 - int(pi.y*100.0/_grid_size);

  int row_max = 5*(range_front+range_back);
  int col_max = 5*(range_left+range_right);

  if(0 <= row && row < row_max && 0 <= col && col < col_max){
    if(_grid_attr[row][col].valid == false){
      _grid_attr[row][col].valid = true;
      _grid_attr[row][col].min_height = 100.0 * pi.z;
      _grid_attr[row][col].max_height = 100.0 * pi.z;
    }else{
      if(100.0*pi.z < _grid_attr[row][col].min_height){
        _grid_attr[row][col].min_height = 100.0*pi.z;
      }
      if(100.0*pi.z > _grid_attr[row][col].max_height){
        _grid_attr[row][col].max_height = 100.0*pi.z;
      }
    }
    _grid_attr[row][col].cloud.push_back(pi);
  }

}

void ObstacleDetection::gridAttrToMat(Grid** _grid_attr, cv::Mat& img){
  int row = 5*(range_front+range_back);
  int col = 5*(range_left+range_right);

  for(int i=0; i<row; i++){
    for(int j=0; j<col; j++){
      if(_grid_attr[i][j].valid){
        float height_diff = _grid_attr[i][j].max_height - _grid_attr[i][j].min_height;
        if(height_diff > _obsHeightThreshhold){
          _grid_attr[i][j].attribute = OBS;
        }else{
          _grid_attr[i][j].attribute = FLAT;
        }
      }
    }
  }

  //find car top
  for(int i = 0; i <row; i++){
    for(int j = 0; j < col; j++){
      if(_grid_attr[i][j].valid && _grid_attr[i][j].attribute != OBS){
        float ground_height = 0;
        int count = 0 ;
        for(int loop1 = i - ground_win_size;loop1<=i+ground_win_size; loop1++){
          for(int loop2 = j - ground_win_size; loop2<=j+ground_win_size; loop2++){
            if(loop1>=0 && loop1<row && loop2>=0 && loop2 <col){
              if(_grid_attr[loop1][loop2].attribute == FLAT){
                ground_height += _grid_attr[loop1][loop2].min_height;
                count++;
              }
            }
          }
        }
        if(count >= 1){
          ground_height /= count;
          if(_grid_attr[i][j].max_height - ground_height > _cartopHeightThreshhold){
            //_grid_attr[i][j].attribute = CAR_TOP;
            _grid_attr[i][j].attribute = OBS;
          }
        }
      }
    }
  }

  //find suspend
  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      if(_grid_attr[i][j].attribute == OBS){
        sort(_grid_attr[i][j].cloud.begin(),
             _grid_attr[i][j].cloud.end(),
             [](const pcl::PointXYZI &p1, const pcl::PointXYZI &p2)->bool{return p1.z < p2.z;});
        int count = _grid_attr[i][j].cloud.size();
        if(count > 0){
          float h_pre = _grid_attr[i][j].cloud[0].z*100.0;
          float h_now = _grid_attr[i][j].cloud[0].z*100.0;
          float h_dis = h_now - h_pre;
          for(int index = 0; index<count; index++){
            h_now = _grid_attr[i][j].cloud[index].z*100.0;
            h_dis = h_now - h_pre;
            if(h_dis > _susHeightThreshhold){
              //_grid_attr[i][j].attribute = SUSPEND;
              _grid_attr[i][j].attribute = FLAT;
              break;
            }
            h_pre = _grid_attr[i][j].cloud[index].z*100.0;
          }
        }
      }
    }
  }

  //draw
  for(int i=0; i<row; i++){
    for(int j=0; j<col; j++){
      if(_grid_attr[i][j].attribute == UNKNOWN){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
      }
      if(_grid_attr[i][j].attribute == FLAT){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr[i][j].attribute == OBS){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  //red

        //
      }
      if(_grid_attr[i][j].attribute == SUSPEND){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);  //blue
        //img.at<cv::Vec3b>(i, j) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr[i][j].attribute == CAR_TOP){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 255);  //yellow
      }
    }
  }

  //expand grid
//  for(int i =0; i<row; i++){
//    for(int j=0; j<col; j++){
//      if(_grid_attr[i][j].attribute == OBS){
//        for(int loop1 = i - exp_grid_size; loop1 <= i + exp_grid_size; loop1++){
//          for(int loop2 = j - exp_grid_size; loop2 <= j + exp_grid_size; loop2++){
//            if(loop1>=0 && loop1<row && loop2>=0 && loop2<col){
//              img.at<cv::Vec3b>(loop1,loop2) = cv::Vec3b(0, 0, 255);
//            }
//          }
//        }
//      }
//    }
//  }

  for(int i=5*range_front -5; i<5*range_front+5; i++){
    for(int j=5*range_left-5; j<5*range_left+5; j++){
      //current car position, blue
      img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    }
  }

}

void ObstacleDetection::cluster(Grid** _grid_attr, cv::Mat& img_gray, cv::Mat& img_color){
  int row = 5*(range_front+range_back);
  int col = 5*(range_left+range_right);

  for(int i = 0; i<row; i++){
    for(int j = 0; j<col; j++){
      if(_grid_attr[i][j].attribute == OBS){
        for(int loop1 = i - exp_grid_size; loop1 <= i + exp_grid_size; loop1++){
          for(int loop2 = j - exp_grid_size; loop2 <= j + exp_grid_size; loop2++){
            if(loop1>=0 && loop1<row && loop2>=0 && loop2<col){
              img_gray.at<uchar>(loop1, loop2) = 255;
            }
          }
        }
      }
    }
  }

  //findcontours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  //blue
  //cv::drawContours(img_color, contours, -1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, hierarchy);
  for(int i =0; i < contours.size(); i++){
    std::vector<cv::Point> points;
    for(int j =0; j<contours[i].size(); j++){
      points.push_back(contours[i][j]);
    }
    cv::RotatedRect box = cv::minAreaRect(points);
    double area = box.size.width * box.size.height;
    if(area > 36 && area < 300
       && box.size.width >= 6 && box.size.width <=25
       && box.size.height >= 6 &&box.size.height <=25){
      cv::Point2f vtx[4];
      box.points(vtx);
      for(int k=0; k<4; k++){
        cv::line(img_color, vtx[k], vtx[(k+1)%4], cv::Scalar(255,0,0), 1, cv::LINE_AA);
      }
    }
  }
}

void ObstacleDetection::griddiffer(){
  int row = 5*(range_front+range_back);
  int col = 5*(range_left+range_right);


  int win_size = 1;
  int win_thresh = 1;
  for(int i=0; i<row; i++){
    for(int j=0; j<col; j++){
      if(_grid_attr_pre[i][j].attribute == OBS && _grid_attr_single[i][j].attribute != OBS){
        obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  //red
      }else{
        obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
      }
//      int count = 0;
//      if(_grid_attr_pre[i][j].attribute == OBS){
//        for(int loop1 = i - win_size; loop1 <= i + win_size; loop1++){
//          for(int loop2 = j - win_size; loop2 <= j + win_size; loop2++){
//            if(loop1>=0 && loop1 < row && loop2 >= 0 && loop2 < col){
//              if(_grid_attr_single[loop1][loop2].attribute == OBS){
//                count++;
//              }
//            }
//          }
//        }
//        if(count >= win_thresh){
//          obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
//        }else{
//          obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  //red
//        }
//      }
//    }
  }

    for(int i=5*range_front -5; i<5*range_front+5; i++){
      for(int j=5*range_left-5; j<5*range_left+5; j++){
        //current car position, blue
        obs_differ.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
      }
    }
  }
}

void ObstacleDetection::saveResult(){
  std::string img_save_path;
  std::string mat_single_path;
  std::string mat_single_draw_path;
  std::string mat_multi_path;
  std::string cloud_single_path;
  std::string cloud_multi_path;
  std::string mat_diff_path;
  std::string obs_cluster_path;
  std::stringstream ss;
  ss<<result_save_location<<_sequence<<"_img.png";
  ss>>img_save_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_single.png";
  ss>>mat_single_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_single_draw.png";
  ss>>mat_single_draw_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_multi.png";
  ss>>mat_multi_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_single.pcd";
  ss>>cloud_single_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_multi.pcd";
  ss>>cloud_multi_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_diff.png";
  ss>>mat_diff_path;

  ss.clear();
  ss<<result_save_location<<_sequence<<"_gray.png";
  ss>>obs_cluster_path;

  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);  //0-9, 0 is best quality

  pcl::PointCloud<pcl::PointXYZI> cloudSum;
  for(int i=0; i<_fusion_num; i++){
    cloudSum += *(_laserCloudStack[i]);
  }

  cv::imwrite(img_save_path, img, compression_params);
  cv::imwrite(mat_single_path, obs_single, compression_params);
  cv::imwrite(mat_single_draw_path, obs_single_draw, compression_params);
  //cv::imwrite(mat_multi_path, obs_multi, compression_params);
  //cv::imwrite(mat_diff_path, obs_pre, compression_params);
  cv::imwrite(obs_cluster_path, obs_cluster, compression_params);
  //pcl::io::savePCDFileASCII(cloud_single_path, *_laserCloud);
  //pcl::io::savePCDFileASCII(cloud_multi_path, cloudSum);
  if(_sequence > 0){
    pcl::io::savePCDFileASCII(cloud_single_path, *_laserCloudStack[_cur_pos]);
  }
}

void ObstacleDetection::laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg){
  _timeLaserCloud = laserCloudMsg->header.stamp;
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

void ObstacleDetection::imageSeqHandler(const std_msgs::Int32::ConstPtr &seqIn)
{
  _newImageSeq = true;
  _sequence = seqIn->data;
}

}
