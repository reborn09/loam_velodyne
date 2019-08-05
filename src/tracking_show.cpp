#include "loam_velodyne/basictrackingshow.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking_show");
  ros::NodeHandle nh;

  int start_frame = 86;
  int end_frame = 95;
  int pose_frame = 95;
  string pcd_location = "/home/jiapengz/data/dynamic/input/";
  string save_location = "/home/jiapengz/data/dynamic/output/";
  pcl::PointCloud<pcl::PointXYZI> cloudsum;
  pcl::PointCloud<pcl::PointXYZI> cloudsave;
  pcl::PointCloud<pcl::PointXYZI> cloudtemp;
  //init
  int row = 5*(range_front_1 + range_back_1);
  int col = 5*(range_left_1 + range_right_1);
  Grid** _grid_attr_single = NULL;
  _grid_attr_single = new Grid *[row];
  for(int i=0; i<row; i++){
    _grid_attr_single[i] = new Grid[col];
  }

  cv::Mat obs_single;
  obs_single = cv::Mat::zeros(row, col, CV_8UC3);

  cv::Mat obs_gray;
  obs_gray = cv::Mat::zeros(row, col, CV_8UC1);

  stringstream ss;
  string pose_path;
  ss<<pcd_location<<pose_frame<<".txt";
  ss>>pose_path;

  float x,y,z;
  double rx,ry,rz;
  ifstream f1(pose_path, ifstream::in);
  f1>>x>>y>>z>>rx>>ry>>rz;
  f1.close();

  cout<<x<<" "<<y<<" "<<z<<endl;
  _transformSum.pos.x() = x;
  _transformSum.pos.y() = y;
  _transformSum.pos.z() = z;
  _transformSum.rot_x = rx;
  _transformSum.rot_y = ry;
  _transformSum.rot_z = rz;

  //reset
  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      _grid_attr_single[i][j].valid = false;
      _grid_attr_single[i][j].attribute = UNKNOWN;
      _grid_attr_single[i][j].cloud.clear();
    }
  }

  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      obs_single.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
    }
  }

  for(int i = 0; i<row; i++){
    for(int j = 0; j < col; j++){
      obs_gray.at<uchar>(i, j) = 0; //black
    }
  }


  //process
  for(int i = start_frame ; i <=end_frame; i++){
    stringstream ss;
    string filename;
    ss.clear();
    ss<<pcd_location<<i<<".pcd";
    ss>>filename;
    cloudtemp.clear();
    pcl::io::loadPCDFile(filename, cloudtemp);
    cloudsum += cloudtemp;
  }

  pcl::PointXYZI pointsel;
  for(auto const& pt : cloudsum.points){
    pointToOrigin(pt, pointsel);
    cloudsave.push_back(pointsel);
    projectPointToGrid(pointsel, _grid_attr_single);
  }
  gridAttrToMat(_grid_attr_single, obs_single);
  cluster(_grid_attr_single, obs_gray, obs_single);

  //show
  cv::imshow("tracking", obs_single);
  cv::waitKey(0);

  //save
#if true
  ss.clear();
  string allcloudsavepath;
  ss<<save_location<<"cloud.pcd";
  ss>>allcloudsavepath;
  cout<<"cloud size:"<<cloudsave.size()<<endl;
  pcl::io::savePCDFileASCII(allcloudsavepath, cloudsave);

  ss.clear();
  string resultpath;
  ss<<save_location<<"result.png";
  ss>>resultpath;
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);  //0-9, 0 is best quality
  cv::imwrite(resultpath, obs_single, compression_params);
#endif

  //destroy
  for(int i = 0; i < row; i++){
    delete[] _grid_attr_single[i];
  }
  delete[] _grid_attr_single;

  return 0;
}
