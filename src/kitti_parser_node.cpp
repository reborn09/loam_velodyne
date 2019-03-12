#include <ctime>
#include "ros/ros.h"
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
{
    struct dirent *ptr;
    DIR *dir;
    dir = opendir(dir_path.c_str());
    out_filelsits.clear();
    while ((ptr = readdir(dir)) != NULL){
        std::string tmp_file = ptr->d_name;
        if (tmp_file[0] == '.')continue;
        if (type.size() <= 0){
            out_filelsits.push_back(ptr->d_name);
        }else{
            if (tmp_file.size() < type.size())continue;
            std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
            if (tmp_cut_type == type){
                out_filelsits.push_back(ptr->d_name);
            }
        }
    }
}

bool computePairNum(std::string pair1,std::string pair2)
{
    return pair1 < pair2;
}

void sort_filelists(std::vector<std::string>& filists,std::string type)
{
    if (filists.empty())return;

    std::sort(filists.begin(),filists.end(),computePairNum);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_parser");
    ros::NodeHandle n;
    ros::Publisher velodyne_pub = n.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 2);

    //pub frequence
    int frequence=10;
    ros::Rate loop_rate(frequence);

    //need to change for different sequence
    string base_dir="/home/jiapengz/data/kitti/data_odometry_velodyne/dataset/sequences/";
    string sequence="03";

    string bin_path;
    stringstream ss;
    ss<<base_dir<<sequence<<"/velodyne/";
    ss>>bin_path;
    std::vector<std::string> file_lists;
    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );

    for (int i = 0; i < file_lists.size(); i++){
      if(ros::ok()){
        std::string bin_file = bin_path + file_lists[i];
        // load point cloud
        std::fstream input(bin_file.c_str(), std::ios::in | std::ios::binary);
        if(!input.good()){
          std::cerr << "Could not read file: " << bin_file << std::endl;
          exit(EXIT_FAILURE);
        }
        input.seekg(0, std::ios::beg);

        pcl::PointCloud<pcl::PointXYZI> cloud64;
        cloud64.clear();
        cloud64.width = 150000;			// 预设大一点的空间
        cloud64.height = 1;
        cloud64.is_dense = true;
        cloud64.resize(cloud64.width*cloud64.height);
        int point_count;
        for (point_count=0; input.good() && !input.eof(); ) {
          pcl::PointXYZI point;
          input.read((char *) &point.x, sizeof(float));
          input.read((char *) &point.y, sizeof(float));
          input.read((char *) &point.z, sizeof(float));
          input.read((char *) &point.intensity, sizeof(float));
          cloud64.points[point_count].x=point.x;
          cloud64.points[point_count].y=point.y;
          cloud64.points[point_count].z=point.z;
          cloud64.points[point_count].intensity=point.intensity;
          point_count++;
        }
        cloud64.width =point_count;
        cloud64.resize(cloud64.width*cloud64.height); // 重新调整点云尺寸至真实值
        input.close();

        // 将pcl::PCLPointCLoud2格式转换成sensor_msgs::PointCloud2格式
        sensor_msgs::PointCloud2 output64;
        pcl::toROSMsg(cloud64,output64);

        output64.header.seq=i;
        output64.header.frame_id="/velodyne";
        output64.header.stamp=ros::Time::now();
        velodyne_pub.publish(output64);

        ros::spinOnce();
        loop_rate.sleep();
      }
  }
  ROS_INFO("KITTI Sequence reach the end!");

  return 0;
}
