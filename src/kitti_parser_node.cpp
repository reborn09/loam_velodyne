#include <ctime>
#include "ros/ros.h"
#include <fstream>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "loam_velodyne/paremeterUse.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_parser");
    ros::NodeHandle n;
    ros::Publisher velodyne_pub = n.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 2);
    ros::Publisher seq_pub = n.advertise<std_msgs::Int32>("/seq_parser", 2);

    //pub frequence
    ros::Rate loop_rate(frequence);

    string bin_path;
    stringstream ss;
    ss<<lidar_base_dir<<sequence<<"/velodyne/";
    ss>>bin_path;
    std::vector<std::string> file_lists;
    read_filelists( bin_path, file_lists, "bin" );
    sort_filelists( file_lists, "bin" );

    for (int i = start_frame; i < file_lists.size(); i++){
      if(ros::ok()){
        std::cout<<"sequence: "<<i<<std::endl;
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
          input.read((char *) &point.x, sizeof(float)); //m
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

#ifdef SAVE_PARSER_LIDAR
        string file_path;
        stringstream ss;
        ss<<save_lidar_location<<i<<".txt";
        ss>>file_path;

        fstream fwriter;
        fwriter.open(file_path,ios::out);
        if(!fwriter.is_open()){
          cerr<<"can't open file: "<<file_path<<endl;
        }else{
          for(int index=0;index<cloud64.points.size();index++){
            double angle=cloud64.points[index].z/sqrt(cloud64.points[index].x*cloud64.points[index].x+cloud64.points[index].y*cloud64.points[index].y);
            angle=angle/3.1415926*180;
            fwriter<<cloud64.points[index].x<<"  "
                   <<cloud64.points[index].y<<"  "
                   <<cloud64.points[index].z<<"  "
                   <<angle<<endl;
          }
          fwriter.close();
        }
#endif

        // 将pcl::PCLPointCLoud2格式转换成sensor_msgs::PointCloud2格式
        sensor_msgs::PointCloud2 output64;
        pcl::toROSMsg(cloud64,output64);

        output64.header.frame_id="/velodyne";
        output64.header.stamp=ros::Time::now();
        velodyne_pub.publish(output64);

        std_msgs::Int32  image_seq;
        image_seq.data = i;
        seq_pub.publish(image_seq);

        ros::spinOnce();
        loop_rate.sleep();
      }
  }
  ROS_INFO("KITTI Sequence reach the end!");

  return 0;
}
