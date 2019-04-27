// 标准C++头文件
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
// linux读取目录文件的头文件
#include <unistd.h>
#include <dirent.h>
// ros相关头文件
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
// pcl相关头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
          // 将 pcl::PointCloud<PointCloudT> 转成 pcl::PCLPointCloud2格式的函数toPCLPointCloud2()的头文件
#include <pcl_conversions/pcl_conversions.h>
          // 将 pcl::PCLPointCloud2 转成 sensor_msgs::PointCloud2格式的函数pcl_conversions::fromPCL()的头文件
// OpenCV headers
#include <opencv/cv.hpp>
#include <opencv2/highgui/highgui.hpp>

// 自定义头文件
#include "loam_velodyne/velodyne_points_parser.h"
#include "loam_velodyne/paremeterUse.h"

using namespace std;
const int MAX_POINT_SIZE32 = 60000;	// 最大的点云数量

// 用于退出循环的函数，关联 ctrl+'C'
bool loop_done = false;  // flag indicating main loop is to terminate
extern "C" void PointsParseLoop_quit(int sig);
void PointsParseLoop_quit(int sig)
{
    loop_done = true;
}

int main(int argc, char **argv)
{
  // ROS初始化
  ros::init (argc, argv, "Hdl_Parser");
  ros::NodeHandle nh;
  ros::Publisher pub32 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_points", 2);	// 声明一下将要在points这个topic上发布消息
  ros::Publisher seq_pub = nh.advertise<std_msgs::Int32>("/seq_parser", 2);

  // parameter reader
  ros::Rate loop_rate(frequence);	// 循环频率
  // 新建一个VELODYNE_PARSER实例
  VELODYNE_PARSER *pvelodyneParser = new VELODYNE_PARSER;
  pvelodyneParser->para_table.print_base_dir();
  if(!pvelodyneParser->init_para())
  {
    cerr<<"***Error: Can't init parameter table!"<<endl;
    return 0;
  }


  // 读取所有文件序号
  string HDL32Dir = base_dir+"/LIDAR_DATA/"+folder;
  DIR *dir;
  struct dirent *ptr;
  std::vector<std::string> frameNo;
  if ((dir=opendir(HDL32Dir.c_str())) == NULL)
  {
    cerr<<"Open dir error..."<<endl;
    exit(1);
  }
  while ((ptr=readdir(dir)) != NULL)
  {
    if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    //current dir OR parrent dir
      continue;
    else if(ptr->d_type == 8)   //file
    {
      string name(ptr->d_name);
      name.erase(0,6);        // erase the prefix "Lidar_"
      int pos = name.find(".bin");
      name.erase(pos, 4);     // erase the subfix ".bin"
      frameNo.push_back(name);
    }
    else
      continue;
  }
  closedir(dir);
  sort(frameNo.begin(), frameNo.end());   // 升序排列，表示时间顺序

  // 主循环
  int i = startFrameNo;
  while(ros::ok() && !loop_done && i < frameNo.size())
  {
    cout<<"Frame No."<<i << ",\t"<<frameNo[i]<<endl;

    pvelodyneParser->clear_points();    // DO NOT FORGET TO CLEANUP !!!

    // parse data
    stringstream ss;
    string lidar32filename;

    ss.str("");
    ss.clear();
    ss << base_dir << "/LIDAR_DATA/" << folder << "/Lidar_" << frameNo[i] <<".bin";
    ss >> lidar32filename;
    ss.str("");
    ss.clear();

    pvelodyneParser->parse_lidar32_data(lidar32filename);

    // 将点云存入pcl数据结构
    pcl::PointCloud<pcl::PointXYZI> cloud32;
    cloud32.width = MAX_POINT_SIZE32;			// 预设大一点的空间
    cloud32.height = 1;
    cloud32.is_dense = true;
    cloud32.resize(cloud32.width*cloud32.height);

    int num = 0;
    bool PtNumExceed = false;
    // lidar32
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
      if(PtNumExceed)
        break;
      for(int cnt = 0; cnt < HDL32_BEAM_POINTSIZE; cnt++)
      {
        if(pvelodyneParser->lidar32_pointcloud[beam][cnt].valid)
        {
          cloud32.points[num].x = pvelodyneParser->lidar32_pointcloud[beam][cnt].x/100.0;
          cloud32.points[num].y = pvelodyneParser->lidar32_pointcloud[beam][cnt].y/100.0;
          cloud32.points[num].z = pvelodyneParser->lidar32_pointcloud[beam][cnt].z/100.0;
          cloud32.points[num].intensity = pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        - (int)pvelodyneParser->lidar32_pointcloud[beam][cnt].intensity
                                        + float(beam);
          // 重新分配beam数值，即替换掉intensity整数部分
          num++;
          if(num >= MAX_POINT_SIZE32)
          {
            PtNumExceed = true;
            break;
          }
        }
      }
    }
    cloud32.width = num;
    cloud32.height = 1;
    cloud32.resize(cloud32.width*cloud32.height);	// 重新调整点云尺寸至真实值
    cout<<"cloud32 points size: "<<cloud32.points.size()<<endl;

    // 将pcl::PointCloud<pcl::PointCloudXYZI>格式转换成pcl::PCLPointCLoud2格式
    pcl::PCLPointCloud2 tmp_cloud32;
    pcl::toPCLPointCloud2(cloud32, tmp_cloud32);
    // 将pcl::PCLPointCLoud2格式转换成sensor_msgs::PointCloud2格式
    sensor_msgs::PointCloud2 output32;
    pcl_conversions::fromPCL(tmp_cloud32, output32);
    // 发布消息
    output32.header.frame_id = "/velodyne";
    output32.header.stamp = ros::Time::now();
    pub32.publish(output32);

    std_msgs::Int32 image_seq;
    image_seq.data = i;
    seq_pub.publish(image_seq);
    // spin & sleep
    i++;
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete pvelodyneParser;
  return 0;
}
