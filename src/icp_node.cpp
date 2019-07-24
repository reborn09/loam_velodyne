#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <fstream>
#include <string>


pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);
bool is_init = false;
int count = 0;
std::string file_path1 = "/home/jiapengz/data/time_save/icp_all.txt";
std::string file_path2 = "/home/jiapengz/data/time_save/icp_ave.txt";
float time_all = 0;
void icpCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

  cloud_in->clear();
  pcl::fromROSMsg(*laserCloudMsg, *cloud_in);

  if(is_init){
    struct timeval start;
    struct timeval end;
    unsigned long diff;
    gettimeofday(&start, NULL);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZI> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    gettimeofday(&end, NULL);
    diff = 1000000 * (end.tv_sec-start.tv_sec)+ end.tv_usec-start.tv_usec;
    float consume_time = diff/1000;
    time_all+=consume_time;
    std::cout << "time:" << consume_time <<std::endl;

    //write
    count++;
    if(count<=100){
      std::ofstream f1(file_path1, std::ofstream::app);
      f1<<count<<"  "<<consume_time<<std::endl;
      f1.close();
    }
    if(count == 100 ){
      time_all = time_all / count;
      std::ofstream f2(file_path2);
      f2<<time_all<<std::endl;
      f2.close();
    }
    if(count > 100){
      std::cout<<"write finish!!!"<<std::endl;
    }

  }else{
    is_init = true;
  }
  cloud_out->clear();
  *cloud_out = *cloud_in;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, icpCallback);
  ros::spin();

  return 0;
}
