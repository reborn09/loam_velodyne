#include "loam_velodyne/ObstacleDetection.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ObstacleDetection");
  ros::NodeHandle node;

  //use fusion frame num to init
  loam::ObstacleDetection detect;
  if(detect.setup(node)){
    detect.spin();
  }

  return 0;
}
