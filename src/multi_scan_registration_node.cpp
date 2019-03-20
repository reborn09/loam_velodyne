#include <ros/ros.h>
#include "loam_velodyne/MultiScanRegistration.h"

/*
 * frame_id describetion
 * camera_init: origin location
 * camera: current location
 */

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle node;
  ros::NodeHandle privateNode("~");

  loam::MultiScanRegistration multiScan;

  if (multiScan.setup(node, privateNode)) {
    // initialization successful
    ros::spin();
  }

  return 0;
}
