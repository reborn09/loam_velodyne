#ifndef PAREMETERUSE_H
#define PAREMETERUSE_H

#include <string>
#include <vector>
#include <dirent.h>
#include <algorithm>

//#define SAVE_PARSER_LIDAR
//#define SAVE_RESULT
const int frequence = 1;
const int _fusion_num = 10;
const int startFrameNo = 0;
const int endFrameNo = 100000;

//std::vector<std::string> frameNo;

const std::string base_dir = "/home/jiapengz/data/alv_data";
const std::string folder = "1122_1426";
const std::string lidar32_para_location = "/home/jiapengz/data/alv_data/parameters";
const std::string save_lidar_location = "/home/jiapengz/data/lidar_save/";
const std::string img_base_dir = "/home/jiapengz/data/kitti/data_odometry_color/dataset/sequences/";
const std::string result_save_location = "/home/jiapengz/data/result_save/";

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);

bool computePairNum(std::string pair1,std::string pair2);

void sort_filelists(std::vector<std::string>& filists,std::string type);

#endif // PAREMETERUSE_H
