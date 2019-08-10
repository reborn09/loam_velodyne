#ifndef PAREMETERUSE_H
#define PAREMETERUSE_H

#include <string>
#include <vector>
#include <dirent.h>
#include <algorithm>

#define SAVE_RESULT
//#define ANALYSE_OBS
const int frequence = 2;
const int _fusion_num = 10;
const int startFrameNo = 8730;
const int endFrameNo = 100000;

//std::vector<std::string> frameNo;

const std::string base_dir = "/home/jiapengz/data/alv_data";
const std::string folder = "1124_1016";
const std::string lidar32_para_location = "/home/jiapengz/data/alv_data/parameters";
const std::string save_lidar_location = "/home/jiapengz/data/lidar_save/";
const std::string result_save_location = "/home/jiapengz/data/result_save/";


//origin 90 30 40 40
//grid range, dimension m
const int range_front = 90;
const int range_back = 90;
const int range_left = 90;
const int range_right = 90;

//obs detect threshhold
const float _obsHeightThreshhold = 30.0; //cm
const float _cartopHeightThreshhold = 50.0; //cm
const float _susHeightThreshhold = 120.0; //cm
const int ground_win_size = 3; //estimate ground height during the window size
const int exp_grid_size = 1; //expand grid

void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type);

bool computePairNum(std::string pair1,std::string pair2);

void sort_filelists(std::vector<std::string>& filists,std::string type);

#endif // PAREMETERUSE_H
