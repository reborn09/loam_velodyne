#ifndef PAREMETERUSE_H
#define PAREMETERUSE_H

#include <string>

//#define SAVE_PARSER_LIDAR
//#define SAVE_RESULT
int start_frame = 0;
int frequence = 1;
int _fusion_num = 10;
std::string lidar_base_dir = "/home/jiapengz/data/kitti/data_odometry_velodyne/dataset/sequences/";
std::string sequence = "05";
std::string save_lidar_location = "/home/jiapengz/data/lidar_save/";

std::string img_base_dir = "/home/jiapengz/data/kitti/data_odometry_color/dataset/sequences/";
std::string result_save_location = "/home/jiapengz/data/result_save/";

//grid range, dimension m
int range_front = 90;
int range_back = 30;
int range_left = 40;
int range_right = 40;

//obs detect threshhold
float _obsHeightThreshhold = 30.0; //cm
float _cartopHeightThreshhold = 50.0; //cm
float _susHeightThreshhold = 160.0; //cm
int ground_win_size = 3; //estimate ground height during the window size
int exp_grid_size = 1; //expand grid

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

#endif // PAREMETERUSE_H
