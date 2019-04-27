#ifndef VELODYNEPOINTSPARSER_H
#define VELODYNEPOINTSPARSER_H

#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include "loam_velodyne/paremeterUse.h"
#include "loam_velodyne/Lidar32DataStruct.h"

struct mPoint3f
{
    mPoint3f(float xval = 0.0, float yval = 0.0, float zval = 0.0):x(xval),y(yval),z(zval){}

    float x;            // in cm
    float y;            // in cm
    float z;            // in cm
    bool valid;         // flag indicating whether this point is valid
    float intensity;    // 0-255
    u_int16_t angleH;   // 0-35999, in 0.01 degree
    int16_t angleV;     // -18000~18000, in 0.01 degree
    int distance;       // real distance, 0-65536*2, in mm
};

struct HDL32_INTRINSIC_PARA
{
    HDL32_INTRINSIC_PARA();
    HDL32_INTRINSIC_PARA & operator = (const HDL32_INTRINSIC_PARA & other)
    {
        memcpy(angleV, other.angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(sin_angleV, other.sin_angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(cos_angleV, other.cos_angleV, sizeof(float)*HDL32_BEAM_NUM);
        memcpy(sin_angleH, other.sin_angleH, sizeof(float)*36000);
        memcpy(cos_angleH, other.cos_angleH, sizeof(float)*36000);
        memcpy(beam_order, other.beam_order, sizeof(int)*HDL32_BEAM_NUM);
        return *this;
    }

    float angleV[HDL32_BEAM_NUM];
    float sin_angleV[HDL32_BEAM_NUM];
    float cos_angleV[HDL32_BEAM_NUM];
    float sin_angleH[36000];
    float cos_angleH[36000];
    int beam_order[HDL32_BEAM_NUM];			//beam_order[i] 表示俯仰角从低到高第i个beam对应原始数据packet中的第 beam_order[i] 组
  int beam_reverse_order[HDL32_BEAM_NUM]; //beam_reverse_order[i]表示原始数据packet中的第i组数据对应俯仰角由低到高第beam_reverse_order[i]大
};

struct LIDAR_EXTRINSIC_PARA
{
    LIDAR_EXTRINSIC_PARA();
    LIDAR_EXTRINSIC_PARA & operator = (const LIDAR_EXTRINSIC_PARA & other)
    {
        memcpy(R, other.R, sizeof(float)*3*3);
        memcpy(T, other.T, sizeof(float)*3);
        return *this;
    }

    float R[3][3];
    float T[3];
};


struct PARA_TABLE
{
    PARA_TABLE();
    ~PARA_TABLE();

    void print_base_dir();

    HDL32_INTRINSIC_PARA lidar32_inpara;
    LIDAR_EXTRINSIC_PARA lidar32_expara;
};


class VELODYNE_PARSER
{
public:
    VELODYNE_PARSER();
    ~VELODYNE_PARSER();

    // configuration
    bool init_para();
    bool arrange_lidar32_beam_order();
    bool init_lidar32_para();
    void clear_points();

    // lidar 32 part
    bool parse_lidar32_process(u_int8_t *cache);
    bool parse_lidar32_data(const std::string filename);
    void calib_lidar32_data();
    void save_lidar32_txt(const std::string &filename);
    int get_points_num_32();

    // point clouds storage
    mPoint3f lidar32_pointcloud[HDL32_BEAM_NUM][HDL32_BEAM_POINTSIZE];   // hdl32 lidar point cloud

    // parameters table
    PARA_TABLE para_table;
    int lidar32_pointNum;
};

#endif // VELODYNEPOINTSPARSER_H
