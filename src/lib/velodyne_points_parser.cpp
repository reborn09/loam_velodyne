#include "loam_velodyne/velodyne_points_parser.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cmath>
#include <cctype>
using namespace std;

const float LIDAR_SPIN_CYCLE = 0.1;

/* HDL32_INTRINSIC_PARA part:
 *  constructor
*/
HDL32_INTRINSIC_PARA::HDL32_INTRINSIC_PARA()
{
    memset(angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(beam_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(beam_reverse_order, 0, sizeof(int)*HDL32_BEAM_NUM);
    memset(sin_angleH, 0, sizeof(float)*36000);
    memset(cos_angleH, 0, sizeof(float)*36000);
    memset(sin_angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
    memset(cos_angleV, 0, sizeof(float)*HDL32_BEAM_NUM);
}

/* LIDAR_EXTRINSIC_PARA part:
 *  constructor
*/
LIDAR_EXTRINSIC_PARA::LIDAR_EXTRINSIC_PARA()
{
    memset(R, 0, sizeof(float)*3*3);
    memset(T, 0, sizeof(float)*3);
}


PARA_TABLE::PARA_TABLE()
{

}

void PARA_TABLE::print_base_dir()
{
    cout<<"---------------- NOTICE !!! ----------------"<<endl<<endl;
    cout<<" The parameters for lidar32"<<endl;
    cout<<"\t\""<<lidar32_para_location<<"\""<<endl<<endl;
    cout<<"-------------- END NOTICE !!! --------------"<<endl<<endl;
}

PARA_TABLE::~PARA_TABLE(){}

/* VELODYNE_PARSER part:
 *  constructor & destructor
 *  initiation of parameters
 *  setup
 *  clear_points
*/
VELODYNE_PARSER::VELODYNE_PARSER()
{
    memset(lidar32_pointcloud, 0, sizeof(mPoint3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    lidar32_pointNum = 0;
}

VELODYNE_PARSER::~VELODYNE_PARSER(){}

bool VELODYNE_PARSER::init_para()
{
    if( init_lidar32_para() )
        return true;
    else
        return false;
}

bool CMP(pair<float, int> a, pair<float, int> b)
{
    return a.first < b.first;
}

bool VELODYNE_PARSER::arrange_lidar32_beam_order()
{
    vector<pair<float, int> > beams;
    for(int i=0; i<HDL32_BEAM_NUM; i++)
    {
        beams.push_back(pair<float, int>(para_table.lidar32_inpara.angleV[i], i));
    }
    sort(beams.begin(), beams.end(), CMP);
    for(int i=0; i<beams.size(); i++)
  {
        para_table.lidar32_inpara.beam_order[i] = beams[i].second;
    para_table.lidar32_inpara.beam_reverse_order[beams[i].second] = i;
  }
}

bool VELODYNE_PARSER::init_lidar32_para()
{
    string lidar32para_filename = lidar32_para_location + "/lidar32para.ini";
    ifstream infile;
    stringstream sline;
    string line;

    for(int k = 0; k<36000; k++)
    {
        para_table.lidar32_inpara.sin_angleH[k] = (float)sin((double)k/18000.0*M_PI);
        para_table.lidar32_inpara.cos_angleH[k] = (float)cos((double)k/18000.0*M_PI);
    }

    infile.open(lidar32para_filename, ios::in);
    if(!infile)
    {
        cerr<<"***Error: can't open hdl32 para file \""<<lidar32para_filename<<"\""<<endl;
        return false;
    }

    sline.str("");
    sline.clear();
    line.clear();
    while(getline(infile, line))
    {
        if(line.empty())
            continue;
        if(line[0] == '#')      // '#' means comment
            continue;
        if(line[0] == '[')      // '[' means this is a flag line
        {
            string flag;
            for(auto c : line)
            {
                if(c != '[' && c != ']' && c != '\\')
                    flag = flag + c;
                if(c == '\\')
                    break;
            }
            if(flag == "Angle_V")
            {
                for(int i=0; i<32; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_inpara.angleV[i];

                    para_table.lidar32_inpara.sin_angleV[i] = (float)sin(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                    para_table.lidar32_inpara.cos_angleV[i] = (float)cos(para_table.lidar32_inpara.angleV[i]/180.0*M_PI);
                }
                arrange_lidar32_beam_order();
            }
            else if(flag == "Rotation")
            {
                for(int i=0; i<3; i++)
                {
                    line.clear();
                    getline(infile, line);
                    sline.str("");
                    sline.clear();
                    sline<<line;
                    sline>>para_table.lidar32_expara.R[i][0]>>para_table.lidar32_expara.R[i][1]>>para_table.lidar32_expara.R[i][2];
                }
            }
            else if(flag == "Translation")
            {
                line.clear();
                getline(infile, line);
                sline.str("");
                sline.clear();
                sline<<line;
                sline>>para_table.lidar32_expara.T[0]>>para_table.lidar32_expara.T[1]>>para_table.lidar32_expara.T[2];
            }
        }
        line.clear();
    }
    infile.close();
    return true;
}

void VELODYNE_PARSER::clear_points()
{
    memset(lidar32_pointcloud, 0, sizeof(mPoint3f)*HDL32_BEAM_NUM*HDL32_BEAM_POINTSIZE);
    lidar32_pointNum = 0;
}

/*
 * Lidar32 part
*/
bool VELODYNE_PARSER::parse_lidar32_process(u_int8_t *cache)
{
    int packet_cnt = 0;
    u_int8_t *fp = cache + HDL32_PACKET_SIZE*packet_cnt;
    u_int8_t *fq = fp + 1;
    u_int16_t val = (*fp)*256 + *fq;

    float *cos_angH = para_table.lidar32_inpara.cos_angleH;
    float *sin_angH = para_table.lidar32_inpara.sin_angleH;
    float *cos_angV = para_table.lidar32_inpara.cos_angleV;
    float *sin_angV = para_table.lidar32_inpara.sin_angleV;
    float *angleV = para_table.lidar32_inpara.angleV;


    u_int16_t start_angleH = 1;	// 记录这一帧初始的水平向激光角度
    bool start_angleH_generated = false;
    while(val == 0xffee)
    {
        // parse packet, 1206 bytes
        for(int i=0; i<HDL32_NUM_SHOTS; i++)    // 12
        {
            u_int8_t *pAngL = fp + 100*i + 2;
            u_int8_t *pAngH = pAngL + 1;
            u_int16_t angleH = ((*pAngH)*256 + (*pAngL) + 9000)%36000;
            u_int8_t *pL = fp + 100*i + 4;
            u_int8_t *pH = pL + 1;
            u_int8_t *pVal = pH + 1;
            int idx = packet_cnt*HDL32_NUM_SHOTS + i;
            if(!start_angleH_generated)
            {
              start_angleH = angleH;
              start_angleH_generated = true;
            }
            // time_ratio指的是当前扫扫描点相对于帧头的时间，用于后续畸变补偿
            float angleH_dis = float(angleH) - float(start_angleH);
            if(angleH_dis < 0.0)
              angleH_dis += 36000.0;
            float time_ratio = LIDAR_SPIN_CYCLE * angleH_dis / 36000.0;

            for(int j=0; j<HDL32_BEAM_NUM; j++) // 32
            {
                int distance = ((*pH)*256 + (*pL))*2;       // in mm
                if(distance >= HDL32_VALID_RADIUS)
                {
                    int beamInd = para_table.lidar32_inpara.beam_reverse_order[j];	//beamInd表示这个点实际上对应这按顺序排列的第beamInd个圈
                    lidar32_pointcloud[beamInd][idx].valid = true;
                    lidar32_pointcloud[beamInd][idx].distance = distance;
                    //lidar32_pointcloud[beamInd][idx].intensity = *pVal;
                    lidar32_pointcloud[beamInd][idx].intensity = (float)beamInd + time_ratio; // 改进的intensity整数部分存beam信息，
                                                                                              // 小数部分存当前点在周期中的时刻比值

                    lidar32_pointcloud[beamInd][idx].angleH = angleH;
                    lidar32_pointcloud[beamInd][idx].angleV = int16_t(angleV[j]*100);
                    lidar32_pointcloud[beamInd][idx].x = (float)distance/10.0 * cos_angV[j] * sin_angH[angleH];
                    lidar32_pointcloud[beamInd][idx].y = (float)distance/10.0 * cos_angV[j] * cos_angH[angleH];
                    lidar32_pointcloud[beamInd][idx].z = (float)distance/10.0 * sin_angV[j];
                    lidar32_pointNum++;
                }
                pL += 3;
                pH += 3;
                pVal += 3;
            }
        }

        // next packet
        packet_cnt++;
        if(packet_cnt > HDL32_PACKET_NUM - 1)       // in case of exceeding memory
            break;
        fp = cache + HDL32_PACKET_SIZE*packet_cnt;
        fq = fp + 1;
        val = (*fp)*256 + *fq;
    }
    return true;
}

bool VELODYNE_PARSER::parse_lidar32_data(const std::string filename)
{
    char file[256];
    strcpy(file, filename.c_str());
    FILE *fp = fopen(file, "rb");
    if(fp==NULL)
        return false;

    u_int8_t *cache = new u_int8_t[HDL32_BUFFER_SIZE];
    if(!fread(cache, sizeof(u_int8_t), HDL32_BUFFER_SIZE, fp))
    {
        cerr<<"***Error: fread file "<<filename<<" failed!"<<endl;
        return false;
    }
    fclose(fp);

    parse_lidar32_process(cache);
    //calib_lidar32_data();	//似乎不应该进行标定，因为loam里面使用了光线追踪的方法，要求点云坐标应该是以雷达为中心的
    delete[] cache;
    return true;
}

void VELODYNE_PARSER::calib_lidar32_data()
{
    for(int beam=0; beam<HDL32_BEAM_NUM; beam++)
    {
        for(int cnt=0; cnt<HDL32_BEAM_POINTSIZE; cnt++)
        {
            mPoint3f *p = &lidar32_pointcloud[beam][cnt];
            const mPoint3f tmp = lidar32_pointcloud[beam][cnt];
            if(p->valid)
            {
                p->x = tmp.x*para_table.lidar32_expara.R[0][0]+tmp.y*para_table.lidar32_expara.R[0][1]+tmp.z*para_table.lidar32_expara.R[0][2] + para_table.lidar32_expara.T[0];
                p->y = tmp.x*para_table.lidar32_expara.R[1][0]+tmp.y*para_table.lidar32_expara.R[1][1]+tmp.z*para_table.lidar32_expara.R[1][2] + para_table.lidar32_expara.T[1];
                p->z = tmp.x*para_table.lidar32_expara.R[2][0]+tmp.y*para_table.lidar32_expara.R[2][1]+tmp.z*para_table.lidar32_expara.R[2][2] + para_table.lidar32_expara.T[2];
            }
        }
    }
}

void VELODYNE_PARSER::save_lidar32_txt(const std::string &filename)
{
    fstream fileout;
    fileout.open(filename, ios::out);
    if(!fileout)
    {
        cerr<<"***Error: Can't open file \""<<filename<<"\""<<endl;
        return;
    }
    for(int beam = 0; beam < HDL32_BEAM_NUM; beam++)
    {
        for(int angle = 0; angle < HDL32_BEAM_POINTSIZE; angle++)
        {
            if(lidar32_pointcloud[beam][angle].valid)
            {
                fileout << lidar32_pointcloud[beam][angle].x/100.0 << "\t\t"<<
                           lidar32_pointcloud[beam][angle].y/100.0 << "\t\t"<<
                           lidar32_pointcloud[beam][angle].z/100.0 << "\t\t"<<
               lidar32_pointcloud[beam][angle].intensity<<endl;
            }
        }
    }
    fileout.close();
}

int VELODYNE_PARSER::get_points_num_32()
{
  return lidar32_pointNum;
}
