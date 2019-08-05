#include "loam_velodyne/basictrackingshow.h"
#include "math_utils.h"

void pointToOrigin(const pcl::PointXYZI &pi, pcl::PointXYZI &po){
  pcl::PointXYZI pt_tmp;
  pt_tmp.x = pi.x - _transformSum.pos.x();
  pt_tmp.y = pi.y - _transformSum.pos.y();
  pt_tmp.z = pi.z - _transformSum.pos.z();
  pt_tmp.intensity = pi.intensity;

  //cout<<"before: "<<pt_tmp.x<<endl;
  rotateYXZ(pt_tmp, -_transformSum.rot_y, -_transformSum.rot_x, -_transformSum.rot_z);
  //cout<<"after: "<<pt_tmp.x<<endl;

  po.x = pt_tmp.z;
  po.y = pt_tmp.x;
  po.z = pt_tmp.y;
  po.intensity = pt_tmp.intensity;
}

void projectPointToGrid(pcl::PointXYZI pi, Grid** _grid_attr){
  int row = 5*range_front_1 - 1 - int(pi.x*100.0/_grid_size_1);
  int col = 5*range_left_1 - 1 - int(pi.y*100.0/_grid_size_1);

  int row_max = 5*(range_front_1+range_back_1);
  int col_max = 5*(range_left_1+range_right_1);

  if(0 <= row && row < row_max && 0 <= col && col < col_max){
    if(_grid_attr[row][col].valid == false){
      _grid_attr[row][col].valid = true;
      _grid_attr[row][col].min_height = 100.0 * pi.z;
      _grid_attr[row][col].max_height = 100.0 * pi.z;
    }else{
      if(100.0*pi.z < _grid_attr[row][col].min_height){
        _grid_attr[row][col].min_height = 100.0*pi.z;
      }
      if(100.0*pi.z > _grid_attr[row][col].max_height){
        _grid_attr[row][col].max_height = 100.0*pi.z;
      }
    }
    _grid_attr[row][col].cloud.push_back(pi);
  }

}

void gridAttrToMat(Grid** _grid_attr, cv::Mat& img){
  int row = 5*(range_front_1+range_back_1);
  int col = 5*(range_left_1+range_right_1);

  for(int i=0; i<row; i++){
    for(int j=0; j<col; j++){
      if(_grid_attr[i][j].valid){
        float height_diff = _grid_attr[i][j].max_height - _grid_attr[i][j].min_height;
        if(height_diff > _obsHeightThreshhold_1){
          _grid_attr[i][j].attribute = OBS;
        }else{
          _grid_attr[i][j].attribute = FLAT;
        }
      }
    }
  }

  //find car top
  for(int i = 0; i <row; i++){
    for(int j = 0; j < col; j++){
      if(_grid_attr[i][j].valid && _grid_attr[i][j].attribute != OBS){
        float ground_height = 0;
        int count = 0 ;
        for(int loop1 = i - ground_win_size_1;loop1<=i+ground_win_size_1; loop1++){
          for(int loop2 = j - ground_win_size_1; loop2<=j+ground_win_size_1; loop2++){
            if(loop1>=0 && loop1<row && loop2>=0 && loop2 <col){
              if(_grid_attr[loop1][loop2].attribute == FLAT){
                ground_height += _grid_attr[loop1][loop2].min_height;
                count++;
              }
            }
          }
        }
        if(count >= 1){
          ground_height /= count;
          if(_grid_attr[i][j].max_height - ground_height > _cartopHeightThreshhold_1){
            //_grid_attr[i][j].attribute = CAR_TOP;
            _grid_attr[i][j].attribute = OBS;
          }
        }
      }
    }
  }

  //find suspend
  for(int i = 0; i < row; i++){
    for(int j = 0; j < col; j++){
      if(_grid_attr[i][j].attribute == OBS){
        sort(_grid_attr[i][j].cloud.begin(),
             _grid_attr[i][j].cloud.end(),
             [](const pcl::PointXYZI &p1, const pcl::PointXYZI &p2)->bool{return p1.z < p2.z;});
        int count = _grid_attr[i][j].cloud.size();
        if(count > 0){
          float h_pre = _grid_attr[i][j].cloud[0].z*100.0;
          float h_now = _grid_attr[i][j].cloud[0].z*100.0;
          float h_dis = h_now - h_pre;
          for(int index = 0; index<count; index++){
            h_now = _grid_attr[i][j].cloud[index].z*100.0;
            h_dis = h_now - h_pre;
            if(h_dis > _susHeightThreshhold_1){
              //_grid_attr[i][j].attribute = SUSPEND;
              _grid_attr[i][j].attribute = FLAT;
              break;
            }
            h_pre = _grid_attr[i][j].cloud[index].z*100.0;
          }
        }
      }
    }
  }

  //draw
  for(int i=0; i<row; i++){
    for(int j=0; j<col; j++){
      if(_grid_attr[i][j].attribute == UNKNOWN){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  //black
      }
      if(_grid_attr[i][j].attribute == FLAT){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr[i][j].attribute == OBS){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  //red

        //
      }
      if(_grid_attr[i][j].attribute == SUSPEND){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);  //blue
        //img.at<cv::Vec3b>(i, j) = cv::Vec3b(50, 50, 50);  //gray
      }
      if(_grid_attr[i][j].attribute == CAR_TOP){
        img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 255);  //yellow
      }
    }
  }

  //expand grid
//  for(int i =0; i<row; i++){
//    for(int j=0; j<col; j++){
//      if(_grid_attr[i][j].attribute == OBS){
//        for(int loop1 = i - exp_grid_size; loop1 <= i + exp_grid_size; loop1++){
//          for(int loop2 = j - exp_grid_size; loop2 <= j + exp_grid_size; loop2++){
//            if(loop1>=0 && loop1<row && loop2>=0 && loop2<col){
//              img.at<cv::Vec3b>(loop1,loop2) = cv::Vec3b(0, 0, 255);
//            }
//          }
//        }
//      }
//    }
//  }

  for(int i=5*range_front_1 -5; i<5*range_front_1+5; i++){
    for(int j=5*range_left_1-5; j<5*range_left_1+5; j++){
      //current car position, blue
      img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0, 0);
    }
  }

}

void cluster(Grid** _grid_attr, cv::Mat& img_gray, cv::Mat& img_color){
  int row = 5*(range_front_1+range_back_1);
  int col = 5*(range_left_1+range_right_1);

  for(int i = 0; i<row; i++){
    for(int j = 0; j<col; j++){
      if(_grid_attr[i][j].attribute == OBS){
        for(int loop1 = i - exp_grid_size_1; loop1 <= i + exp_grid_size_1; loop1++){
          for(int loop2 = j - exp_grid_size_1; loop2 <= j + exp_grid_size_1; loop2++){
            if(loop1>=0 && loop1<row && loop2>=0 && loop2<col){
              img_gray.at<uchar>(loop1, loop2) = 255;
            }
          }
        }
      }
    }
  }

  //findcontours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img_gray, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  //blue
  //cv::drawContours(img_color, contours, -1, cv::Scalar(255, 0, 0), 1, cv::LINE_AA, hierarchy);
  for(int i =0; i < contours.size(); i++){
    std::vector<cv::Point> points;
    for(int j =0; j<contours[i].size(); j++){
      points.push_back(contours[i][j]);
    }
    cv::RotatedRect box = cv::minAreaRect(points);
    double area = box.size.width * box.size.height;
    if(area > 36 && area < 300
       && box.size.width >= 6 && box.size.width <=25
       && box.size.height >= 6 &&box.size.height <=25
       && box.center.x >= 125 && box.center.x <= 150
       && box.center.y >= 300){
      cv::Point2f vtx[4];
      box.points(vtx);
      for(int k=0; k<4; k++){
        cv::line(img_color, vtx[k], vtx[(k+1)%4], cv::Scalar(255,0,0), 1, cv::LINE_AA);
      }
    }
  }
}
