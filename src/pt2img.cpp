/*
 *  Oct. 19 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  project 3d point cloud onto top view, resulting in a gray image 
 *
 * */

#include "pt2img.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>

#define SPAN 200
#define MAX_H 250

using namespace std; 

// X-Z horizontal -> X-Y image space 
void topviewPts(std::vector<rs::float3>& pts, bool save_img, int si, bool show_img)
{
  float min_x, max_x, min_z, max_z; 
  minmaxX(pts, min_x, max_x);
  minmaxZ(pts, min_z, max_z); 
  
  float span_x = max_x - min_x; 
  float span_z = max_z - min_z; 
  
  // cout <<"pt2img.cpp: span_x: "<<span_x<<" span_z: "<<span_z<<endl; 

  float span = span_x > span_z ? span_x : span_z; 
  // assert(span > 0); 
  float scale = SPAN/span; 
  
  int X_SPAN = (span_x * scale) + 1; 
  int Y_SPAN = (span_z * scale) + 1; 
  
  cv::Mat img = cv::Mat::zeros(Y_SPAN, X_SPAN, CV_8UC1); 
  vector<vector<unsigned char> > vH(Y_SPAN, vector<unsigned char>(X_SPAN, 0)); 

  int row; // [0~Y_SPAN-1]
  int col; 
  int max_v = 1; 
  // unsigned char tmp; 
  for(int i=0; i<pts.size(); i++)
  {
    rs::float3& pt = pts[i]; 
    row = Y_SPAN-1- (int)((pt.z - min_z)*scale);
    col = (int)((pt.x - min_x) * scale); 
    if(col < 0 || col >= X_SPAN || row < 0) continue; 
    
    // tmp = img.at<unsigned char>(row)(col) ++; 
    vH[row][col]++;
    if(vH[row][col] > MAX_H) vH[row][col] = MAX_H;  
    if(vH[row][col] > max_v) max_v = vH[row][col]; 
  }
  
  // rescale 
  if(max_v > 1){
    for(int r = 0 ; r<Y_SPAN; r ++)
      for(int c = 0; c<X_SPAN; c++)
      {
        img.at<unsigned char>(r,c) = (int)(((float)vH[r][c]/(float)max_v)*MAX_H);
      }
  }
  
  if(show_img)
  {
    // show and save the image 
    cv::imshow("topview", img); 
    cv::waitKey(20);
  }
  if(save_img)
  {
    // static int index = si; 
    stringstream ss; 
    ss <<"./tmp/"<<si<<".png"; 
    cv::imwrite(ss.str().c_str(), img); 
  }
  return ; 
}

void extractNonZero(std::vector<rs::float3>& pts, vector<float>& px, vector<float>& py)
{
  float min_x, max_x, min_z, max_z; 
  minmaxX(pts, min_x, max_x);
  minmaxZ(pts, min_z, max_z); 
  
  float span_x = max_x - min_x; 
  float span_z = max_z - min_z; 

  float span = span_x ; 
  float scale = SPAN/span; 
  
  int X_SPAN = (span_x * scale) + 1; 
  int Y_SPAN = (span_z * scale) + 1; 
  
  vector<vector<unsigned char> > vH(Y_SPAN, vector<unsigned char>(X_SPAN, 0)); 

  int row; // [0~Y_SPAN-1]
  int col; 
  int max_v = 1; 
  // unsigned char tmp; 
  for(int i=0; i<pts.size(); i++)
  {
    rs::float3& pt = pts[i]; 
    row = Y_SPAN-1- (int)((pt.z - min_z)*scale);
    col = (int)((pt.x - min_x) * scale); 
    if(col < 0 || col >= X_SPAN || row < 0) continue; 
    
    // tmp = img.at<unsigned char>(row)(col) ++; 
    vH[row][col]++;
    if(vH[row][col] == 1)
    {
      px.push_back(col);
      py.push_back(row); 
    }
    // if(vH[row][col] > MAX_H) vH[row][col] = MAX_H;  
    // if(vH[row][col] > max_v) max_v = vH[row][col]; 
  }
  
return ;
}

void minmaxX(std::vector<rs::float3>& pts, float& min, float & max)
{
  min = 10000;
  max = 0; 

  for(int i=0; i<pts.size(); i++)
  {
    rs::float3& pt = pts[i]; 
    if(pt.x > max) max = pt.x;
    if(pt.x < min) min = pt.x; 
  }
  // cout << "pt2img.cpp: range x : "<<min<<" - "<<max<<endl; 
  return ; 
}

void minmaxZ(std::vector<rs::float3>& pts, float& min, float & max)
{
  min = 10000;
  max = 0; 

  for(int i=0; i<pts.size(); i++)
  {
    rs::float3& pt = pts[i]; 
    if(pt.z > max) max = pt.z;
    if(pt.z < min) min = pt.z; 
  }
  // cout << "pt2img.cpp: range z : "<<min<<" - "<<max<<endl; 
  return ; 
}






