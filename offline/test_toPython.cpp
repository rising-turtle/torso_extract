/*
 *  Oct. 21 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  read an image, send to python polyfit.py for processing
 *
 * */

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include "toPython.h"

using namespace std; 

void findNonZero(cv::Mat img, vector<float>& px, vector<float>& py);

int main()
{
  // 1. load image, and extract nonzero pts 
  cv::Mat img = cv::imread("141.png"); 
  vector<float> px, py; 
  findNonZero(img, px, py); 

  // 2. transfer to python for processing 
  ToPython topy("/home/davidz/work/github/torso_extract/offline", "polyfit"); 
  float result =  topy.List2F1("point_pipeline",(float*)(px.data()), (float*)(py.data()), px.size());

  cout <<"test_toPython: result: "<<result<<endl;
  return 0; 
}

void findNonZero(cv::Mat img, vector<float>& px, vector<float>& py)
{
  for(int r=0; r<img.rows; r++)
    for(int c=0; c<img.cols; c++)
    {
      unsigned char v = img.at<unsigned char>(r,c); 
      if(v > 0){
        px.push_back(c);
        py.push_back(r); 
      }
    }
  return ;
}

