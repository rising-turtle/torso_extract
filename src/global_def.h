/*
 *  David Z 26. Oct. 2014
 *  
 *  global variables, functions
 *
 * */

#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <limits>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>
#include <Eigen/Eigen>
#include <Eigen/Core>

#define SQ(x) ((x)*(x))
#define R2D(r) (((r)*180.)/(M_PI))
#define D2R(d) (((d)*M_PI)/(180.))

using namespace std;

typedef enum {RED = 0, GREEN, BLUE, PURPLE, WHITE, YELLOW} COLOR; 
// extern bool markColor( color_point_cloud& , boost::shared_ptr<pcl::PointIndices>& index, COLOR );
// extern bool markColor( color_point_cloud& , COLOR);
// extern bool markColor( color_point_cloud&, int);

extern const unsigned char* translate(uint8_t* p8, uint16_t* p16, int N);
extern const unsigned char* translate(vector<uint8_t>& v8, vector<uint16_t>& v16);
extern const unsigned char* translateScale(vector<uint8_t>& v8, vector<uint16_t>& v16);

extern bool computeEigenVector2D(Eigen::Matrix2f& covariance_matrix, Eigen::Vector2f& major_axis, Eigen::Vector2f& minor_axis, 
                            float& major_value, float& minor_value);
extern bool computeEigenVector(Eigen::Matrix3f& cov, Eigen::Vector3f& major_axis, Eigen::Vector3f& middle_axis, 
                              Eigen::Vector3f& minor_axis, float& major_value, float& middle_value, float& minor_value);
extern int g_color[][3];

// #include "global_def.hpp" // compute centroid and normal of a point cloud  

#endif
