/*
 *  Oct. 19 2017, He Zhang, hxzhang1@ualr.edu
 *
 *  project 3d point cloud onto top view, resulting in a gray image 
 *
 * */

#pragma once 

#include "global_def.h"
#include <librealsense/rs.hpp>
#include <vector>

extern void minmaxX(std::vector<rs::float3>& pt, float& min, float & max); 
extern void minmaxZ(std::vector<rs::float3>& pt, float& min, float & max); 
extern void topviewPts(std::vector<rs::float3>&, bool save_img = false, int start_i = 1, bool show_img = true); 

extern void extractNonZero(std::vector<rs::float3>& pts, vector<float>& px, vector<float>& py); 

