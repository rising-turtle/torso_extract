/*
 * Oct. 26, 2016 David Z 
 *
 * Integrate the histogram filter functions into a class 
 *
 * */


#ifndef HISTOGRAM_FILTER_H
#define HISTOGRAM_FILTER_H

#include "global_def.h"
#include <vector>
#include <iostream>

/////////////////////////////////////////////////////////////////
// 
// Histogram filter to exclude points has less accumulated value 
// 
//  size x [-1.5, 1.5], y [0 1.5]
//  map  x [0, 300],  y [0 150]
//


#define CAP_X 300
#define CAP_Y 150 
#define RESOLUTION 5 
#define MAX_X (CAP_X/RESOLUTION)
#define MAX_Y (CAP_Y/RESOLUTION)
#define SHIFT_X 1.5
#define SHIFT_Y 0
#define SCALE 100

using namespace std; 

class CHisFilter
{
  public:
    CHisFilter(); 
    ~CHisFilter(); 

    bool histogramFilter(void** pts, vector<int>& indices, vector<bool>& ); 
    void computeHisValue(void** pts, vector<int>& indices); 
    void selectPoints(void** pts, vector<int>& indices, vector<bool>& );
    void clearHisValue(); 
    void neighbor(int casen, int mx, int my, int& nx, int& ny);
    int avgNeighbor(int mx, int my); 
    void meanFilter();  
    inline int x2map(float x) {return (int)(((x+SHIFT_X)*SCALE)/RESOLUTION); }
    inline int y2map(float y) {return (int)(((y+SHIFT_Y)*SCALE)/RESOLUTION); }
    bool isIllegal(int mx, int my); 
  
    void test(void** pts, vector<int>& indices, vector<bool>& bs); // use OpenCV's mat to show the histogram value 
    int regionGrow(); // region grow with m_sx m_sy, return the number of grids in the obtained region 
    void rescaleAvgV(int S = 255); 

    int m_HisV[MAX_X][MAX_Y]; 
    int m_AvgV[MAX_X][MAX_Y]; 
    int m_maxHisV; 
    int m_maxAvgV;

    // used for region growing 
    int m_ResV[MAX_X][MAX_Y];
    int m_Used[MAX_X][MAX_Y]; 
    int m_sx; 
    int m_sy; 
};

#endif
