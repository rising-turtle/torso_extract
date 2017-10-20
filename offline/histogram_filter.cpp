#include "histogram_filter.h"
// #include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <queue>
#include <fstream>

CHisFilter::CHisFilter(){}
CHisFilter::~CHisFilter(){}


bool CHisFilter::histogramFilter(void** pts, vector<int>& indices, vector<bool>& bs)
{
  computeHisValue(pts, indices);  
  meanFilter();
  rescaleAvgV();
  int area = regionGrow(); 
  selectPoints(pts, indices, bs); 
  return true; 
}

void CHisFilter::selectPoints(void** pts, vector<int>& indices, vector<bool>& bs)
{
  rs::float3* p = reinterpret_cast<rs::float3*>(*pts); 
  unsigned int N = indices.size(); 
  bs.resize(N, false); 
  for(int i=0; i<N; i++)
  {
    rs::float3* pt = p + indices[i]; 
    
    int mx = x2map(pt->x); 
    int my = y2map(pt->z);  // y is vertical, [x, z] to 2D [mx,my],
    if(isIllegal(mx, my)) continue; 
    
    if(m_ResV[mx][my] > 0) bs[i] = true;

  }
}


void CHisFilter::computeHisValue(void** pts, vector<int>& indices)
{
  clearHisValue(); 
  unsigned int N = indices.size(); 
  rs::float3* p = reinterpret_cast<rs::float3*>(*pts); 
  
  for(unsigned int i=0; i<N; i++)
  {
    rs::float3* pt = p + indices[i]; 
    
    int mx = x2map(pt->x); 
    int my = y2map(pt->z);  // y is vertical, [x, z] to 2D [mx,my],

    if(mx < 0 || mx >= MAX_X || my < 0 || my >= MAX_Y) // out of range 
    {
      continue; 
    }
    m_HisV[mx][my] ++; 
  }

  return ;
}

void CHisFilter::clearHisValue()
{
  memset(m_HisV, 0, MAX_X*MAX_Y*sizeof(int)); 
  memset(m_ResV, 0, MAX_X*MAX_Y*sizeof(int));
  memset(m_Used, 0, MAX_X*MAX_Y*sizeof(int)); 
  m_maxHisV = 1; 
  m_maxAvgV = 1;
  m_sx = m_sy = -1; 
}

bool CHisFilter::isIllegal(int mx, int my)
{
  if(mx < 0 || my <0 || mx >= MAX_X || my >= MAX_Y) 
    return true; 
  return false; 
}

void CHisFilter::neighbor(int casen, int mx, int my, int &nx, int &ny)
{
  switch (casen)
  {
    case 1: {nx = mx - 1; ny = my    ; break; }
    case 2: {nx = mx;     ny = my - 1; break; }
    case 3: {nx = mx + 1; ny = my    ; break; }
    case 4: {nx = mx;     ny = my + 1; break; }
    case 5: {nx = mx + 1; ny = my + 1; break; }
    case 6: {nx = mx + 1; ny = my - 1; break; }
    case 7: {nx = mx - 1; ny = my + 1; break; }
    case 8: {nx = mx - 1; ny = my - 1; break; }
  }
  return ; 
}


int CHisFilter::avgNeighbor(int mx, int my)
{
  int cnt = 1; 
  int total = 0; 
  int nx, ny; 
  if(isIllegal(mx, my)) return 0; 
  // if(m_HisV[mx][my].his_v == 0) return 0; 
  total += m_HisV[mx][my]; 
  for(int i=0; i<8; i++)
  {
    neighbor(i+1, mx, my , nx, ny); 
    if(isIllegal(nx, ny)) continue; 
    // if(H[nx][nz].his_v == 0) continue; 
    cnt ++ ; 
    total += m_HisV[nx][ny]; 
  }
  return (int)(total/cnt);

}

void CHisFilter::meanFilter()
{
  for(int i=0; i<MAX_X; i++)
    for(int j=0; j<MAX_Y; j++)
    {
       if(m_HisV[i][j] >= m_maxHisV)
       {
          m_maxHisV = m_HisV[i][j]; 
       }

       int avgH = avgNeighbor(i, j); 
       m_AvgV[i][j] = avgH; 
       // m_maxAvgV = m_maxAvgV >= avgH ? m_maxAvgV : avgH; 

       if(avgH > m_maxAvgV)
       {
         m_maxAvgV = avgH; 
         m_sx = i; 
         m_sy = j; 
       }
    }
}

void CHisFilter::rescaleAvgV(int S)
{
  if(m_maxAvgV <= 1) return ; 

  for(int i = 0; i<MAX_X; i++)
    for(int j=0 ; j<MAX_Y; j++)
    {
      m_AvgV[i][j] = (int)(((float)m_AvgV[i][j]/(float)m_maxAvgV)*S);
    }
  m_maxAvgV = S; 
}

int CHisFilter::regionGrow()
{
  if(m_sx < 0 || m_sy < 0) 
    return -1; 
  int ret = 0;
  queue<int> qx; 
  queue<int> qy; 
  qx.push(m_sx); 
  qy.push(m_sy); 
  
  int mx, my, nx, ny; 
  int h; 
  // cout << __FILE__<< " start seed: "<<m_sx<<" "<<m_sy<<endl; 
  while(!qx.empty())
  {
    mx = qx.front();  qx.pop();
    my = qy.front();  qy.pop();
    m_Used[mx][my] = 1; 
    m_ResV[mx][my] = 1;
    h = m_AvgV[mx][my] ; 
    ++ret;
    // find its neighbor 
    for(int i=0; i<8; i++)
    {
      neighbor(i + 1, mx, my, nx, ny); 
      if(isIllegal(nx, ny)) continue; 
      if(m_Used[nx][ny]) continue; 
      
      // if(abs(m_AvgV[nx][ny] - h) < 50 && m_AvgV[nx][ny] > 50)
      if(m_AvgV[nx][ny] > 50)
      {
        qx.push(nx); 
        qy.push(ny); 
      }else{
        // cout<<" (nx, ny) = "<<nx<<" "<<ny<<" v[nx][ny] = "<<m_AvgV[nx][ny]<<" h = "<<h<<endl; 
      }
    }
  }
  // cout <<__FILE__<<" has found "<<ret<<" grids"<<endl;
  return ret; 
}

namespace{

  cv::Mat generateAMat(int h[MAX_X][MAX_Y], int max_h)
  {
    int BLOCK = 15;
    cv::Mat ret(MAX_Y*BLOCK, MAX_X*BLOCK, CV_8UC1);
    for(int i=0; i<MAX_X; i++)
    {
      for(int j=0; j<MAX_Y; j++)
      {
        for(int m=0; m<BLOCK; m++)
        {
          for(int n=0; n<BLOCK; n++)
          {
            ret.at<unsigned char>((MAX_Y-j-1)*BLOCK + m, i*BLOCK + n) = (int)(((float)h[i][j]/(float)max_h)*255);
          }
        }
      }
    }
    return ret;
  }

  void dumpToFile(int h[MAX_X][MAX_Y], int max_h)
  { 
    ofstream ouf("hisV.log"); 
    for(int j=0; j<MAX_Y; j++)
    {   for(int i=0; i<MAX_X; i++)
      {
        // ouf<<i+1<<" "<<j+1<<" "<< (int)(((float)h[i][j]/(float)max_h)*255) <<endl;
        ouf<< (int)(((float)h[i][j]/(float)max_h)*255) <<" ";
      }
      ouf << endl;
    }
    ouf.close(); 
    return ;
  }
}

void CHisFilter::test(void** pts, vector<int>& indices, vector<bool>& bs)
{
  computeHisValue(pts, indices);  
  meanFilter();
  rescaleAvgV();
  int area = regionGrow(); 
  
  selectPoints(pts, indices, bs); 

  // generate a cv::mat 
  // cv::Mat rawMat = generateAMat(m_HisV, m_maxHisV); 
  // cv::imshow("rawMat", rawMat);
  // cv::waitKey(10); 
  cv::Mat avgMat = generateAMat(m_AvgV, m_maxAvgV); 
  cv::imshow("avgMat", avgMat); 
  // dumpToFile(m_AvgV, m_maxAvgV); 
  cv::Mat resMat = generateAMat(m_ResV, 1); 
  cv::imshow("resMat", resMat);
  cv::waitKey(10);
}


