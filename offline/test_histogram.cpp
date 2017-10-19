/*
 *  Oct. 20, 2016 David Z
 *  
 *  Test the result of histogram filter 
 *  
 * */


#include <librealsense/rs.hpp>
#include "example.hpp"
#include "body_extract.h"
#include "global.h"
#include "histogram_filter.h"

#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <fstream>

inline void glVertex(const rs::float3 & vertex) { glVertex3fv(&vertex.x); }
inline void glTexCoord(const rs::float2 & tex_coord) { glTexCoord2fv(&tex_coord.x); }

// struct state { double yaw, pitch, lastX, lastY; bool ml; std::vector<rs::stream> tex_streams; int index; rs::device * dev; };

struct state { double yaw, pitch, lastX, lastY; bool ml; int index; };

void setWinUI(GLFWwindow **pwin); 

void histogramFilter(void** pts, vector<int>& indices, vector<bool>& bs);

void test_body_orientation(int, char**);

int main(int argc, char * argv[]) try
{
    test_body_orientation(argc, argv); 

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}


bool getPTFromlog(vector<rs::float3>& pts, vector<int>& indices, const char* f); 

float compute_angle(float n[3])
{ 
    float l = sqrt(n[0]*n[0] + n[2]*n[2]); 
    if (l <= 1e-5) return 0; 
    return (asin(n[0]/l) * 180./M_PI);
}

void test_body_orientation(int argc, char* argv[])
{	
    // rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

   
    glfwInit();
    // state app_state = {0, 0, 0, 0, false, {rs::stream::color, rs::stream::depth, rs::stream::infrared}, 0, &dev};
    state app_state = {0, 0, 0, 0, false, 0};
    GLFWwindow * win = glfwCreateWindow(640, 480, "test_histogram", 0, 0);
    glfwSetWindowUserPointer(win, &app_state);
     
	setWinUI(&win); 
   
    glfwMakeContextCurrent(win);
    texture_buffer tex;

    CBodyExtract body_extract; 

    string fname = "./log/pts/log"; 
    if(argc > 1)
      fname = string(argv[1]); 
    vector<rs::float3> pv; 
    vector<int> indices; 
    if(!getPTFromlog(pv, indices, fname.c_str()))
    {
      cout <<" what? failed to load pt!"<<endl; 
      return ;
    }

    CHisFilter hf;

    while (!glfwWindowShouldClose(win))
    {
        glfwPollEvents();
          
        glPushAttrib(GL_ALL_ATTRIB_BITS);

        int width, height;
        glfwGetFramebufferSize(win, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(52.0f/255, 72.f/255, 94.0f/255.0f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        gluPerspective(60, (float)width/height, 0.01f, 20.0f);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        gluLookAt(0,0,0, 0,0,1, 0,-1,0);

        glTranslatef(0,0,+0.5f);
        glRotated(app_state.pitch, 1, 0, 0);
        glRotated(app_state.yaw, 0, 1, 0);
        glTranslatef(0,0,-0.5f);

        glPointSize((float)width/640);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, tex.get_gl_handle());
        glBegin(GL_POINTS);

        // auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
	
        // for display orientation 
        float len = 0.2; // 20 cm line represent the body direction 
        bool display_body_orientation = false; 
        float uncertainty = 0;   // how confidence for the body direction 
        float centroid_pt[3]; 
        float nv_direction[3]; 
        vector<bool> b_remained;
        rs::float3 * points = &(pv[0]); 
        vector<int> remained_indices; 
	// if(body_extract.segmentFromCentral((void**)(&points), 640, 480, indices))
	{
                // histogramFilter((void**)(&points), indices, b_remained); 
            
                hf.test((void**)(&points), indices, b_remained); 

		for(int i=0; i< indices.size(); i++)
		{
			rs::float3 * pt = points + indices[i]; 
                        if(b_remained[i]){
                          glColor3f(0, 1.0, 0.0);
                          remained_indices.push_back(indices[i]);
                        }
                        else{
                          glColor3f(1.0, 0, 0);
                        }
			glVertex(*pt);
		}

                // uncertainty = body_extract.extractOrientation((void**)(&points), indices, centroid_pt, nv_direction); 
                uncertainty = body_extract.extractOrientation((void**)(&points), remained_indices, centroid_pt, nv_direction); 
                display_body_orientation = true; 
	}

        glEnd();

        // cout << " compute nv: "<<nv_direction[0]<<" "<<nv_direction[1]<<" "<<nv_direction[2]<<" theta = "<<compute_angle(nv_direction)<<endl; 

        if(display_body_orientation)
        {
          glColor3ub((unsigned char)(255*(1-uncertainty)), (unsigned char)(255*(uncertainty)), 0);
          glLineWidth(2.);
          glBegin(GL_LINE_STRIP); 
            glVertex3f(centroid_pt[0], centroid_pt[1], centroid_pt[2]); 
            glVertex3f(centroid_pt[0] + len*nv_direction[0], centroid_pt[1] + len* nv_direction[1], centroid_pt[2] + len*nv_direction[2]);
          glEnd();
          glPointSize(20.0);
          glBegin(GL_POINTS); 
            glColor3f(1.0, 1.0, 0.0); 
            glVertex3f(centroid_pt[0], centroid_pt[1], centroid_pt[2]);
            glColor3f(1.0, 0., 1.0);
            glVertex3f(centroid_pt[0] + len*nv_direction[0], centroid_pt[1] + len* nv_direction[1], centroid_pt[2] + len*nv_direction[2]); 
          glEnd();
        }
        glPopMatrix();
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glPopAttrib();

        glfwGetWindowSize(win, &width, &height);
        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glPushMatrix();
        glOrtho(0, width, height, 0, -1, +1);
        
        glPopMatrix();

        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();

}

bool getPTFromlog(vector<rs::float3>& pts, vector<int>& indices, const char* f)
{
  ifstream inf(f); 
  if(!inf.is_open()) return false; 

  int i= 0; 
  while(!inf.eof())
  {
    rs::float3 pt; 
    inf>>pt.x>>pt.y>>pt.z; 
    pts.push_back(pt);
    indices.push_back(i++);
  }
  cout <<" succeed to load "<<pts.size()<<" pts"<<endl;
  return true;
}

/////////////////////////////////////////////////////////////////
// 
// Histogram filter to exclude points has less accumulated value 
// 
//  size x [-1.5, 1.5], z [0 1.5]
//  map  x [0, 300],  z [0 150]
//

static const int S = 100; 
static const int R = 3; // resolution 5 cm
// static const int MAX_X = 300/R; 
static const int MAX_Z = 150/R; 
static const int HIS_THRE = 500;

inline int x2map(float x){return ((int)((x+1.5)*S))/R;}
inline int z2map(float z){ return ((int)(z*S))/R;}

typedef struct _HisElem{int his_v;
  _HisElem():his_v(0){}
} HisElem;

int avgNeighbor(HisElem H[MAX_X][MAX_Z], int mx, int mz);

void histogramFilter(void** pts, vector<int>& indices, vector<bool>& bs)
{
  rs::float3* p = reinterpret_cast<rs::float3*>(*pts); 
  
  unsigned int N = indices.size(); 
  // static vector<int> histogram_v; 
  // histogram_v.clear(); 
  // histogram_v.resize(N, 0); 
  bs.resize(N, false); 
  HisElem his[MAX_X][MAX_Z]; 

  static vector<int> tmp_x; tmp_x.reserve(MAX_X*MAX_Z/2); 
  tmp_x.clear(); 
  static vector<int> tmp_z; tmp_z.reserve(MAX_X*MAX_Z/2);
  tmp_z.clear(); 

  int added_cnt = 0; 
  // 1. map to histogram grids
  for(int i=0; i<N; i++)
  {
    bs[i] = false; 
    rs::float3* pt = p + indices[i]; 
    
    int mx = x2map(pt->x);
    int mz = z2map(pt->z); 
    if(mx < 0 || mx >= MAX_X || mz <0 || mz >= MAX_Z) 
    {
      continue ;
    }

    ++ added_cnt; 
    HisElem& t= his[mx][mz]; 
    if(t.his_v == 0)
    {
     tmp_x.push_back(mx); 
     tmp_z.push_back(mz); 
    }
    t.his_v ++ ;
    // t.index.push_back(i);
  }
  // cout <<" added_cnt : "<<added_cnt<<endl;

  /*
  // 2. statistical compute histogram values
  unsigned int M = tmp_x.size(); 
  static vector<unsigned int> hs; // (M, 0);
  hs.clear();
  hs.reserve(N); 
  cout<< " after histogram computation M = "<<M<<" N = "<<N<<endl;

  static int num_id = 0; 
  stringstream ss; 
  ss << "./log/histogram_"<<++num_id<<".log";
  // ofstream ouf(ss.str().c_str());

  for(int i=0,k=0; i<M; i++)
  {
  //  cout << "i = "<<i<< " tmp_x[i] "<<tmp_x[i]<<" tmp_z[i] "<<tmp_z[i];
    HisElem& t = his[tmp_x[i]][tmp_z[i]]; 
    
    // for(int j=0; j<t.his_v; j++)
    {
    //  cout<<" j= "<<j<< " t.index[j] = "<<t.index[j] ;
    //  histogram_v[t.index[j]] = t.his_v; 
    //  cout << " k = "<<k<<endl;
      hs.push_back(t.his_v);
      // ouf<<t.his_v<<endl;
      // hs[k++] = t.his_v;
    }
    // t.his_v = 0; 
    // t.index.clear();
  }
  
  cout <<" before compute mu and sigma hs.size() "<<hs.size()<<endl; 
  // compute mean and sigma 
  unsigned int mu, sigma; 
  compute_mu_sigma<unsigned int>(&hs[0], hs.size(), mu, sigma); 
  
  cout<<" mu = "<<mu<<" sigma = "<<sigma<<endl;
*/
  // 3. for each 3d point, get its histogram value, and set its validity 
  int cnt = 0; 
  int out_range_cnt = 0; 
  int zero_cnt = 0; 
  int away_sig_cnt = 0;

  for(int i=0; i<N; i++)
  {
    rs::float3* pt = p + indices[i]; 
    int mx = x2map(pt->x);
    int mz = z2map(pt->z); 
    if(mx < 0 || mx >= MAX_X || mz <0 || mz >= MAX_Z) 
    {
      ++out_range_cnt;
      continue ;
    }
    
    int h = avgNeighbor(his, mx, mz); 
    
    // HisElem& t= his[mx][mz]; 
    // if(t.his_v == 0 ) 
    if(h == 0)
    {
      zero_cnt ++ ;
      continue ;
    }

    // if(t.his_v > HIS_THRE)
    if(h >= HIS_THRE)
    {
      bs[i] = true;
      ++ cnt; 
    }else{
      ++ away_sig_cnt; 
    }
    // t.his_v = 0;
  }
  
  // cout<<" remained "<<cnt<<" out_range:  "<<out_range_cnt<<" zero: "<<zero_cnt<<" away_sig: "<<away_sig_cnt<<" total: "<<(cnt + out_range_cnt + zero_cnt+ away_sig_cnt)<<endl;

  return ;
}

bool isIllegal(int nx, int nz)
{
  if(nx < 0 || nz <0 || nx >= MAX_X || nz >= MAX_Z) 
    return true;
  return false;
}

void neighbor(int casen, int mx, int mz, int& nx, int& nz)
{
  switch (casen)
  {
    case 1: {nx = mx - 1; nz = mz    ; break; }
    case 2: {nx = mx;     nz = mz - 1; break; }
    case 3: {nx = mx + 1; nz = mz    ; break; }
    case 4: {nx = mx;     nz = mz + 1; break; }
    case 5: {nx = mx + 1; nz = mz + 1; break; }
    case 6: {nx = mx + 1; nz = mz - 1; break; }
    case 7: {nx = mx - 1; nz = mz + 1; break; }
    case 8: {nx = mx - 1; nz = mz - 1; break; }
  }
  return ; 
}

int avgNeighbor(HisElem H[MAX_X][MAX_Z], int mx, int mz)
{
  int cnt = 1; 
  int total = 0; 
  int nx, nz; 
  if(isIllegal(mx, mz)) return 0; 
  if(H[mx][mz].his_v == 0) return 0; 
  total += H[mx][mz].his_v; 
  for(int i=0; i<8; i++)
  {
    neighbor(i+1, mx, mz, nx, nz); 
    if(isIllegal(nx, nz)) continue; 
    if(H[nx][nz].his_v == 0) continue; 
    cnt ++ ; 
    total += H[nx][nz].his_v; 
  }
  return (int)(total/cnt);
}


///////////////////////////////////////////////////
//
void setWinUI(GLFWwindow **pwin)
{
    GLFWwindow * win = *pwin; 

    glfwSetMouseButtonCallback(win, [](GLFWwindow * win, int button, int action, int mods)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if(button == GLFW_MOUSE_BUTTON_LEFT) s->ml = action == GLFW_PRESS;
        // if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) s->index = (s->index+1) % s->tex_streams.size();
    });
        
    glfwSetCursorPosCallback(win, [](GLFWwindow * win, double x, double y)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if(s->ml)
        {
            s->yaw -= (x - s->lastX);
            s->yaw = std::max(s->yaw, -120.0);
            s->yaw = std::min(s->yaw, +120.0);
            s->pitch += (y - s->lastY);
            s->pitch = std::max(s->pitch, -80.0);
            s->pitch = std::min(s->pitch, +80.0);
        }
        s->lastX = x;
        s->lastY = y;
    });
        
    glfwSetKeyCallback(win, [](GLFWwindow * win, int key, int scancode, int action, int mods)
    {
        auto s = (state *)glfwGetWindowUserPointer(win);
        if (action == GLFW_RELEASE)
        {
            if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(win, 1);
            else if (key == GLFW_KEY_F1)
            {
               // if (!s->dev->is_streaming()) s->dev->start();
            }
            else if (key == GLFW_KEY_F2)
            {
               // if (s->dev->is_streaming()) s->dev->stop();
            }
        }
    });

}

