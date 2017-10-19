/*
 *  Oct. 18 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  extract torso using camera data stored in disk
 *  
 *  designed to test algorithm 
 *
 * */

// #include <librealsense/rs.hpp>
#include "body_extract.h"
#include "global.h"
#include "global_def.h"
#include <GLFW/glfw3.h>
#include <GL/glu.h>
#include <GL/gl.h>
// #include "example.hpp"
#include "histogram_filter.h"

#include <chrono>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <algorithm>
#include <fstream>
#include "rs_r200_wrapper.h"
#include "cam_model.h"


using namespace std; 

struct state { double yaw, pitch, lastX, lastY; bool ml; int index; };

inline void glVertex(const rs::float3 & vertex) { glVertex3fv(&vertex.x); }

void setWinUI(GLFWwindow **pwin); 

void offline_pipeline(); 

void computePts(vector<rs::float3>& pts, vector<int>& indices, CamModel& cam, cv::Mat& rgb, cv::Mat& dpt, float scale); 

bool readData(string dir, int index, CamModel& cam, vector<rs::float3>& pts); 

int main(int argc, char* argv[])
{
  offline_pipeline(); 
  return 0; 
}

void offline_pipeline()
{
  // 
  glfwInit();
  // state app_state = {0, 0, 0, 0, false, {rs::stream::color, rs::stream::depth, rs::stream::infrared}, 0, &dev};
  state app_state = {0, 0, 0, 0, false, 0};
  GLFWwindow * win = glfwCreateWindow(640, 480, "offline_pipeline", 0, 0);
  glfwSetWindowUserPointer(win, &app_state);

  setWinUI(&win); 

  glfwMakeContextCurrent(win);
  // texture_buffer tex;
  GLuint texture;

  string dir("/home/davidz/work/data/up/rollator/dataset1");

  // generate 3d point cloud 
  vector<rs::float3> pv; 
  float d = 1.;
  // CamModel c2h(608.1673/d, 605.7/2,  323.717/d, 228.8423/d, 0.18159, -0.60858); 
  CamModel c2h(591.6589, 591.6589, 310.7385, 239.500);

  // point to the rgb data 
  // unsigned char* prgb = rgb.data; 

  // cout<<"torso_offline.cpp: read N of pts: "<<indices.size()<<endl; 

  CBodyExtract body_extract; 
  CHisFilter hf;

  // ofstream ouf("pts.log"); 

  // while (!glfwWindowShouldClose(win))
  for(int j=20; j<400 && !glfwWindowShouldClose(win); j++)
  {
    // cv::imshow("rgb", rgb);
    // cv::waitKey(10); 
    // cv::imshow("dpt", dpt);
    // cv::waitKey(10); 
  
    if(!readData(dir, j, c2h, pv))
    {
      break; 
    }

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

    glTranslatef(0,0,+0.6f); // 0.5
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    // glTranslatef(0,0,-0.5f);
    glTranslatef(0,0,-0.5f);

    glPointSize((float)width/640);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texture);

    // draw the human points 
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
    vector<int> indices; 
    if(body_extract.segmentFromCentral((void**)(&points), 640, 480, indices))
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
        // glColor3f(1.0, 0.0, 0); 

        // unsigned char* pt_bgr = prgb + indices[i]*3; 
        // glColor3f(((float)pt_bgr[2])/255., ((float)pt_bgr[1])/255.,((float)pt_bgr[2])/255.); 
        glVertex(*pt);
        // ouf<<pt->x<<", "<<pt->y<<", "<<pt->z<<endl;
      }

      // uncertainty = body_extract.extractOrientation((void**)(&points), indices, centroid_pt, nv_direction); 
      uncertainty = body_extract.extractOrientation((void**)(&points), remained_indices, centroid_pt, nv_direction); 
      display_body_orientation = true; 
    }else{
      cerr <<"torso_offline.cpp: no body extracted ! "<<endl; 
      sleep(2); 
    }

    glEnd();
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
    usleep(1000*20); 
  }
  glfwDestroyWindow(win);
  glfwTerminate();
}

bool readData(string dir, int index, CamModel& cam, vector<rs::float3>& pts)
{
  pts.clear(); 
  // read image from disk 
  string f_rgb, f_dpt; 
  stringstream rgb_ss, dpt_ss; 
  rgb_ss <<"/color/"<<std::setfill('0')<<setw(6)<<index<<".png"; 
  dpt_ss <<"/depth/"<<std::setfill('0')<<setw(6)<<index<<".png"; 

  f_rgb = dir + rgb_ss.str(); 
  f_dpt = dir + dpt_ss.str(); 
  cv::Mat rgb, dpt; 
  CRSR200Wrapper r200; 
  if(!r200.readOneFrameCV(f_rgb, f_dpt, rgb, dpt))  
  {
    cerr <<"torso_offline.cpp: failed to load image: "<<f_rgb<<endl; 
    return false; 
  }else{
    cout <<"torso_offline.cpp: handle image: "<<f_rgb<<endl; 
  }
  vector<int> indices; 
  computePts(pts, indices, cam, rgb, dpt, 0.001); 
  return true; 
}


void computePts(vector<rs::float3>& pts, vector<int>& indices, CamModel &cam, cv::Mat& rgb, cv::Mat& dpt,  float d_scale)
{
  int i = 0; 
  pts.resize(rgb.rows * rgb.cols); 
  indices.resize(rgb.rows * rgb.cols); 
  double px, py, pz;  
  float z; 
  for(int r = 0; r < rgb.rows; r++)
    for(int c = 0; c < rgb.cols; c++)
    {
      rs::float3 pt; 
     
      // z = depth.at<unsigned short>(int(v+0.5), int(u+0.5)) * depth_scale; 
      
      z = dpt.at<unsigned short>((int)(r), (int)(c))*d_scale;
      if(std::isnan(z) || z <= 0.0) 
      {
        indices[i++] = -1; 
        // ++d_nanz; 
        continue;
      }
      cam.convertUVZ2XYZ(c, r, z, px, py, pz); 
      if(std::isnan (px) || std::isnan(py) || std::isnan(pz))
      {
        // ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
        //FIXME Use parameter here to choose whether to use
        indices[i++] = -1; 
        continue;
      }else
      {
        pt.x = px; pt.y = py; pt.z = pz; 
        pts[i] = pt; 
        // indices.push_back(i++); 
        // indices.push_back(r*rgb.cols + c); 
        indices[i] = i; 
      }
      i++;  
    }
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
