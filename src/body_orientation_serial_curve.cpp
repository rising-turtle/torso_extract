// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include "body_extract.h"
#include "serial_com.h"
#include "histogram_filter.h"

#include <cmath>
#include <chrono>
#include <vector>
#include <sstream>
#include <iostream>
#include <algorithm>

#include "pt2img.h"
#include "../offline/toPython.h"
#include "smooth.h"

extern unsigned short encoder(unsigned short v, bool neg);
extern unsigned short encoder(short v); 
extern int decoder(unsigned char b1, unsigned char b2); 

struct state { double yaw, pitch, lastX, lastY; bool ml; std::vector<rs::stream> tex_streams; int index; rs::device * dev; };

extern int CreatCRC(unsigned char* CommData, unsigned int uLen) ; 

void histogramFilter(void** pts, vector<int>& indices, vector<bool>& bs);

void test_body_orientation(int argc, char* argv[]);

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

///               / nz 
//		 /
//		/
// 		------> nx
// 		|
//		|
//		| ny

float compute_angle(float n[3])
{ 
    float l = sqrt(n[0]*n[0] + n[2]*n[2]); 
    if (l <= 1e-5) return 0; 
    return (asin(n[0]/l) * 180./M_PI);
}

void test_body_orientation(int argc, char* argv[])
{
  // 1. realsense setup 
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.enable_stream(rs::stream::infrared, rs::preset::best_quality);
    try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
    dev.start();
    
    state app_state = {0, 0, 0, 0, false, {rs::stream::color, rs::stream::depth, rs::stream::infrared}, 0, &dev};
    if(dev.is_stream_enabled(rs::stream::infrared2)) app_state.tex_streams.push_back(rs::stream::infrared2);
    
    std::ostringstream ss; ss << "CPP Point Cloud Example (" << dev.get_name() << ")";
      
  // 2. body extract 
    CBodyExtract body_extract; 
  
  // 3. serial port setup 
    CSerialCom serial_port; 
    char devicename[32] = "/dev/ttyUSB0"; 
    if(argc > 1) 
    {
      printf("devicename= %s\n", argv[1]);
      strcpy(devicename, argv[1]); 
    }
    if(!serial_port.open_serial(devicename)) 
    {
      printf("body_orientation_serial.cpp: failed to open serial port %s\n", devicename);
      throw std::runtime_error("Failed to open serial port");
    }

    unsigned short header = 0xFFFF; // 32700 
    int LS = sizeof(short); 
    unsigned char sbuf[5*LS] ; 
    char buf[32] = {0}; 
    const int F3 = sizeof(float)*3;

    int frames = 0; float time = 0, fps = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    
    CHisFilter hf; 

    // Smooth 
    CSmooth<double> smooth; 

    // interface to python script
    ToPython topy("/home/hub_robotics/work/body_orientation_new/ipynb", "polyfit"); 

    // while (!glfwWindowShouldClose(win))
    while(true)
    {
        if(dev.is_streaming()) dev.wait_for_frames();

        auto t1 = std::chrono::high_resolution_clock::now();
        time += std::chrono::duration<float>(t1-t0).count();
        t0 = t1;
        ++frames;
        if(time > 0.5f)
        {
            fps = frames / time;
            frames = 0;
            time = 0;
        }

        const rs::stream tex_stream = app_state.tex_streams[app_state.index];
        const float depth_scale = dev.get_depth_scale();
        const rs::extrinsics extrin = dev.get_extrinsics(rs::stream::depth, tex_stream);
        const rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
        const rs::intrinsics tex_intrin = dev.get_stream_intrinsics(tex_stream);
        bool identical = depth_intrin == tex_intrin && extrin.is_identity();
      
        int width, height;
             // auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
        auto depth = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));
	const rs::float3 * points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
	vector<int> indices; 
	
        // for display orientation 
        float len = 0.2; // 20 cm line represent the body direction 
        float uncertainty = 0;   // how confidence for the body direction 
        float centroid_pt[3]; 
        float nv_direction[3];
	float yaw = 0.;  
        vector<bool> b_remained;
        vector<int> remained_indices; 

        vector<rs::float3> pbody; 

        vector<float> nonzero_x; 
        vector<float> nonzero_y;

	if(body_extract.segmentFromCentral((void**)(&points), depth_intrin.width, depth_intrin.height, indices))
	{
            // histogramFilter((void**)(&points), indices, b_remained); 
            hf.histogramFilter((void**)(&points), indices, b_remained); 

            remained_indices.reserve(indices.size());
            for(int i=0; i<indices.size(); i++)
            {
              const rs::float3 *pt = points + indices[i];
              if(b_remained[i]) 
              {
                remained_indices.push_back(indices[i]);
                pbody.push_back(*pt); 
              }
            }

            // uncertainty = body_extract.extractOrientation((void**)(&points), indices, centroid_pt, nv_direction); 
            body_extract.extractOrientation((void**)(&points), remained_indices, centroid_pt, nv_direction);
	    // yaw = compute_angle(nv_direction); 
            extractNonZero(pbody, nonzero_x, nonzero_y); 

            // compute theta using polyfit 
            yaw =  topy.List2F1("point_pipeline",(float*)(nonzero_x.data()), (float*)(nonzero_y.data()), nonzero_x.size());

            // smooth the result
            smooth.push(yaw); 
            yaw = smooth.pop();

            // added Jan. 15 2018 He Zhang
            
            if(fabs(yaw) > 3.2)// degree
            {
              double scale = 2.87309; 
              double b = 0.64167;
              yaw = scale *(yaw + b); 
            }

            printf("body_orientation_serial.cpp: frame capture at %f fps, body orientation %f %f %f, yaw = %f\n", fps, nv_direction[0], nv_direction[1], nv_direction[2], yaw); 
		
            // memcpy(buf, (char*)nv_direction, F3); 
            // buf[F3] = '#'; 
	    // memcpy(sbuf, (char*)(&yaw), sizeof(float)); 
            // memcpy(sbuf+4, (char*)(&centroid_pt[0]), sizeof(float));
            // memcpy(sbuf+8, (char*)(&centroid_pt[2]), sizeof(float));

            // newly added Mar.2 2017 David Z
          short de[3] ; 
          short syaw = yaw*10; 
          short scentrol_ptx = centroid_pt[0]*1000; 
          short scentrol_ptz = centroid_pt[2]*1000; 
          // syaw = -10; scentrol_ptx = -21; scentrol_ptz = 32;
          unsigned short tmp; 
          memcpy(sbuf, &header, LS);
          tmp = encoder(syaw); 
          memcpy(sbuf+LS, &tmp, LS);  de[0] = decoder(sbuf[2], sbuf[3]); printf("syaw = %d encoder = 0x%04x de = %d \n", syaw, tmp, de[0]); 
          tmp = encoder(scentrol_ptx); 
          memcpy(sbuf+LS*2, &tmp, LS);  de[1] = decoder (sbuf[4], sbuf[5]); printf("sx = %d encoder = 0x%04x de = %d \n", scentrol_ptx, tmp, de[1]); 
          tmp = encoder(scentrol_ptz); 
          memcpy(sbuf+LS*3, &tmp, LS);  de[2] = decoder (sbuf[6], sbuf[7]); printf("sz = %d encoder = 0x%04x de = %d \n", scentrol_ptz, tmp, de[2]);
          unsigned short checksum = CreatCRC(sbuf, LS*4); 
          // tmp = encoder(checksum, false); 
          memcpy(sbuf+LS*4, &checksum, LS); 

          printf("send out syaw = %d sx = %d sz = %d checksum = %d\n", syaw, scentrol_ptx, scentrol_ptz, checksum); 

	    // make simple compression for the orientation data, ignore 0.001
	    /*for(int i=0; i<=1; i++)
	    {  
		    sbuf[i] = (int)fabs(nv_direction[i]*100);
		    sbuf[i] = sbuf[i] + (nv_direction[i] > 0 ? 100 : 0);
	    } */

            // if(serial_port.send(buf, F3+1))
            // sbuf[0] = 0xFF; sbuf[1] = 0xFF; sbuf[2] = 0x00; sbuf[3] = 0x01; sbuf[4] = 0x00; sbuf[5] = 0x02; 
            // sbuf[6] = 0x00; sbuf[7] = 0x03; sbuf[8] = 0x11; sbuf[9] = 0x22; 
	    if(serial_port.send((char*)sbuf, 5*LS))
            {
	      // printf("body_orientation_serial.cpp: succeed to send data: %i %i!\n", sbuf[0], sbuf[1]); 
              printf("body_orientation_serial.cpp: succeed to send data yaw, x, z: %f %f %f!\n", yaw, centroid_pt[0], centroid_pt[2]); 
            }else
            {
              printf("body_orientation_serial.cpp: failed to send data!\n"); 
              throw std::runtime_error("Failed to send data through serial port");
            }
	}
	else
	{
          // printf("body_orientation_serial.cpp: frame capture at %f fps, no body detected! \n", fps);
          // when person is out of view, reset the result  
          smooth.clear(); 
	}
    }
}


/////////////////////////////////////////////////////////////////
// 
// Histogram filter to exclude points has less accumulated value 
// 
//  size x [-1.5, 1.5], z [0 1.5]
//  map  x [0, 300],  z [0 150]
//

static const int S = 100; 
static const int R = 3; // resolution 3 cm
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
  /*
  cout <<" added_cnt : "<<added_cnt<<endl;

  // 2. statistical compute histogram values
  unsigned int M = tmp_x.size(); 
  static vector<unsigned int> hs; // (M, 0);
  hs.clear();
  hs.reserve(N); 
  cout<< " after histogram computation M = "<<M<<" N = "<<N<<endl;

  static int num_id = 0; 
  stringstream ss; 
  ss << "./log/histogram_"<<++num_id<<".log";
  ofstream ouf(ss.str().c_str());

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
      ouf<<t.his_v<<endl;
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
    // if(abs(t.his_v-mu) <= 2*sigma || t.his_v > HIS_THRE)
    if(h >= HIS_THRE)
    {
      bs[i] = true;
      ++ cnt; 
    }else{
      ++ away_sig_cnt; 
    }
    // t.his_v = 0;
  }
  
  cout<<" remained "<<cnt<<" out_range:  "<<out_range_cnt<<" zero: "<<zero_cnt<<" away_sig: "<<away_sig_cnt<<" total: "<<(cnt + out_range_cnt + zero_cnt+ away_sig_cnt)<<endl;

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




