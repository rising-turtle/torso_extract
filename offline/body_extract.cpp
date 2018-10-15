#include "body_extract.h"
#include "global_def.h"
// #include <librealsense/rs.hpp>
#include <queue>

CBodyExtract::CBodyExtract(){}
CBodyExtract::~CBodyExtract(){}

// #define SQ(x) ((x)*(x))

namespace{

	float sqrDis(rs::float3* p1, rs::float3* p2)
	{
		return (SQ(p1->x - p2->x) + SQ(p1->y - p2->y) + SQ(p1->z - p2->z));
	}
}

int CBodyExtract::validIniPt(void** pts, int pw, int ph, int W, vector<bool>& flags)
{
  rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
  int pindex = ph * W + pw;
  rs::float3 * p_cur = p + pindex; 
  if(p_cur->z <= 2. && p_cur->z >= 0.4 && flags[pindex] == true)
    return pindex;
  return -1;
} 

int CBodyExtract::findInitialFromCentralWithFlag(void** pts, int cw, int ch, vector<bool>& flags)
{
  int h = ch * 2; 
  int w = cw * 2;
  int r = 100; 
  for(int ir = 0; ir<=r; ir++)
  {
  for(int ih=0; ih<=ir; ih++)
  // for(int iw=0; iw<=r; iw++)
    {
      int iw = ir - ih; 
      // int pw = cw + iw; 
      // int ph = ch + ih; 
      
      int ret = validIniPt(pts, cw + iw, ch + ih, w, flags); 
      if( ret >= 0) return ret; 
      
      ret = validIniPt(pts, cw - iw, ch + ih, w, flags); 
      if(ret >= 0) return ret; 

      ret = validIniPt(pts, cw + iw, ch - ih, w, flags); 
      if(ret >=0 ) return ret; 

      ret = validIniPt(pts, cw -iw, ch - ih, w, flags); 
      if(ret >= 0) return ret; 
    }
  }
  return -1;
}



int CBodyExtract::findInitialFromCentral(void** pts, int cw, int ch)
{
  int h = ch * 2; 
  int w = cw * 2;
  rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
  int r = 200; 
  for(int d=0; d<=200; d++)
  {
     for(int id=0; id<=d; id++)
     {
	int jd = d - id; 
	for(int ih = -id; ih <= id ; ih+= 2*id)
	{
	    for(int iw = -jd; iw <= jd ; iw += 2*jd)
	    {
		int pw = cw + iw; 
		int ph = ch + ih; 
		int pindex = ph * w + pw;
		rs::float3 * p_cur = p + pindex; 
		// cout <<"at x= "<<pw<<" y= "<<ph<<" pt: "<<p_cur->x<<" "<<p_cur->y<<" "<<p_cur->z<<endl;
		if(p_cur->z <= 2. && p_cur->z >= 0.4)
		    return pindex;
		if(jd == 0) break; 
	    }
	    if(id == 0) break; 
	}
     }/*
      for(int ih=0; ih<=d; ih++)
	  for(int iw=-r; iw<=r; iw+=2)
	  {
	      int pw = cw + iw; 
	      int ph = ch + ih; 
	      int pindex = ph * w + pw;
	      rs::float3 * p_cur = p + pindex; 
	      cout <<"at x= "<<pw<<" y= "<<ph<<" pt: "<<p_cur->x<<" "<<p_cur->y<<" "<<p_cur->z<<endl;
	      if(p_cur->z <= 2. && p_cur->z >= 0.4)
		  return pindex;
	  }*/
  }
  return -1;
}

bool CBodyExtract::segmentFromCentralWithFlag(void** pts, int w, int h, vector<int>& indices, vector<bool>& fvalid, float dis_threshold, int num_threshold)
{
	bool ret = false; 
	rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
	vector<vector<bool> > flags(h, vector<bool>(w, false)); 
	
	int cw = w/2; 
	int ch = h/2; 
	int cur_index = ch*w + cw; 
	rs::float3 *p_cur = p + cur_index;

	if(p_cur->z > 2. || p_cur->z < 0.4)
	{
          cur_index = findInitialFromCentralWithFlag(pts, cw, ch, fvalid);
          if(cur_index < 0)
          {
		cout<<"body_extract.cpp: no good point around central point z: "<<p_cur->z<<" not good!"<<endl;
		return ret; 
          }
	}	
	
	// collect the body points 
	vector<int> pt_index; 
	pt_index.reserve(num_threshold); 	

	flags[ch][cw] = true; 
	queue<int> Neigh; 
	Neigh.push(cur_index); 
	while(!Neigh.empty())
	{
		int seed_index = Neigh.front(); // get a seed  
		pt_index.push_back(seed_index);
		Neigh.pop(); 
		rs::float3 * p_seed = p + seed_index; 
		int sh = seed_index/w;
		int sw = seed_index - sh*w; 
		// search for neighbors 
		for(int ih=-1; ih<=1; ih++)
		for(int iw=-1; iw<=1; iw++)
		{
			ch = sh + ih; 
			cw = sw + iw; 
                        if(ch <0 || ch>= h || cw <0 || cw >= w) continue;
			if(flags[ch][cw]) continue; 	// this point has been visited 
			cur_index = ch * w + cw; 
			p_cur = p + cur_index; 
			if(p_cur->z < 0.2 || p_cur->z >= 2 || !fvalid[cur_index]) continue; // not valid point 
			if(sqrDis(p_seed, p_cur) <= dis_threshold) // find a good neighbor 
			{
				Neigh.push(cur_index); 
				flags[ch][cw] = true; 
			}
		}
	}

	if(pt_index.size() >= num_threshold)
	{
		// cout<<"body_extract.cpp: successfully extract body with "<<pt_index.size()<<" points"<<endl;
		indices = pt_index; 
		ret = true;
	}else
	{
	    cout<<"body_extract.cpp: failed to extract body with pt_index.size(): "<<pt_index.size()<<" threshold: "<<num_threshold<<endl;
	}

	return ret; 


}

bool CBodyExtract::segmentFromCentral(void** pts, int w, int h, vector<int>& indices, float dis_threshold, int num_threshold)
{
	bool ret = false; 
	rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
	vector<vector<bool> > flags(h, vector<bool>(w, false)); 
	
	int cw = w/2; 
	int ch = h/2; 
	int cur_index = ch*w + cw; 
	rs::float3 *p_cur = p + cur_index;

	if(p_cur->z > 2. || p_cur->z < 0.4)
	{
          cur_index = findInitialFromCentral(pts, cw, ch);
          if(cur_index < 0)
          {
		cout<<"body_extract.cpp: no good point around central point z: "<<p_cur->z<<" not good!"<<endl;
		return ret; 
          }
	}	
	
	// cout <<"cur_index: "<<cur_index<<endl;
	

	// collect the body points 
	vector<int> pt_index; 
	pt_index.reserve(num_threshold); 	
	
	int neighbor_range = 20; 

	flags[ch][cw] = true; 
	queue<int> Neigh; 
	Neigh.push(cur_index); 
	while(!Neigh.empty())
	{
		int seed_index = Neigh.front(); // get a seed  
		pt_index.push_back(seed_index);
		Neigh.pop(); 
		rs::float3 * p_seed = p + seed_index; 
		// cout <<"pt_index.size= "<<pt_index.size()<<" p_seed: "<<p_seed->x<<" "<<p_seed->y<<" "<<p_seed->z<<endl;
		int sh = seed_index/w;
		int sw = seed_index - sh*w; 
		// search for neighbors 
		for(int ih=-neighbor_range; ih<=neighbor_range; ih++)
		for(int iw=-neighbor_range; iw<=neighbor_range; iw++)
		{
			ch = sh + ih; 
			cw = sw + iw; 
                        if(ch <0 || ch>= h || cw <0 || cw >= w) continue;
			if(flags[ch][cw]) continue; 	// this point has been visited 
			cur_index = ch * w + cw; 
			p_cur = p + cur_index; 
			// cout <<"cw = "<<cw<<" ch = "<<ch<<" pt: "<<p_cur->x<<" "<<p_cur->y<<" "<<p_cur->z<<endl;  
			if(p_cur->z < 0.2 || p_cur->z >= 2) continue; // not valid point 
			if(sqrDis(p_seed, p_cur) <= dis_threshold) // find a good neighbor 
			{
				Neigh.push(cur_index); 
				flags[ch][cw] = true; 
			}
		}
	}

	if(pt_index.size() >= num_threshold)
	{
		// cout<<"body_extract.cpp: successfully extract body with "<<pt_index.size()<<" points"<<endl;
		indices = pt_index; 
		ret = true;
	}else{
            cout <<"body_extract.cpp: failed! only find "<<pt_index.size()<<" points"<<endl;
        }

	return ret; 
}


float CBodyExtract::extractOrientation(void ** pts, vector<int>& indices, float f[3], float nv[3])
{
  rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
  
  computeCentroid(pts, indices, f); // compute the centriod point of the input point cloud 

  // compute covariance matrix 
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();   
  Eigen::Vector3f pt; 
  for(int i=0; i<indices.size(); i++)
  {
    rs::float3 * p_cur = p + indices[i]; 
    pt[0] = p_cur->x - f[0]; pt[1] = p_cur->y - f[1]; pt[2] = p_cur->z - f[2]; 

    cov(1,1) += pt.y()*pt.y(); 
    cov(1,2) += pt.y()*pt.z(); 
    cov(2,2) += pt.z()*pt.z(); 
    pt *= pt.x(); 
    
    cov(0,0) += pt.x();
    cov(0,1) += pt.y(); 
    cov(0,2) += pt.z(); 
  }

  cov(1,0) = cov(0,1); 
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2); 

  // PCA decomposition for the covariance matrix to compute body orientation
  typedef Eigen::Vector3f V3f; 
  V3f major_axis, middle_axis, minor_axis; 
  float major_w, middle_w, minor_w; 
  computeEigenVector(cov, major_axis, middle_axis, minor_axis, major_w, middle_w, minor_w); 
  
  if(minor_axis[2] >= 0)
    minor_axis*= -1. ;

  nv[0] = minor_axis[0]; nv[1] = minor_axis[1]; nv[2] = minor_axis[2]; 


  return (2*minor_w)/(middle_w + major_w);

}


void CBodyExtract::computeCentroid(void** pts, vector<int>& indices, float pt[3])
{
  rs::float3 * p = reinterpret_cast< rs::float3 *>(*pts);
  
  double px, py, pz; 
  px = py = pz = 0; 
  for(int i=0; i<indices.size(); i++)
  {
    rs::float3 * p_cur = p + indices[i]; 
    px += p_cur->x; py += p_cur->y; pz += p_cur->z;  
  }
  
  double n = (double)indices.size();
  pt[0] = px/n; 
  pt[1] = py/n; 
  pt[2] = pz/n; 

  return ;
}

