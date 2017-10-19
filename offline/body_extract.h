/*
	Aug.26, 2016 David Z 
	
	Extract body segment, using flood flow from the central point, and then applying PCA to extract orientation 

*/

#ifndef BODY_EXTRACT_H
#define BODY_EXTRACT_H

#include <string>
#include <iostream>
#include <vector>

using namespace std; 

class CBodyExtract
{
public:
	CBodyExtract(); 
	~CBodyExtract(); 

	/* segmentation using flood flow start from the central point*/
	bool segmentFromCentral(void** pts, int w, int h, vector<int>& indices, float dis_threshold = 0.05*0.05, int num_threshold = 30000);
	
	/* segmentation using flood flow start from the central point with flags*/
	bool segmentFromCentralWithFlag(void** pts, int w, int h, vector<int>& indices, vector<bool>& flags, float dis_threshold = 0.05*0.05, int num_threshold = 30000);

        /* find a good start point in the central area around central point with flags*/
        int validIniPt(void** pt, int pw, int ph, int W, vector<bool>& flags); 
        int findInitialFromCentralWithFlag(void** pts, int cw, int ch, vector<bool>& flags); 

        /* find a good start point in the central area around central point */
        int findInitialFromCentral(void** pts, int cw, int ch); 
	
        /* compute the centroid point of a given point cloud*/
        void computeCentroid(void** pts, vector<int>& indices, float pt[3]);

	/* compute Orientation given the body segmentation */
	float extractOrientation(void **pts, vector<int>& indices, float f[3], float nv[3]); 
	
};





#endif
