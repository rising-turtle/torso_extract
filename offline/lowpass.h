/**
    May 29 2018, He Zhang, hxzhang1@ualr.edu
    
    low pass filter 

*/

#pragma once 

using namespace std; 

class CLowPass
{
    public:
	CLowPass(double tau, double ts); 
	double filt(double x); 
	double m_a; 
	double m_b; 
	double m_pre;
	bool m_first; 
};
