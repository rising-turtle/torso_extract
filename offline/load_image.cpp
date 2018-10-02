/*
    process log to load image dirs 

*/

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std; 

bool loadImages(string filename, vector<string>& vrgb, vector<string>& vdpt, vector<double>& vt)
{
    ifstream inf(filename);
    if(!inf.is_open())
    {
	cout<<"load_image: failed to read file: "<<filename<<endl; 
	return false; 
    }
    
    vrgb.clear(); 
    vdpt.clear(); 
    vt.clear(); 

    while(!inf.eof())
    {
	string s;
	getline(inf, s); 
	if(!s.empty())
	{
	    stringstream ss;
	    ss << s; 
	    double t; 
	    ss >> t;
	    vt.push_back(t); 
	    string rgb_name; 
	    ss >> rgb_name; 
	    vrgb.push_back(rgb_name); 
	    ss >> t; 
	    string dpt_name;
	    ss >> dpt_name; 
	    vdpt.push_back(dpt_name); 
	}
    }
    cout <<"load_image: succeed to load "<<vt.size()<<" images!"<<endl; 
    return true; 
}
