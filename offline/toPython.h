/*  
 *  Oct. 21 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  toPython transfers data to python 
 *
 * */

#pragma once 

#include <Python.h>
#include <string>
#include <vector>

using namespace std; 

class ToPython
{
  public:
    ToPython(string dir = "./", string modulename=""); 
    ~ToPython();
    void init(); 
    void uninit(); 
    float List2F1(string module_name, float* l1, float* l2, int N); 
    string mModulePath; // python script directory 
    string mModuleName; // module name 
    PyObject * mpModule;    // module pointer 
    PyObject * mpDict;      // dictionary attribution
};
