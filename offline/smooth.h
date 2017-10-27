/*
 *  Oct. 27 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  Smooth the input , mean filter first 
 *
 * */

#ifndef SMOOTH_H
#define SMOOTH_H

#include <vector>
#include <deque>
#include <assert.h>

using namespace std; 

template<typename T, int L = 5>
class CSmooth
{
  public:
    CSmooth(); 
    virtual ~CSmooth(); 
    
    void push(T e);
    T pop();
    bool empty(); 
    void clear(); 

    deque<T> mv_raw; // raw data input  
};

#include "smooth.hpp"


#endif
