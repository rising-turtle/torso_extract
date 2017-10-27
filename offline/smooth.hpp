/*
 *  Oct. 27 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  Smooth the input , mean filter first 
 *
 * */

#include <iostream>

template<typename T, int L>
CSmooth<T,L>::CSmooth()
{
  assert(L >= 1); 
}

template<typename T, int L>
CSmooth<T,L>::~CSmooth()
{}

template<typename T, int L>
void CSmooth<T,L>::push(T e)
{
  mv_raw.push_back(e); 
  if(mv_raw.size() > L)
  {
    mv_raw.pop_front();
  }
  // cout <<"mv_raw.size() = "<<mv_raw.size()<<" L = "<<L<<endl;
}

template<typename T, int L>
T CSmooth<T,L>::pop()
{
  int N = mv_raw.size(); 
  assert(N > 0); 
  typename deque<T>::iterator it = mv_raw.begin(); 
  T sum = 0; 
  while(it != mv_raw.end())
  {
    sum += (*it); 
    ++it; 
  }
  
  return (sum)/(T)(N); 
}

template<typename T, int L>
void CSmooth<T,L>::clear()
{
  mv_raw.clear(); 
}

template<typename T, int L>
bool CSmooth<T,L>::empty()
{
  return (mv_raw.size() == 0);
}




