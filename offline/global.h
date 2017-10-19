#ifndef GLOBAL_H
#define GLOBAL_H

#include <cmath>

#define SQ(x) ((x)*(x))

template<typename T>
bool compute_mu_sigma(T* in, int N, T& mu, T& sigma)
{
  if(N<=0)
  {
    mu = 0; 
    sigma = 0;
    return false;
  }

  // compute mu
  T total = 0; 
  for(int i=0; i<N; i++)
  {
    total += in[i];
  }
  mu = total/(T)(N); 
  
  // compute sigma
  total = 0;
  for(int i=0; i<N; i++)
  {
      total += SQ(in[i]-mu);
  }
  sigma = (T)sqrt(total/(T)N);
  return true;
}


#endif
