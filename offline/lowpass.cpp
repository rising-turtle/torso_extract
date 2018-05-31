#include "lowpass.h"


CLowPass::CLowPass(double tau, double ts)
{
    m_a = 1./(tau/ts + 1.);
    m_b = tau/ts/(tau/ts + 1.); 
    m_pre = 0; 
    m_first = true; 
}

double CLowPass::filt(double x)
{
    if(m_first == true)
    {
	m_first = false; 
    }else
    {
	x = m_a * x + m_b * m_pre; 
    }
    m_pre = x; 
    return x; 
}
