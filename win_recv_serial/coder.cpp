#include <stdio.h>
#define MKSHORT(a,b) ((unsigned short)(a) | ((unsigned short)(b) << 8))

unsigned short encoder(unsigned short v, bool neg)
{
  unsigned char H8 = v/128; 
  unsigned char L8 = v - 128*H8; 
 
  if(neg) H8 |= 0x40; 
  return MKSHORT(L8, H8); 
}
unsigned short encoder(short v)
{
  if(v < 0) return encoder(v*-1, true); 
  return encoder(v, false); 
}

int decoder(unsigned char B1, unsigned char B2)
{
  if(B1 == 0xFF && B2 == 0xFF)
  {
    printf("header: 0xFFFF\n");
    return 0xFFFF; 
  }
  int l8 = B1 & 0x7F; 
  int h8 = B2 & 0x3F; 
  int ret = h8 * 128 + l8; 
  
  if(B2 & 0x40) ret *= -1; 
  return ret; 
}



