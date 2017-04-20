
#include "serial_com.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>

void test_write(CSerialCom& s); 
void test_read(CSerialCom& s); 
void test2(CSerialCom& s);
void test3(CSerialCom& s);

const int N = 3; 
const char stop_sign = '#';

int main(int argc, char* argv[])
{
  CSerialCom serial; 
  if(!serial.open_serial("/dev/ttyUSB0"))
  {
    printf("failed to open serial port"); 
    return -1;
  }

  // test_read(serial); 
  for(int i=0; i<10000; i++)
  {
    printf("test_write at %d times\n",i);
	test3(serial);
     // test2(serial);
     // test_write(serial);
    usleep(1000*100);
  }

  return 0; 
}

void test3(CSerialCom& s)
{
  static const int M = 3;  
  static float buf[M] = {0.295, -0.312};
  buf[M-1] = -sqrt(1 - buf[0]*buf[0] - buf[1]*buf[1]);  
  static unsigned char sbuf[2]; 
printf("send floats: ");
for(int i=0;i<M; i++)
{
	printf("%f ", buf[i]);
}

for(int i=0; i<=1; i++)
{  
  sbuf[i] = (int)fabs(buf[i]*100); 
  sbuf[i] = sbuf[i] + (buf[i] > 0 ? 100 : 0); 
   printf("%d ", sbuf[i]);
 } 

printf("\n");

sbuf[0] = 'a'; 
sbuf[1] = 'b';

  if(s.send((char*)sbuf, 2))
  {
    // printf("succeed send %d floats\n", N); 
  }else
  {
    printf("failed send floats!\n");
  }

  //  delete []tbuf; 

}



void test2(CSerialCom& s)
{
  static const int M = 13;  
  static char buf[M] = {'a', 'b', 'c'}; 
  printf("send %d chars :%s \n", M, buf); 
  for(int i=0; i<M; i++)
  {
buf[i] = 'a' + i;
   // buf[i] = (i+1); 
   printf("%d ", buf[i]);
  }
buf[M-1] = '\n';  
printf("\n");

  int Len = sizeof(char)*M;
  // char* tbuf = new char[Len]; 
  // strncpy(tbuf, (char*)buf, Len-1); 
  // tbuf[Len-1] = stop_sign; 

  if(s.send(buf, Len))
  {
    // printf("succeed send %d floats\n", N); 
  }else
  {
    printf("failed send floats!\n");
  }

  //  delete []tbuf; 

}

void test_write(CSerialCom& s)
{
  static float buf[N]; 
  // static char buf[N*4]; 
  printf("send %d floats: \n", N); 
  for(int i=0; i<N; i++)
  {
      buf[i] = (i+1)*2.4; 
      printf("%f ", buf[i]);
//	 buf[i] = 'a'+i; 
//	 printf("%c ", buf[i]);
  }
printf("\n");
char *pbuf = (char*)buf; 
  for(int i=0; i<N*4; i++)
{
	pbuf[i] = (60 + i)* (i%2?-1:1);
	printf("%d ", pbuf[i]);
}
printf("\n");

  int Len = sizeof(float)*N + 1;
  char* tbuf = new char[Len]; 
  strncpy(tbuf, (char*)buf, Len-1); 
  tbuf[Len-1] = '\n'; //stop_sign; 

  if(s.send(tbuf, Len))
  // if(s.send((char*)buf, N*4))
  {
    printf("succeed send %d floats\n", N); 
  }else
  {
    printf("failed send floats!\n");
  }

  delete []tbuf; 
}

void test_read(CSerialCom& s)
{
  float buf[N] = {0};
  int Len = sizeof(float)*N + 1; 
  if(s.recv((char*)buf, Len, stop_sign))
  {
    printf("succeed read %d floats: \n", N);
    for(int i=0; i<N; i++)
      printf("%f ", buf[i]); 
    printf("\n");
  }else
  {
    printf("failed to read %d floats: \n", N); 
  }
  return ;
}



