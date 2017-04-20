
#include "serial_com.h"
#include <stdio.h>
#include <string.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <csignal>

void test_write(CSerialCom& s); 
void test_read(CSerialCom& s);
void signal_handler_IO (int status);
void signal_handler_INT(int );

bool wait_flag = true; 
const int N = 3; 
const char stop_sign = '#';

int main(int argc, char* argv[])
{
  CSerialCom serial; 
  char devicename[32] = {0}; 
  strcpy(devicename, "/dev/ttyS4"); 
  if(argc >1)
    strcpy(devicename, argv[1]); 


  if(!serial.open_serial(devicename))
  {
    printf("failed to open serial port"); 
    return -1;
  }

  // register SIG_IO handler 
  /*struct sigaction saio;               //definition of signal action
  saio.sa_handler = signal_handler_IO;
  sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
  saio.sa_flags = 0;
  saio.sa_restorer = NULL;
  sigaction(SIGIO,&saio,NULL);
  */
  signal(SIGIO, signal_handler_IO);

  int i=0;
  while(true)
  {
    if(wait_flag == false)
    {
      wait_flag = true;
      // printf("test_read at %d times\n", i++);
      test_read(serial); 
    }else
      usleep(100*1000);
  }
  // test_write(serial);

  return 0; 
}

void test_write(CSerialCom& s)
{
  float buf[N]; 
  printf("send %d floats: \n", N); 
  for(int i=0; i<N; i++)
  {
    buf[i] = (i+1)*2.4; 
    printf("%f ", buf[i]);
  }
  
  int Len = sizeof(float)*N + 1;
  char* tbuf = new char[Len]; 
  strncpy(tbuf, (char*)buf, Len-1); 
  tbuf[Len-1] = stop_sign; 

  if(s.send(tbuf, Len))
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
    printf("succeed read %d floats: ", N);
    for(int i=0; i<N; i++)
      printf("%f ", buf[i]); 
    printf("\n");
  }else
  {
    printf("failed to correctly read %d floats, received: ", N); 
    for(int i=0; i<N; i++)
    {
      printf("%f ", buf[i]); 
    }
    printf("\n");
  }
  return ;
}

void signal_handler_IO (int status)
{
   // printf("received SIGIO signal.\n");
   wait_flag = false;
}

void signal_handler_INT(int )
{
  printf("in singal_handler_INT\n");
  exit(1);
}


