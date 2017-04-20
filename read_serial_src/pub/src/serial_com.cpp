#include "serial_com.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

using namespace std;

CSerialCom::CSerialCom(){}
CSerialCom::~CSerialCom(){}

bool CSerialCom::open_serial(char* devicename)
{

  // open the serial port 
  if((m_fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK))<0)
  {
    printf("asy_write_com.c: failed to open %s\n", devicename); 
    return false; 
  }

    // get old fd state 
  tcgetattr(m_fd, &old_state); 

  // set serial 
  tio.c_cflag = B38400 | CS8 | CREAD | CRTSCTS | CLOCAL; 
  // tio.c_cflag = B19200 | CS8 | CREAD | CRTSCTS | CLOCAL; 

  // tio.c_cflag &= ~HUPCL; // clear the HUPCL 
  tio.c_lflag = 0; // set input noncanonical, no processing 
  tio.c_iflag = IGNPAR; // ignore party errors 
  tio.c_oflag = 0; // set output flag noncanonical 
  tio.c_cc[VTIME] = 0; // no time delay 
  tio.c_cc[VMIN] = 1; // no char delay 
  tcflush(m_fd, TCIFLUSH); // flush buffer 
  tcsetattr(m_fd, TCSANOW, &tio);

  // fcntl(m_fd, F_SETFL, FNDELAY);
  fcntl(m_fd, F_SETFL, FASYNC);

  return true; 
}

bool CSerialCom::send(char* buf, int len)
{
  int result = write(m_fd, buf, len); 
  if(result < 0 || result != len)
  {
    close_serial();
    return false;
  }
  return true;
}

bool CSerialCom::recv(char* buf, int len, char stop_sign)
{
  char tbuf[1024]; // currently do not make len > 1024
  char last_sign;
  int tlen;
  int left = len;
  int i;
  while(left > 0)
  {
    int res = read(m_fd, tbuf, 1024); 
    // printf("read %d characters: \n", res);
    for(int i=0; i<res; i++)
    {
      if(tbuf[i] == stop_sign)
      {
        if(left == 1) // all received 
        {
          // buf[len-left] = tbuf[i];
          return true; 
        }

        tlen = res - i -1; 
        strncpy(buf, tbuf + i + 1, tlen); 
        left = len - tlen; 
        break; 
      }
      if(left > 1)
        buf[len-left] = tbuf[i];
      else
        last_sign = tbuf[i];
      left--; 
    }
    if(res <= 0)
    {
      usleep(100);   
    }
  }
  // if(buf[len-1] != stop_sign)
  if(last_sign != stop_sign)
  {
    printf("serial_com.cpp: the last character is %c not %c, something error!", last_sign, stop_sign); 
    return false;
  }
  return true;
}

void CSerialCom::close_serial()
{
  tcsetattr(m_fd, TCSANOW, &old_state); 
  close(m_fd);
}
