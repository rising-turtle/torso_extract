#include <stdio.h>
#include <tchar.h>
#include "Serial.h"	// Library described above
#include <string>
#include <cmath>
#include <string.h>

bool recv_orientation_data(Serial** sp, char* buf, int len, char stop_sign);
bool recv_orientation_data2(Serial** p_sp, char* buf, int len);
bool recv_orientation_data3(Serial** p_sp, float* buf);
bool recv_orientation_data4(Serial** p_sp, char* buf, short d[3]);

extern int decoder(unsigned char B1, unsigned char B2);

// application reads from the specified serial port and reports the collected data
int main1(int argc, char* argv[])
{
	printf("Welcome to the serial test app!\n\n");

	// Serial* SP = new Serial("\\\\.\\COM10");    // adjust as needed

	// Here is the COM port number you need to figure out by yourself
	Serial* SP = new Serial("\\\\.\\COM4"); // no 'device' here, just COM4, ok?yes good   // adjust as needed
	if (SP->IsConnected())
		printf("We're connected");


	char incomingData[256] = "";			// don't forget to pre-allocate memory
	//printf("%s\n",incomingData);
	int dataLength = 3; // 255
	int readResult = 0;

	const static int N = 3;
	// const char stop_sign = '#';
    // float buf[N] ;
    // char buf[N*4+1];
    char buf[10];
	// int len = sizeof(float)*N +1 ;
    // float* pf = (float*)buf;
    short d[3] = {0};

	while(SP->IsConnected())
	{
		// readResult = SP->ReadData(incomingData,dataLength);
		// printf("Bytes read: (0 means no data available) %i\n",readResult);
         //       incomingData[readResult] = 0;
        //if(readResult > 0)
        //	printf("receive: %s\n",incomingData);

		// Sleep(500);

		if(recv_orientation_data4(&SP, (char*)buf, d))
		// if(recv_orientation_data3(&SP, buf))
		// if(recv_orientation_data2(&SP, (char*)buf, len))
        // if(recv_orientation_data(&SP, (char*)buf, len, stop_sign))
            {
                // printf("succeed to read %d floats: \n", N);
                //for(int i=0; i<N; i++)
                    {
                        // printf("%f ", pf[i]);
                //        printf("%f ", buf[i]);
                printf("read data yaw = %d, x = %d, z = %d \n", d[0], d[1], d[2]);
                printf("read data yaw = %f, x = %f, z = %f \n", d[0]*0.1, d[1]*0.001, d[2]*0.001);
                    }
                // printf("\n");
                Sleep(1);
            }

	}
	return 0;
}

bool recv_orientation_data4(Serial** p_sp, char* buf, short d[3])
{
     Serial * sp = *p_sp;

    char* pd = (char*)buf;
    short yaw, x, z;
    unsigned short checksum1;
    unsigned short checksum2;
    while(1)
    {
        int i = sp->ReadData2(pd, 1);
        if(i == 0) // no data is received
        {
            Sleep(1);
            continue;
        }
        printf("read pd1 = %d\n", *pd);
        if(*pd == -1) // package header is 0xFFFF
        {
            pd ++ ;
            sp->ReadData2(pd,1);
            printf("read pd2 = %d\n", *pd);
            if(*pd == -1)
            {
                pd++;
                sp->ReadData2(pd, 8);

                yaw = decoder(*(pd), *(pd+1));
                x = decoder(*(pd+2), *(pd+3));
                z = decoder(*(pd+4), *(pd+5));

                d[0] = yaw;
                d[1] = x;
                d[2] = z;
                break;
            }
        }
        Sleep(1);
    }
    return true;
}

bool recv_orientation_data3(Serial** p_sp, float* buf)
{
    Serial* sp = *p_sp;
    int left = 2;
    static char tbuf[2];
    while(true)
    {
    int res = sp->ReadData2(tbuf,left);
    if(res <= 0)
    {
        Sleep(1);
        continue;
    }
    int i;
    // printf("received: ");
    for(i=0; i<left; i++)
    {
        // printf("%d ", tbuf[i]);
        if(tbuf[i] > 100)
            buf[i] = (tbuf[i] - 100)*(0.01);
        else
            buf[i] = (tbuf[i])*(-0.01);
    }
    // printf("\n");
    float t = 1-(buf[0]*buf[0]) - (buf[1]*buf[1]);
    if(t<0) continue; // transmission error happens
    buf[2] = -sqrt(t);
    break;
    }
    return true;
}


bool recv_orientation_data2(Serial** p_sp, char* buf, int len)
{
    Serial* sp = *p_sp;
    int left = len;
    char tbuf[1024];
    int i = 0;
    while(left >0 )
    {
        int res = sp->ReadData2(tbuf,left);
        if(res > 0)
        {
            printf("read %d data from COM\n", res);
            strncpy(buf+i, tbuf, res);
            left -= res;
            i+= res;
        }
        Sleep(1);
    }
    return true;
}

bool recv_orientation_data(Serial** p_sp, char* buf, int len, char stop_sign)
{
    Serial* sp = *p_sp;
    char tbuf[1024];
    char last_sign;
    int tlen;
    int left = len;
    int i;
    while(left > 0)
    {
        int res = sp->ReadData(tbuf,len);
        if(res > 0) printf("read %d data from COM\n", res);
        if(res == left && tbuf[res-1] == stop_sign)
        {
            strncpy(buf, tbuf, len-1);
            return true;
        }

        for(i=0; i<res; i++)
        {
            if(tbuf[i] == stop_sign)
            {
                if(res == left && i == res-1) // all received
                    return true;
                tlen = res - i - 1;
                strncpy(buf, tbuf+i+1, tlen);
                left = len - tlen;
                break;
            }
            if(left > 1)
                buf[len - left] = tbuf[i];
            else
                last_sign = tbuf[i];
            left--;
        }
        if(res <= 0)
        {
            Sleep(1);
        }
    }
    if(last_sign != stop_sign)
        {
            printf("the last char is %c not %c, something error\n", last_sign, stop_sign);
            return false;
        }
    return true;
}



