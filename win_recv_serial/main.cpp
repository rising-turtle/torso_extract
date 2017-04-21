#include <stdio.h>
#include <tchar.h>
#include "Serial.h"	// Library described above
#include <string>
#include <cmath>
#include <string.h>

bool recv_orientation_data4(Serial** p_sp, char* buf, short d[3]);

extern int decoder(unsigned char B1, unsigned char B2);

// application reads from the specified serial port and reports the collected data
int main(int argc, char* argv[])
{
	// Here is the COM port number you need to figure out by yourself
	Serial* SP = new Serial("\\\\.\\COM4"); // no 'device' here, just COM4, ok?yes good   // adjust as needed
	if (SP->IsConnected())
		printf("We're connected");

	char incomingData[256] = "";			// don't forget to pre-allocate memory
	int dataLength = 3; // 255
	int readResult = 0;

	const static int N = 3;
    char buf[10];
    short d[3] = {0};

	while(SP->IsConnected())
	{
		if(recv_orientation_data4(&SP, (char*)buf, d))
        {

       // you can display it using arduino's function
            printf("read data yaw = %d, x = %d, z = %d \n", d[0], d[1], d[2]);
            printf("read data yaw = %f, x = %f, z = %f \n", d[0]*0.1, d[1]*0.001, d[2]*0.001);

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
