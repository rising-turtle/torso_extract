#define CRC16_GEN_POL 0x8005
#define MKSHORT(a,b) ((unsigned short)(a) | ((unsigned short)(b) << 8))

unsigned short CreatCRC(unsigned char *CommData, unsigned int uLen)
{
	unsigned short uCrc16;
	unsigned char abData[2];

	uCrc16=0;
	abData[0]=0;

	while(uLen--)
	{
		abData[1]=abData[0];
		abData[0]=*CommData++;
		if(uCrc16&0x8000)
		{
			uCrc16=(uCrc16&0x7fff)<<1;
			uCrc16^=CRC16_GEN_POL;
		}

		else
			uCrc16<<=1;
		
		uCrc16^=MKSHORT(abData[0], abData[1]);
	}

	return (uCrc16);
}


