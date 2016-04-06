#include "stdafx.h"

#include "CRC_Algorithm.h"

bool crc_check(unsigned char *pBuffer,unsigned len)
{
	unsigned char rcvbuf[2];
	unsigned int crc(crc_16(pBuffer, rcvbuf, len - 2)); 
	if (pBuffer[len - 2] != rcvbuf[0] || pBuffer[len - 1] != rcvbuf[1])
		return false;
	else
		return true;
}


// unsigned short crc_16(const char *Array, char *Rcvbuf,unsigned int Len)
// {
// 	unsigned int  IX, IY, CRC;
// 	CRC = 0xFFFF;//set all 1
// 
// 	if (Len <= 0)
// 		CRC = 0;
// 	else
// 	{
// 		Len--; 
// 		for (IX = 0; IX <= Len; IX++)
// 		{
// 			CRC = CRC ^ (unsigned int)(Array[IX]);
// 			for(IY = 0; IY <= 7; IY++)
// 			{
// 				if ((CRC & 1) != 0 )
// 				{
// 					CRC = (CRC >> 1) ^ 0xA001;
// 				}
// 	
// 					CRC = CRC >> 1;
// 			}
// 		}
// 	}
// 
// 	Rcvbuf[1] = (CRC & 0xff00) >> 8;//高位置
// 	Rcvbuf[0] = (CRC & 0x00ff);     //低位置
// 
// // 	CRC  = Rcvbuf[0];
// // 	CRC += Rcvbuf[1] << 8;
// 	
// 
// 	return CRC;
// }

unsigned short crc_16(unsigned char *Array, unsigned char *Rcvbuf,unsigned int Len)
{
	unsigned int  IX,IY,CRC;
	CRC=0xFFFF;//set all 1

	if (Len<=0)
		CRC = 0;
	else
	{
		Len--;
		for (IX=0;IX<=Len;IX++)
		{
			CRC=CRC^(unsigned int)(Array[IX]);
			for(IY=0;IY<=7;IY++)
			{
				if ((CRC&1)!=0 )
					CRC=(CRC>>1)^0xA001;
				else
					CRC=CRC>>1;    
			}
		}
	}

	Rcvbuf[1] = (CRC & 0xff00)>>8;//高位置
	Rcvbuf[0] = (CRC & 0x00ff);   //低位置

	CRC= Rcvbuf[1]<<8;
	CRC+= Rcvbuf[0];

	return CRC;
}