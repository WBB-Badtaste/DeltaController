#pragma once

//crc校验
//pBuffer[in]	-要校验的字符串
//len[in]		-字符串长度
bool crc_check(unsigned char *pBuffer, unsigned len);

//生成CRC校验码
//Array			-源字符串
//Rcvbuf		-CRC码字符串
//Len			-源字符串长度
//返回CRC码
unsigned short crc_16(unsigned char *Array, unsigned char *Rcvbuf,unsigned int Len);