#pragma once

bool crc_check(unsigned char *pBuffer, unsigned len);

//unsigned crc_clac(const char *pBuffer, unsigned len);

//unsigned short crc_16(const char *Array, char *Rcvbuf,unsigned int Len);
unsigned short crc_16(unsigned char *Array, unsigned char *Rcvbuf,unsigned int Len);