#pragma once

bool crc_check(const char *pBuffer, unsigned len);

//unsigned crc_clac(const char *pBuffer, unsigned len);

unsigned short crc_16(const char *Array, char *Rcvbuf,unsigned int Len);