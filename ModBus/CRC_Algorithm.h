#pragma once

//crcУ��
//pBuffer[in]	-ҪУ����ַ���
//len[in]		-�ַ�������
bool crc_check(unsigned char *pBuffer, unsigned len);

//����CRCУ����
//Array			-Դ�ַ���
//Rcvbuf		-CRC���ַ���
//Len			-Դ�ַ�������
//����CRC��
unsigned short crc_16(unsigned char *Array, unsigned char *Rcvbuf,unsigned int Len);