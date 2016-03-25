#pragma once

 
#define DEFAULT_COMM_NAME _T("COM1")
#define DEFAULT_COMM_RATE CBR_19200

// 读取总超时 = ReadTotalTimeoutMultiplier * 字节数 + ReadTotalTimeoutConstant  
#define DEFAULT_COMM_RIT 1000		// 读间隔超时。 接收时，两字符间最大的时延。
#define DEFAULT_COMM_RTTM 200		// 读时间系数。 读取每字节的超时。
#define DEFAULT_COMM_RTTC 1000		// 读时间常量。 读串口数据的固定超时。

 // 写入总超时 = WriteTotalTimeoutMultiplier * 字节数 + WriteTotalTimeoutConstant 
#define DEFAULT_COMM_WTTM 500		// 写时间系数。 写每字节的超时。
#define DEFAULT_COMM_WTTC 2000		// 写时间常量。 写串口数据的固定超时。  

class CComController
{
private:
	HANDLE m_hComm;
	unsigned m_nNumberOfBytesToWrite;	//将要写入的数据长度
	unsigned m_nBytesSent;				//实际写入的数据长度  
	unsigned m_nNumberOfBytesToRead;   //将要读取的数据长度
	unsigned m_nBytesRead;				//实际读取的数据长度

public:
	CComController(void);
	CComController(LPCSTR name, DWORD baudRate);
	~CComController(void);
	BOOL Send(char* lpSendBuffer, const unsigned &nNumberOfBytesToSend, unsigned &nBytesSend);
	BOOL Read(char* lpReadBuffer, const unsigned &nNumberOfBytesToRead, unsigned &nBytesRead);
};


