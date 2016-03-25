#include "StdAfx.h"
#include "ComController.h"



CComController::CComController(void):
							m_hComm(NULL),
							m_nNumberOfBytesToWrite(0),
							m_nBytesSent(0),			
							m_nNumberOfBytesToRead(0),
							m_nBytesRead(0)				
{
	m_hComm = CreateFile(DEFAULT_COMM_NAME,   // 串口名称(COMx)

		GENERIC_READ | GENERIC_WRITE,         // 串口属性为可读/写

		0,                                    // 串口设备必须被独占性的访问

		NULL,                                 // 无安全属性

		OPEN_EXISTING,                        // 串口设备必须使用OPEN_EXISTING参数

		0,									  // 同步式 I/O, FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED, 重叠方式

		NULL);                                // 对于串口设备而言此参数必须为0

	if (m_hComm == INVALID_HANDLE_VALUE)
	{

		//如果该串口不存在或者正被另外一个应用程序使用，

		//则打开失败，本程序退出

		//AfxMessageBox("打开COM失败!");

		DWORD dw_err = GetLastError();

		LPVOID lpMsgBuf;

		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
			NULL,
			dw_err,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &lpMsgBuf,
			0, NULL );

		TCHAR szBuf[128];

		wsprintf(szBuf,
			_T("%s 出错信息 (出错码=%d): %s"),
			_T("CreateDirectory"), dw_err, lpMsgBuf);
		LocalFree(lpMsgBuf);

		OutputDebugString(szBuf);

		return ;

	}

	//2.设置串口参数：波特率、数据位、校验位、停止位等信息

	DCB dcb;

	GetCommState(m_hComm, &dcb);    //获取该端口的默认参数

	//修改波特率

	dcb.BaudRate = DEFAULT_COMM_RATE;

	//重新设置参数

	SetCommState(m_hComm, &dcb);

	SetupComm(m_hComm, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024

	COMMTIMEOUTS TimeOuts;

	//设定读超时
	TimeOuts.ReadIntervalTimeout = DEFAULT_COMM_RIT;			//读间隔超时
	TimeOuts.ReadTotalTimeoutMultiplier = DEFAULT_COMM_RTTM;	//读时间系数
	TimeOuts.ReadTotalTimeoutConstant = DEFAULT_COMM_RTTC;		//读时间常量

	//读间隔超时被设置成MAXDWORD并且读时间系数和读时间常量都为0，
	//那么在读一次输入缓冲区的内容后读操作就立即返回，
	//而不管是否读入了要求的字符。

	//设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = DEFAULT_COMM_WTTM;	//写时间系数
	TimeOuts.WriteTotalTimeoutConstant = DEFAULT_COMM_WTTC;		//写时间常量

	SetCommTimeouts(m_hComm,&TimeOuts);			//设置超时
}

CComController::CComController(LPCSTR name, DWORD baudRate)
{
	m_hComm = CreateFile(name,				  // 串口名称(COMx)

		GENERIC_READ | GENERIC_WRITE,         // 串口属性为可读/写

		0,                                    // 串口设备必须被独占性的访问

		NULL,                                 // 无安全属性

		OPEN_EXISTING,                        // 串口设备必须使用OPEN_EXISTING参数

		0,                // 同步式 I/O

		0);                                   // 对于串口设备而言此参数必须为0

	if (m_hComm == INVALID_HANDLE_VALUE)
	{

		//如果该串口不存在或者正被另外一个应用程序使用，

		//则打开失败，本程序退出

		return ;

	}

	//设置串口参数：波特率、数据位、校验位、停止位等信息

	DCB dcb;

	//获取该端口的默认参数
	GetCommState(m_hComm, &dcb);

	//修改波特率
	dcb.BaudRate = baudRate;

	//重新设置参数
	SetCommState(m_hComm, &dcb);

	SetupComm(m_hComm,1024,1024); //输入缓冲区和输出缓冲区的大小都是1024

	COMMTIMEOUTS TimeOuts;

	//设定读超时
	TimeOuts.ReadIntervalTimeout = 1000;		//读间隔超时
	TimeOuts.ReadTotalTimeoutMultiplier = 500;	//读时间系数
	TimeOuts.ReadTotalTimeoutConstant = 5000;	//读时间常量

	//读间隔超时被设置成MAXDWORD并且读时间系数和读时间常量都为0，
	//那么在读一次输入缓冲区的内容后读操作就立即返回，
	//而不管是否读入了要求的字符。

	//设定写超时
	TimeOuts.WriteTotalTimeoutMultiplier = 500;	//写时间系数
	TimeOuts.WriteTotalTimeoutConstant = 2000;	//写时间常量

	SetCommTimeouts(m_hComm,&TimeOuts);			//设置超时
}

CComController::~CComController(void)
{
	  CloseHandle(m_hComm); 
}

BOOL CComController::Send(char* lpSendBuffer, const unsigned &nNumberOfBytesToSend, unsigned &nBytesSend)
{
	BOOL bRes(m_hComm != INVALID_HANDLE_VALUE);

	bRes = bRes ? PurgeComm(m_hComm, PURGE_RXABORT|PURGE_RXCLEAR) : bRes;//中断所有读操作,清除输入缓冲区

	m_nBytesSent = m_nNumberOfBytesToWrite = 0;

	if (nNumberOfBytesToSend != 0)
		m_nNumberOfBytesToWrite = nNumberOfBytesToSend;
	else
		return FALSE;

	bRes = bRes ? WriteFile(m_hComm, lpSendBuffer, m_nNumberOfBytesToWrite, (LPDWORD)(&m_nBytesSent), NULL) : bRes;

	bRes = bRes ? m_nNumberOfBytesToWrite == m_nBytesSent : bRes;

	nBytesSend = m_nBytesSent;

	return bRes;
}

BOOL CComController::Read(char* lpReadBuffer, const unsigned &nNumberOfBytesToRead, unsigned &nBytesRead)
{
	BOOL bRes(m_hComm != INVALID_HANDLE_VALUE);

	bRes = bRes ? PurgeComm(m_hComm, PURGE_RXABORT|PURGE_RXCLEAR) : bRes;//中断所有读操作,清除输入缓冲区

	m_nBytesRead = m_nNumberOfBytesToRead = 0;

	if (nNumberOfBytesToRead != 0)
		m_nNumberOfBytesToRead = nNumberOfBytesToRead;
	else
		return FALSE;

	bRes = bRes ? ReadFile(m_hComm, lpReadBuffer, m_nNumberOfBytesToRead, (LPDWORD)(&m_nBytesRead), NULL) : bRes;

	nBytesRead = m_nBytesRead;

	return bRes;
}