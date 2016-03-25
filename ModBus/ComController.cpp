#include "StdAfx.h"
#include "ComController.h"



CComController::CComController(void):
							m_hComm(NULL),
							m_nNumberOfBytesToWrite(0),
							m_nBytesSent(0),			
							m_nNumberOfBytesToRead(0),
							m_nBytesRead(0)				
{
	m_hComm = CreateFile(DEFAULT_COMM_NAME,   // ��������(COMx)

		GENERIC_READ | GENERIC_WRITE,         // ��������Ϊ�ɶ�/д

		0,                                    // �����豸���뱻��ռ�Եķ���

		NULL,                                 // �ް�ȫ����

		OPEN_EXISTING,                        // �����豸����ʹ��OPEN_EXISTING����

		0,									  // ͬ��ʽ I/O, FILE_ATTRIBUTE_NORMAL|FILE_FLAG_OVERLAPPED, �ص���ʽ

		NULL);                                // ���ڴ����豸���Դ˲�������Ϊ0

	if (m_hComm == INVALID_HANDLE_VALUE)
	{

		//����ô��ڲ����ڻ�����������һ��Ӧ�ó���ʹ�ã�

		//���ʧ�ܣ��������˳�

		//AfxMessageBox("��COMʧ��!");

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
			_T("%s ������Ϣ (������=%d): %s"),
			_T("CreateDirectory"), dw_err, lpMsgBuf);
		LocalFree(lpMsgBuf);

		OutputDebugString(szBuf);

		return ;

	}

	//2.���ô��ڲ����������ʡ�����λ��У��λ��ֹͣλ����Ϣ

	DCB dcb;

	GetCommState(m_hComm, &dcb);    //��ȡ�ö˿ڵ�Ĭ�ϲ���

	//�޸Ĳ�����

	dcb.BaudRate = DEFAULT_COMM_RATE;

	//�������ò���

	SetCommState(m_hComm, &dcb);

	SetupComm(m_hComm, 1024, 1024); //���뻺����������������Ĵ�С����1024

	COMMTIMEOUTS TimeOuts;

	//�趨����ʱ
	TimeOuts.ReadIntervalTimeout = DEFAULT_COMM_RIT;			//�������ʱ
	TimeOuts.ReadTotalTimeoutMultiplier = DEFAULT_COMM_RTTM;	//��ʱ��ϵ��
	TimeOuts.ReadTotalTimeoutConstant = DEFAULT_COMM_RTTC;		//��ʱ�䳣��

	//�������ʱ�����ó�MAXDWORD���Ҷ�ʱ��ϵ���Ͷ�ʱ�䳣����Ϊ0��
	//��ô�ڶ�һ�����뻺���������ݺ���������������أ�
	//�������Ƿ������Ҫ����ַ���

	//�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = DEFAULT_COMM_WTTM;	//дʱ��ϵ��
	TimeOuts.WriteTotalTimeoutConstant = DEFAULT_COMM_WTTC;		//дʱ�䳣��

	SetCommTimeouts(m_hComm,&TimeOuts);			//���ó�ʱ
}

CComController::CComController(LPCSTR name, DWORD baudRate)
{
	m_hComm = CreateFile(name,				  // ��������(COMx)

		GENERIC_READ | GENERIC_WRITE,         // ��������Ϊ�ɶ�/д

		0,                                    // �����豸���뱻��ռ�Եķ���

		NULL,                                 // �ް�ȫ����

		OPEN_EXISTING,                        // �����豸����ʹ��OPEN_EXISTING����

		0,                // ͬ��ʽ I/O

		0);                                   // ���ڴ����豸���Դ˲�������Ϊ0

	if (m_hComm == INVALID_HANDLE_VALUE)
	{

		//����ô��ڲ����ڻ�����������һ��Ӧ�ó���ʹ�ã�

		//���ʧ�ܣ��������˳�

		return ;

	}

	//���ô��ڲ����������ʡ�����λ��У��λ��ֹͣλ����Ϣ

	DCB dcb;

	//��ȡ�ö˿ڵ�Ĭ�ϲ���
	GetCommState(m_hComm, &dcb);

	//�޸Ĳ�����
	dcb.BaudRate = baudRate;

	//�������ò���
	SetCommState(m_hComm, &dcb);

	SetupComm(m_hComm,1024,1024); //���뻺����������������Ĵ�С����1024

	COMMTIMEOUTS TimeOuts;

	//�趨����ʱ
	TimeOuts.ReadIntervalTimeout = 1000;		//�������ʱ
	TimeOuts.ReadTotalTimeoutMultiplier = 500;	//��ʱ��ϵ��
	TimeOuts.ReadTotalTimeoutConstant = 5000;	//��ʱ�䳣��

	//�������ʱ�����ó�MAXDWORD���Ҷ�ʱ��ϵ���Ͷ�ʱ�䳣����Ϊ0��
	//��ô�ڶ�һ�����뻺���������ݺ���������������أ�
	//�������Ƿ������Ҫ����ַ���

	//�趨д��ʱ
	TimeOuts.WriteTotalTimeoutMultiplier = 500;	//дʱ��ϵ��
	TimeOuts.WriteTotalTimeoutConstant = 2000;	//дʱ�䳣��

	SetCommTimeouts(m_hComm,&TimeOuts);			//���ó�ʱ
}

CComController::~CComController(void)
{
	  CloseHandle(m_hComm); 
}

BOOL CComController::Send(char* lpSendBuffer, const unsigned &nNumberOfBytesToSend, unsigned &nBytesSend)
{
	BOOL bRes(m_hComm != INVALID_HANDLE_VALUE);

	bRes = bRes ? PurgeComm(m_hComm, PURGE_RXABORT|PURGE_RXCLEAR) : bRes;//�ж����ж�����,������뻺����

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

	bRes = bRes ? PurgeComm(m_hComm, PURGE_RXABORT|PURGE_RXCLEAR) : bRes;//�ж����ж�����,������뻺����

	m_nBytesRead = m_nNumberOfBytesToRead = 0;

	if (nNumberOfBytesToRead != 0)
		m_nNumberOfBytesToRead = nNumberOfBytesToRead;
	else
		return FALSE;

	bRes = bRes ? ReadFile(m_hComm, lpReadBuffer, m_nNumberOfBytesToRead, (LPDWORD)(&m_nBytesRead), NULL) : bRes;

	nBytesRead = m_nBytesRead;

	return bRes;
}