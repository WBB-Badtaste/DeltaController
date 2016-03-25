#pragma once

 
#define DEFAULT_COMM_NAME _T("COM1")
#define DEFAULT_COMM_RATE CBR_19200

// ��ȡ�ܳ�ʱ = ReadTotalTimeoutMultiplier * �ֽ��� + ReadTotalTimeoutConstant  
#define DEFAULT_COMM_RIT 1000		// �������ʱ�� ����ʱ�����ַ�������ʱ�ӡ�
#define DEFAULT_COMM_RTTM 200		// ��ʱ��ϵ���� ��ȡÿ�ֽڵĳ�ʱ��
#define DEFAULT_COMM_RTTC 1000		// ��ʱ�䳣���� ���������ݵĹ̶���ʱ��

 // д���ܳ�ʱ = WriteTotalTimeoutMultiplier * �ֽ��� + WriteTotalTimeoutConstant 
#define DEFAULT_COMM_WTTM 500		// дʱ��ϵ���� дÿ�ֽڵĳ�ʱ��
#define DEFAULT_COMM_WTTC 2000		// дʱ�䳣���� д�������ݵĹ̶���ʱ��  

class CComController
{
private:
	HANDLE m_hComm;
	unsigned m_nNumberOfBytesToWrite;	//��Ҫд������ݳ���
	unsigned m_nBytesSent;				//ʵ��д������ݳ���  
	unsigned m_nNumberOfBytesToRead;   //��Ҫ��ȡ�����ݳ���
	unsigned m_nBytesRead;				//ʵ�ʶ�ȡ�����ݳ���

public:
	CComController(void);
	CComController(LPCSTR name, DWORD baudRate);
	~CComController(void);
	BOOL Send(char* lpSendBuffer, const unsigned &nNumberOfBytesToSend, unsigned &nBytesSend);
	BOOL Read(char* lpReadBuffer, const unsigned &nNumberOfBytesToRead, unsigned &nBytesRead);
};


