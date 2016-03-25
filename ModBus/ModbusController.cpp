#include "StdAfx.h"
#include "ModbusController.h"


short CModbusController::m_regDatas[SYSTEM_REG_NUM];	//�Ĵ�������


CRITICAL_SECTION CModbusController::m_cs_reg;		//�Ĵ��������ٽ���


//DWORD WINAPI CModbusController::ReadThread(LPVOID pParam)
 unsigned WINAPI CModbusController::ReadThread(void *pParam)
{
// 	ARG_LIST *pReadArgs = (ARG_LIST *)pArgList;
// 	HANDLE *pReadEvent_threading = pReadArgs->pEvent_running;
// 	HANDLE *pReadEvent_read = pReadArgs->pEvent_work;
// 	CComController *pComm = pReadArgs->pComm;
// 	unsigned *pModubsId = pReadArgs->pModbusID;
// 	int *pRegDatas = pReadArgs->pRegDatas;
// 	CModbusController *pModbusObj = pReadArgs->pObj;
// 
// 	delete pReadArgs;//�������ٴ���ṹ��

	CModbusController *pModbusObj = (CModbusController*)pParam;

	char commReadBuffer[COMM_READ_BUFFER_SIZE];	//���ջ�����
	char commSendBuffer[COMM_SEND_BUFFER_SIZE];	//���ջ�����
	unsigned readNumber(0);						//ʵ�ʽ��ճ���

	unsigned regBeginIndex(0);		//�������ļĴ�����ID
	unsigned regNum(0);				//�������ļĴ�������
	unsigned commDataLen(0);		//�������ݿ鳤��
//	unsigned bufferLen(0);			//�������ļĴ�������
// 	char *pData(nullptr);			//�������ݸ���ָ��
// 	int *plVal(nullptr);			//λ���㸨��ָ��
//	int index(0);					//�������

	while (WaitForSingleObject(pModbusObj->m_hReadEvent_threading, 0) == WAIT_OBJECT_0)//��ȡѭ���¼�
	{
		ZeroMemory(commReadBuffer, COMM_READ_BUFFER_SIZE);//��ջ�����

		if (!pModbusObj->m_comm.Read(commReadBuffer, COMM_READ_BUFFER_SIZE, readNumber) || readNumber < 1)
			continue;

#ifdef _DEBUG
		PrintStr(commReadBuffer, readNumber, true);
#endif

		//SetEvent(pModbusObj->m_hReadEvent_read);//��ͬ����ȡʹ��	

		if (!crc_check(commReadBuffer, readNumber))//CRCУ��
		{
			continue;
		}


		if (commReadBuffer[0] != pModbusObj->m_modbusID)//�ж��Ƿ񷢸�����
			continue;

		char func_code = commReadBuffer[1];//��ַ����

		switch (func_code)
		{
		case 0x10://д�Ĵ���

// 			regIndex = ((commReadBuffer[2] & 0xf0) * 16 + commReadBuffer[2] & 0x0f) * 10 + ((commReadBuffer[3] & 0xf0) * 16 + commReadBuffer[3] & 0x0f);
// 			commDataLen = (commReadBuffer[6] & 0xf0) * 16 + commReadBuffer[6] & 0x0f;

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = (short)commReadBuffer[6];

			pModbusObj->CtrlModbusReg(regBeginIndex, commDataLen, &commReadBuffer[7], false);

			//Ӧ��
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//��ջ�����

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = commReadBuffer[2];
			commSendBuffer[3] = commReadBuffer[3];

			crc_16(commSendBuffer, &commSendBuffer[4], 4);
			
			pModbusObj->Send(commSendBuffer, 6, FALSE);//��Ӧ����

#ifdef _DEBUG
			PrintStr(commSendBuffer, 6, false);
#endif
			break;

		case 0x03://���Ĵ���

// 			regIndex = (commReadBuffer[2] & 0xf0 * 16 + commReadBuffer[2] & 0x0f) * 10 + (commReadBuffer[3] & 0xf0 * 16 + commReadBuffer[3] & 0x0f);
// 			commDataLen = ((commReadBuffer[4] & 0xf0 * 16 + commReadBuffer[4] & 0x0f) * 10 + (commReadBuffer[5] & 0xf0 * 16 + commReadBuffer[5] & 0x0f)) * 2;
			
			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = regNum * 2;

			//Ӧ��
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//��ջ�����

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = (char)commDataLen;

			pModbusObj->CtrlModbusReg(regBeginIndex, commDataLen, &commSendBuffer[3], true);

			crc_16(commSendBuffer, &commSendBuffer[commDataLen + 3], commDataLen + 3);

			pModbusObj->Send(commSendBuffer, commDataLen + 5, FALSE);//��Ӧ����

#ifdef _DEBUG
			PrintStr(commSendBuffer, commDataLen + 5, false);
#endif
			break;

		case 0x01://����Ȧ

// 			bufferIndex = (commReadBuffer[2] & 0xf0 * 16 + commReadBuffer[2] & 0x0f) * 10 + (commReadBuffer[3] & 0xf0 * 16 + commReadBuffer[3] & 0x0f);
// 			bufferEndIndex = bufferIndex + (commReadBuffer[4] & 0xf0 * 16 + commReadBuffer[4] & 0x0f) * 10 + (commReadBuffer[5] & 0xf0 * 16 + commReadBuffer[5] & 0x0f);

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = regNum;

			//Ӧ��
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//��ջ�����

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = (char)commDataLen;

			pModbusObj->CtrlModbusCoil(regBeginIndex, commDataLen, &commSendBuffer[3], true);

			crc_16(commSendBuffer, &commSendBuffer[commDataLen + 3], commDataLen + 3);

			pModbusObj->Send(commSendBuffer, commDataLen + 5, FALSE);//��Ӧ����

#ifdef _DEBUG
			PrintStr(commSendBuffer, commDataLen + 5, false);
#endif
			break;

		case 0x0f://д��Ȧ

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = (short)commReadBuffer[6];

			pModbusObj->CtrlModbusCoil(regBeginIndex, commDataLen, &commReadBuffer[7], false);

			//Ӧ��
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//��ջ�����

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = commReadBuffer[2];
			commSendBuffer[3] = commReadBuffer[3];

			crc_16(commSendBuffer, &commSendBuffer[4], 5);

			pModbusObj->Send(commSendBuffer, 6, FALSE);//��Ӧ����

#ifdef _DEBUG
			PrintStr(commSendBuffer, 6, false);
#endif
			break;

		default://��֧�ֵĹ�����
			continue;
		}

	}

	return 0;

} 


//DWORD WINAPI CModbusController::SendThread(LPVOID pParam)
unsigned WINAPI CModbusController::SendThread(void* pParam)
{
// 	ARG_LIST *pSendArgs = (ARG_LIST *)pArgList;
// 	HANDLE *pSendEvent_threading = pSendArgs->pEvent_running;
// 	HANDLE *pSendEvent_send = pSendArgs->pEvent_work;
// 	CComController *pComm = pSendArgs->pComm;
// 	unsigned *pModubsId = pSendArgs->pModbusID;
// 	int *pRegDatas = pSendArgs->pRegDatas;
// 	CModbusController *pModbusObj = pSendArgs->pObj;

//	delete pSendArgs;//�������ٴ���ṹ��

	CModbusController *pModbusObj = (CModbusController*)pParam;

	vector<MODBUS_SEND_BUFFER>::iterator iter;

	unsigned lenSent;

	while (WaitForSingleObject(pModbusObj->m_hSendEvent_threading, 0) == WAIT_OBJECT_0)//����ѭ���¼�
	{
		WaitForSingleObject(pModbusObj->m_hSendEvent_sent, INFINITE);

		for (iter = pModbusObj->m_vec_commSendBuffer.begin(); iter != pModbusObj->m_vec_commSendBuffer.end(); iter = pModbusObj->m_vec_commSendBuffer.begin())
		{
			Sleep(MODBUS_SEND_DELAY);

			if (!pModbusObj->m_comm.Send(iter->buffer, iter->bufLen, lenSent))
				continue;

			pModbusObj->m_vec_commSendBuffer.erase(iter);
		}

		ResetEvent(pModbusObj->m_hSendEvent_sent);
	}

	return 0;
}


BOOL CModbusController::Init()
{
	DWORD dErrorCode(0);

	m_hReadEvent_threading = CreateEvent(NULL, TRUE, TRUE, NULL);
	if (!m_hReadEvent_threading)
		dErrorCode = GetLastError();

	m_hReadEvent_read = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (!m_hReadEvent_read)
		dErrorCode = GetLastError();

	m_hSendEvent_threading = CreateEvent(NULL, TRUE, TRUE, NULL);
	if (!m_hSendEvent_threading)
		dErrorCode = GetLastError();

	m_hSendEvent_sent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (!m_hSendEvent_sent)
		dErrorCode = GetLastError();
	
	if (dErrorCode == 0)
	{
// 		ARG_LIST *pReadArgs = new ARG_LIST;//��ֹ�߳̿�ʼǰ�ͱ�ϵͳ���٣���������ѿռ�
// 		pReadArgs->pEvent_running = &m_hReadEvent_threading;
// 		pReadArgs->pEvent_work = &m_hReadEvent_read;
// 		pReadArgs->pComm = &m_comm;
// 		pReadArgs->pModbusID = &m_modbusID;
// 		pReadArgs->pRegDatas = m_regDatas;
// 		pReadArgs->pObj = this;

		m_hReadThread = 0;
		//m_hReadThread = ::CreateThread (NULL, 0, ReadThread, this, 0, &m_uReadId );
		m_hReadThread = (HANDLE)_beginthreadex(NULL, 0, ReadThread, this, 0, &m_uReadId );

		if (!m_hReadThread)
			dErrorCode = GetLastError();
	}

	if (dErrorCode == 0)
	{
// 		ARG_LIST *pSendArgs = new ARG_LIST;//��ֹ�߳̿�ʼǰ�ͱ�ϵͳ���٣���������ѿռ�
// 		pSendArgs->pEvent_running = &m_hSendEvent_threading;
// 		pSendArgs->pEvent_work = &m_hSendEvent_sent;
// 		pSendArgs->pComm = &m_comm;
// 		pSendArgs->pModbusID = &m_modbusID;
// 		pSendArgs->pRegDatas = m_regDatas;
// 		pSendArgs->pObj = this;

		m_hSendThread = 0;
		//m_hSendThread = ::CreateThread (NULL, 0, SendThread, this, 0, &m_uSendId );
		m_hSendThread = (HANDLE)_beginthreadex(NULL, 0, SendThread, this, 0, &m_uSendId );

		if (!m_hSendThread)
			dErrorCode = GetLastError();
	}

	if (dErrorCode)
	{
		//�쳣����
		return FALSE;
	}
	else
		return TRUE;
}


CModbusController::CModbusController(void):
	m_comm(),
	m_modbusID(DEFAULT_COMM_ID),
	m_hReadThread(0),
	m_hReadEvent_threading(0),
	m_hReadEvent_read(0),
	m_hSendThread(0),
	m_hSendEvent_threading(0),
	m_hSendEvent_sent(0),
	m_uReadId(0),
	m_uSendId(0)
{
	InitializeCriticalSection(&m_cs_reg);
	ZeroMemory(m_regDatas, SYSTEM_REG_NUM * sizeof(short));
	Init();
}


CModbusController::CModbusController(LPCSTR name, DWORD baudRate):
	m_comm(name, baudRate),
	m_modbusID (DEFAULT_COMM_ID),
	m_hReadThread(0),
	m_hReadEvent_threading(0),
	m_hReadEvent_read(0),
	m_hSendThread(0),
	m_hSendEvent_threading(0),
	m_hSendEvent_sent(0),
	m_uReadId(0),
	m_uSendId(0)
{
	InitializeCriticalSection(&m_cs_reg);
	ZeroMemory(m_regDatas, SYSTEM_REG_NUM * sizeof(short));
 	Init();
}


CModbusController::CModbusController(const unsigned modbusID):
	m_comm(),
	m_modbusID(modbusID),
	m_hReadThread(0),
	m_hReadEvent_threading(0),
	m_hReadEvent_read(0),
	m_hSendThread(0),
	m_hSendEvent_threading(0),
	m_hSendEvent_sent(0),
	m_uReadId(0),
	m_uSendId(0)
{
	InitializeCriticalSection(&m_cs_reg);
	ZeroMemory(m_regDatas, SYSTEM_REG_NUM * sizeof(short));
	Init();
}
 

CModbusController::CModbusController(LPCSTR name, DWORD baudRate, const unsigned modbusID):
	m_comm(name, baudRate),
	m_modbusID (modbusID),
	m_hReadThread(0),
	m_hReadEvent_threading(0),
	m_hReadEvent_read(0),
	m_hSendThread(0),
	m_hSendEvent_threading(0),
	m_hSendEvent_sent(0),
	m_uReadId(0),
	m_uSendId(0)
{
	InitializeCriticalSection(&m_cs_reg);
	ZeroMemory(m_regDatas, SYSTEM_REG_NUM * sizeof(short));
 	Init();
}


CModbusController::~CModbusController(void)
{
	ResetEvent(m_hReadEvent_threading);
	WaitForSingleObject(m_hReadThread, INFINITE);
	ResetEvent(m_hSendEvent_threading);
	SetEvent(m_hSendEvent_sent);
	WaitForSingleObject(m_hSendThread, INFINITE);
 	DeleteCriticalSection(&m_cs_reg); 
}


BOOL CModbusController::Send(const char *pSendBuffer, const unsigned &len, const BOOL &bSyncSingal = TRUE)
{
	if (bSyncSingal == TRUE)
		WaitForSingleObject(m_hSendEvent_sent, INFINITE);

	MODBUS_SEND_BUFFER sendBuf;
	for (unsigned i = 0; i < len; i++)
	{
		sendBuf.buffer[i] = pSendBuffer[i];
	}
	sendBuf.bufLen = len;

	m_vec_commSendBuffer.push_back(sendBuf);

	SetEvent(m_hSendEvent_sent);

	return FALSE;
}


BOOL CModbusController::CtrlRegSync(const unsigned &index, const unsigned &num, short *pRecvRegData, const bool &ctrlType)
{
	unsigned regIndex(index), regEndIndex(index + num), bufIndex(0);

	if (regEndIndex > SYSTEM_REG_NUM)//�����Ĵ�������
		return FALSE;

	EnterCriticalSection(&m_cs_reg);// �����ٽ����������߳����޷�����  

	if (ctrlType) //���Ĵ���
		for (; regIndex < regEndIndex; regIndex++)
			pRecvRegData[bufIndex++] = m_regDatas[regIndex];
	else//д�Ĵ���
		for (; regIndex < regEndIndex; regIndex++)
			m_regDatas[regIndex] = pRecvRegData[bufIndex++];

	LeaveCriticalSection(&m_cs_reg);  // �뿪�ٽ����������߳̿��Խ���

	return TRUE;
}


BOOL CModbusController::ReadReg(const unsigned &index, const unsigned &num, short *pRegData)
{
	return CtrlRegSync(index, num, pRegData, true);
}


BOOL CModbusController::WriteReg(const unsigned &index, const unsigned &num, short *pRegData)
{
	if (CtrlRegSync(index, num, pRegData, false))
		return FALSE;

	return TRUE;
}


BOOL CModbusController::CtrlModbusCoil(const unsigned &index, const unsigned &CommDataLen, char *pCommData, const bool &ctrlType)
{
	unsigned regDataLen(CommDataLen);
	short regData[SYSTEM_REG_NUM];

	ZeroMemory(regData, SYSTEM_REG_NUM * sizeof(short));

	if (ctrlType)//����Ȧ
	{
		if(!ReadReg(index, regDataLen, regData))
			return FALSE;

		for (unsigned i = 0; i < regDataLen; i++)
			pCommData[i] = regData[i] & 0x00FF;

		return TRUE;
	}
	else//д��Ȧ
	{
		for (unsigned i = 0; i < regDataLen; i++)
			regData[i] = pCommData[i];

		return WriteReg(index, regDataLen, regData);
	}
}


BOOL CModbusController::CtrlModbusReg(const unsigned &regIndex, const unsigned &CommDataLen, char *pCommData, const bool &ctrlType)
{
	unsigned regDataLen(CommDataLen / 2);
	short regData[SYSTEM_REG_NUM];

	ZeroMemory(regData, SYSTEM_REG_NUM * sizeof(short));

	if (ctrlType)//���Ĵ���
	{
		if(!ReadReg(regIndex, regDataLen, regData))
			return FALSE;

		for (unsigned i = 0; i < regDataLen; i++)
		{
			pCommData[i * 2] = (regData[i] & 0xFF00) >> 8;
			pCommData[i * 2 + 1] = regData[i] & 0x00FF;
		}

		return TRUE;
	}
	else//д�Ĵ���
	{
		for (unsigned i = 0; i < regDataLen; i++)
		{
			regData[i]  = pCommData[i * 2] << 8;
			regData[i] += pCommData[i * 2 + 1];
		}
		return WriteReg(regIndex, regDataLen, regData);
	}
}


BOOL CModbusController::TransStrToHexFromat(const char* const source, const unsigned &sourceLen, TCHAR* const target, unsigned &targetLen)
{
	const unsigned buffLen(sourceLen * 3);
	char char_H4, char_L4;

	char *pb = new char[buffLen];

	for (unsigned i = 0; i < sourceLen; ++i)
	{
		char_H4 = (source[i] & 0xf0) >> 4;
		char_L4 = source[i] & 0x0f;

		if (char_H4 > 0x09)
			pb[i * 3] = 0x37 + char_H4;
		else
			pb[i * 3] = 0x30 + char_H4;

		if (char_L4 > 0x09)
			pb[i * 3 + 1] = 0x37 + char_L4;
		else
			pb[i * 3 + 1] = 0x30 + char_L4;

		pb[i * 3 + 2] = (' ');
	}

#ifdef UNICODE  
	targetLen = MultiByteToWideChar(CP_ACP, 0, pb, buffLen, target, buffLen);
	if (targetLen == 0)
		return FALSE;
#else
	strcpy(target, pb);
	targetLen = buffLen;
#endif

	delete[] pb;

	target[buffLen - 1] = _T('\n');

	return TRUE;
}


BOOL CModbusController::PrintStr(const char* const source, const unsigned &sourceLen, const bool &ctrlType)
{
	const unsigned defaultLen(max(COMM_READ_BUFFER_SIZE, COMM_SEND_BUFFER_SIZE) * 2);
	TCHAR commStrHex[defaultLen];
	ZeroMemory(commStrHex, sizeof(TCHAR) * defaultLen);
	unsigned commStrHexLen(0);

	TransStrToHexFromat(source, sourceLen, commStrHex, commStrHexLen);

	if (ctrlType)
		OutputDebugString(_T("Recv:"));
	else
		OutputDebugString(_T("Send:"));

	OutputDebugString(commStrHex);

	return TRUE;
}