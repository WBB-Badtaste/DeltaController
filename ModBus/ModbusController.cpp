#include "StdAfx.h"
#include "ModbusController.h"


// unsigned short CModbusController::m_regDatas[SYSTEM_REG_NUM];	//寄存器数据
// 
// 
// CRITICAL_SECTION CModbusController::m_cs_reg;		//寄存器控制临界区


//DWORD WINAPI CModbusController::ReadThread(LPVOID pParam)
 unsigned WINAPI CModbusController::ReadThread(void *pParam)
{
	CModbusController *pModbusObj = (CModbusController*)pParam;

	unsigned char commReadBuffer[COMM_READ_BUFFER_SIZE];	//接收缓冲区
	unsigned char commSendBuffer[COMM_SEND_BUFFER_SIZE];	//接收缓冲区
	unsigned readNumber(0);						//实际接收长度

	unsigned regBeginIndex(0);		//待操作的寄存器首ID
	unsigned regNum(0), coilNum;	//待操作的寄存器，线圈个数
	unsigned commDataLen(0);		//串口数据块长度


	while (WaitForSingleObject(pModbusObj->m_hReadEvent_threading, 0) == WAIT_OBJECT_0)//读取循环事件
	{
		ZeroMemory(commReadBuffer, COMM_READ_BUFFER_SIZE);//清空缓冲区

		if (!pModbusObj->m_comm.Read(commReadBuffer, COMM_READ_BUFFER_SIZE, readNumber) || readNumber < 1)
			continue;

#ifdef _DEBUG
		pModbusObj->PrintStr(commReadBuffer, readNumber, true);
#endif

		if (!crc_check(commReadBuffer, readNumber))//CRC校验
		{
			continue;
		}

		if (commReadBuffer[0] != pModbusObj->m_modbusID)//判断是否发给本机
			continue;

		char func_code = commReadBuffer[1];//地址代码

		switch (func_code)
		{
		case 0x10://写寄存器

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = (short)commReadBuffer[6];

			pModbusObj->CtrlModbusReg(regBeginIndex, commDataLen, &commReadBuffer[7], false);

			//应答
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//清空缓冲区

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = commReadBuffer[2];
			commSendBuffer[3] = commReadBuffer[3];
			commSendBuffer[4] = commReadBuffer[4];
			commSendBuffer[5] = commReadBuffer[5];

			crc_16((unsigned char*)commSendBuffer, (unsigned char*)&commSendBuffer[6], 6);
			
			pModbusObj->Send(commSendBuffer, 8, FALSE);//响应报文

#ifdef _DEBUG
			pModbusObj->PrintStr(commSendBuffer, 8, false);
#endif
			break;

		case 0x03://读寄存器

			
			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			regNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = regNum * 2;

			//应答
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//清空缓冲区

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = (char)commDataLen;

			pModbusObj->CtrlModbusReg(regBeginIndex, commDataLen, &commSendBuffer[3], true);

			crc_16((unsigned char*)commSendBuffer, (unsigned char*)&commSendBuffer[commDataLen + 3], commDataLen + 3);

			pModbusObj->Send(commSendBuffer, commDataLen + 5, FALSE);//响应报文

#ifdef _DEBUG
			pModbusObj->PrintStr(commSendBuffer, commDataLen + 5, false);
#endif
			break;

		case 0x01://读线圈

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			coilNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = (coilNum - 1) / 8 + 1;

			//应答
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//清空缓冲区

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = (char)commDataLen;

			pModbusObj->CtrlModbusCoil(regBeginIndex, coilNum, commDataLen, &commSendBuffer[3], true);

			crc_16((unsigned char*)commSendBuffer, (unsigned char*)&commSendBuffer[commDataLen + 3], commDataLen + 3);

			pModbusObj->Send(commSendBuffer, commDataLen + 5, FALSE);//响应报文

#ifdef _DEBUG
			pModbusObj->PrintStr(commSendBuffer, commDataLen + 5, false);
#endif
			break;

		case 0x0f://写线圈

			regBeginIndex = ((short)commReadBuffer[2] << 8) + (short)commReadBuffer[3];
			coilNum = ((short)commReadBuffer[4] << 8) + (short)commReadBuffer[5];
			commDataLen = (short)commReadBuffer[6];

			pModbusObj->CtrlModbusCoil(regBeginIndex, coilNum, commDataLen, &commReadBuffer[7], false);

			//应答
			ZeroMemory(commSendBuffer, COMM_SEND_BUFFER_SIZE);//清空缓冲区

			commSendBuffer[0] = commReadBuffer[0];
			commSendBuffer[1] = commReadBuffer[1];
			commSendBuffer[2] = commReadBuffer[2];
			commSendBuffer[3] = commReadBuffer[3];
			commSendBuffer[4] = commReadBuffer[4];
			commSendBuffer[5] = commReadBuffer[5];

			crc_16((unsigned char*)commSendBuffer, (unsigned char*)&commSendBuffer[6], 6);

			pModbusObj->Send(commSendBuffer, 8, FALSE);//响应报文

#ifdef _DEBUG
			pModbusObj->PrintStr(commSendBuffer, 8, false);
#endif
			break;

		default://不支持的功能码
			continue;
		}

	}

	return 0;

} 


unsigned WINAPI CModbusController::SendThread(void* pParam)
{

	CModbusController *pModbusObj = (CModbusController*)pParam;

	vector<MODBUS_SEND_BUFFER>::iterator iter;

	unsigned lenSent;

	while (WaitForSingleObject(pModbusObj->m_hSendEvent_threading, 0) == WAIT_OBJECT_0)//发送循环事件
	{
		WaitForSingleObject(pModbusObj->m_hSendEvent_sent, INFINITE);

		EnterCriticalSection(&pModbusObj->m_cs_sendBuffer);
		for (iter = pModbusObj->m_vec_commSendBuffer.begin(); iter != pModbusObj->m_vec_commSendBuffer.end(); )
		{
			Sleep(MODBUS_SEND_DELAY);

			if (!pModbusObj->m_comm.Send(iter->buffer, iter->bufLen, lenSent))
				continue;
#ifdef _DEBUG
			unsigned len = pModbusObj->m_vec_commSendBuffer.size();
#endif
			iter = pModbusObj->m_vec_commSendBuffer.erase(iter);
		}
		LeaveCriticalSection(&pModbusObj->m_cs_sendBuffer);
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
// 		ARG_LIST *pReadArgs = new ARG_LIST;//防止线程开始前就被系统销毁，所以申请堆空间
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
// 		ARG_LIST *pSendArgs = new ARG_LIST;//防止线程开始前就被系统销毁，所以申请堆空间
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
		//异常处理
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
	InitializeCriticalSection(&m_cs_sendBuffer);
	ZeroMemory(m_regDatas, SYSTEM_BUFFER_NUM * sizeof(short));
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
	InitializeCriticalSection(&m_cs_sendBuffer);
	ZeroMemory(m_regDatas, SYSTEM_BUFFER_NUM * sizeof(short));
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
	InitializeCriticalSection(&m_cs_sendBuffer);
	ZeroMemory(m_regDatas, SYSTEM_BUFFER_NUM * sizeof(short));
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
	InitializeCriticalSection(&m_cs_sendBuffer);
	ZeroMemory(m_regDatas, SYSTEM_BUFFER_NUM * sizeof(short));
 	Init();
}


CModbusController::~CModbusController(void)
{
	ResetEvent(m_hReadEvent_threading);
	WaitForSingleObject(m_hReadThread, INFINITE);
	ResetEvent(m_hSendEvent_threading);
	SetEvent(m_hSendEvent_sent);
	WaitForSingleObject(m_hSendThread, INFINITE);
	DeleteCriticalSection(&m_cs_sendBuffer); 
 	DeleteCriticalSection(&m_cs_reg); 
}


BOOL CModbusController::Send(const unsigned char *pSendBuffer, const unsigned &len, const BOOL &bSyncSingal = TRUE)
{
	if (bSyncSingal == TRUE)
		WaitForSingleObject(m_hSendEvent_sent, INFINITE);

	MODBUS_SEND_BUFFER sendBuf;
	for (unsigned i = 0; i < len; i++)
	{
		sendBuf.buffer[i] = pSendBuffer[i];
	}
	sendBuf.bufLen = len;

	EnterCriticalSection(&m_cs_sendBuffer);
	m_vec_commSendBuffer.push_back(sendBuf);
	LeaveCriticalSection(&m_cs_sendBuffer);

	SetEvent(m_hSendEvent_sent);

	return FALSE;
}


BOOL CModbusController::CtrlRegSync(const unsigned &index, const unsigned &num, unsigned short *pRecvRegData, const bool ctrlType)
{
	unsigned regIndex(index), regEndIndex(index + num), bufIndex(0);

	if (regEndIndex > SYSTEM_BUFFER_NUM)//超出寄存器数量
		return FALSE;

	EnterCriticalSection(&m_cs_reg);// 进入临界区，其它线程则无法进入  

	if (ctrlType) //读寄存器
		for (; regIndex < regEndIndex; regIndex++)
			pRecvRegData[bufIndex++] = m_regDatas[regIndex];
	else//写寄存器
		for (; regIndex < regEndIndex; regIndex++)
			m_regDatas[regIndex] = pRecvRegData[bufIndex++];

	LeaveCriticalSection(&m_cs_reg);  // 离开临界区，其它线程可以进入

	return TRUE;
}


BOOL CModbusController::ReadReg(const unsigned &index, const unsigned &num, unsigned short *pRegData)
{
	return CtrlRegSync(index, num, pRegData, true);
}


BOOL CModbusController::WriteReg(const unsigned &index, const unsigned &num,  unsigned short *pRegData)
{

	if (CtrlRegSync(index, num, pRegData, false))
		return FALSE;

	return TRUE;
}


BOOL CModbusController::CtrlModbusCoil(const unsigned &index, const unsigned &coilNum, const unsigned &CommDataLen, unsigned char *pCommData, const bool &ctrlType)
{
	unsigned regDataLen((coilNum - 1) / 16 + 1);
	unsigned short regData[COIL_NUM_OF_BUFFER];

	unsigned regIndex(COIL_INDEX_OF_BUFFER + index / 16);
	unsigned coilIndex(index % 16);

	ZeroMemory(regData, COIL_NUM_OF_BUFFER * sizeof(short));

	unsigned offset((16 - coilIndex));//偏移量
	

	if (ctrlType)//读线圈
	{
		if(!ReadReg(regIndex, regDataLen, regData))
			return FALSE;

		unsigned short mask(coilIndex == 0 ? 0x0000 : 0xFFFF >> offset);

		for (unsigned i = 0; i < regDataLen; ++i)
		{
			const unsigned short buffer((regData[i] >> coilIndex) + ((regData[i + 1] & mask) << offset));

			pCommData[i * 2] = buffer & 0x00ff;
			pCommData[i * 2 + 1] = (buffer & 0xff00) >> 8;
		}

		return TRUE;
	}
	else//写线圈
	{
		unsigned bufferLen((coilNum + coilIndex - 1) / 16 + 1); //移位后需要的缓冲区长度
		unsigned short mask(coilIndex == 0 ? 0x0000 : 0xFFFF << offset);

		for (unsigned i = 0; i < bufferLen; ++i)
		{
			if (i)
			{
				regData[i] = (pCommData[i] << coilIndex + ((pCommData[i - 1] & mask) >> offset));
			}
			else
			{
				regData[i] = (pCommData[i] << coilIndex);
			}
		}

		unsigned short mask_head(~(0xFFFF << coilIndex));
		unsigned short mask_tail(0xFFFF << ((coilNum + coilIndex - 1) % 16 + 1));

		EnterCriticalSection(&m_cs_reg);// 进入临界区，其它线程则无法进入  

		for (unsigned i = 0; i < bufferLen; ++i)
		{
			mask = 0x0000;

			if (i == 0) mask |= mask_head;

			if(i == bufferLen - 1) mask |= mask_tail;
			
			m_regDatas[regIndex + i] = (m_regDatas[regIndex + i] & mask) + regData[i];
		}

		LeaveCriticalSection(&m_cs_reg);  // 离开临界区，其它线程可以进入

		return TRUE;
	}
}


BOOL CModbusController::CtrlModbusReg(const unsigned &regIndex, const unsigned &CommDataLen, unsigned char *pCommData, const bool &ctrlType)
{
	unsigned regDataLen(CommDataLen / 2);
	unsigned short regData[REG_NUM_OF_BUFFER];

	ZeroMemory(regData, REG_NUM_OF_BUFFER * sizeof(short));

	if (ctrlType)//读寄存器
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
	else//写寄存器
	{
		for (unsigned i = 0; i < regDataLen; i++)
		{
			regData[i]  = pCommData[i * 2] << 8;
			regData[i] += pCommData[i * 2 + 1];
		}
		return WriteReg(regIndex, regDataLen, regData);
	}
}


BOOL CModbusController::TransStrToHexFromat(const unsigned char* const source, const unsigned &sourceLen, TCHAR* const target, unsigned &targetLen)
{
	const unsigned buffLen(sourceLen * 3);
	char char_H4, char_L4;

	char *pb = new char[buffLen];

	ZeroMemory(pb, buffLen * sizeof(char));

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
	strcpy_s(target, buffLen, pb);
	targetLen = buffLen;
#endif

	delete[] pb;

	target[buffLen - 1] = _T('\n');

	return TRUE;
}


BOOL CModbusController::PrintStr(const unsigned char* const source, const unsigned &sourceLen, const bool &ctrlType)
{
	const unsigned defaultLen(max(COMM_READ_BUFFER_SIZE, COMM_SEND_BUFFER_SIZE) * 3);
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

