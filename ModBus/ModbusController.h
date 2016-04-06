#pragma once

#include "ComController.h"
#include "CRC_Algorithm.h"
#include <vector>

//#include <process.h>

using std::vector;

#define DEFAULT_COMM_ID			1											//默认ID
#define COMM_READ_BUFFER_SIZE	100											//串口读取缓冲区大小
#define COMM_SEND_BUFFER_SIZE	100											//串口读取缓冲区大小
#define SYSTEM_BUFFER_NUM		400											//用于modbus的系统缓存个数
#define	COIL_INDEX_OF_BUFFER	300											//线圈在缓存中的起始地址
#define REG_NUM_OF_BUFFER		COIL_INDEX_OF_BUFFER						//寄存器个数
#define COIL_NUM_OF_BUFFER		(SYSTEM_BUFFER_NUM - REG_NUM_OF_BUFFER)		//线圈个数
#define MODBUS_SEND_DELAY		0											//发送延时，降低示教器的数据处理量


typedef struct modbus_send_buffer
{
	unsigned char buffer[COMM_SEND_BUFFER_SIZE];
	unsigned bufLen;
}MODBUS_SEND_BUFFER;


class CModbusController
{
public:
	
	CModbusController(void);

 	CModbusController(LPCSTR name, DWORD baudRate);
  	CModbusController(const unsigned modbusID);
 	CModbusController(LPCSTR name, DWORD baudRate, const unsigned modbusID);
	~CModbusController(void);

	BOOL ReadReg(const unsigned &index, const unsigned &num, unsigned short *pRegData);
	BOOL WriteReg(const unsigned &index, const unsigned &num, unsigned short *pRegData);

	BOOL Send(const unsigned char *pSendBuffer, const unsigned &len, const BOOL &bSyncSingal);


private:
	BOOL Init();

 	CComController m_comm;

	unsigned m_modbusID, m_uReadId, m_uSendId;
	HANDLE m_hReadThread, m_hReadEvent_threading, m_hReadEvent_read;
	HANDLE m_hSendThread, m_hSendEvent_threading, m_hSendEvent_sent;
	
	unsigned short m_regDatas[SYSTEM_BUFFER_NUM];	//寄存器数据
	
	CRITICAL_SECTION m_cs_reg;			//寄存器控制临界区

	BOOL CtrlRegSync(const unsigned &index, const unsigned &num, unsigned short *pRegData, const bool ctrlType);

	BOOL CtrlModbusCoil(const unsigned &index, const unsigned &coilNum, const unsigned &num, unsigned char *pCommData, const bool &ctrlType);

	BOOL CtrlModbusReg(const unsigned &index, const unsigned &num, unsigned char *pCommData, const bool &ctrlType);

	BOOL TransStrToHexFromat(const unsigned char* const source, const unsigned &sourceLen, TCHAR* const target, unsigned &targetLen);

	BOOL PrintStr(const unsigned char* const source, const unsigned &sourceLen, const bool &ctrlType);

	static unsigned WINAPI ReadThread(void *);
	static unsigned WINAPI SendThread(void *);

	//发送内容缓存区
	vector<MODBUS_SEND_BUFFER> m_vec_commSendBuffer;
	CRITICAL_SECTION m_cs_sendBuffer;
};
