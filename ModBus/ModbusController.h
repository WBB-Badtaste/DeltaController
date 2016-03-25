#pragma once

#include "ComController.h"

#include "CRC_Algorithm.h"
#include <vector>

//#include <process.h>

using std::vector;

#define DEFAULT_COMM_ID			1	//默认ID
#define COMM_READ_BUFFER_SIZE	100	//串口读取缓冲区大小
#define COMM_SEND_BUFFER_SIZE	100	//串口读取缓冲区大小
#define SYSTEM_REG_NUM			40	//系统寄存器个数
#define MODBUS_SEND_DELAY		0	//发送延时，降低示教器的数据处理量


typedef struct modbus_send_buffer
{
	char buffer[COMM_SEND_BUFFER_SIZE];
	unsigned bufLen;
}MODBUS_SEND_BUFFER;


typedef enum regIndex
{
	//当前机器人坐标值
	ROBOT_POS_X_1 = 0,
	ROBOT_POS_X_2 = 1,
	ROBOT_POS_X_3 = 2,
	ROBOT_POS_X_4 = 3,

	ROBOT_POS_Y_1 = 4,
	ROBOT_POS_Y_2 = 5,
	ROBOT_POS_Y_3 = 6,
	ROBOT_POS_Y_4 = 7,

	ROBOT_POS_Z_1 = 8,
	ROBOT_POS_Z_2 = 9,
	ROBOT_POS_Z_3 = 10,
	ROBOT_POS_Z_4 = 11
}REG_INDEX;

class CModbusController
{
public:
	
	CModbusController(void);

 	CModbusController(LPCSTR name, DWORD baudRate);
  	CModbusController(const unsigned modbusID);
 	CModbusController(LPCSTR name, DWORD baudRate, const unsigned modbusID);
	~CModbusController(void);

	static BOOL ReadReg(const unsigned &index, const unsigned &num, short *pRegData);
	static BOOL WriteReg(const unsigned &index, const unsigned &num, short *pRegData);

	BOOL Send(const char *pSendBuffer, const unsigned &len, const BOOL &bSyncSingal);

	vector<MODBUS_SEND_BUFFER> m_vec_commSendBuffer;

private:
	BOOL Init();

 	CComController m_comm;

	unsigned m_modbusID, m_uReadId, m_uSendId;
	HANDLE m_hReadThread, m_hReadEvent_threading, m_hReadEvent_read;
	HANDLE m_hSendThread, m_hSendEvent_threading, m_hSendEvent_sent;
// 	DWORD m_uReadId;
// 	DWORD m_uSendId;

//	BOOL Recv(const BOOL &bSyncSingal);
	
	static short m_regDatas[SYSTEM_REG_NUM];	//寄存器数据
	
	static CRITICAL_SECTION m_cs_reg;			//寄存器控制临界区

	static BOOL CtrlRegSync(const unsigned &index, const unsigned &num, short *pRegData, const bool &ctrlType);

	static unsigned WINAPI ReadThread(void *);
	//static DWORD WINAPI ReadThread(LPVOID);

	static unsigned WINAPI SendThread(void *);
	//static DWORD WINAPI SendThread(LPVOID);

	static BOOL CtrlModbusCoil(const unsigned &index, const unsigned &num, char *pCommData, const bool &ctrlType);

	static BOOL CtrlModbusReg(const unsigned &index, const unsigned &num, char *pCommData, const bool &ctrlType);

	static BOOL TransStrToHexFromat(const char* const source, const unsigned &sourceLen, TCHAR* const target, unsigned &targetLen);

	static BOOL PrintStr(const char* const source, const unsigned &sourceLen, const bool &ctrlType);
};


//typedef struct ArgList
// {
// 	CComController *pComm;		//串口控制对象指针
// 	HANDLE *pEvent_running;		//收发线程生命标志事件指针
// 	HANDLE *pEvent_work;		//收发成功标志事件指针
//	unsigned *pModbusID;		//本机ID指针
//	int *pRegDatas;				//寄存器指针
//	CModbusController *pObj;	//对象指针
//}ARG_LIST;