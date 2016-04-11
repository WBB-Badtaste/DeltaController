#pragma once


// #include "AxisControl.h"
// #include "RocksControl.h"
// #include "NyceExErrorHandle.h"
//MODBUS
#include "ModbusController.h"


//机器人运动状态
typedef enum motionState
{
	HOME		= 0,//机器人回零
	PTP			= 1,
	CIRC		= 2,
	DOOR		= 3,
	READY		= 4,
	CATCH		= 5,
	INIT		= 6,
	CTRL_BRAKE  = 7,
	CTRL_CARMERA = 8
}MOTION_STATE;


class CMotionStateMach
{
public:
	CMotionStateMach(void);
	CMotionStateMach(HWND);
	~CMotionStateMach(void);

	bool FinlishMatch(const double &x, const double &y, const double &angle, const bool &bSuccess);
	bool SwitchToCatchState();
	bool SwitchToInitState();
	bool SwitchToDoorState(const double &vel);
	bool SwitchToPtpState(const double &x,const double &y,const double &z,const double &vel);
	bool SwitchToHomeState();
	bool SwitchToCircState(const double &vel);
	bool SwitchToJogState(const double &dist, const int &dire);
	bool SwitchToCtrlBrakeState();
	bool SwitchToCtrlCarmeraState();
	

private:
	//匹配结果
	//ROCKS_COORD m_targetPos_camera;
	//double m_dAngle;

	//主窗口句柄
	const HWND m_hMainWnd;

	//工作线程
	HANDLE m_hStateThread, m_hEvST, m_hEvMove;
	HANDLE m_hReadPosThread, m_hEvRPT;
	MOTION_STATE m_status;
	static unsigned WINAPI StateThread(void *);
	static unsigned WINAPI AssistThread(void *);

	//工作函数
	const uint32_t Catch();
	const uint32_t Init();
	const uint32_t Door();
	const uint32_t Ptp();
	const uint32_t Home();
	const uint32_t Circ();
	const uint32_t CtrlBrake();
	const uint32_t CtrlCarmera();



	//ROCKS状态处理函数
	void StatusHandler(uint32_t &);
	void SendString(const CString &str = "");

	//匹配结果
	bool m_bWaitForMatchRes;
	HANDLE m_hEvFinlishMatch;
	double m_dTargetPos_x;
	double m_dTargetPos_y;
	double m_dTargetAngle;
	//初始化标志
	bool m_bInit;
	//运动速度
	double m_motionVel;
	//移动位置
	double m_nextPos_x;
	double m_nextPos_y;
	double m_nextPos_z;
	bool   m_bPtpRelative;

	//modbus
	CModbusController m_mc;

	//窗口显示信息
	CString m_str;
};

