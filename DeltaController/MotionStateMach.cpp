

#include "StdAfx.h"


#include "MotionStateMach.h"

#include "AxisControl.h"
#include "RocksControl.h"
#include "NyceExErrorHandle.h"

CMotionStateMach::CMotionStateMach(HWND hMainWnd)
	: m_hMainWnd(hMainWnd)
	, m_hStateThread(0)
	, m_hReadPosThread(0)
	, m_hEvST(0)
	, m_hEvRPT(0)
	, m_status(READY)
	, m_hEvMove(0)
	, m_hEvFinlishMatch(0)
	, m_mc()
	, m_str("")
	, m_bInit(false)
	, m_motionVel(0.0)
	, m_nextPos_x(0.0)
	, m_nextPos_y(0.0)
	, m_nextPos_z(0.0)
	, m_bPtpRelative(false)
	, m_dTargetPos_x(0.0)
	, m_dTargetPos_y(0.0)
	, m_dTargetAngle(0.0)
	, m_bWaitForMatchRes(false)
{
	m_hEvFinlishMatch = CreateEvent(NULL, FALSE, FALSE, NULL);

	//����״̬���߳�
	m_hEvST = CreateEvent(NULL, TRUE, TRUE, NULL);
	m_hEvMove = CreateEvent(NULL, TRUE, FALSE, NULL);
	m_hStateThread = (HANDLE)_beginthreadex(NULL, 0, StateThread, this, 0, NULL );

	//������λ���߳�
	m_hEvRPT = CreateEvent(NULL, TRUE, TRUE, NULL);
	m_hReadPosThread = (HANDLE)_beginthreadex(NULL, 0, AssistThread, this, 0, NULL );
}



CMotionStateMach::~CMotionStateMach(void)
{
	m_bInit = false;
	//�رն�λ���߳�
	ResetEvent(m_hEvRPT);
	WaitForSingleObject(m_hReadPosThread, 500);
	//�ر�״̬���߳�
	ResetEvent(m_hEvST);
	SetEvent(m_hEvMove);
	WaitForSingleObject(m_hStateThread, 500);

	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES, axId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES_ROTATION, rotationId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES_BELT, beltId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiDisconnect(noId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksTermMatrix();
}


unsigned WINAPI CMotionStateMach::AssistThread(void *pParam)
{
	CMotionStateMach *pMSM = (CMotionStateMach *)pParam;

	NYCE_STATUS myStatus(NYCE_OK);

	double dRobotPos[6];

	while(WaitForSingleObject(pMSM->m_hEvRPT, 0) == WAIT_OBJECT_0)
	{
		Sleep(ASSIST_DELAY);

		if (!pMSM->m_bInit)
			continue;
		
		//��ȡ������λ��
		myStatus = RocksReadPosDelta(dRobotPos);

		if(NyceError(myStatus))
		{
			pMSM->StatusHandler(myStatus);
			Sleep(1000);
			continue;
		}

		::SendMessage(pMSM->m_hMainWnd, WM_UPDATE_ROBOT_POS, NULL, (LPARAM)dRobotPos);

		//��ȡmodbusָ��
		float ptp_x(0.0), ptp_y(0.0), ptp_z(0.0), ptp_vel(0.0);
		if(pMSM->m_mc.GetPtpComand(ptp_x, ptp_y, ptp_z, ptp_vel))
			pMSM->SwitchToPtpState(ptp_x, ptp_y, ptp_z, ptp_vel);

		if (pMSM->m_mc.GetHomeComand())
			pMSM->SwitchToHomeState();

		float jog_dist(0.0), jog_dire(0.0);
		if (pMSM->m_mc.GetJogComand(jog_dist, jog_dire))
			pMSM->SwitchToJogState(jog_dist, jog_dire);
	}
	return 0;
}


unsigned WINAPI CMotionStateMach::StateThread(void *pParam)
{
	CMotionStateMach *pMSM = (CMotionStateMach *)pParam;

	NYCE_STATUS myStatus(NYCE_OK);

	while(WaitForSingleObject(pMSM->m_hEvMove, INFINITE) == WAIT_OBJECT_0)
	{
		if (WaitForSingleObject(pMSM->m_hEvST, 0) != WAIT_OBJECT_0)//�˳��̱߳�־
			break;

		if (!pMSM->m_bInit && pMSM->m_status != INIT)
		{
			pMSM->SendString("System should be initialed first.");
			continue;
		}

		switch(pMSM->m_status)
		{
		case HOME:
			myStatus = pMSM->Home();
			pMSM->m_status = READY;
			break;
		case PTP:
			myStatus = pMSM->Ptp();
			pMSM->m_status = READY;
			break;
		case CIRC:
			myStatus = pMSM->Circ();
			pMSM->m_status = READY;
			break;
		case DOOR:
			myStatus = pMSM->Door();
			pMSM->m_status = READY;
			break;
		case READY:
			myStatus = NYCE_OK;
			break;
		case CATCH:
			myStatus = pMSM->Catch();
			pMSM->m_status = READY;
			break;
		case INIT:
			myStatus = pMSM->Init();
			pMSM->m_status = READY;
			break;
		default:
			pMSM->m_status = READY;
			break;
		}
		pMSM->StatusHandler(myStatus);
		ResetEvent(pMSM->m_hEvMove);
	}
	return 0;
}


const uint32_t CMotionStateMach::Catch()
{

	NYCE_STATUS nyceStatus(NYCE_OK);

	//ֹͣ���ʹ�
	SAC_STATE state;
	SAC_SPG_STATE spgState;
	SacReadState(beltId[0], &state, &spgState);

	SAC_JOG_PARS jogPars;
	jogPars.velocity = BELT_VEL * BELT_BASE_RATE;
	jogPars.acceleration = jogPars.velocity * 10;
	jogPars.jerk = jogPars.velocity * 100;

	if (state != SAC_READY)
		SacStopJog(beltId[0], &jogPars);

	//���ò���
	TRAJ_PARS trajPars;
	trajPars.velocity = 800 * KIN_BASE_RATE;
	trajPars.acceleration = trajPars.velocity * 100;
	trajPars.splineTime = 0.005;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksHomeDelta(trajPars);

	DOOR_TRAJ_PARS doorPars1, doorPars2, doorPars3;

	doorPars1.startPos.type = KIN_COORD;
	doorPars1.startPos.position.x = -300;
	doorPars1.startPos.position.y = 0 ;
	doorPars1.startPos.position.z = -870;
	doorPars1.endPos.type = KIN_COORD;
	doorPars1.endPos.position.x = 50;
	doorPars1.endPos.position.y = -400;
	doorPars1.endPos.position.z = -873;
	doorPars1.radius = 8;
	doorPars1.riseHeight = max(doorPars1.startPos.position.z, doorPars1.endPos.position.z) + 25 * KIN_BASE_RATE - doorPars1.startPos.position.z;
	doorPars1.trajPars.velocity = BELT_VEL * 9 * KIN_BASE_RATE;
	doorPars1.trajPars.acceleration = doorPars1.trajPars.velocity * 10;
	doorPars1.trajPars.splineTime = 0.005;

	doorPars2.startPos.type = KIN_COORD;
	doorPars2.startPos.position.x = doorPars1.endPos.position.x;
	doorPars2.startPos.position.y = doorPars1.endPos.position.y;
	doorPars2.startPos.position.z = doorPars1.endPos.position.z;
	doorPars2.endPos.type = KIN_COORD;
	doorPars2.endPos.position.x = 0;
	doorPars2.endPos.position.y = 0;
	doorPars2.endPos.position.z = 0;
	doorPars2.radius = 8;
	doorPars2.riseHeight = 0;
	doorPars2.trajPars.velocity = doorPars1.trajPars.velocity;
	doorPars2.trajPars.acceleration = doorPars1.trajPars.acceleration;
	doorPars2.trajPars.splineTime = doorPars1.trajPars.splineTime;

	doorPars3.startPos.type = KIN_COORD;
	doorPars3.startPos.position.x = 0;
	doorPars3.startPos.position.y = 0;
	doorPars3.startPos.position.z = 0;
	doorPars3.endPos.type = KIN_COORD;
	doorPars3.endPos.position.x = doorPars1.startPos.position.x;
	doorPars3.endPos.position.y = doorPars1.startPos.position.y;
	doorPars3.endPos.position.z = doorPars1.startPos.position.z;
	doorPars3.radius = 8;
	doorPars3.riseHeight = 0;
	doorPars3.trajPars.velocity = doorPars1.trajPars.velocity;
	doorPars3.trajPars.acceleration = doorPars1.trajPars.velocity * 10;
	doorPars3.trajPars.splineTime = 0.005;

	TRAJ_PARS readyPars, catchPars;
	readyPars.velocity = 500 * KIN_BASE_RATE;
	readyPars.acceleration = readyPars.velocity * 100;
	readyPars.splineTime = 0.005;

	catchPars.velocity = BELT_VEL * 7 * KIN_BASE_RATE;
	catchPars.acceleration = catchPars.velocity * 100;
	catchPars.splineTime = 0.005;

	ROCKS_COORD ptpPos;
	ptpPos.type = KIN_COORD;
	ptpPos.position.x = doorPars1.startPos.position.x;
	ptpPos.position.y = doorPars1.startPos.position.y;
	ptpPos.position.z = doorPars1.startPos.position.z + 40 * KIN_BASE_RATE;
	ptpPos.cuEncoderValue = 0;

	ROCKS_COORD catchPos;
	catchPos.type = KIN_COORD;
	catchPos.position.x = 0;
	catchPos.position.y = 0;
	catchPos.position.z = 0;
	catchPos.cuEncoderValue = 0;

	NYCE_DIGITAL_IO_ID io1, io2;
	io1.slotId = NYCE_SLOT3;
	io1.digIONr = NYCE_DIGOUT2;

	io2.slotId = NYCE_SLOT3;
	io2.digIONr = NYCE_DIGOUT3;

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = 0  * KIN_BASE_RATE;
	pos.position.y = 0  * KIN_BASE_RATE;
	pos.position.z = 30 * KIN_BASE_RATE;

	TRAJ_PARS trajPars2;
	trajPars2.velocity = 50 * KIN_BASE_RATE;
	trajPars2.acceleration = trajPars2.velocity * 100;
	trajPars2.splineTime = 0.005;

	double beltEncoderVal(0.0);

	//�ƶ���ץȡ���Ϸ�
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(ptpPos, readyPars);

	ROCKS_COORD targetPos_kin,targetPos_carmera;
	targetPos_kin.type = KIN_COORD;
	targetPos_kin.position.x = 0;
	targetPos_kin.position.y = 0;
	targetPos_kin.position.z = 0;
	targetPos_carmera.type = CAMERA_COORD;
	targetPos_carmera.position.x = 0;
	targetPos_carmera.position.y = 0;
	targetPos_carmera.position.z = 0;

	//��ʱ
	ROCKS_COORD catchUpPos;
	catchUpPos.type = KIN_COORD;
	catchUpPos.position.x = 0;
	catchUpPos.position.y = 60;
	catchUpPos.position.z = 0;

	TRAJ_PARS catchUpPars;
	catchUpPars.velocity = 230;
	catchUpPars.acceleration = catchUpPars.velocity * 100;
	catchUpPars.splineTime = 0.01;

	for (int i = 0; i <= 5; ++i)
	{
		//�ƶ���Ŀ�����
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(doorPars1.startPos, readyPars);

		//��������
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io1);//pick

		//ͣ0.3s��Ŀ������
		if (NyceSuccess(nyceStatus))
			Sleep(300);

		//���͹켣�ߵ�Ŀ����õ�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars1);

		//�ر�������
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io1);//place

		//�򿪴�����
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io2);//place

		//ͣ1s��Ŀ�����
		if (NyceSuccess(nyceStatus))
			Sleep(3000);

		//�رմ�����
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io2);//place

		SAC_PTP_PARS ptpPos_belt;
		ptpPos_belt.positionReference = SAC_RELATIVE;
		ptpPos_belt.position = -550 * BELT_BASE_RATE;
		ptpPos_belt.velocity = BELT_VEL * BELT_BASE_RATE;
		ptpPos_belt.acceleration = ptpPos_belt.velocity * 10;
		ptpPos_belt.jerk = ptpPos_belt.velocity * 100;

		//����Ƥ�������˶�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacPointToPoint(beltId[0], &ptpPos_belt);

		//�ȴ�Ƥ���˶����
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacSynchronize( beltId[0], SAC_REQ_MOTION_STOPPED, SAC_INDEFINITE );

		//��ȡƤ��������ֵ
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacReadVariable(beltId[0], SAC_VAR_AXIS_POS, &targetPos_kin.cuEncoderValue);

		//ƥ���ķ�Ӧʱ�䣬���Բ�Ҫ
		Sleep(500);

		if (NyceSuccess(nyceStatus))
		{
			m_bWaitForMatchRes = true;
			::SendMessage(m_hMainWnd, WM_MATCH_MODEL, NULL, NULL);
			WaitForSingleObject(m_hEvFinlishMatch, INFINITE);
			m_bWaitForMatchRes = false;
			
			if (m_dTargetPos_x == 0 && m_dTargetPos_y ==0)
			{
				return CAMERA_ERR_MATCH_FAIL;
			}
			targetPos_carmera.position.x = m_dTargetPos_x;
			targetPos_carmera.position.y = m_dTargetPos_y;
			ConvertTwoCoordinate(targetPos_carmera, targetPos_kin);
			
		}

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksRotateAngle(-m_dTargetAngle);

		//����Ƥ�������˶�
		SAC_JOG_PARS jogPars_belt;
		jogPars_belt.velocity = BELT_VEL * BELT_BASE_RATE;
		jogPars_belt.acceleration = jogPars_belt.velocity * 10;
		jogPars_belt.jerk = jogPars_belt.velocity * 100;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStartJog(beltId[0], &jogPars_belt);

		if (NyceSuccess(nyceStatus))
			Sleep(4500 * 200 / BELT_VEL);

		//��ʼ����ץȡ��
		//--JoMar--20160202

		//һ�μ���ץȡλ��
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCalcCatchPos(doorPars2.trajPars, doorPars2.startPos, targetPos_kin , 25 * KIN_BASE_RATE, doorPars2.endPos);

		//�����µ�ĩλ�����������߶�
		doorPars2.riseHeight = max(doorPars2.startPos.position.z, doorPars2.endPos.position.z) + 25 * KIN_BASE_RATE - doorPars2.startPos.position.z;

		//���͹켣��ץȡ���Ϸ�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars2);

		//���μ���ץȡλ��
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCalcCatchPos(catchPars, doorPars2.endPos, targetPos_kin, -20 * KIN_BASE_RATE, catchPos);

		//��������
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io1);//pick

		//PTP��ʵ��ץȡλ��
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(catchPos, catchPars);

// 		//��ʱ
 //		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(catchUpPos, catchUpPars, TRUE);

// 		//ͣ0.2s
// 		if (NyceSuccess(nyceStatus))
// 			Sleep(700);

		doorPars3.startPos.position.x = catchPos.position.x;
		doorPars3.startPos.position.y = catchPos.position.y;
		doorPars3.startPos.position.z = catchPos.position.z;
		doorPars3.riseHeight = max(doorPars3.startPos.position.z, doorPars3.endPos.position.z) + 25 * KIN_BASE_RATE - doorPars3.startPos.position.z;

		//�ƶ������õ�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars3);

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksRotateAngle(m_dTargetAngle);

		//�ر�������
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io1);	//place

		//�򿪴���
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io2);	//place

		//ͣ2s
		if (NyceSuccess(nyceStatus))
			Sleep(500);

		//�رմ�����
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io2);	//place

		//�ƶ���Ŀ���Ϸ�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars2, TRUE);

		//ֹͣ���ʹ�
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStopJog(beltId[0], &jogPars_belt);

		Sleep(1000);
	}

	return nyceStatus;
}

const uint32_t CMotionStateMach::Init()
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceInit(NYCE_ETH);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiConnect(noName[0], &noId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES_ROTATION, rotationId, rotationName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES_BELT, beltId, beltaName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInitDelta(NUM_AXES, axId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInitMatrix();

	if (NyceSuccess(nyceStatus))
		m_bInit = true;
	else
		m_bInit = false;

	return nyceStatus;
}

const uint32_t CMotionStateMach::Door()
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	DOOR_TRAJ_PARS doorPars1, doorPars2;
	doorPars1.startPos.type = KIN_COORD;
	doorPars1.startPos.position.x = -DOOR_PAR_X;
	doorPars1.startPos.position.y = -DOOR_PAR_Y;
	doorPars1.startPos.position.z = DOOR_HIGHT1;

	doorPars1.endPos.type = KIN_COORD;
	doorPars1.endPos.position.x = DOOR_PAR_X;
	doorPars1.endPos.position.y = DOOR_PAR_Y;
	doorPars1.endPos.position.z = DOOR_HIGHT2;

	doorPars1.riseHeight = 25.0;
	doorPars1.radius = 8.0;
	doorPars1.trajPars.velocity = m_motionVel;
	doorPars1.trajPars.acceleration = doorPars1.trajPars.velocity * 100;
	doorPars1.trajPars.splineTime = 0.001;

	doorPars2.startPos.type = KIN_COORD;
	doorPars2.startPos.position.x = DOOR_PAR_X;
	doorPars2.startPos.position.y = DOOR_PAR_Y;
	doorPars2.startPos.position.z = DOOR_HIGHT1;

	doorPars2.endPos.type = KIN_COORD;
	doorPars2.endPos.position.x = -DOOR_PAR_X;
	doorPars2.endPos.position.y = -DOOR_PAR_Y;
	doorPars2.endPos.position.z = DOOR_HIGHT2;
	doorPars2.riseHeight = 25.0 + DOOR_HIGHT1 - DOOR_HIGHT2;
	doorPars2.radius = 8.0;
	doorPars2.trajPars.velocity = doorPars1.trajPars.velocity;
	doorPars2.trajPars.acceleration = doorPars2.trajPars.velocity * 100;
	doorPars2.trajPars.splineTime = 0.001;

	TRAJ_PARS readyPars;
	readyPars.velocity = 800;
	readyPars.acceleration = 800 * 100;
	readyPars.splineTime = 0.01;

	double position[6];
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksReadPosDelta(position);

	if (position[0] - doorPars1.startPos.position.x >  0.1 ||
		position[1] - doorPars1.startPos.position.y >  0.1 ||
		position[2] - doorPars1.startPos.position.z >  0.1 ||
		position[0] - doorPars1.startPos.position.x < -0.1 ||
		position[1] - doorPars1.startPos.position.y < -0.1 ||
		position[2] - doorPars1.startPos.position.z < -0.1 )
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(doorPars1.startPos, readyPars);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars1);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars2);

	return nyceStatus;
}

const uint32_t CMotionStateMach::Ptp()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	
	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = m_nextPos_x;
	pos.position.y = m_nextPos_y;
	pos.position.z = m_nextPos_z;
	
	TRAJ_PARS trajPars;
	trajPars.velocity = m_motionVel;
	trajPars.acceleration = trajPars.velocity * 100;
	trajPars.splineTime = 0.01;
	
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, m_bPtpRelative);
	
	return nyceStatus;
}

const uint32_t CMotionStateMach::Home()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	
	TRAJ_PARS trajPars;
	trajPars.velocity = HOME_VEL;
	trajPars.acceleration = trajPars.velocity * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksHomeDelta(trajPars);
	
	return nyceStatus;
}

const uint32_t CMotionStateMach::Circ()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	
	ROCKS_COORD readyPos;
	readyPos.type = KIN_COORD;
	readyPos.position.x = -200;
	readyPos.position.y = 0; 
	readyPos.position.z = -835;
	
	TRAJ_PARS trajPars;
	trajPars.velocity = m_motionVel;
	trajPars.acceleration = trajPars.velocity * 100;
	trajPars.splineTime = 0.005;
	
	TRAJ_PARS readyPars;
	readyPars.velocity = 100;
	readyPars.acceleration = readyPars.velocity * 100;
	readyPars.splineTime = 0.01;
	
	double position[6];
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksReadPosDelta(position);
	
	if (position[0] - readyPos.position.x >  0.1 ||
		position[1] - readyPos.position.y >  0.1 ||
		position[2] - readyPos.position.z >  0.1 ||
		position[0] - readyPos.position.x < -0.1 ||
		position[1] - readyPos.position.y < -0.1 ||
		position[2] - readyPos.position.z < -0.1 )
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(readyPos, readyPars);
	
	CARTESIAN_COORD centerOffset;
	centerOffset.x = 200;
	centerOffset.y = 0;
	centerOffset.z = 0;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCricleDelta(centerOffset, -M_PI * 8, trajPars);
	
	return nyceStatus;
}

bool CMotionStateMach::FinlishMatch(const double &x, const double &y, const double &angle, const bool &bSuccess)
{
	if (bSuccess)
	{
		m_dTargetPos_x = x;
		m_dTargetPos_y = y;
		m_dTargetAngle = angle;
	}
	else
	{
		m_dTargetPos_x = 0.0;
		m_dTargetPos_y = 0.0;
		m_dTargetAngle = 0.0;
	}

	if (!m_bWaitForMatchRes)
		return false;
	SetEvent(m_hEvFinlishMatch);
	return true;
}

bool CMotionStateMach::SwitchToCatchState()
{
	if (m_status != READY)
		return false;
	else
	{
		m_status = CATCH;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToInitState()
{
	if (m_status != READY)
		return false;
	else
	{
		m_status = INIT;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToDoorState(const double &vel)
{
	if (m_status != READY)
		return false;
	else
	{
		m_motionVel = vel;
		m_status = DOOR;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToPtpState(const double &x, const double &y, const double &z, const double &vel)
{
	if (m_status != READY)
		return false;
	else
	{
		m_nextPos_x = x;
		m_nextPos_y = y;
		m_nextPos_z = z;
		m_motionVel = vel;
		m_bPtpRelative = false;
		m_status = PTP;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToHomeState()
{
	if (m_status != READY)
		return false;
	else
	{
		m_status = HOME;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToCircState(const double &vel)
{
	if (m_status != READY)
		return false;
	else
	{
		m_motionVel = vel;
		m_status = CIRC;
		SetEvent(m_hEvMove);
		return true;
	}
}

bool CMotionStateMach::SwitchToJogState(const double &dist, const int &dire)
{
	if (m_status != READY)
		return false;
	else
	{
		m_nextPos_x = 0;
		m_nextPos_y = 0;
		m_nextPos_z = 0;
		switch (dire)
		{
		case 0:
			m_nextPos_x += dist;
			break;
		case 1:
			m_nextPos_y += dist;
			break;
		case 2:
			m_nextPos_z += dist;
			break;
		}
		m_motionVel = JOG_VEL;
		m_bPtpRelative = true;
		m_status = PTP;
		SetEvent(m_hEvMove);
		return true;
	}
}

// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	ROCKS_COORD pos;
// 	pos.type = KIN_COORD;
// 	pos.position.x = 0;
// 	pos.position.y = 0;
// 	pos.position.z = 0;

// 	UpdateData(TRUE);
// 	switch (m_motion_par_direc)
// 	{
// 	case 0:
// 		pos.position.x = -10;
// 		break;
// 	case 1:
// 		pos.position.y = -10;
// 		break;
// 	case 2:
// 		pos.position.z = -10;
// 		break;
// 	}
// 
// 	TRAJ_PARS trajPars;
// 	trajPars.velocity = JOG_VEL;
// 	trajPars.acceleration = trajPars.velocity * 100;
// 	trajPars.splineTime = 0.01;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);
// 
// 	StatusHandle(nyceStatus);

void CMotionStateMach::StatusHandler(uint32_t &nyceStatus)
{
	SendString(NyceGetStatusStringEx(nyceStatus));
}

void CMotionStateMach::SendString(const CString &str)
{
	SYSTEMTIME time;
	GetSystemTime(&time);
	m_str = "";
	m_str.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
	m_str += ": ";
	m_str += str;
	m_str += "\n";
	::SendMessage(m_hMainWnd, WM_HANDLE_NYCESTATUS, NULL, (LPARAM)&m_str);
}