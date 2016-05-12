#pragma once

#include <fstream>
#include <iostream>
using namespace std;

#include "nyceapi.h"
#include "NyceExDefs.h"

#include "CoordinateAlgorithm.h"
#include "DeltaKinAlgorithm.h"

static DELTA_MECH_PARS delta_mech_pars;
static double rate_angle2pu[ROCKS_MECH_MAX_NR_OF_JOINTS];

static HANDLE evExportDatas(CreateEvent(NULL,TRUE,FALSE,NULL));

//设置Delta机构参数函数
static NYCE_STATUS RocksSetMechParsDelta(const double &lenOfBasePlatform, const double &lenOfTravelPlatform, const double &lenOfActiveArm, const double &lenOfPassiveArm)
{
	delta_mech_pars.e = lenOfTravelPlatform;
	delta_mech_pars.f = lenOfBasePlatform;
	delta_mech_pars.re = lenOfPassiveArm;
	delta_mech_pars.rf = lenOfActiveArm;

	return NYCE_OK;
}

//设置比例函数
static NYCE_STATUS RocksSetPuRateDelta(const double &rate_robot, const double &rate_belt)
{
	rate_angle2pu[0] = rate_robot;
	rate_angle2pu[1] = rate_robot;
	rate_angle2pu[2] = rate_robot;
	rate_angle2pu[3] = rate_belt;

	return NYCE_OK;
}

//检查比例值是否有误
inline NYCE_STATUS RocksCheckRate(ROCKS_MECH* pMech)
{
	for (uint32_t ax = 0; ax < pMech->nrOfJoints; ax++)
	{
		if (rate_angle2pu[ax] < 1)
			return ROCKS_ERR_PU_RATE_ERROR;
	}
	return NYCE_OK;
}

//将角度转换成PU
//angle[in]				-角度
//positionUnit[out]		-PU
//index[in]				-比例值序号，轴ID
inline void ConvertAngleToPU(const double &angle, double &positionUnit, const uint32_t &index)
{
	positionUnit = angle * rate_angle2pu[index];
}

//将PU转换成角度
//angle[out]			-角度
//positionUnit[in]		-PU
//index[in]				-比例值序号，轴ID
inline void ConvertPUToAngle(const double &positionUnit, double &angle, const uint32_t &index)
{
	if (rate_angle2pu[index])
		angle = positionUnit / rate_angle2pu[index];
	else
		angle = 0.0;
}

//Delta的正向坐标转换函数
//pJointPos[in]			-轴坐标
//pMechPos[out]			-笛卡尔坐标
static NYCE_STATUS RocksKinForwardDelta(ROCKS_MECH* pMech, const double pJointPos[], double pMechPos[])
{
	//检查机器人参数
	if (delta_mech_pars.e <= 0 || delta_mech_pars.f <= 0 || delta_mech_pars.re <= 0 || delta_mech_pars.rf <= 0 )
		return ROCKS_ERR_DELTA_PARS_ERROR;

	//检查比例
	NYCE_STATUS status = RocksCheckRate(pMech);
	if (NyceError(status))
		return status;

	//置零笛卡尔坐标数组
	ZeroMemory(pMechPos, sizeof(double) * ROCKS_MECH_MAX_DOF);

	double jointAnglePos[ROCKS_MECH_MAX_NR_OF_JOINTS];//角度坐标
	for (uint32_t ax = 0; ax < 3; ax++)
	{
		//将轴坐标转换成角度
		ConvertPUToAngle(pJointPos[ax], jointAnglePos[ax], ax);
		if (jointAnglePos[ax] < -M_PI_2 || jointAnglePos[ax] > M_PI_2)
			return ROCKS_ERR_DELTA_POSTURE_ERROR;
	}

	//正向计算
	if(!DeltaCalcPosForward(delta_mech_pars, jointAnglePos, pMechPos))
		return ROCKS_ERR_DELTA_JOINT_POS_ERROR;

	return NYCE_OK;
}

//获取Delta的机构位置
//pPos[out]			-笛卡尔坐标
static NYCE_STATUS RocksKinDeltaPosition(ROCKS_MECH* pMech, double pPos[])
{
	//置零数组
	ZeroMemory(pPos, ROCKS_MECH_MAX_DOF * sizeof(double));
	double pJointPos[ROCKS_MECH_MAX_DOF];
	NYCE_STATUS status = NYCE_OK;
	for (uint32_t ax = 0; ax < pMech->nrOfJoints; ax++)
	{
		//读取轴坐标系的当前位置
		status = NyceError(status) ? status : SacReadVariable(pMech->jointAxisId[ax], SAC_VAR_SETPOINT_POS, &pJointPos[ax]);
	}
	//轴坐标转换成笛卡尔坐标
	status = NyceError(status) ? status : RocksKinForwardDelta(pMech, pJointPos, pPos);	
	return status;
}

//Delta的逆向坐标转换函数
//将笛卡尔坐标转换成轴坐标，数据在pMech内部缓存
static NYCE_STATUS RocksKinInverseDelta(ROCKS_MECH* pMech, const ROCKS_KIN_INV_PARS* pKin)
{
	if (pMech->var.mechStep != ROCKS_MECH_STEP_VALID_PATH)
		return ROCKS_ERR_NO_VALID_PATH;

	//检查机构参数
	if (delta_mech_pars.e <= 0 || delta_mech_pars.f <= 0 || delta_mech_pars.re <= 0 || delta_mech_pars.rf <= 0 )
		return ROCKS_ERR_DELTA_PARS_ERROR;

	//检查比例值
	NYCE_STATUS status = RocksCheckRate(pMech);
	if (NyceError(status))
		return status;

	uint32_t ax = 0;

	//如果轴坐标内存被分配过要释放掉旧的
	if (pMech->var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (pMech->var.pJointPositionBufferC[i] != NULL)
			{
				free(pMech->var.pJointPositionBufferC[i]);
				pMech->var.pJointPositionBufferC[i] = NULL;
			}
			if (pMech->var.pJointVelocityBufferC[i] != NULL)
			{
				free(pMech->var.pJointVelocityBufferC[i]);
				pMech->var.pJointVelocityBufferC[i] = NULL;
			}
		}
		pMech->var.jointBuffersAllocated = FALSE;
	}

	//分配轴坐标内存
	while (!pMech->var.jointBuffersAllocated)
	{
		pMech->var.pJointPositionBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));
		pMech->var.pJointVelocityBufferC[ax] = (double*)malloc(pMech->var.maxNrOfSplines * sizeof(double));

		ZeroMemory(pMech->var.pJointPositionBufferC[ax], pMech->var.maxNrOfSplines * sizeof(double));
		ZeroMemory(pMech->var.pJointVelocityBufferC[ax], pMech->var.maxNrOfSplines * sizeof(double));

		//设置标志位和运动学函数地址
		if(++ax == ROCKS_MECH_MAX_NR_OF_JOINTS) 
		{
			pMech->var.jointBuffersAllocated = TRUE;
			pMech->var.pApplyForwardKinFunc = RocksKinForwardDelta;
			pMech->var.pApplyInverseKinFunc = RocksKinInverseDelta;
			pMech->var.pGetWorldSetpointPosFunc = RocksKinDeltaPosition;
		}
	}

	//分配笛卡尔坐标内存
	uint32_t realSegNum(0);
	double (*pPosition)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();
	double (*pVelocity)[ROCKS_MECH_MAX_DOF] = new double[pMech->var.maxNrOfSplines][ROCKS_MECH_MAX_DOF]();

	//输出轨迹数据，合速度和沿径方向的路程
	if (WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0 )
	{
		ofstream file("..//trajectoryDatas.txt");	
		file<<"|Index|pVelocitySplineBuffer|pPositionSplineBuffer"<<endl<<"|:-:|:-:|:-:"<<endl;
		for (uint32_t i = 0; i < pMech->var.usedNrOfSplines; ++i)
		{
			file<<"|"<<i<<"|"<<pMech->var.pVelocitySplineBuffer[i]<<"|"<<pMech->var.pPositionSplineBuffer[i]<<endl;
		}
		file.close();
	}
	
	//将轨迹数据，转换成笛卡尔坐标
	for (uint32_t index = 0; index < pMech->var.usedNrOfSplines; ++index, ++realSegNum)
		ConvertPathToWorldCoordinate(pMech, index, pPosition[realSegNum], pVelocity[realSegNum]);
		
	//注意usedNrOfSplines要重新赋值为实际的轨迹段数
	pMech->var.usedNrOfSplines = realSegNum;

	//输出笛卡尔坐标系的轨迹数据
	if (WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0 )
	{
		ofstream file("..//xyzDatas.txt");	
		file<<pMech->var.startPos[0]<<"|"<<pMech->var.startPos[1]<<"|"<<pMech->var.startPos[2]<<endl;
		file<<"|Index|x_pos|y_pos|z_pos|x_vel|y_vel|z_vel|"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|"<<endl;
		for (uint32_t i = 0; i < realSegNum; ++i)
		{
			file<<"|"<<i<<"|"<<pPosition[i][0]<<"|"<<pPosition[i][1]<<"|"<<pPosition[i][2]<<"|"<<pVelocity[i][0]<<"|"<<pVelocity[i][1]<<"|"<<pVelocity[i][2]<<"|"<<endl;
		}
		file.close();
	}
	
	//轴坐标系的轨迹数据缓冲区
	double jointAnglePos[ROCKS_MECH_MAX_NR_OF_JOINTS];
	double jointAngleVel[ROCKS_MECH_MAX_NR_OF_JOINTS];

	//通过运动学将笛卡尔坐标转换成轴坐标
	for(uint32_t index = 0; index < realSegNum; ++index)
	{
		//位置逆运算
		if(!DeltaCalcPosInverse(delta_mech_pars, pPosition[index], jointAnglePos))
			return ROCKS_ERR_DELTA_TAJ_OVER_WORKSPACE;
		//速度逆运算
		if(!DeltaCalcVelInverse(delta_mech_pars, pPosition[index], pVelocity[index], jointAnglePos, jointAngleVel))
			return ROCKS_ERR_DELTA_TAJ_VEL_ERROR;
		//将角度转换成轴坐标
		for (uint32_t ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ax++)
		{
			ConvertAngleToPU(jointAnglePos[ax], pMech->var.pJointPositionBufferC[ax][index], ax);
			ConvertAngleToPU(jointAngleVel[ax], pMech->var.pJointVelocityBufferC[ax][index], ax);
		}
	}
	//输出轴坐标系下的轨迹数据
	if (WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0 )
	{
		ofstream file("..//jointDatas.txt");	
		file<<pMech->var.startPos[0]<<"|"<<pMech->var.startPos[1]<<"|"<<pMech->var.startPos[2]<<endl;
		file<<"|Index|joint1_pos|joint2_pos|joint3_pos|joint1_vel|joint2_vel|joint3_vel|"<<endl<<"|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|"<<endl;
		for (uint32_t i = 0; i < realSegNum; ++i)
			file<<"|"<<i<<"|"<<pMech->var.pJointPositionBufferC[0][i]<<"|"<<pMech->var.pJointPositionBufferC[1][i]<<"|"<<pMech->var.pJointPositionBufferC[2][i]<<"|"<<pMech->var.pJointVelocityBufferC[0][i]<<"|"<<pMech->var.pJointVelocityBufferC[1][i]<<"|"<<pMech->var.pJointVelocityBufferC[2][i]<<"|"<<endl;
		
		file.close();
	}

	//清除缓存
	delete []pPosition;
	delete []pVelocity;

	//设置标志位
	pMech->var.mechStep = ROCKS_MECH_STEP_VALID_INV_KINEMATICS;
	return NYCE_OK;
}

//输出spline数据功能的控制函数
//调试时使用
static NYCE_STATUS RocksExExportSplineDatas(const BOOL &signal)
{
	if (signal && WaitForSingleObject(evExportDatas, 0) != WAIT_OBJECT_0)
		SetEvent(evExportDatas);

	if (!signal && WaitForSingleObject(evExportDatas, 0) == WAIT_OBJECT_0)
		ResetEvent(evExportDatas);

	return NYCE_OK;
}