#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "CoordinateStructs.h"
#include "SpaceAlgorithm.h"
#include "SpiralTrajAlgorithm.h"
#include "RocksSpiralBufferManager.h"

#include <rocksapi.h>

static uint32_t mix_Boundary = 0;
static ROCKS_PLANE mix_plane = ROCKS_PLANE_XY;
static ROCKS_POSE mix_pose;
static uint32_t mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
static double mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, mix_center1, mix_center2, mix_angle, mix_endPos_x, mix_endPos_y, mix_endPos_z;

//将分段的圆弧轨迹数据转换成笛卡尔坐标数据，用在混合轨迹中
//注意该函数只是将某一段数据转换，不是整体转换
//plane[in]				-轨迹所在实际平面
//pose[in]				-变换矩阵
//startPos1[in]			-起始点值1
//startPos2[in]			-起始点值2
//center1[in]			-中心点值1
//center2[in]			-中心点值2
//angle[in]				-轨迹的整体旋转角度，有方向
//CurrentDistance[in]	-当前点已经走过的路程
//CurrentVelocity[in]	-当前点的速度
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConvertCriclePath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &startPos2, const double &center1, const double &center2, const double &angle, const  double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	//计算半径
	double radius = sqrt((center1 - startPos1) * (center1 - startPos1) + (center2 - startPos2) * (center2 - startPos2));

	//先假设圆弧在XY平面
	//计算起始点和圆心连线与x轴的夹角
	double beta(0.0);
	CalcRotateAngle(beta, center1, center2, startPos1, startPos2);

	//根据当前分段位置到起始点的路程，计算当前点和圆心连线与x轴的夹角
	double absoluteAngle(0.0), absoluteVelocity(0.0);
	if (angle>0)
	{
		absoluteAngle = beta - CurrentDistance / radius;
		absoluteVelocity = CurrentVelocity;
	}
	else
	{
		absoluteAngle = beta + CurrentDistance / radius;
		absoluteVelocity = -CurrentVelocity;
	}

	//计算XY平面内，当前点的笛卡尔坐标值
	//并将假设的XY平面轨迹，转换到实际的平面
	switch(plane)
	{
	case ROCKS_PLANE_XY:

		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[1] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[1] = -absoluteVelocity * cos(absoluteAngle);
		break;

	case ROCKS_PLANE_YZ:

		pPosition[1] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[1] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;

	case ROCKS_PLANE_ZX:

		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;

	default:
		break;
	}

	//变换矩阵计算
	//旋转
	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	//平移
	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

//将分段的圆弧轨迹数据转换成笛卡尔坐标数据，用在非混合轨迹中
//注意该函数只是将某一段数据转换，不是整体转换
//pStartPos[in]			-起始点
//totalAngle[in]		-轨迹的整体旋转角度，有方向
//CurrentDistance[in]	-当前点已经走过的路程
//CurrentVelocity[in]	-当前点的速度
//plane					-轨迹所在平面
//radius				-圆弧半径
//pCenter				-圆心
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConvertCriclePath(const double *const pStartPos, const double &totalAngle, const double &CurrentDistance, const double &CurrentVelocity, const ROCKS_PLANE &plane, const double &radius, const double *const pCenter, double *const pPosition, double *const pVelocity)
{
	double angle(0.0);
	double beta(0.0);
	switch(plane)
	{
	case ROCKS_PLANE_XY:
		//计算起始偏角
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[0], pStartPos[1]);

		//根据路程计算实际笛卡尔坐标
		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] =  CurrentVelocity * sin(angle);
			pVelocity[1] = -CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] = -CurrentVelocity * sin(angle);
			pVelocity[1] =  CurrentVelocity * cos(angle);
			pVelocity[2] =  0;
		}

		pPosition[0] = pCenter[0] + radius * cos(angle);
		pPosition[1] = pCenter[1] + radius * sin(angle);
		pPosition[2] = pStartPos[2];
		break;

	case ROCKS_PLANE_YZ:
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[1], pStartPos[2]);

		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] =  0;
			pVelocity[1] =  CurrentVelocity * sin(angle);
			pVelocity[2] = -CurrentVelocity * cos(angle);
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] =  0;
			pVelocity[1] = -CurrentVelocity * sin(angle);
			pVelocity[2] =  CurrentVelocity * cos(angle);
		}

		pPosition[0] = pStartPos[0];
		pPosition[1] = pCenter[0] + radius * cos(angle);
		pPosition[2] = pCenter[1] + radius * sin(angle);
		break;

	case ROCKS_PLANE_ZX:
		CalcRotateAngle(beta, pCenter[0], pCenter[1], pStartPos[2], pStartPos[0]);

		if (totalAngle > 0)
		{
			angle = beta - CurrentDistance / radius;

			pVelocity[0] = -CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] =  CurrentVelocity * sin(angle);
		}
		else
		{
			angle = beta + CurrentDistance / radius;

			pVelocity[0] =  CurrentVelocity * cos(angle);
			pVelocity[1] =  0;
			pVelocity[2] = -CurrentVelocity * sin(angle);
		}

		pPosition[0] = pCenter[1] + radius * sin(angle);
		pPosition[1] = pStartPos[1];
		pPosition[2] = pCenter[0] + radius * cos(angle);
		break;
	}
}


//将分段的螺旋线轨迹数据转换成笛卡尔坐标数据
//注意该函数只是将某一段数据转换，不是整体转换
//plane[in]				-轨迹所在实际平面
//pose[in]				-变换矩阵
//startPos1[in]			-起始点值1
//startPos2[in]			-起始点值2
//endPos1[in]			-终点值1
//endPos2[in]			-终点值2
//center1[in]			-中心点值1
//center2[in]			-中心点值2
//CurrentDistance[in]	-当前点已经走过的路程
//CurrentVelocity[in]	-当前点的速度
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConverSpiralPath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &startPos2, const double &endPos1, const double &endPos2, const double &center1, const double &center2, const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double stratRadius(0.0), endRadius(0.0), a(0.0), b(0.0);
	//计算螺旋线参数a，b
	CalcArchimedeSpiralPars(startPos1, startPos2, endPos1, endPos2, center1, center2, stratRadius, endRadius, a, b);

	double absoluteAngle(0.0), radius(0.0), absoluteVelocity(0.0);
	switch(plane)
	{
	case ROCKS_PLANE_XY:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[1] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[1] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_YZ:
		pPosition[1] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[1] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	case ROCKS_PLANE_ZX:
		pPosition[0] = center1 + radius * cos(absoluteAngle);
		pPosition[2] = center2 + radius * sin(absoluteAngle);

		pVelocity[0] =  absoluteVelocity * sin(absoluteAngle);
		pVelocity[2] = -absoluteVelocity * cos(absoluteAngle);
		break;
	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}

//将分段的直线轨迹数据转换成笛卡尔坐标数据，用在混合轨迹中
//注意该函数只是将某一段数据转换，不是整体转换
//plane[in]				-轨迹所在实际平面
//pose[in]				-变换矩阵
//startPos1[in]			-起始点值1
//startPos2[in]			-起始点值2
//endPose1[in]			-终点值1
//endPose2[in]			-终点值2
//CurrentDistance[in]	-当前点已经走过的路程
//CurrentVelocity[in]	-当前点的速度
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConverLinePath(const ROCKS_PLANE &plane, const ROCKS_POSE &pose, const double &startPos1, const double &endPose1, const double &startPos2, const double &endPose2 , const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double distance = sqrt((endPose1 - startPos1) * (endPose1 - startPos1) + (endPose2 - startPos2) * (endPose2 - startPos2));
	double rate1 = (endPose1 - startPos1) / distance;
	double rate2 = (endPose2 - startPos2) / distance;

	switch(plane)
	{
	case ROCKS_PLANE_XY:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = CurrentDistance * rate2 + startPos2;
		pPosition[2] = 0;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = CurrentVelocity * rate2;
		pVelocity[2] = 0;
		break;
	case ROCKS_PLANE_YZ:

		pPosition[0] = 0;
		pPosition[1] = CurrentDistance * rate1 + startPos1;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = 0;
		pVelocity[1] = CurrentVelocity * rate1;
		pVelocity[2] = CurrentVelocity * rate2;
		break;
	case ROCKS_PLANE_ZX:

		pPosition[0] = CurrentDistance * rate1 + startPos1;
		pPosition[1] = 0;
		pPosition[2] = CurrentDistance * rate2 + startPos2;

		pVelocity[0] = CurrentVelocity * rate1;
		pVelocity[1] = 0;
		pVelocity[2] = CurrentVelocity * rate2;
		break;
	default:
		break;
	}

	if (pose.r.x)
	{
		Roll(pPosition, startPos1, startPos2, pose.r.x);
		Roll(pVelocity, pose.r.x);
	}

	if (pose.r.y)
	{
		Pitch(pPosition, startPos1, startPos2, pose.r.y);
		Pitch(pVelocity, pose.r.y);
	}

	if (pose.r.z)
	{
		Yaw(pPosition, startPos1, startPos2, pose.r.z);
		Yaw(pVelocity, pose.r.z);
	}

	pPosition[0] += pose.t.x;
	pPosition[1] += pose.t.y;
	pPosition[2] += pose.t.z;
}


//将分段的圆弧轨迹数据转换成笛卡尔坐标数据，用在非混合轨迹中
//注意该函数只是将某一段数据转换，不是整体转换
//pStartPos[in]			-起始点
//pEndPos[in]			-终点
//totalDistance			-总路程
//CurrentDistance[in]	-当前点已经走过的路程
//CurrentVelocity[in]	-当前点的速度
//pCenter				-圆心
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConverLinePath(const double *const pStartPos, const double *const pEndPos, const double &totalDistance, const double &CurrentDistance, const double &CurrentVelocity, double *const pPosition, double *const pVelocity)
{
	double rate_x = (pEndPos[0] - pStartPos[0]) / totalDistance;
	double rate_y = (pEndPos[1] - pStartPos[1]) / totalDistance;
	double rate_z = (pEndPos[2] - pStartPos[2]) / totalDistance;

	pPosition[0] = CurrentDistance * rate_x + pStartPos[0];
	pPosition[1] = CurrentDistance * rate_y + pStartPos[1];
	pPosition[2] = CurrentDistance * rate_z + pStartPos[2];

	pVelocity[0] = CurrentVelocity * rate_x;
	pVelocity[1] = CurrentVelocity * rate_y;
	pVelocity[2] = CurrentVelocity * rate_z;
}
//将轨迹描述数据转换成笛卡尔坐标
//注意该函数只是将某一段数据转换，不是整体转换
//index[in]				-缓冲区序号
//pPosition[out]		-笛卡尔坐标位置
//pVelocity[out]		-笛卡尔坐标速度
static void ConvertPathToWorldCoordinate(const ROCKS_MECH* const pMech, uint32_t &index, double *const pPosition, double *const pVelocity)
{
	//圆弧轨迹
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_CIRCULAR)
	{
		ConvertCriclePath(pMech->var.startPos, pMech->var.angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index],pMech->var.plane, pMech->var.radius, pMech->var.center, pPosition, pVelocity);
	} 

	//直线轨迹
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_LINEAR)
	{
		ConverLinePath(pMech->var.startPos, pMech->var.endPos, pMech->var.pPositionSplineBuffer[pMech->var.usedNrOfSplines - 1], pMech->var.pPositionSplineBuffer[index],pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
	}

	//混合轨迹
	if (pMech->var.moveType == ROCKS_MOVE_TYPE_MIX)
	{
		//当前是第一段时，混合轨迹临界点序号置零
		if (index == 0)
		{
			mix_Boundary = 0;
		}
		//当前段号为混合轨迹临界点序号，提取下一种轨迹的轨迹描述信息
		if (index == mix_Boundary)
		{
			//更新临界点序号
			mix_Boundary = (uint32_t)(pMech->var.pVelocitySplineBuffer[index + 3] + index + 7);

			//获得所在平面信息
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) / 256)
			{
			case ROCKS_PLANE_XY:
				mix_plane = ROCKS_PLANE_XY;
				break;
			case ROCKS_PLANE_YZ:
				mix_plane = ROCKS_PLANE_YZ;
				break;
			case ROCKS_PLANE_ZX:
				mix_plane = ROCKS_PLANE_ZX;
				break;
			default:
				break;
			}

			//获得轨迹类型信息
			switch ((int)(pMech->var.pPositionSplineBuffer[index + 3]) % 256)
			{
			case ROCKS_MOVE_TYPE_LINEAR:
				mix_moveType = ROCKS_MOVE_TYPE_LINEAR;
				break;
			case ROCKS_MOVE_TYPE_CIRCULAR:
				mix_moveType = ROCKS_MOVE_TYPE_CIRCULAR;
				break;
			case ROCKS_MOVE_TYPE_SPIRAL:
				mix_moveType = ROCKS_MOVE_TYPE_SPIRAL;
				break;
			case ROCKS_MOVE_TYPE_SPIRAL_EX:
				mix_moveType = ROCKS_MOVE_TYPE_SPIRAL_EX;
				break;
			default:
				break;
			}
			//获得终点三维坐标
			mix_endPos_x  = pMech->var.pVelocitySplineBuffer[index];
			mix_endPos_y  = pMech->var.pPositionSplineBuffer[index];
			mix_endPos_z  = pMech->var.pVelocitySplineBuffer[index + 1];
			//获得旋转矩阵
			mix_pose.r.x  = pMech->var.pPositionSplineBuffer[index + 1];
			mix_pose.r.y  = pMech->var.pPositionSplineBuffer[index + 2];
			mix_pose.r.z  = pMech->var.pVelocitySplineBuffer[index + 2];
			//获取起始点二维坐标
			mix_startPos1 = pMech->var.pVelocitySplineBuffer[index + 4];
			mix_startPos2 = pMech->var.pPositionSplineBuffer[index + 4];

			//根据不同轨迹类型获得特殊信息
			switch (mix_moveType)
			{
			case ROCKS_MOVE_TYPE_LINEAR:
				//获得直线终点二维坐标
				mix_endPos1 = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_endPos2 = pMech->var.pPositionSplineBuffer[index + 5];
				break;
			case ROCKS_MOVE_TYPE_CIRCULAR:
				//获得圆弧旋转角和圆心位置
				mix_angle = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_center1 = pMech->var.pVelocitySplineBuffer[index + 6];
				mix_center2 = pMech->var.pPositionSplineBuffer[index + 6];
				break;
			case ROCKS_MOVE_TYPE_SPIRAL:
				//螺旋线终点二维坐标和中心位置二维坐标
				mix_endPos1 = pMech->var.pVelocitySplineBuffer[index + 5];
				mix_endPos2 = pMech->var.pPositionSplineBuffer[index + 5];
				mix_center1 = pMech->var.pVelocitySplineBuffer[index + 6];
				mix_center2 = pMech->var.pPositionSplineBuffer[index + 6];
				break;
			default:
				break;
			}
			//跳过轨迹描述信息区，序号进入真正的分段数据区
			index += 7;
		}
		//根据不同轨迹类型转换数据
		switch (mix_moveType)
		{
		case ROCKS_MOVE_TYPE_LINEAR:
			ConverLinePath(mix_plane, mix_pose, mix_startPos1, mix_endPos1, mix_startPos2, mix_endPos2, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		case ROCKS_MOVE_TYPE_CIRCULAR:
			ConvertCriclePath(mix_plane, mix_pose, mix_startPos1, mix_startPos2, mix_center1, mix_center2, mix_angle, pMech->var.pPositionSplineBuffer[index], pMech->var.pVelocitySplineBuffer[index], pPosition, pVelocity);
			break;
		case ROCKS_MOVE_TYPE_SPIRAL:
			//ConverSpiralPath();
			break;
		case ROCKS_MOVE_TYPE_SPIRAL_EX:
			pPosition[0] = pPosSpiralSplineBuffer[index][0];
			pPosition[1] = pPosSpiralSplineBuffer[index][1];
			pPosition[2] = pPosSpiralSplineBuffer[index][2];

			pVelocity[0] = pVelSpiralSplineBuffer[index][0];
			pVelocity[1] = pVelSpiralSplineBuffer[index][1];
			pVelocity[2] = pVelSpiralSplineBuffer[index][2];

			return;

			break;
		default:
			break;
		}
		switch (mix_plane)
		{
		case ROCKS_PLANE_XY://XY-PLANE
			pPosition[2] = mix_endPos_z;
			break;
		case ROCKS_PLANE_YZ://YZ-PLANE
			pPosition[0] = mix_endPos_x;
			break;
		case ROCKS_PLANE_ZX://ZX-PLANE
			pPosition[1] = mix_endPos_y;
			break;
		default:
			break;
		}
	}
	//整体轨迹旋转
	if (pMech->var.refFramePose2.r.x)
	{
		Roll(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.x);
		Roll(pVelocity, pMech->var.refFramePose2.r.x);
	}

	if (pMech->var.refFramePose2.r.y)
	{
		Pitch(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.y);
		Pitch(pVelocity, pMech->var.refFramePose2.r.y);
	}

	if (pMech->var.refFramePose2.r.z)
	{
		Yaw(pPosition, pMech->var.startPos, pMech->var.refFramePose2.r.z);
		Yaw(pVelocity, pMech->var.refFramePose2.r.z);
	}
	//整体轨迹平移
	pPosition[0] += pMech->var.refFramePose2.t.x;
	pPosition[1] += pMech->var.refFramePose2.t.y;
	pPosition[2] += pMech->var.refFramePose2.t.z;
} 

//不同坐标系的转换函数
//先转换成基础坐标系，再转换成目标坐标系
//按照（Roll->Pitch->Yaw->平移->缩放）顺序转换成基础坐标系
//按照（缩放->平移->Yaw->Pitch->Roll）顺序转换成目标坐标系
//origin[in]		-源坐标系数据
//target[out]		-目标坐标系数据
static void ConvertTwoCoordinate(const ROCKS_COORD &origin, ROCKS_COORD &target)
{

	if (origin.type == target.type)
	{
		target.position.x = origin.position.x;
		target.position.y = origin.position.y;
		target.position.z = origin.position.z;

		return;
	}

	ROCKS_COORD buffer;
	buffer.position.x = origin.position.x;
	buffer.position.y = origin.position.y;
	buffer.position.z = origin.position.z;
	buffer.type = BASE_COORD;

	//转化为基础坐标系
	if (origin.type != BASE_COORD)
	{
		double buf(0.0);
		uint32_t index(origin.type);
		double angleX(g_pTransfMatrix[index].r.x);
		double angleY(g_pTransfMatrix[index].r.y);
		double angleZ(g_pTransfMatrix[index].r.z);

		//Roll
					  buf =  buffer.position.y * cos(angleX) + buffer.position.z * sin(angleX);
		buffer.position.z = -buffer.position.y * sin(angleX) + buffer.position.z * cos(angleX);
		buffer.position.y =  buf;

		//Pitch
					  buf =	 buffer.position.x * cos(angleY) - buffer.position.z * sin(angleY);
		buffer.position.z =	 buffer.position.x * sin(angleY) + buffer.position.z * cos(angleY);
		buffer.position.x =	 buf;

		//Yaw
					  buf =  buffer.position.x * cos(angleZ) + buffer.position.y * sin(angleZ);
		buffer.position.y = -buffer.position.x * sin(angleZ) + buffer.position.y * cos(angleZ);
		buffer.position.x =  buf;

		buffer.position.x += g_pTransfMatrix[index].t.x;
		buffer.position.y += g_pTransfMatrix[index].t.y;
		buffer.position.z += g_pTransfMatrix[index].t.z;

		if (g_pTransfMatrix[index].zoom != 0 && g_pTransfMatrix[index].zoom != 1)
		{
			buffer.position.x /= g_pTransfMatrix[index].zoom;
			buffer.position.y /= g_pTransfMatrix[index].zoom;
			buffer.position.z /= g_pTransfMatrix[index].zoom;
		}
	}

	target.position.x = buffer.position.x;
	target.position.y = buffer.position.y;
	target.position.z = buffer.position.z;

	//转化为目标坐标系
	if (target.type != BASE_COORD)
	{
		double buf(0.0);
		uint32_t index(target.type);
		double angleX(-g_pTransfMatrix[index].r.x);
		double angleY(-g_pTransfMatrix[index].r.y);
		double angleZ(-g_pTransfMatrix[index].r.z);

		if (g_pTransfMatrix[index].zoom != 0 && g_pTransfMatrix[index].zoom != 1)
		{
			target.position.x *= g_pTransfMatrix[index].zoom;
			target.position.y *= g_pTransfMatrix[index].zoom;
			target.position.z *= g_pTransfMatrix[index].zoom;
		}

		target.position.x -= g_pTransfMatrix[index].t.x;
		target.position.y -= g_pTransfMatrix[index].t.y;
		target.position.z -= g_pTransfMatrix[index].t.z;

		//Yaw
					  buf =  target.position.x * cos(angleZ) + target.position.y * sin(angleZ);
		target.position.y = -target.position.x * sin(angleZ) + target.position.y * cos(angleZ);
		target.position.x =  buf;

		//Pitch
					  buf =  target.position.x * cos(angleY) - target.position.z * sin(angleY);
		target.position.z =  target.position.x * sin(angleY) + target.position.z * cos(angleY);
		target.position.x =  buf;

		//Roll
					  buf =  target.position.y * cos(angleX) + target.position.z * sin(angleX);
		target.position.z = -target.position.y * sin(angleX) + target.position.z * cos(angleX);
		target.position.y =  buf;
	}
}

// static bool SetCameraMatrix(const ROCKS_E3_VECTOR &t, const ROCKS_E3_VECTOR &r)
// {
// 	if (g_pTransfMatrix)
// 	{
// 		g_pTransfMatrix[CAMERA_COORD].r.x = r.x;
// 		g_pTransfMatrix[CAMERA_COORD].r.y = r.y;
// 		g_pTransfMatrix[CAMERA_COORD].r.z = r.z;
// 
// 		g_pTransfMatrix[CAMERA_COORD].t.x = t.x;
// 		g_pTransfMatrix[CAMERA_COORD].t.y = t.y;
// 		g_pTransfMatrix[CAMERA_COORD].t.z = t.z;
// 		return true;
// 	}
// 	else
// 		return false;
// }
// 
// static bool SetTargetMatrix(const ROCKS_E3_VECTOR &t, const ROCKS_E3_VECTOR &r)
// {
// 	if (g_pTransfMatrix)
// 	{
// 		g_pTransfMatrix[TARGET_COORD].r.x = r.x;
// 		g_pTransfMatrix[TARGET_COORD].r.y = r.y;
// 		g_pTransfMatrix[TARGET_COORD].r.z = r.z;
// 
// 		g_pTransfMatrix[TARGET_COORD].t.x = t.x;
// 		g_pTransfMatrix[TARGET_COORD].t.y = t.y;
// 		g_pTransfMatrix[TARGET_COORD].t.z = t.z;
// 
// 		return true;
// 	}
// 	else
// 		return false;
// }
// 
// static bool SetBeltMatrix(const ROCKS_E3_VECTOR &t, const ROCKS_E3_VECTOR &r, const double &beltLenght, const double encoderMinRange, const double encoderMaxRange)
// {						
// 	if (g_pTransfMatrix)
// 	{
// 		g_pTransfMatrix[BELT_COORD].r.x = r.x;
// 		g_pTransfMatrix[BELT_COORD].r.y = r.y;
// 		g_pTransfMatrix[BELT_COORD].r.z = r.z;
// 
// 		g_pTransfMatrix[BELT_COORD].t.x = t.x;
// 		g_pTransfMatrix[BELT_COORD].t.y = t.y;
// 		g_pTransfMatrix[BELT_COORD].t.z = t.z;
// 
// 		return true;
// 	}
// 	else
// 		return false;
// }