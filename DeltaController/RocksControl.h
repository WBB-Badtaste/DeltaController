/**
*Copyright (C), 2015-2016, Shenzhen Sunet Industry co.,Ltd.
*@file  文件名，存储编码格式说明（ANSI、UNICODE、UTF8等）
*@brief 概要说明
*@author JoMar[sos901012@gmail.com]
*@version 0.1
*@date 2016-01-22
*
*/


#pragma once

#include "Defines.h"
#include "RocksExtern_Delta.h"
#include "RocksExtren_Spiral.h"

static ROCKS_MECH  m_mech;

typedef struct cartesianCoordinate
{
	double x;
	double y;
	double z;
}CARTESIAN_COORD;

typedef struct trajectoryPars
{
	double velocity;
	double acceleration;
	double splineTime;
}TRAJ_PARS;


/**
*
*	@brief 
*
*	@author JoMar[sos901012@gmail.com]
*
*	@date 2016-01-22
*
*
*/
static NYCE_STATUS RocksInitDelta(const uint32_t &axesNum, const SAC_AXIS* const axId)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = axesNum; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( uint32_t ax = 0; ax < axesNum; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );

// 	double e = 40;     
// 	double f = 105;    
// 	double re = 194;
// 	double rf = 90;

//相聚本体
// 	double e = 65;     
// 	double f = 220;    
// 	double re = 650;
// 	double rf = 330;

	double e = 70;  
	double f = 220; 
	double re = 1000;
	double rf = 400;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksSetMechParsDelta(f, e, rf, re);

/*	double rate_angle2pu = 131072 * 11 / (2 * M_PI);//山洋机器人*/
/*	double rate_angle2pu_robot = 131072 * 40 / (2 * M_PI);//相聚机器人*/
	double rate_angle2pu_robot = 131072 * 33 / (2 * M_PI);//大族机器人
	double rate_angle2pu_belta = 131072 * 5 / (2 * M_PI);
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksSetPuRateDelta(rate_angle2pu_robot, rate_angle2pu_belta);

	return nyceStatus;
}

static NYCE_STATUS RocksInitCartesian(const uint32_t &axesNum, const SAC_AXIS* const axId)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	// Create mechanism
	// ----------------
	m_mech.nrOfJoints = axesNum; // X1, X2, Y and Z
	m_mech.dof[ 0 ]	  = TRUE;     // X
	m_mech.dof[ 1 ]	  = TRUE;     // Y
	m_mech.dof[ 2 ]	  = TRUE;     // Z
	m_mech.dof[ 3 ]	  = FALSE;    // Rx
	m_mech.dof[ 4 ]	  = FALSE;    // Ry
	m_mech.dof[ 5 ]	  = FALSE;    // Rz
	for ( uint32_t ax = 0; ax < axesNum; ax++ )
	{
		m_mech.jointAxisId[ ax ] = axId[ ax ];
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechCreate( &m_mech );

	return nyceStatus;
}

static NYCE_STATUS RocksTerm()
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (m_mech.var.jointBuffersAllocated)
	{
		int i;
		for (i = 0; i < ROCKS_MECH_MAX_NR_OF_JOINTS; i++)
		{
			if (m_mech.var.pJointPositionBufferC[i] != NULL)
			{
				free(m_mech.var.pJointPositionBufferC[i]);
				m_mech.var.pJointPositionBufferC[i] = NULL;
			}
			if (m_mech.var.pJointVelocityBufferC[i] != NULL)
			{
				free(m_mech.var.pJointVelocityBufferC[i]);
				m_mech.var.pJointVelocityBufferC[i] = NULL;
			}
		}
	}

	// Delete mechanism
	// ----------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksMechDelete( &m_mech );

	return nyceStatus;
}

static NYCE_STATUS RocksPtpDelta(const ROCKS_COORD &rocksCoord, const TRAJ_PARS &trajPars, BOOL bRelative = FALSE, const double timeout = SAC_INDEFINITE)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_PTP_PARS sinePtpPars;
	ROCKS_KIN_INV_PARS kinPars;

	ROCKS_COORD kinCoord;
	kinCoord.type = KIN_COORD;
	ConvertTwoCoordinate(rocksCoord, kinCoord);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, sinePtpPars.startPos);
	if (bRelative)
	{
		sinePtpPars.endPos[0] = sinePtpPars.startPos[0] + kinCoord.position.x;
		sinePtpPars.endPos[1] = sinePtpPars.startPos[1] + kinCoord.position.y;
		sinePtpPars.endPos[2] = sinePtpPars.startPos[2] + kinCoord.position.z;
	}
	else
	{
		sinePtpPars.endPos[0] = kinCoord.position.x;
		sinePtpPars.endPos[1] = kinCoord.position.y;
		sinePtpPars.endPos[2] = kinCoord.position.z;
	}
	
	sinePtpPars.maxVelocity = trajPars.velocity;
	sinePtpPars.maxAcceleration = trajPars.acceleration;
	sinePtpPars.splineTime = trajPars.splineTime;
	sinePtpPars.maxNrOfSplines = 0;
	sinePtpPars.pPositionSplineBuffer = NULL;
	sinePtpPars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccPtp(&m_mech,&sinePtpPars);

	for (int ax = 0; ax < 3; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

	// Synchronize on motion complete
	// ------------------------------
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout);

	return nyceStatus;
}


//centerOffset是圆形轨迹中心相对起始位置的偏移量
static NYCE_STATUS RocksCricleDelta(const CARTESIAN_COORD &centerOffset, const double &angle, const TRAJ_PARS &trajPars, const double &timeout = SAC_INDEFINITE, const int &repeatTimes = -1)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccCirclePars;
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	const double radius(sqrt(centerOffset.x * centerOffset.x + centerOffset.y * centerOffset.y + centerOffset.z * centerOffset.z));

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, 	sineAccCirclePars.startPos);

	sineAccCirclePars.maxVelocity = trajPars.velocity;
	sineAccCirclePars.maxAcceleration = trajPars.acceleration;
	sineAccCirclePars.splineTime = trajPars.splineTime;
	sineAccCirclePars.center[ 0 ] = sineAccCirclePars.startPos[0] - radius;
	sineAccCirclePars.center[ 1 ] = sineAccCirclePars.startPos[1];
	sineAccCirclePars.angle = angle;
	sineAccCirclePars.plane = ROCKS_PLANE_XY;
	sineAccCirclePars.maxNrOfSplines = 0;
	sineAccCirclePars.pPositionSplineBuffer = NULL;
	sineAccCirclePars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccCirclePars);
	
	ROCKS_POSE pose;
	CalcRotateAngle(pose.r.y, -centerOffset.x, -centerOffset.z);
	pose.r.x = 0;
	pose.r.y = pose.r.y;
	pose.r.z = 0;
	pose.t.x = 0;
	pose.t.y = 0;
	pose.t.z = 0;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &pose );

	switch(repeatTimes)
	{
	case -1:
		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		break;
	case 0:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		while(nyceStatus == NYCE_OK)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	default:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		for(int i = 0; i < repeatTimes && nyceStatus == NYCE_OK; ++i)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	}
	
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinResetOrigin(&m_mech);

	return nyceStatus;
}
static NYCE_STATUS RocksCricleCartesian(const CARTESIAN_COORD &centerOffset, const double &angle, const TRAJ_PARS &trajPars, const double &timeout = SAC_INDEFINITE, const int &repeatTimes = -1)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_TRAJ_SINE_ACC_CIRCLE_PARS sineAccCirclePars;
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	const double radius(sqrt(centerOffset.x * centerOffset.x + centerOffset.y * centerOffset.y + centerOffset.z * centerOffset.z));

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinCartesianPosition(&m_mech, 	sineAccCirclePars.startPos);

	sineAccCirclePars.maxVelocity = trajPars.velocity;
	sineAccCirclePars.maxAcceleration = trajPars.acceleration;
	sineAccCirclePars.splineTime = trajPars.splineTime;
	sineAccCirclePars.center[ 0 ] = sineAccCirclePars.startPos[0] - radius;
	sineAccCirclePars.center[ 1 ] = sineAccCirclePars.startPos[1];
	sineAccCirclePars.angle = angle;
	sineAccCirclePars.plane = ROCKS_PLANE_XY;
	sineAccCirclePars.maxNrOfSplines = 0;
	sineAccCirclePars.pPositionSplineBuffer = NULL;
	sineAccCirclePars.pVelocitySplineBuffer = NULL;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSineAccCircle( &m_mech, &sineAccCirclePars);

	ROCKS_POSE pose;
	CalcRotateAngle(pose.r.y, -centerOffset.x, -centerOffset.z);
	pose.r.x = 0;
	pose.r.y = pose.r.y;
	pose.r.z = 0;
	pose.t.x = 0;
	pose.t.y = 0;
	pose.t.z = 0;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &pose );

	switch(repeatTimes)
	{
	case -1:
		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		break;

	case 0:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		while(nyceStatus == NYCE_OK)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;

	default:
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

		for(int i = 0; i < repeatTimes && nyceStatus == NYCE_OK; ++i)
		{
			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

			for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
			{
				kinPars.pJointPositionBuffer[ ax ] = NULL;
				kinPars.pJointVelocityBuffer[ ax ] = NULL;
			}

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseCartesian( &m_mech, &kinPars );

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

			nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );
		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );
		break;
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinResetOrigin(&m_mech);

	return nyceStatus;	
}

//const double DOOR_SPLINETIME = 0.0005;
const double DOOR_SPEED = 500;
const double DOOR_ACC = DOOR_SPEED * 100;
const double DOOR_HEIGHT = 20;
const double DOOR_WIDTH = 20;
const double DOOR_FILLET = 40;

// const double OPT_DOOR_POINT_1[3] = {-65,0,-220};
// const double OPT_DOOR_POINT_2[3] = {-25,0,-170};
// const double OPT_DOOR_POINT_3[3] = { 25,0,-170};
// const double OPT_DOOR_POINT_4[3] = { 65,0,-220};

static const double OPT_DOOR_POINT_1[3] = {-65,0,-320};
static const double OPT_DOOR_POINT_2[3] = {-25,0,-270};
static const double OPT_DOOR_POINT_3[3] = { 25,0,-270};
static const double OPT_DOOR_POINT_4[3] = { 65,0,-320};	

static ROCKS_TRAJ_SEGMENT_START_PARS segStartPars;
static ROCKS_TRAJ_SEGMENT_LINE_PARS segLinePars1,segLinePars2,segLinePars3,segLinePars4;
static ROCKS_TRAJ_SEGMENT_ARC_PARS segArcPars1,segArcPars2;

static NYCE_STATUS RocksDoorDelta()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	double line = sqrt((-138.75 - OPT_DOOR_POINT_2[2]) * (-138.75 - OPT_DOOR_POINT_2[2]) + OPT_DOOR_POINT_2[0] * OPT_DOOR_POINT_2[0]);
	double z = -(138.75 + line * sqrt(4100.0) / 50);
	double center[3] = {0,0,z};

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	segStartPars.splineTime = 0.002;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;

	segLinePars1.plane = ROCKS_PLANE_ZX;
	segLinePars1.endPos[0] = OPT_DOOR_POINT_2[0];
	segLinePars1.endPos[1] = OPT_DOOR_POINT_2[2];
	segLinePars1.endVelocity = DOOR_SPEED;
	segLinePars1.maxAcceleration = DOOR_SPEED * 100;

	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = center[0];
	segArcPars1.center[1] = center[2];
	segArcPars1.endPos[0] = OPT_DOOR_POINT_3[0];
	segArcPars1.endPos[1] = OPT_DOOR_POINT_3[2];
	segArcPars1.endVelocity = DOOR_SPEED;
	segArcPars1.maxAcceleration = DOOR_SPEED * 100;
	segArcPars1.positiveAngle = TRUE;

	segLinePars2.plane = ROCKS_PLANE_ZX;
	segLinePars2.endPos[0] = OPT_DOOR_POINT_4[0];
	segLinePars2.endPos[1] = OPT_DOOR_POINT_4[2];
	segLinePars2.endVelocity = 0;
	segLinePars2.maxAcceleration = DOOR_SPEED * 100;

	segLinePars3.plane = ROCKS_PLANE_ZX;
	segLinePars3.endPos[0] = OPT_DOOR_POINT_3[0];
	segLinePars3.endPos[1] = OPT_DOOR_POINT_3[2];
	segLinePars3.endVelocity = DOOR_SPEED;
	segLinePars3.maxAcceleration = DOOR_SPEED * 100;

	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = center[0];
	segArcPars2.center[1] = center[2];
	segArcPars2.endPos[0] = OPT_DOOR_POINT_2[0];
	segArcPars2.endPos[1] = OPT_DOOR_POINT_2[2];
	segArcPars2.endVelocity = DOOR_SPEED;
	segArcPars2.maxAcceleration = DOOR_SPEED * 100;
	segArcPars2.positiveAngle = FALSE;

	segLinePars4.plane = ROCKS_PLANE_ZX;
	segLinePars4.endPos[0] = OPT_DOOR_POINT_1[0];
	segLinePars4.endPos[1] = OPT_DOOR_POINT_1[2];
	segLinePars4.endVelocity = 0;
	segLinePars4.maxAcceleration = DOOR_SPEED * 100;

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars1);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars2);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars3);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech,&segLinePars4);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );
	
	while(nyceStatus == NYCE_OK)
	{
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);

		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
		{
			kinPars.pJointPositionBuffer[ ax ] = NULL;
			kinPars.pJointVelocityBuffer[ ax ] = NULL;
		}
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE );
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );

	return nyceStatus;
}

static const double DOOR_EX_SPLINETIME = 0.01;
static const double DOOR_EX_SPEED = 100;
static const double DOOR_EX_ACC = DOOR_SPEED * 100;
static const double SPIRAL_MAX_RADIAL_SPEED = 100;
static const double SPIRAL_MAX_RADIAL_ACC = SPIRAL_MAX_RADIAL_SPEED * 100;

static ROCKS_TRAJ_SEGMENT_SPIRAL_PARS_EX segSpiralPars1, segSpiralPars2, segSpiralPars3, segSpiralPars4;

static NYCE_STATUS RocksSpiralExDoorDelta()
{
	NYCE_STATUS nyceStatus(NYCE_OK);
	ROCKS_KIN_INV_PARS kinPars;
	ROCKS_TRAJ_PATH rocksTrajPath;

	double center[2] = {(OPT_DOOR_POINT_2[0] + OPT_DOOR_POINT_3[0]) / 2.0, OPT_DOOR_POINT_1[2]};
	double radius[2] = {sqrt((OPT_DOOR_POINT_2[0] - center[0]) * (OPT_DOOR_POINT_2[0] - center[0]) + (OPT_DOOR_POINT_2[2] - center[1]) * (OPT_DOOR_POINT_2[2] - center[1])), sqrt((OPT_DOOR_POINT_3[0] - center[0]) * (OPT_DOOR_POINT_3[0] - center[0]) + (OPT_DOOR_POINT_3[2] - center[1]) * (OPT_DOOR_POINT_3[2] - center[1]))};
	double angleSpeed(DOOR_EX_SPEED / radius[1]);
	double angleMaxAcc(angleSpeed * 100);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	segStartPars.splineTime = DOOR_EX_SPLINETIME;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;

	segSpiralPars1.plane = ROCKS_PLANE_ZX;
	segSpiralPars1.center[0] = center[0];
	segSpiralPars1.center[1] = center[1];
	segSpiralPars1.endPos[0] = OPT_DOOR_POINT_2[0];
	segSpiralPars1.endPos[1] = OPT_DOOR_POINT_2[2];
	segSpiralPars1.endAngleVelocity = angleSpeed;
	segSpiralPars1.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars1.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars1.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars1.originOffset.r.x = 0;
	segSpiralPars1.originOffset.r.y = 0;
	segSpiralPars1.originOffset.r.z = 0;
	segSpiralPars1.originOffset.t.x = 0;
	segSpiralPars1.originOffset.t.y = 0;
	segSpiralPars1.originOffset.t.z = 0;
	
	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = center[0];
	segArcPars1.center[1] = center[1];
	segArcPars1.endPos[0] = OPT_DOOR_POINT_3[0];
	segArcPars1.endPos[1] = OPT_DOOR_POINT_3[2];
	segArcPars1.endVelocity = DOOR_EX_SPEED;
	segArcPars1.maxAcceleration = DOOR_EX_ACC;
	segArcPars1.positiveAngle = TRUE;
	segArcPars1.originOffset.r.x = 0;
	segArcPars1.originOffset.r.y = 0;
	segArcPars1.originOffset.r.z = 0;
	segArcPars1.originOffset.t.x = 0;
	segArcPars1.originOffset.t.y = 0;
	segArcPars1.originOffset.t.z = 0;

	segSpiralPars2.plane = ROCKS_PLANE_ZX;
	segSpiralPars2.center[0] = center[0];
	segSpiralPars2.center[1] = center[1];
	segSpiralPars2.endPos[0] = OPT_DOOR_POINT_4[0];
	segSpiralPars2.endPos[1] = OPT_DOOR_POINT_4[2];
	segSpiralPars2.endAngleVelocity = 0;
	segSpiralPars2.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars2.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars2.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars2.originOffset.r.x = 0;
	segSpiralPars2.originOffset.r.y = 0;
	segSpiralPars2.originOffset.r.z = 0;
	segSpiralPars2.originOffset.t.x = 0;
	segSpiralPars2.originOffset.t.y = 0;
	segSpiralPars2.originOffset.t.z = 0;

	segSpiralPars3.plane = ROCKS_PLANE_ZX;
	segSpiralPars3.center[0] = center[0];
	segSpiralPars3.center[1] = center[1];
	segSpiralPars3.endPos[0] = OPT_DOOR_POINT_3[0];
	segSpiralPars3.endPos[1] = OPT_DOOR_POINT_3[2];
	segSpiralPars3.endAngleVelocity = angleSpeed;
	segSpiralPars3.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars3.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars3.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars3.originOffset.r.x = 0;
	segSpiralPars3.originOffset.r.y = 0;
	segSpiralPars3.originOffset.r.z = 0;
	segSpiralPars3.originOffset.t.x = 0;
	segSpiralPars3.originOffset.t.y = 0;
	segSpiralPars3.originOffset.t.z = 0;

	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = center[0];
	segArcPars2.center[1] = center[1];
	segArcPars2.endPos[0] = OPT_DOOR_POINT_2[0];
	segArcPars2.endPos[1] = OPT_DOOR_POINT_2[2];
	segArcPars2.endVelocity = DOOR_EX_SPEED;
	segArcPars2.maxAcceleration = DOOR_EX_ACC;
	segArcPars2.positiveAngle = FALSE;
	segArcPars2.originOffset.r.x = 0;
	segArcPars2.originOffset.r.y = 0;
	segArcPars2.originOffset.r.z = 0;
	segArcPars2.originOffset.t.x = 0;
	segArcPars2.originOffset.t.y = 0;
	segArcPars2.originOffset.t.z = 0;

	segSpiralPars4.plane = ROCKS_PLANE_ZX;
	segSpiralPars4.center[0] = center[0];
	segSpiralPars4.center[1] = center[1];
	segSpiralPars4.endPos[0] = OPT_DOOR_POINT_1[0];
	segSpiralPars4.endPos[1] = OPT_DOOR_POINT_1[2];
	segSpiralPars4.endAngleVelocity = 0;
	segSpiralPars4.maxAngleAcceleration = angleMaxAcc;
	segSpiralPars4.maxRadialVelocity = SPIRAL_MAX_RADIAL_SPEED;
	segSpiralPars4.maxRadialAcceleration = SPIRAL_MAX_RADIAL_ACC;
	segSpiralPars4.originOffset.r.x = 0;
	segSpiralPars4.originOffset.r.y = 0;
	segSpiralPars4.originOffset.r.z = 0;
	segSpiralPars4.originOffset.t.x = 0;
	segSpiralPars4.originOffset.t.y = 0;
	segSpiralPars4.originOffset.t.z = 0;

//	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajGetPath( &m_mech, &rocksTrajPath );

	while(nyceStatus == NYCE_OK)
	{
// 		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajLoadPath(&m_mech, &rocksTrajPath);
// 
// 		for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
// 		{
// 			kinPars.pJointPositionBuffer[ ax ] = NULL;
// 			kinPars.pJointVelocityBuffer[ ax ] = NULL;
// 		}

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech,&segStartPars);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars1);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars1);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars3);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech,&segArcPars2);
		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentSpiral(&m_mech,&segSpiralPars4);

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );	

		nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, SAC_INDEFINITE );
	}

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajDeletePath( &m_mech, &rocksTrajPath );

	return nyceStatus;
}


typedef struct doorTrajPars
{
	ROCKS_COORD startPos;
	ROCKS_COORD endPos;
	double riseHeight;
	double radius;
	TRAJ_PARS trajPars;
}DOOR_TRAJ_PARS;

/**
*	@author JoMar
*	@date 2016-01-22
*	@brief Control Delta-Robot to complete the door type path movement.
*	@param [in] doorPars - The informations of the door type path.
*	@param [in] timeout - The time limitation of waiting Delta-Robot to complete the path.
*	@return NYCE_STATUS
*/

static NYCE_STATUS RocksDoorDelta(const DOOR_TRAJ_PARS &doorPars, const double &timeout = SAC_INDEFINITE)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	//始末点水平距离
	const double distance(sqrt((doorPars.endPos.position.x - doorPars.startPos.position.x) * (doorPars.endPos.position.x - doorPars.startPos.position.x) + (doorPars.endPos.position.y - doorPars.startPos.position.y) * (doorPars.endPos.position.y - doorPars.startPos.position.y)));

	//速度比率
	const double velRatio1(doorPars.riseHeight / (distance / 2 + doorPars.riseHeight));
	const double velRatio2((-(doorPars.endPos.position.z - doorPars.startPos.position.z) +  doorPars.riseHeight) / (distance / 2 + (-(doorPars.endPos.position.z - doorPars.startPos.position.z) +  doorPars.riseHeight)));

	//始末点连线在水平面的投影方向
	const double angleZ(atan2(doorPars.endPos.position.y - doorPars.startPos.position.y, doorPars.endPos.position.x - doorPars.startPos.position.x));
	
	//获取机器人当前位置
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, segStartPars.startPos);

	//设置路径的起始位置和相关参数
	segStartPars.splineTime = doorPars.trajPars.splineTime;
	segStartPars.maxNrOfSplines = 0;
	segStartPars.pPositionSplineBuffer = NULL;
	segStartPars.pVelocitySplineBuffer = NULL;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentStart(&m_mech, &segStartPars);

	//设置上升段直线路径
	segLinePars1.plane = ROCKS_PLANE_ZX;
	segLinePars1.endPos[0] = doorPars.startPos.position.x;
	segLinePars1.endPos[1] = doorPars.startPos.position.z + doorPars.riseHeight - doorPars.radius;
	segLinePars1.endVelocity = doorPars.trajPars.velocity * velRatio1;
	segLinePars1.maxAcceleration = doorPars.trajPars.acceleration;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech, &segLinePars1);

	//设置转向路径
	segArcPars1.plane = ROCKS_PLANE_ZX;
	segArcPars1.center[0] = segLinePars1.endPos[0] + doorPars.radius;
	segArcPars1.center[1] = segLinePars1.endPos[1];
	segArcPars1.endPos[0] = segLinePars1.endPos[0] + doorPars.radius;
	segArcPars1.endPos[1] = segLinePars1.endPos[1] + doorPars.radius;
	segArcPars1.endVelocity = doorPars.trajPars.velocity * velRatio1;
	segArcPars1.maxAcceleration = doorPars.trajPars.acceleration;
	segArcPars1.positiveAngle = TRUE;//注意旋转方向
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech, &segArcPars1);

	//设置加速平移路径
	segLinePars2.plane = ROCKS_PLANE_ZX;
	segLinePars2.endPos[0] = segArcPars1.endPos[0] + distance * 0.5 - doorPars.radius;
	segLinePars2.endPos[1] = segArcPars1.endPos[1];
	segLinePars2.endVelocity = doorPars.trajPars.velocity;
	segLinePars2.maxAcceleration = doorPars.trajPars.acceleration;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech, &segLinePars2);

	//设置减速平移路径
	segLinePars3.plane = ROCKS_PLANE_ZX;
	segLinePars3.endPos[0] = segLinePars2.endPos[0] + distance * 0.5 - doorPars.radius;
	segLinePars3.endPos[1] = segLinePars2.endPos[1];
	segLinePars3.endVelocity = doorPars.trajPars.velocity * velRatio2;
	segLinePars3.maxAcceleration = doorPars.trajPars.acceleration;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech, &segLinePars3);
	
	//设置转向路径
	segArcPars2.plane = ROCKS_PLANE_ZX;
	segArcPars2.center[0] = segLinePars3.endPos[0];
	segArcPars2.center[1] = segLinePars3.endPos[1] - doorPars.radius;
	segArcPars2.endPos[0] = segLinePars3.endPos[0] + doorPars.radius;
	segArcPars2.endPos[1] = segLinePars3.endPos[1] - doorPars.radius;
	segArcPars2.endVelocity = doorPars.trajPars.velocity * velRatio2;
	segArcPars2.maxAcceleration = doorPars.trajPars.acceleration;
	segArcPars2.positiveAngle = TRUE;//注意旋转方向
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentArc(&m_mech, &segArcPars2);

	//设置下降路径
	segLinePars4.plane = ROCKS_PLANE_ZX;
	segLinePars4.endPos[0] = segArcPars2.endPos[0];
	segLinePars4.endPos[1] = doorPars.endPos.position.z;
	segLinePars4.endVelocity = 0;
	segLinePars4.maxAcceleration = doorPars.trajPars.acceleration;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksTrajSegmentLine(&m_mech, &segLinePars4);

	//设置整体偏移
	ROCKS_POSE pose;
	pose.r.x = 0;
	pose.r.y = 0;
	pose.r.z = -angleZ;
	pose.t.x = 0;
	pose.t.y = 0;
	pose.t.z = 0;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinMoveOrigin( &m_mech, &pose );

	ROCKS_KIN_INV_PARS kinPars;
	for (int ax = 0; ax < ROCKS_MECH_MAX_NR_OF_JOINTS; ++ax)
	{
		kinPars.pJointPositionBuffer[ ax ] = NULL;
		kinPars.pJointVelocityBuffer[ ax ] = NULL;
	}
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinInverseDelta( &m_mech, &kinPars );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStream( &m_mech );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksStreamSynchronize( &m_mech, timeout );

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinResetOrigin( &m_mech );

	return nyceStatus;
}

static NYCE_STATUS RocksSetHomePos(const ROCKS_COORD &rocksCoord)
{
	g_bInitHomePos = TRUE;
	g_homePos.type = KIN_COORD;
	ConvertTwoCoordinate(rocksCoord, g_homePos);

	return NYCE_OK;
}

static NYCE_STATUS RocksSetPlacePos(const ROCKS_COORD &rocksCoord)
{
	g_homePos.type = KIN_COORD;
	ConvertTwoCoordinate(rocksCoord, g_placePos);

	return NYCE_OK;
}

static NYCE_STATUS RocksHomeDelta(const TRAJ_PARS &trajPars)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (!g_bInitHomePos)
	{
		double joinPos[6];
		joinPos[0] = 0.0;
		joinPos[1] = 0.0;
		joinPos[2] = 0.0;

		double cartesianPos[6];
		ZeroMemory(cartesianPos, 6 * sizeof(double));
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksKinForwardDelta(&m_mech, joinPos, cartesianPos);

		ROCKS_COORD ptpPos;
		ptpPos.position.x = cartesianPos[0];
		ptpPos.position.y = cartesianPos[1];
		ptpPos.position.z = cartesianPos[2] + 80;
		ptpPos.type = KIN_COORD;

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksSetHomePos(ptpPos);
	}

	double curCarPos[6];
	ZeroMemory(curCarPos, ROCKS_MECH_MAX_DOF * sizeof(double));
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksKinDeltaPosition(&m_mech, curCarPos);

	if (curCarPos[0] - g_homePos.position.x >  1 ||
		curCarPos[1] - g_homePos.position.y >  1 ||
		curCarPos[2] - g_homePos.position.z >  1 ||
		curCarPos[0] - g_homePos.position.x < -1 ||
		curCarPos[1] - g_homePos.position.y < -1 ||
		curCarPos[2] - g_homePos.position.z < -1 )
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(g_homePos, trajPars);
	else
		return NYCE_OK;
	
	return nyceStatus;
}

static NYCE_STATUS RocksReadPosDelta(double *position)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : RocksKinDeltaPosition(&m_mech, position);

	return nyceStatus;
}

static NYCE_STATUS RocksInitMatrix()
{
	g_pTransfMatrix = new TRANSF_MATRIX[NUM_COORD_TYPES]();

	double x_base(0.0), y_belt(-13107200), x_camera(799.107), y_camera(726.886), z_kin(-876.43);
	g_pTransfMatrix[CAMERA_COORD].r.x	= 0;
	g_pTransfMatrix[CAMERA_COORD].r.y	= M_PI;
	g_pTransfMatrix[CAMERA_COORD].r.z	= -0.014813;
	g_pTransfMatrix[CAMERA_COORD].t.x	= x_camera * cos(g_pTransfMatrix[CAMERA_COORD].r.z) - y_camera * sin(g_pTransfMatrix[CAMERA_COORD].r.z);
	g_pTransfMatrix[CAMERA_COORD].t.y	= y_belt / BELT_BASE_RATE * PIXEL_BASE_RATE - x_camera * sin(g_pTransfMatrix[CAMERA_COORD].r.z) - y_camera * cos(g_pTransfMatrix[CAMERA_COORD].r.z);
	g_pTransfMatrix[CAMERA_COORD].t.z	= z_kin / KIN_BASE_RATE * PIXEL_BASE_RATE;
	g_pTransfMatrix[CAMERA_COORD].zoom	= PIXEL_BASE_RATE;

	g_pTransfMatrix[KIN_COORD].zoom = KIN_BASE_RATE;

	g_pTransfMatrix[BELT_COORD].zoom = BELT_BASE_RATE;

// 	ROCKS_COORD test_camera, test_blet, test_kin;
// 	test_camera.type = CAMERA_COORD;
// 	test_blet.type = BELT_COORD;
// 	test_kin.type = KIN_COORD;
// 	test_camera.position.x = x_camera;
// 	test_camera.position.y = y_camera;
// 	test_camera.position.z = 0;
// 	ConvertTwoCoordinate(test_camera, test_blet);
// 	ConvertTwoCoordinate(test_camera, test_kin);
// 	ConvertTwoCoordinate(test_kin, test_camera );
// 	ConvertTwoCoordinate(test_kin, test_camera);

	return NYCE_OK;
}

static NYCE_STATUS RocksTermMatrix()
{

//	timeKillEvent(g_wTimerID);

	delete[] g_pTransfMatrix;

	return NYCE_OK;
}

/**
*	@author JoMar
*	@date 2016-01-21
*	@brief This is a function calculating the position where Delta-Robot can catch up and pick up the target at.
*	@param [in] motionPars - The parameters of motion of the door trajectory 
*	@param [in] placeTargetPos_kin - The position in kinematic coordinate system of the target where the Delta-Robot place it on the belt.
*	@param [in] endBufHeightOffset - The height offset which use to keep some time to calculate and finish the second motion. It can improve the accuracy.
*	@param [out] meetingPos_kin - The position in kinematic coordinate system of the target where the Delta-Robot can catch it up.
*/
static NYCE_STATUS RocksCalcCatchPos(const TRAJ_PARS &motionPars, const ROCKS_COORD &placeRobotPos_kin, const ROCKS_COORD &placeTargetPos_kin, const double endBufHeightOffset, ROCKS_COORD &meetingPos_kin)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	//读取当前的皮带编码器位置和速度
	double encoderVel, encoderPos;
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : SacReadVariable(beltId[0], SAC_VAR_AXIS_VEL, &encoderVel);
	nyceStatus = NyceError( nyceStatus ) ? nyceStatus : SacReadVariable(beltId[0], SAC_VAR_AXIS_POS, &encoderPos);

	//目标在机构坐标系的当前位置
	double moveLen = (encoderPos - placeTargetPos_kin.cuEncoderValue) / BELT_BASE_RATE;//相对放置点而言，目标已经向前行了一段距离
	ROCKS_COORD cuTargetPos_kin;
	cuTargetPos_kin.type = KIN_COORD;
	cuTargetPos_kin.position.x = placeTargetPos_kin.position.x;
	cuTargetPos_kin.position.y = placeTargetPos_kin.position.y + moveLen * KIN_BASE_RATE;
	cuTargetPos_kin.position.z = placeTargetPos_kin.position.z + endBufHeightOffset * KIN_BASE_RATE;

	//传送带在皮带坐标系中的速度
	ROCKS_COORD beltVel_belt;
	beltVel_belt.type = BELT_COORD;
	beltVel_belt.position.x = 0;
	beltVel_belt.position.y = encoderVel;
	beltVel_belt.position.z = 0;

	//传送带在机构坐标系中的速度
	ROCKS_COORD beltVel_kin;
	beltVel_kin.type = KIN_COORD;
	ConvertTwoCoordinate(beltVel_belt, beltVel_kin);

	//计算由于时间残生的目标位置误差
	ROCKS_COORD threshold_kin;
	threshold_kin.type = KIN_COORD;
	threshold_kin.position.x = beltVel_kin.position.x * THRESHOLD_TIME;
	threshold_kin.position.y = beltVel_kin.position.y * THRESHOLD_TIME;
	threshold_kin.position.z = beltVel_kin.position.z * THRESHOLD_TIME;

	//方程参数简化
	double parX(placeRobotPos_kin.position.x - cuTargetPos_kin.position.x);
	double parY(placeRobotPos_kin.position.y - cuTargetPos_kin.position.y); 
	double parZ(placeRobotPos_kin.position.z - cuTargetPos_kin.position.z);


	//计算参数项
	double A(beltVel_kin.position.x * beltVel_kin.position.x + beltVel_kin.position.y * beltVel_kin.position.y - motionPars.velocity * motionPars.velocity / 4 );
	double B(-2 * (parX * beltVel_kin.position.x + parY * beltVel_kin.position.y + parZ * beltVel_kin.position.z));
	double C(parX * parX + parY * parY + parZ * parZ);

	//韦达定理计算时间
	double delta(B * B - 4 * A * C);
	if (delta < 0)
		return ROCKS_ERR_CALC_CATCH_POS_FAIL;

	double time, x1, x2;
	if (delta == 0)
	{
		x1 = (sqrt(delta) - B) / 2 / A;
		if (x1 < 0)
			return ROCKS_ERR_CALC_CATCH_POS_FAIL;
		else time = x1;
	}
	else
	{
		x1 = ( sqrt(delta) - B) / 2 / A;
		x2 = (-sqrt(delta) - B) / 2 / A;
		if (x1 <= 0 && x2 <= 0)
			return ROCKS_ERR_CALC_CATCH_POS_FAIL;
		else 
			time = max(x1, x2);
	}

	//计算相遇点
	meetingPos_kin.position.x = cuTargetPos_kin.position.x + beltVel_kin.position.x * time + threshold_kin.position.x;
	meetingPos_kin.position.y = cuTargetPos_kin.position.y + beltVel_kin.position.y * time + threshold_kin.position.y;
	meetingPos_kin.position.z = cuTargetPos_kin.position.z + beltVel_kin.position.z * time + threshold_kin.position.z;

	//这里添加判断是否在抓取区

	return nyceStatus;
}


//暂时使用
static NYCE_STATUS RocksSetTargetPos()
{

// 	for (uint32_t i = 0; i < NUM_TARGETS; ++i)
// 	{
// 		g_targetPos[i].type = BELT_COORD;
// 		g_targetPos[i].position.x = ;
// 		g_targetPos[i].position.y = ;
// 		g_targetPos[i].position.z = 0;
// 		g_targetPos[i].cuEncoderValue = 
// 	}

	return NYCE_OK;
}

static NYCE_STATUS RocksGetTargetPos(ROCKS_COORD &targetPos)
{
	NYCE_STATUS nyceSatus(NYCE_OK);

	double beltPos;
	nyceSatus = NyceError(nyceSatus) ? nyceSatus : SacReadVariable(beltId[0], SAC_VAR_AXIS_POS, &beltPos);

// 	//不能计算重复长度，很不精确
// 	int fac = beltPos;
// 	beltPos -= fac
// 	//过滤掉反面

	return nyceSatus;
}

/**
*	@author JoMar
*	@date 2016-01-29
*	@brief This is a function controlling the rotation motor.
*	@param [in] angle - The angle which the rotation motor move with. There is no limit to the range of this value.
*/
static NYCE_STATUS RocksRotateAngle(const double angle)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	//将角度取值为0~360度
	double correctAngle(angle - ((int)angle / 360) * 360.0);

	//将角度取值为-180~180度
	if (correctAngle >= 0)
		correctAngle = correctAngle <=  180.0 ? correctAngle : correctAngle - 360.0;
	else
		correctAngle = correctAngle >= -180.0 ? correctAngle : correctAngle + 360.0;

	const double pos(correctAngle * ROTATE_ANGLE_RATE);

	SAC_PTP_PARS ptpPos_rotation;
	ptpPos_rotation.positionReference = SAC_RELATIVE;
	ptpPos_rotation.position = pos;
	ptpPos_rotation.velocity = ROTATE_VEL * ROTATE_ANGLE_RATE;
	ptpPos_rotation.acceleration = ptpPos_rotation.velocity * 10;
	ptpPos_rotation.jerk = ptpPos_rotation.velocity * 100;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacPointToPoint(rotationId[0], &ptpPos_rotation);

	return nyceStatus;
}