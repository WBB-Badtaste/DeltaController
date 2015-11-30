#pragma once

#include "mmsystem.h"  
#include "rockstypes.h"
#include "CoordinateStructs.h"
#include "sacapi.h"
#include "nhiapi.h"

#define NUM_AXES 3
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_3", "DEF_AXIS_2"};
SAC_AXIS axId[ NUM_AXES ];

#define NUM_AXES_BELT 1
const char *beltaName[ NUM_AXES_BELT ] = {"DEF_AXIS_4"};
SAC_AXIS beltId[ NUM_AXES_BELT ];

#define NUM_NODE 1
const char *noName[ NUM_NODE ] = { "NY411x_node" };
NHI_NODE noId[ NUM_NODE ];

//皮带相关
#define NUM_BELTPOS_QUE 20
double g_beltPos[NUM_BELTPOS_QUE], g_beltVel(0.0);
uint32_t g_beltPos_index(0);
uint32_t g_readBeltPos_delayTime(500);
MMRESULT g_wTimerID(0);
BOOL g_readingBeltPos(FALSE);

double g_beltLenght;
double g_encoderMinRange;
double g_encoderMaxRange;

//坐标系相关
TRANSF_MATRIX *g_pTransfMatrix;

//特殊坐标 
ROCKS_COORD g_homePos;
BOOL g_bInitHomePos;
ROCKS_COORD g_placePos;
ROCKS_COORD g_pickPos;