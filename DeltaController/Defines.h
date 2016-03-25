#pragma once

#include "mmsystem.h"  
#include "rockstypes.h"
#include "CoordinateStructs.h"
#include "sacapi.h"
#include "nhiapi.h"


//位姿轴
#define NUM_AXES 3
static const char *axName[ NUM_AXES ] = { "DEF_AXIS_3", "DEF_AXIS_1", "DEF_AXIS_2"};
static SAC_AXIS axId[ NUM_AXES ];

//旋转轴
#define NUM_AXES_ROTATION 1
static const char *rotationName[ NUM_AXES_ROTATION ] = {"DEF_AXIS_4"};
static SAC_AXIS rotationId[ NUM_AXES_ROTATION ];

//皮带轴
#define NUM_AXES_BELT 1
static const char *beltaName[ NUM_AXES_BELT ] = {"DEF_AXIS_5"};
static SAC_AXIS beltId[ NUM_AXES_BELT ];

//节点
#define NUM_NODE 1
static const char *noName[ NUM_NODE ] = { "NY411x_node" };
static NHI_NODE noId[ NUM_NODE ];

//皮带轴
#define BELT_VEL 200 //传送带速度，单位mm/s
#define BELT_BASE_RATE 15518.2130329563124//15548.8947763849895 //实际移动长度与编码器值的比值
static double THRESHOLD_TIME = 0.085; //计算时间阈值，单位S
// double g_encoderMinRange;
// double g_encoderMaxRange;

//旋转轴
#define ROTATE_ANGLE_RATE (131072 / 360)
#define ROTATE_VEL M_PI

//机构位移调整参数,实际长度与机构行程的比值
#define KIN_BASE_RATE 0.9737

//坐标系相关
static TRANSF_MATRIX *g_pTransfMatrix;

//相机相关
#define PIXEL_BASE_RATE 3.1266775702479

//特殊坐标 
static BOOL		g_bInitHomePos;
static ROCKS_COORD g_homePos;
static ROCKS_COORD g_placePos;
static ROCKS_COORD g_pickPos;

//门型运动参数
#define DOOR_HIGHT1 -835
#define DOOR_HIGHT2 -835
#define DOOR_PAR_X 152.5
#define DOOR_PAR_Y 0

#define READ_POS_DELAY 500//循环读取机器人位置的时间间隔

//回零运动参数
#define HOME_VEL 500

//Jog参数
#define JOG_VEL 50

#define WM_MATCH_MODEL			WM_USER + 1
#define WM_HANDLE_NYCESTATUS	WM_USER + 2
#define WM_UPDATE_ROBOT_POS		WM_USER + 3