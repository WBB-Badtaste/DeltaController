#pragma once

#include "sacapi.h"
#include "nhiapi.h"

#define NUM_AXES 4
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_3", "DEF_AXIS_2", "DEF_AXIS_4" };

SAC_AXIS axId[ NUM_AXES ];

#define NUM_NODE 1
const char *noName[ NUM_NODE ] = { "NY411x_node" };

NHI_NODE noId[ NUM_NODE ];


//皮带位置和速度
#define NUM_BELTPOS_QUE 20
double g_beltPos[NUM_BELTPOS_QUE], g_beltVel(0.0);
uint32_t g_beltPos_index(0);
uint32_t g_readBeltPos_delayTime(500);