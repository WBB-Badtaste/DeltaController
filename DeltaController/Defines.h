#pragma once

#include "sacapi.h"

#define NUM_AXES 3
const char *axName[ NUM_AXES ] = { "DEF_AXIS_1", "DEF_AXIS_2", "DEF_AXIS_3"};

SAC_AXIS axId[ NUM_AXES ];

#define NUM_NODE 1
const char *noName[ NUM_NODE ] = { "NY411x_node" };

NHI_NODE noId[ NUM_NODE ];