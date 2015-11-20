#pragma once

const uint32_t NUM_COORD_TYPES(6);

//坐标系类型
typedef enum CoordType
{
	WORLD_COORD		= NUM_COORD_TYPES - 6,
	KIN_COORD		= NUM_COORD_TYPES - 5,
	CAMERA_COORD	= NUM_COORD_TYPES - 4,
	BELT_COORD		= NUM_COORD_TYPES - 3,
	TARGET_COORD	= NUM_COORD_TYPES - 2,
	JOIN_COORD		= NUM_COORD_TYPES - 1
}COORD_TYPE;

//坐标系结构体
typedef struct RocksCoordinate
{
	ROCKS_E3_VECTOR postion;
	COORD_TYPE type;
}ROCKS_COORD;

//转换矩阵结构体
typedef struct  TransfMatrix
{
	ROCKS_E3_VECTOR t;
	ROCKS_E3_VECTOR r;
}TRANSF_MATRIX;