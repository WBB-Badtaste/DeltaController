#pragma once

static const uint32_t NUM_COORD_TYPES(6);

//坐标系类型
typedef enum CoordType
{
	BASE_COORD		= NUM_COORD_TYPES - 6,
	KIN_COORD		= NUM_COORD_TYPES - 5,
	CAMERA_COORD	= NUM_COORD_TYPES - 4,
	BELT_COORD		= NUM_COORD_TYPES - 3,
	TARGET_COORD	= NUM_COORD_TYPES - 2,
	JOIN_COORD		= NUM_COORD_TYPES - 1//注意转换过程
}COORD_TYPE;

//坐标系结构体
typedef struct RocksCoordinate
{
	ROCKS_E3_VECTOR position;//实际位置
	COORD_TYPE type;//坐标系类型
	double cuEncoderValue;//编码器值，用在抓取时匹配编码器位置
}ROCKS_COORD;

//转换矩阵结构体
typedef struct TransfMatrix
{
	ROCKS_E3_VECTOR t;//平移矩阵
	ROCKS_E3_VECTOR r;//旋转矩阵
	double zoom;//缩放比例
//	ROCKS_E3_VECTOR c;
}TRANSF_MATRIX;