#ifndef __ZQCW_VISION_2014_09_28_H__
#define __ZQCW_VISION_2014_09_28_H__
 
//#include "stdafx.h"
#include "resource.h"
#include <afxwin.h>
#include "ImageDib.h"
#include <vector>


#define DEGBUG_WZQ

/************************************************************
// 百泰源信息技术有限公司 版权所有
// 模块名称：cwvision.dll
// 当前版本：1.0
// 创建时间：2014-9-28
// 修改时间：2015-4-20

说明：
*************************************************************/

// #ifdef _EXPORTS
// #define  CWVISIONAPI extern "C" __declspec(dllexport)
// #else
// #define CWVISIONAPI extern "C" __declspec(dllimport)
// #endif

#define CWVISIONAPI __declspec(dllimport)

//打开程序
//功能:打开程序
//打开程序成功返回句柄
//	HANDLE cwOpenProgram();

#define PI 3.1415926535897932384626433832795028

typedef enum _cwColor{
	COLOR_RED = 0,
	COLOR_GREEN,
	COLOR_YELLOW
}CCOLOR;

// 浮点数坐标点的数据结构
typedef struct _cwDPOINT{
	double	x;			// x方向
	double	y;			// y方向
	_cwDPOINT(double x0=0,double y0=0):x(x0),y(y0){}
	void SetVal(double x0,double y0){
		x = x0;
		y = y0;
	}
} FPOINT;

// 整数矩形的数据结构
typedef struct _cwIRECT{
	int		left;		// 左边
	int		top;		// 上边
	int		width;		// 宽度
	int		height;		// 高度
} IRECT;

// 带旋转的整数矩形的数据结构
typedef struct _cwDRRECT{
	double	centerX;		// 中心x
	double	centerY;		// 中心y
	double	xLength;		// x方向长度(矩形宽度的一半)
	double	yLength;		// y方向长度(矩形高度的一半)
	double  phi;          //旋转角度(弧度)
	_cwDRRECT(double x=0,double y=0,double ag=0,double xl=1,double yl=1):
	centerX(x),centerY(y),phi(ag),xLength(xl),yLength(yl){}
	void SetData(double x=0,double y=0,double ag=0,double xl=1,double yl=1){
		centerX = x;
		centerY = y;
		phi = ag;
		xLength = xl;
		yLength = yl;
	}
} FRRECT;

// 浮点数矩形的数据结构
// typedef struct _cwFRECT{
// 	float	left;		// 左边
// 	float	top;		// 上边
// 	float	width;		// 宽度
// 	float	height;		// 高度
// } FRECT;

// 圆形的数据结构
// typedef struct _cwCIRCLE{
// 	float	cx;			// 中心x
// 	float	cy;			// 中心y
// 	float	r;		// 半径
// } CIRCLE;
//圆的标准方程参数 (x-cx)^2 + (y-cy)^2 = r^2
typedef struct _cwCircle{
	double cx;
	double cy;
	double r;
	_cwCircle(double x=0,double y=0,double radius=0):cx(x),cy(y),r(radius){}
}CIRCLE;

// 标准直线的数据结构参数 ax+by+c=0
typedef struct _cwLine{
	double a;
	double b;
	double c;
	FPOINT start; //线的起点
	FPOINT end;   //线的终点
	_cwLine(double x = 0,double y = 0,double z = 0):a(x),b(y),c(z){
		start.SetVal(0,0);
		end.SetVal(0,0);
	}
} LINE;


typedef enum _cwShap{
	SHAP_NULL = -1,
	SHAP_CIRCLE = 0,      //圆
		SHAP_SQUARE,      //正方形
		SHAP_OTHER,       //其他图形
		SHAP_RECTANGLE,   //矩形
		SHAP_TRIANGLE,    //三角形
		SHAP_LINE,        //线形
		SHAP_CIRCULAR,    //带卡尺的圆
		SHAP_CORNER,      //角点
		SHAP_SRECTANGLE,   //带拟合的矩形
		SHAP_GREY         //灰度匹配
}SHAP;

// ROI类型
typedef enum roi_tpye{
	ROI_NULL=0	,               //NULL
		ROI_CIRCLE,             //圆形ROI
		ROI_LINE,               //线形ROI
		ROI_FSQUARE,            //正方形ROI
		ROI_FRECT,              //矩形ROI(可旋转)
		//ROI_RFRECT,             //可旋转矩形ROI
		//ROI_ANNULUS,          // 圆环形ROI
		//ROI_CIRCULAR,           //带卡尺的圆形ROI
		ROI_CIRCLE_CIRCLE,      //圆圆组合模板
		ROI_CIRCLE_LINE,        //圆线组合模板
		ROI_CORNER,             //角点
		ROI_RECT_FIT            //拟合矩形
}ROI_TYPE;

//模板的数据结构
typedef struct _cwModel MODEL;
SHAP GetModShap(const MODEL* mod);

// ROI句柄
typedef struct _cwROI ROI, *HROI;

// cwFindModel - 搜索模板参数选项
typedef struct _cwFINDOPTION {
	int		minScore;		// 搜索匹配对象的最小分数(0~1000)；
	int		minAngle;		// 最小角度（-180~180）；
	int		maxAngle;		// 最大角度（-180~180）；
	int		minScale;		// 最小比例（按照百分比，100为1倍）；
	int		maxScale;		// 最大比例（按照百分比，100为1倍）；
	int		needEdge_or_level;		// 最短的边缘，用以限制那些长度小于该值的边缘，以便减少噪音；
							// 灰度模板为查找等级
	int		threshold_or_maxError;		// 边缘梯度阈值，只有那些梯度大于该阈值的边缘点才被检测出来；
							// 灰度模板为容许的最大匹配偏差
	int		findCount;		// 需要搜索的个数；
	_cwFINDOPTION(int score=600,int minA=-10,int maxA=10,int minS=0.8,int maxS=1.2,int edgeTh=30,int count=0):minScore(score),
		minAngle(minA),maxAngle(maxA),minScale(minS),maxScale(maxS),threshold_or_maxError(edgeTh),findCount(count){}
} FINDOPTION, *PFINDOPTION;

// cwFindModel - 搜索模板返回数据
typedef struct _cwMATCH 
{
	double	score;			// 相似度；
	double	angle;			// 角度；
	double	scale;			// 比例；
	FRRECT	rectBox;		// 位置框；
	FPOINT	position;		// 匹配的位置；
} MATCH, *PMATCH;

// cwLearnMode - 学习模板参数选项
typedef struct _cwLEARNOPTION 
{
	int		minScore;		// 搜索匹配对象的最小分数(0~1000)；
	int		minAngle;		// 最小角度（-180~180）；
	int		maxAngle;		// 最大角度（-180~180）；
	int		minScale;		// 最小比例（按照百分比，100为1倍）；
	int		maxScale;		// 最大比例（按照百分比，100为1倍）；
	int     edgeThreashold; // 最小边缘阈值
	int		color;          // 模板颜色0为黑色 1为白色
	int     polarity;       // 极性 0为暗到明 1为明到暗 2为任意极性
	double  intersectDiff;  // 角点夹角的角度的正负偏差容忍单位为度
	_cwLEARNOPTION(int score=600,int minA=-10,int maxA=10,int minS=0.8,int maxS=1.2,int edgeTh=30,int cola=0
		,int polar=0,double interDiff = 5):minScore(score),minAngle(minA),maxAngle(maxA),minScale(minS),
		maxScale(maxS),edgeThreashold(edgeTh),color(cola),polarity(polar),intersectDiff(interDiff){}
	_cwLEARNOPTION(const struct _cwLEARNOPTION* option):minScore(option->minScore),minAngle(option->minAngle),
		maxAngle(option->maxAngle),minScale(option->minScale),maxScale(option->maxScale),edgeThreashold(option->edgeThreashold),
		color(option->color),polarity(option->polarity),intersectDiff(option->intersectDiff){}
} LEARNOPTION, *PLEARNOPTION;


typedef enum{
	GREY_TYPE_original = 0, 
		GREY_TYPE_normalized = 1, 
		GREY_TYPE_gradient = 2, 
		GREY_TYPE_sobel = 3
}GREY_VALUE_TYPE;

// cwLearnMode - 学习灰度模板参数选项
typedef struct _cwLEARNOPTIONGREY 
{
	int			findLevel;		//查找等级（1-5等级越高速度越快）
	double		minAngle;		// 最小角度（-180~180）；
	double		maxAngle;		// 最大角度（-180~180）；
	double		angleStep;      //搜索角度的跨度（0~180）；
	bool		bOptimize;		//是否进行优化（创建优化模板的时间会长点，随之带来的是匹配更准确）
	GREY_VALUE_TYPE greyType;   //灰度值类型
	_cwLEARNOPTIONGREY(int level=5,double minA=-10.0,double maxA=10.0,double step=1.0,bool optimize=true,
		GREY_VALUE_TYPE type=GREY_TYPE_original):findLevel(level),minAngle(minA),maxAngle(maxA),
		angleStep(step),bOptimize(optimize),greyType(type){}
	_cwLEARNOPTIONGREY(const struct _cwLEARNOPTIONGREY* option):findLevel(option->findLevel),minAngle(option->minAngle),
		maxAngle(option->maxAngle),angleStep(option->angleStep),bOptimize(option->bOptimize),greyType(option->greyType){}
} LEARNOPTIONGREY, *PLEARNOPTIONGREY;


// 显示图形用的数据结构 display 
typedef struct _cwOFFSETZOOM
{
	double		x;		// 显示图形的坐标原点x
	double		y;		// 显示图形的坐标原点y
	double	scale;	// 显示图形时的缩放倍数
	_cwOFFSETZOOM(double tx=0,double ty=0,double sc = 1):x(tx),y(ty),scale(sc){}
} OFFSETZOOM;

/************************************************************************/
/* 功能: 设置是否翻转图像的y方向 (默认为翻转)
/************************************************************************/
CWVISIONAPI void cwSetReversePara(bool value=true);

/************************************************************************/
/* 功能: 释放一个模板指针 
/************************************************************************/
CWVISIONAPI void cwFreeModel( MODEL* pModel ); 

/************************************************************************/
/* 功能: 释放一个有效的ROI指针
/* 参数: pRoi 一个指向ROI类型的指针，需要释放的ROI
/************************************************************************/

CWVISIONAPI void cwFreeRoi( ROI* pRoi );
/************************************************************************/
/* 功能: 释放一个指针的内存
/* 参数: pMemory 指向需要释放的内存块首地址	
/************************************************************************/
CWVISIONAPI void cwFreeMemory( void* pMemory );

/************************************************************************/
/* 功能: 释放一张以被创建好的图像的内存
/* 参数: pImage 需要释放内存的图像指针	
/************************************************************************/
CWVISIONAPI void cwFreeImage( IMAGE* pImage );

/************************************************************************/
/* 功能: 从文件读模板 
/************************************************************************/
CWVISIONAPI MODEL* cwLoadModel( LPCSTR pszFileName );

/************************************************************************/
/* 功能: 从文件读模板 
/* 参数: pszProductName产品名（在产品文件夹下新建的文件夹）
/* 参数: cameraIndex相机索引编号
/* 参数: modSeq模板序号
/* 参数: isAttach是否附加模板（默认为false），即附加模板在模板序号的基础上+模板偏移值
/* 参数: seqOffset为设定的附加模板偏移量（默认为1000）
/************************************************************************/
CWVISIONAPI MODEL* cwLoadModelP(LPCSTR pszProductName,int cameraIndex,int modSeq,bool isAttach=false ,int seqOffset=1000);

/************************************************************************/
// 功能: 从文件中读取一张BMP图片
// 参数: pszFileName 图片在文件中的全路径名
// 返回: 成功返回一个有效的图像数据指针，否则返回NULL
/************************************************************************/
CWVISIONAPI IMAGE* cwReadBMPFile( LPCSTR pszFileName );

/************************************************************************/
/* 功能: 以BMP格式保存一张图片到文件中去
/* 参数: pszFileName 保存图片的全路径名
/* 参数: pImage 需要保存的图像数据指针
/* 参数: writeWithoutWarn文件存在是否检测存在并提示覆盖（默认为false）
/* 返回: 成功返回true，否则返回false
/************************************************************************/
CWVISIONAPI bool cwWriteBMPFile( LPCSTR pszFileName, IMAGE* pImage ,bool writeWithoutWarm = false);

/************************************************************************/
/* 功能: 保存模板到文件 
/* 参数: writeWithoutWarn文件存在是否检测存在并提示覆盖（默认为false）
/************************************************************************/
CWVISIONAPI bool cwSaveModel( LPCSTR pszFileName, const MODEL* pModel ,bool writeWithoutWarm = false); 

/************************************************************************/
/* 功能: 从文件删除模板 
/************************************************************************/
CWVISIONAPI bool cwDeleteModel( LPCSTR pszFileName); 

/************************************************************************/
/* 功能: 保存模板到文件 
/* 参数: pModel需要保存的模板指针
/* 参数: pszProductName产品名（在产品文件夹下新建的文件夹）
/* 参数: cameraIndex相机索引编号
/* 参数: modSeq模板序号
/* 参数: isAttach是否附加模板（默认为false），即附加模板在模板序号的基础上+模板偏移值
/* 参数: seqOffset为设定的附加模板偏移量（默认为1000）
/* 参数: writeWithoutWarn文件存在是否检测存在并提示覆盖（默认为false）
/************************************************************************/
CWVISIONAPI bool cwSaveModelP( const MODEL* pModel, LPCSTR pszProductName,int cameraIndex,int modSeq,
				 bool isAttach=false ,int seqOffset=1000,bool writeWithoutWarm = false);

/************************************************************************/
/* 功能: 创建一副位图（目前仅支持8位） 
/************************************************************************/
CWVISIONAPI IMAGE* cwCreateImage( BYTE* pData, int BitCount, int Width, int Height );

/************************************************************************/
/* 功能: 更新图像的数据，将pData的数据复制到pImage中去
/* 参数: pImage 目标图象，需要更新数据的图象指针		
/* 参数: pBits 新的图像数据缓冲区的首地址指针，该图像的位数和尺寸必须和目标图象一样
/************************************************************************/
CWVISIONAPI void cwSetImageBits( IMAGE* pImage, const BYTE* pBits, DWORD dPrat );

/************************************************************************/
// 功能: 学习一个外形模板
// 参数: pImage 用来学习的图像数据（目前只支持8位灰度图像）
// 参数: pRoi 学习的roi
// 参数: shap 学习的模板类型
// 参数: Option 相关的参数，参见LEARNOPTION
// 参数: showEditModelDlg为是否显示编辑模板对话框（默认为不显示）
// 参数: searchRegion为搜索区域为NULL表示搜索全图（默认为NULL）
// 返回: 成功返回一个模板句柄，否则返回NULL
/************************************************************************/
CWVISIONAPI MODEL* cwLearnModel(const IMAGE* pImage, ROI* pRoi,const OFFSETZOOM* pZoom,SHAP shap, const LEARNOPTION* option,
								bool showEditModelDlg=false ,FRRECT* searchRegion = NULL);

/************************************************************************/
// 功能: 学习一个灰度模板
// 参数: pImage 用来学习的图像数据（目前只支持8位灰度图像）
// 参数: pRoi 学习的roi
// 参数: shap 学习的模板类型
// 参数: Option 相关的参数，参见LEARNGREYOPTION
// 参数: showEditModelDlg为是否显示编辑模板对话框（默认为不显示）
// 参数: searchRegion为搜索区域为NULL表示搜索全图（默认为NULL）
// 返回: 成功返回一个模板句柄，否则返回NULL
/************************************************************************/
CWVISIONAPI MODEL* cwLearnModelGrey(const IMAGE* pImage, ROI* pRoi,const OFFSETZOOM* pZoom, const LEARNOPTIONGREY* option,
								bool showEditModelDlg = false , FRRECT* searchRegion = NULL);

/************************************************************************/
/* 功能: 在一张图像上搜索跟模板相似的目标(用于匹配任意形状,有扩缩无旋转)*/
/* 参数: pImage 输入的图像数据，支持8位灰度图像                         */
/* 参数: pRect 搜索范围，NULL为学习模板时保存的搜索区域                 */
/* 参数: hModel 模板句柄                                                */
/* 参数: Option 搜索的相关参数，参见FINDOPTION                          */
/* 参数: retCount 返回搜索到的匹配对象的个数                            */
/* 参数: 成功返回所有搜索到的目标的数据，否则返回NULL                   */
/************************************************************************/
CWVISIONAPI MATCH* cwFindModel( const IMAGE* pImage, const FRRECT* pRect, const MODEL* pModel, const FINDOPTION* Option, 
							   int* retCount ,double angle = 0,double cx = 320,double cy = 240 );

CWVISIONAPI LINE* cwFindLine(const IMAGE* pImage,ROI* roi,const OFFSETZOOM* pZoom);
CWVISIONAPI LINE* cwFindLine(const IMAGE* pImage,FRRECT* rect);

//CWVISIONAPI FPOINT* cwFindCorner(const IMAGE* pImage,ROI* roi,const OFFSETZOOM* pZoom);

//用于显示 训练框的函数

/************************************************************************/
/* 功能: 创建一个ROI
/* 参数: Type 需要创建的ROI类型，参见ROI类型的宏定义
/* 返回: 成功返回一个ROI数据指针，否则返回NULL
/************************************************************************/
CWVISIONAPI ROI* cwCreateRoi(int Type);

 /************************************************************************/
 /* 功能: 获取ROI类型
 /* 返回: 返回ROI的类型
 /************************************************************************/
CWVISIONAPI ROI_TYPE cwGetROIType(ROI*roi);

/************************************************************************/
/* 功能: 获取模板类型
/* 返回: 返回模板的类型
/************************************************************************/
CWVISIONAPI SHAP cwGetModelShap(MODEL*mod);

/************************************************************************/
/* 功能: 判断当前的鼠标位置点是否在ROI的调节点内，并获取调节点对应的值
/* 参数: pRoi 一个指向ROI类型的指针，用来操作一个ROI
/* 参数: pZoom为控件左上角坐标的偏移值和扩缩比，外部已转换则不需要可设为NULL
/* 参数: y 当前鼠标点的x轴坐标
/* 参数: x 当前鼠标点的y轴坐标
/* 参数: record是否需要记录当前位置信息（默认为false）鼠标按下时需要设置该值为true
/* 返回: 成功返回对应调节点相应的值，每个调节点都有不同的值，否则返回0
//矩形调节点从上到下依次为1 2 3  4 5 6   7 8 9
//圆形调节点从上到下依次为1  2 3 4  5
/************************************************************************/
CWVISIONAPI int cwGetRoiNode(const ROI* pRoi, const OFFSETZOOM* pZoom, int x,int y ,bool record=false);

/************************************************************************/
/* 功能: 根据调节点的值来对ROI相应的位置进行调整
/* 参数: pRoi 一个指向ROI类型的指针，用来操作一个ROI
/* 参数: pZoom为控件左上角坐标的偏移值和扩缩比，外部已转换则不需要可设为NULL
/* 参数: Node 调节点的值
/* 参数: y 当前鼠标点的x轴坐标
/* 参数: x 当前鼠标点的y轴坐标
/* 返回: 成功返回true，否则返回false
/************************************************************************/
CWVISIONAPI bool cwAdjustRoi( ROI* pRoi, const OFFSETZOOM* pZoom, int Node, int x, int y );

/************************************************************************/
/* 功能: 根据ROI的类型和调节点的值来设置当前的光标类型（即鼠标箭头显示类型）
/* 参数: pRoi 一个指向ROI类型的指针
/* 参数: Node 调节点的值
/* 返回: 成功返回true，否则返回false
/************************************************************************/
CWVISIONAPI bool cwRoiCursor( const ROI* pRoi, int Node );

/************************************************************************/
/* 功能: 根据两个坐标点来画一个ROI，类型不同的ROI产生的效果不同
/* 参数: pRoi 一个指向ROI类型的指针，用来操作一个ROI
/* 参数: pZoom为控件左上角坐标的偏移值和扩缩比，外部已转换则不需要可设为NULL
/* 参数: x1 第一个点的x轴坐标
/* 参数: y1 第一个点的y轴坐标
/* 参数: x2 第二个点的x轴坐标
/* 参数: y2 第二个点的y轴坐标
/* 返回: 成功返回一个ROI数据指针，否则返回NULL
/************************************************************************/
CWVISIONAPI bool cwSetRoi( ROI* pRoi, const OFFSETZOOM* pZoom, int x1, int y1, int x2, int y2);

/************************************************************************/
/* 功能: 在屏幕上显示一个ROI
/* 参数: showWnd 显示设备的绘图窗口句柄
/* 参数: pRoi 需要显示的ROI数据
/* 参数: Color 显示ROI时使用的颜色
/* 参数: clearFormer为是否清除已有绘制痕迹(默认为清除)
/* 参数: showImageWidth为图像的实际显示宽度 默认为640
/* 参数: showImageHeigth为图像的实际显示高度 默认为480
/************************************************************************/
CWVISIONAPI void cwDisplayRoi(HWND showWnd, const OFFSETZOOM* pZoom, const ROI* pRoi, CCOLOR Color,bool clearFormer = true,
							  int showImageWidth=640,int showImageHeigth=480);

/************************************************************************/
/* 功能: 在屏幕上显示一个ROI的所有调节点
/* 参数: showWnd 显示设备的绘图窗口句柄
/* 参数: pRoi 需要显示调节点的ROI数据
/* 参数: Color 显示ROI调节点时使用的颜色
/* 参数: showImageWidth为图像的实际显示宽度 默认为640
/* 参数: showImageHeigth为图像的实际显示高度 默认为480
/************************************************************************/
CWVISIONAPI void cwDisplayRoiNode(HWND showWnd, const OFFSETZOOM* pZoom, const ROI* pRoi, CCOLOR Color ,
								  int showImageWidth=640,int showImageHeigth=480);


/************************************************************************/
/* 功能: 将ROI转换为一个FRRECT型指针
/* 参数: pRoi 需要转换的ROI数据，该ROI的类型必须为ROI_RRECT
/* 返回: 成功返回一个指向该ROI内部的数据FRRECT型指针，否则返回NULL
/************************************************************************/
CWVISIONAPI FRRECT* cwGetFRRect( ROI* pRoi );

/************************************************************************/
/* 功能: 将ROI转换为4个FRRECT型指针 依次为上右下左
/* 参数: pRoi 需要转换的ROI数据，该ROI的类型必须为ROI_SRECT
/* 返回: 成功返回一个指向该ROI内部的数据FRRECT型指针，否则返回NULL
/************************************************************************/
CWVISIONAPI bool cwGetFRRects( ROI* pRoi ,FRRECT rect[4]);

/************************************************************************/
/* 功能: 将ROI转换为一个ROUND型指针
/* 参数: pRoi 需要转换的ROI数据，该ROI的类型必须为ROI_ROUND
/* 返回: 成功返回一个指向该ROI内部的数据ROUND型指针，否则返回NULL
/************************************************************************/
CWVISIONAPI CIRCLE* cwGetRound( ROI* pRoi );

/************************************************************************/
/* 功能: 在屏幕上显示一张图像
/* 参数: showWnd 显示设备的绘图窗口句柄
/* 参数: pImage 需要显示的图像数据指针
/* 参数: pZoom 使用该参数可以进行平移和缩放显示物体，不需要可设为NULL 默认为NULL
/************************************************************************/
CWVISIONAPI void cwDisplayImage( HWND showWnd, const IMAGE* pImage , const OFFSETZOOM* pZoom = NULL);

/************************************************************************/
/* 功能: 在屏幕上显示最近一次匹配结果
/* 参数: showWnd 显示设备的绘图窗口句柄
/* 参数: pZoom 使用该参数可以进行平移和缩放显示物体 不需要可设为NULL 默认为NULL
/* 参数: showImageWidth为图像的实际显示宽度 默认为640
/* 参数: showImageHeigth为图像的实际显示高度 默认为480
/************************************************************************/
CWVISIONAPI int cwDisplayLastMatchFeature( HWND showWnd,OFFSETZOOM* pZoom=NULL,int showImageWidth=640,int showImageHeigth=480, bool bShowResult = true);
CWVISIONAPI void cwSetDisplayMatchFeature(bool disp);//设置是否显示匹配结果
CWVISIONAPI void cwResultRectSeqSet(int seq);//设置匹配结果的序号
CWVISIONAPI void cwResultRectSeqIncrease();//增加匹配结果的序号
CWVISIONAPI void cwResultRectSeqDecrease();//减少匹配结果的序号
CWVISIONAPI void cwResultRectSeqReset();//重置匹配结果的序号

//用于调整模板的函数
CWVISIONAPI void DisableDisplay(bool vale);
CWVISIONAPI int ReLearnModel(MODEL* model,LEARNOPTION* learnOption); 
CWVISIONAPI int ReLearnModelGrey(MODEL* model,LEARNOPTIONGREY* learnOption);
CWVISIONAPI bool LearnModelAnyA(MODEL* model,const FRRECT* pFRRect, const LEARNOPTION* learnOption );
CWVISIONAPI bool LearnModelGreyA(MODEL* pModel,const FRRECT* pFRRect, const LEARNOPTIONGREY* learnOption );
//void FreeModelPart( MODEL* pModel );
CWVISIONAPI void DisplayRect(FRRECT* pFRRect);
CWVISIONAPI void InitLearnRect(const FRRECT* pFRRect);
CWVISIONAPI void SubLearnRect(FRRECT* pFRRect);
CWVISIONAPI void cwSetROIFlush(bool value);//

//标定
CWVISIONAPI void cwCalibration(IMAGE* pImage);

#endif