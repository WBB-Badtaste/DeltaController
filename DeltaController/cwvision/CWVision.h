#ifndef __ZQCW_VISION_2014_09_28_H__
#define __ZQCW_VISION_2014_09_28_H__
 
//#include "stdafx.h"
#include "resource.h"
#include <afxwin.h>
#include "ImageDib.h"
#include <vector>


#define DEGBUG_WZQ

/************************************************************
// ��̩Դ��Ϣ�������޹�˾ ��Ȩ����
// ģ�����ƣ�cwvision.dll
// ��ǰ�汾��1.0
// ����ʱ�䣺2014-9-28
// �޸�ʱ�䣺2015-4-20

˵����
*************************************************************/

// #ifdef _EXPORTS
// #define  CWVISIONAPI extern "C" __declspec(dllexport)
// #else
// #define CWVISIONAPI extern "C" __declspec(dllimport)
// #endif

#define CWVISIONAPI __declspec(dllimport)

//�򿪳���
//����:�򿪳���
//�򿪳���ɹ����ؾ��
//	HANDLE cwOpenProgram();

#define PI 3.1415926535897932384626433832795028

typedef enum _cwColor{
	COLOR_RED = 0,
	COLOR_GREEN,
	COLOR_YELLOW
}CCOLOR;

// ���������������ݽṹ
typedef struct _cwDPOINT{
	double	x;			// x����
	double	y;			// y����
	_cwDPOINT(double x0=0,double y0=0):x(x0),y(y0){}
	void SetVal(double x0,double y0){
		x = x0;
		y = y0;
	}
} FPOINT;

// �������ε����ݽṹ
typedef struct _cwIRECT{
	int		left;		// ���
	int		top;		// �ϱ�
	int		width;		// ���
	int		height;		// �߶�
} IRECT;

// ����ת���������ε����ݽṹ
typedef struct _cwDRRECT{
	double	centerX;		// ����x
	double	centerY;		// ����y
	double	xLength;		// x���򳤶�(���ο�ȵ�һ��)
	double	yLength;		// y���򳤶�(���θ߶ȵ�һ��)
	double  phi;          //��ת�Ƕ�(����)
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

// ���������ε����ݽṹ
// typedef struct _cwFRECT{
// 	float	left;		// ���
// 	float	top;		// �ϱ�
// 	float	width;		// ���
// 	float	height;		// �߶�
// } FRECT;

// Բ�ε����ݽṹ
// typedef struct _cwCIRCLE{
// 	float	cx;			// ����x
// 	float	cy;			// ����y
// 	float	r;		// �뾶
// } CIRCLE;
//Բ�ı�׼���̲��� (x-cx)^2 + (y-cy)^2 = r^2
typedef struct _cwCircle{
	double cx;
	double cy;
	double r;
	_cwCircle(double x=0,double y=0,double radius=0):cx(x),cy(y),r(radius){}
}CIRCLE;

// ��׼ֱ�ߵ����ݽṹ���� ax+by+c=0
typedef struct _cwLine{
	double a;
	double b;
	double c;
	FPOINT start; //�ߵ����
	FPOINT end;   //�ߵ��յ�
	_cwLine(double x = 0,double y = 0,double z = 0):a(x),b(y),c(z){
		start.SetVal(0,0);
		end.SetVal(0,0);
	}
} LINE;


typedef enum _cwShap{
	SHAP_NULL = -1,
	SHAP_CIRCLE = 0,      //Բ
		SHAP_SQUARE,      //������
		SHAP_OTHER,       //����ͼ��
		SHAP_RECTANGLE,   //����
		SHAP_TRIANGLE,    //������
		SHAP_LINE,        //����
		SHAP_CIRCULAR,    //�����ߵ�Բ
		SHAP_CORNER,      //�ǵ�
		SHAP_SRECTANGLE,   //����ϵľ���
		SHAP_GREY         //�Ҷ�ƥ��
}SHAP;

// ROI����
typedef enum roi_tpye{
	ROI_NULL=0	,               //NULL
		ROI_CIRCLE,             //Բ��ROI
		ROI_LINE,               //����ROI
		ROI_FSQUARE,            //������ROI
		ROI_FRECT,              //����ROI(����ת)
		//ROI_RFRECT,             //����ת����ROI
		//ROI_ANNULUS,          // Բ����ROI
		//ROI_CIRCULAR,           //�����ߵ�Բ��ROI
		ROI_CIRCLE_CIRCLE,      //ԲԲ���ģ��
		ROI_CIRCLE_LINE,        //Բ�����ģ��
		ROI_CORNER,             //�ǵ�
		ROI_RECT_FIT            //��Ͼ���
}ROI_TYPE;

//ģ������ݽṹ
typedef struct _cwModel MODEL;
SHAP GetModShap(const MODEL* mod);

// ROI���
typedef struct _cwROI ROI, *HROI;

// cwFindModel - ����ģ�����ѡ��
typedef struct _cwFINDOPTION {
	int		minScore;		// ����ƥ��������С����(0~1000)��
	int		minAngle;		// ��С�Ƕȣ�-180~180����
	int		maxAngle;		// ���Ƕȣ�-180~180����
	int		minScale;		// ��С���������հٷֱȣ�100Ϊ1������
	int		maxScale;		// �����������հٷֱȣ�100Ϊ1������
	int		needEdge_or_level;		// ��̵ı�Ե������������Щ����С�ڸ�ֵ�ı�Ե���Ա����������
							// �Ҷ�ģ��Ϊ���ҵȼ�
	int		threshold_or_maxError;		// ��Ե�ݶ���ֵ��ֻ����Щ�ݶȴ��ڸ���ֵ�ı�Ե��ű���������
							// �Ҷ�ģ��Ϊ��������ƥ��ƫ��
	int		findCount;		// ��Ҫ�����ĸ�����
	_cwFINDOPTION(int score=600,int minA=-10,int maxA=10,int minS=0.8,int maxS=1.2,int edgeTh=30,int count=0):minScore(score),
		minAngle(minA),maxAngle(maxA),minScale(minS),maxScale(maxS),threshold_or_maxError(edgeTh),findCount(count){}
} FINDOPTION, *PFINDOPTION;

// cwFindModel - ����ģ�巵������
typedef struct _cwMATCH 
{
	double	score;			// ���ƶȣ�
	double	angle;			// �Ƕȣ�
	double	scale;			// ������
	FRRECT	rectBox;		// λ�ÿ�
	FPOINT	position;		// ƥ���λ�ã�
} MATCH, *PMATCH;

// cwLearnMode - ѧϰģ�����ѡ��
typedef struct _cwLEARNOPTION 
{
	int		minScore;		// ����ƥ��������С����(0~1000)��
	int		minAngle;		// ��С�Ƕȣ�-180~180����
	int		maxAngle;		// ���Ƕȣ�-180~180����
	int		minScale;		// ��С���������հٷֱȣ�100Ϊ1������
	int		maxScale;		// �����������հٷֱȣ�100Ϊ1������
	int     edgeThreashold; // ��С��Ե��ֵ
	int		color;          // ģ����ɫ0Ϊ��ɫ 1Ϊ��ɫ
	int     polarity;       // ���� 0Ϊ������ 1Ϊ������ 2Ϊ���⼫��
	double  intersectDiff;  // �ǵ�нǵĽǶȵ�����ƫ�����̵�λΪ��
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

// cwLearnMode - ѧϰ�Ҷ�ģ�����ѡ��
typedef struct _cwLEARNOPTIONGREY 
{
	int			findLevel;		//���ҵȼ���1-5�ȼ�Խ���ٶ�Խ�죩
	double		minAngle;		// ��С�Ƕȣ�-180~180����
	double		maxAngle;		// ���Ƕȣ�-180~180����
	double		angleStep;      //�����ǶȵĿ�ȣ�0~180����
	bool		bOptimize;		//�Ƿ�����Ż��������Ż�ģ���ʱ��᳤�㣬��֮��������ƥ���׼ȷ��
	GREY_VALUE_TYPE greyType;   //�Ҷ�ֵ����
	_cwLEARNOPTIONGREY(int level=5,double minA=-10.0,double maxA=10.0,double step=1.0,bool optimize=true,
		GREY_VALUE_TYPE type=GREY_TYPE_original):findLevel(level),minAngle(minA),maxAngle(maxA),
		angleStep(step),bOptimize(optimize),greyType(type){}
	_cwLEARNOPTIONGREY(const struct _cwLEARNOPTIONGREY* option):findLevel(option->findLevel),minAngle(option->minAngle),
		maxAngle(option->maxAngle),angleStep(option->angleStep),bOptimize(option->bOptimize),greyType(option->greyType){}
} LEARNOPTIONGREY, *PLEARNOPTIONGREY;


// ��ʾͼ���õ����ݽṹ display 
typedef struct _cwOFFSETZOOM
{
	double		x;		// ��ʾͼ�ε�����ԭ��x
	double		y;		// ��ʾͼ�ε�����ԭ��y
	double	scale;	// ��ʾͼ��ʱ�����ű���
	_cwOFFSETZOOM(double tx=0,double ty=0,double sc = 1):x(tx),y(ty),scale(sc){}
} OFFSETZOOM;

/************************************************************************/
/* ����: �����Ƿ�תͼ���y���� (Ĭ��Ϊ��ת)
/************************************************************************/
CWVISIONAPI void cwSetReversePara(bool value=true);

/************************************************************************/
/* ����: �ͷ�һ��ģ��ָ�� 
/************************************************************************/
CWVISIONAPI void cwFreeModel( MODEL* pModel ); 

/************************************************************************/
/* ����: �ͷ�һ����Ч��ROIָ��
/* ����: pRoi һ��ָ��ROI���͵�ָ�룬��Ҫ�ͷŵ�ROI
/************************************************************************/

CWVISIONAPI void cwFreeRoi( ROI* pRoi );
/************************************************************************/
/* ����: �ͷ�һ��ָ����ڴ�
/* ����: pMemory ָ����Ҫ�ͷŵ��ڴ���׵�ַ	
/************************************************************************/
CWVISIONAPI void cwFreeMemory( void* pMemory );

/************************************************************************/
/* ����: �ͷ�һ���Ա������õ�ͼ����ڴ�
/* ����: pImage ��Ҫ�ͷ��ڴ��ͼ��ָ��	
/************************************************************************/
CWVISIONAPI void cwFreeImage( IMAGE* pImage );

/************************************************************************/
/* ����: ���ļ���ģ�� 
/************************************************************************/
CWVISIONAPI MODEL* cwLoadModel( LPCSTR pszFileName );

/************************************************************************/
/* ����: ���ļ���ģ�� 
/* ����: pszProductName��Ʒ�����ڲ�Ʒ�ļ������½����ļ��У�
/* ����: cameraIndex����������
/* ����: modSeqģ�����
/* ����: isAttach�Ƿ񸽼�ģ�壨Ĭ��Ϊfalse����������ģ����ģ����ŵĻ�����+ģ��ƫ��ֵ
/* ����: seqOffsetΪ�趨�ĸ���ģ��ƫ������Ĭ��Ϊ1000��
/************************************************************************/
CWVISIONAPI MODEL* cwLoadModelP(LPCSTR pszProductName,int cameraIndex,int modSeq,bool isAttach=false ,int seqOffset=1000);

/************************************************************************/
// ����: ���ļ��ж�ȡһ��BMPͼƬ
// ����: pszFileName ͼƬ���ļ��е�ȫ·����
// ����: �ɹ�����һ����Ч��ͼ������ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI IMAGE* cwReadBMPFile( LPCSTR pszFileName );

/************************************************************************/
/* ����: ��BMP��ʽ����һ��ͼƬ���ļ���ȥ
/* ����: pszFileName ����ͼƬ��ȫ·����
/* ����: pImage ��Ҫ�����ͼ������ָ��
/* ����: writeWithoutWarn�ļ������Ƿ�����ڲ���ʾ���ǣ�Ĭ��Ϊfalse��
/* ����: �ɹ�����true�����򷵻�false
/************************************************************************/
CWVISIONAPI bool cwWriteBMPFile( LPCSTR pszFileName, IMAGE* pImage ,bool writeWithoutWarm = false);

/************************************************************************/
/* ����: ����ģ�嵽�ļ� 
/* ����: writeWithoutWarn�ļ������Ƿ�����ڲ���ʾ���ǣ�Ĭ��Ϊfalse��
/************************************************************************/
CWVISIONAPI bool cwSaveModel( LPCSTR pszFileName, const MODEL* pModel ,bool writeWithoutWarm = false); 

/************************************************************************/
/* ����: ���ļ�ɾ��ģ�� 
/************************************************************************/
CWVISIONAPI bool cwDeleteModel( LPCSTR pszFileName); 

/************************************************************************/
/* ����: ����ģ�嵽�ļ� 
/* ����: pModel��Ҫ�����ģ��ָ��
/* ����: pszProductName��Ʒ�����ڲ�Ʒ�ļ������½����ļ��У�
/* ����: cameraIndex����������
/* ����: modSeqģ�����
/* ����: isAttach�Ƿ񸽼�ģ�壨Ĭ��Ϊfalse����������ģ����ģ����ŵĻ�����+ģ��ƫ��ֵ
/* ����: seqOffsetΪ�趨�ĸ���ģ��ƫ������Ĭ��Ϊ1000��
/* ����: writeWithoutWarn�ļ������Ƿ�����ڲ���ʾ���ǣ�Ĭ��Ϊfalse��
/************************************************************************/
CWVISIONAPI bool cwSaveModelP( const MODEL* pModel, LPCSTR pszProductName,int cameraIndex,int modSeq,
				 bool isAttach=false ,int seqOffset=1000,bool writeWithoutWarm = false);

/************************************************************************/
/* ����: ����һ��λͼ��Ŀǰ��֧��8λ�� 
/************************************************************************/
CWVISIONAPI IMAGE* cwCreateImage( BYTE* pData, int BitCount, int Width, int Height );

/************************************************************************/
/* ����: ����ͼ������ݣ���pData�����ݸ��Ƶ�pImage��ȥ
/* ����: pImage Ŀ��ͼ����Ҫ�������ݵ�ͼ��ָ��		
/* ����: pBits �µ�ͼ�����ݻ��������׵�ַָ�룬��ͼ���λ���ͳߴ�����Ŀ��ͼ��һ��
/************************************************************************/
CWVISIONAPI void cwSetImageBits( IMAGE* pImage, const BYTE* pBits, DWORD dPrat );

/************************************************************************/
// ����: ѧϰһ������ģ��
// ����: pImage ����ѧϰ��ͼ�����ݣ�Ŀǰֻ֧��8λ�Ҷ�ͼ��
// ����: pRoi ѧϰ��roi
// ����: shap ѧϰ��ģ������
// ����: Option ��صĲ������μ�LEARNOPTION
// ����: showEditModelDlgΪ�Ƿ���ʾ�༭ģ��Ի���Ĭ��Ϊ����ʾ��
// ����: searchRegionΪ��������ΪNULL��ʾ����ȫͼ��Ĭ��ΪNULL��
// ����: �ɹ�����һ��ģ���������򷵻�NULL
/************************************************************************/
CWVISIONAPI MODEL* cwLearnModel(const IMAGE* pImage, ROI* pRoi,const OFFSETZOOM* pZoom,SHAP shap, const LEARNOPTION* option,
								bool showEditModelDlg=false ,FRRECT* searchRegion = NULL);

/************************************************************************/
// ����: ѧϰһ���Ҷ�ģ��
// ����: pImage ����ѧϰ��ͼ�����ݣ�Ŀǰֻ֧��8λ�Ҷ�ͼ��
// ����: pRoi ѧϰ��roi
// ����: shap ѧϰ��ģ������
// ����: Option ��صĲ������μ�LEARNGREYOPTION
// ����: showEditModelDlgΪ�Ƿ���ʾ�༭ģ��Ի���Ĭ��Ϊ����ʾ��
// ����: searchRegionΪ��������ΪNULL��ʾ����ȫͼ��Ĭ��ΪNULL��
// ����: �ɹ�����һ��ģ���������򷵻�NULL
/************************************************************************/
CWVISIONAPI MODEL* cwLearnModelGrey(const IMAGE* pImage, ROI* pRoi,const OFFSETZOOM* pZoom, const LEARNOPTIONGREY* option,
								bool showEditModelDlg = false , FRRECT* searchRegion = NULL);

/************************************************************************/
/* ����: ��һ��ͼ����������ģ�����Ƶ�Ŀ��(����ƥ��������״,����������ת)*/
/* ����: pImage �����ͼ�����ݣ�֧��8λ�Ҷ�ͼ��                         */
/* ����: pRect ������Χ��NULLΪѧϰģ��ʱ�������������                 */
/* ����: hModel ģ����                                                */
/* ����: Option ��������ز������μ�FINDOPTION                          */
/* ����: retCount ������������ƥ�����ĸ���                            */
/* ����: �ɹ�����������������Ŀ������ݣ����򷵻�NULL                   */
/************************************************************************/
CWVISIONAPI MATCH* cwFindModel( const IMAGE* pImage, const FRRECT* pRect, const MODEL* pModel, const FINDOPTION* Option, 
							   int* retCount ,double angle = 0,double cx = 320,double cy = 240 );

CWVISIONAPI LINE* cwFindLine(const IMAGE* pImage,ROI* roi,const OFFSETZOOM* pZoom);
CWVISIONAPI LINE* cwFindLine(const IMAGE* pImage,FRRECT* rect);

//CWVISIONAPI FPOINT* cwFindCorner(const IMAGE* pImage,ROI* roi,const OFFSETZOOM* pZoom);

//������ʾ ѵ����ĺ���

/************************************************************************/
/* ����: ����һ��ROI
/* ����: Type ��Ҫ������ROI���ͣ��μ�ROI���͵ĺ궨��
/* ����: �ɹ�����һ��ROI����ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI ROI* cwCreateRoi(int Type);

 /************************************************************************/
 /* ����: ��ȡROI����
 /* ����: ����ROI������
 /************************************************************************/
CWVISIONAPI ROI_TYPE cwGetROIType(ROI*roi);

/************************************************************************/
/* ����: ��ȡģ������
/* ����: ����ģ�������
/************************************************************************/
CWVISIONAPI SHAP cwGetModelShap(MODEL*mod);

/************************************************************************/
/* ����: �жϵ�ǰ�����λ�õ��Ƿ���ROI�ĵ��ڵ��ڣ�����ȡ���ڵ��Ӧ��ֵ
/* ����: pRoi һ��ָ��ROI���͵�ָ�룬��������һ��ROI
/* ����: pZoomΪ�ؼ����Ͻ������ƫ��ֵ�������ȣ��ⲿ��ת������Ҫ����ΪNULL
/* ����: y ��ǰ�����x������
/* ����: x ��ǰ�����y������
/* ����: record�Ƿ���Ҫ��¼��ǰλ����Ϣ��Ĭ��Ϊfalse����갴��ʱ��Ҫ���ø�ֵΪtrue
/* ����: �ɹ����ض�Ӧ���ڵ���Ӧ��ֵ��ÿ�����ڵ㶼�в�ͬ��ֵ�����򷵻�0
//���ε��ڵ���ϵ�������Ϊ1 2 3  4 5 6   7 8 9
//Բ�ε��ڵ���ϵ�������Ϊ1  2 3 4  5
/************************************************************************/
CWVISIONAPI int cwGetRoiNode(const ROI* pRoi, const OFFSETZOOM* pZoom, int x,int y ,bool record=false);

/************************************************************************/
/* ����: ���ݵ��ڵ��ֵ����ROI��Ӧ��λ�ý��е���
/* ����: pRoi һ��ָ��ROI���͵�ָ�룬��������һ��ROI
/* ����: pZoomΪ�ؼ����Ͻ������ƫ��ֵ�������ȣ��ⲿ��ת������Ҫ����ΪNULL
/* ����: Node ���ڵ��ֵ
/* ����: y ��ǰ�����x������
/* ����: x ��ǰ�����y������
/* ����: �ɹ�����true�����򷵻�false
/************************************************************************/
CWVISIONAPI bool cwAdjustRoi( ROI* pRoi, const OFFSETZOOM* pZoom, int Node, int x, int y );

/************************************************************************/
/* ����: ����ROI�����ͺ͵��ڵ��ֵ�����õ�ǰ�Ĺ�����ͣ�������ͷ��ʾ���ͣ�
/* ����: pRoi һ��ָ��ROI���͵�ָ��
/* ����: Node ���ڵ��ֵ
/* ����: �ɹ�����true�����򷵻�false
/************************************************************************/
CWVISIONAPI bool cwRoiCursor( const ROI* pRoi, int Node );

/************************************************************************/
/* ����: �����������������һ��ROI�����Ͳ�ͬ��ROI������Ч����ͬ
/* ����: pRoi һ��ָ��ROI���͵�ָ�룬��������һ��ROI
/* ����: pZoomΪ�ؼ����Ͻ������ƫ��ֵ�������ȣ��ⲿ��ת������Ҫ����ΪNULL
/* ����: x1 ��һ�����x������
/* ����: y1 ��һ�����y������
/* ����: x2 �ڶ������x������
/* ����: y2 �ڶ������y������
/* ����: �ɹ�����һ��ROI����ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI bool cwSetRoi( ROI* pRoi, const OFFSETZOOM* pZoom, int x1, int y1, int x2, int y2);

/************************************************************************/
/* ����: ����Ļ����ʾһ��ROI
/* ����: showWnd ��ʾ�豸�Ļ�ͼ���ھ��
/* ����: pRoi ��Ҫ��ʾ��ROI����
/* ����: Color ��ʾROIʱʹ�õ���ɫ
/* ����: clearFormerΪ�Ƿ�������л��ƺۼ�(Ĭ��Ϊ���)
/* ����: showImageWidthΪͼ���ʵ����ʾ��� Ĭ��Ϊ640
/* ����: showImageHeigthΪͼ���ʵ����ʾ�߶� Ĭ��Ϊ480
/************************************************************************/
CWVISIONAPI void cwDisplayRoi(HWND showWnd, const OFFSETZOOM* pZoom, const ROI* pRoi, CCOLOR Color,bool clearFormer = true,
							  int showImageWidth=640,int showImageHeigth=480);

/************************************************************************/
/* ����: ����Ļ����ʾһ��ROI�����е��ڵ�
/* ����: showWnd ��ʾ�豸�Ļ�ͼ���ھ��
/* ����: pRoi ��Ҫ��ʾ���ڵ��ROI����
/* ����: Color ��ʾROI���ڵ�ʱʹ�õ���ɫ
/* ����: showImageWidthΪͼ���ʵ����ʾ��� Ĭ��Ϊ640
/* ����: showImageHeigthΪͼ���ʵ����ʾ�߶� Ĭ��Ϊ480
/************************************************************************/
CWVISIONAPI void cwDisplayRoiNode(HWND showWnd, const OFFSETZOOM* pZoom, const ROI* pRoi, CCOLOR Color ,
								  int showImageWidth=640,int showImageHeigth=480);


/************************************************************************/
/* ����: ��ROIת��Ϊһ��FRRECT��ָ��
/* ����: pRoi ��Ҫת����ROI���ݣ���ROI�����ͱ���ΪROI_RRECT
/* ����: �ɹ�����һ��ָ���ROI�ڲ�������FRRECT��ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI FRRECT* cwGetFRRect( ROI* pRoi );

/************************************************************************/
/* ����: ��ROIת��Ϊ4��FRRECT��ָ�� ����Ϊ��������
/* ����: pRoi ��Ҫת����ROI���ݣ���ROI�����ͱ���ΪROI_SRECT
/* ����: �ɹ�����һ��ָ���ROI�ڲ�������FRRECT��ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI bool cwGetFRRects( ROI* pRoi ,FRRECT rect[4]);

/************************************************************************/
/* ����: ��ROIת��Ϊһ��ROUND��ָ��
/* ����: pRoi ��Ҫת����ROI���ݣ���ROI�����ͱ���ΪROI_ROUND
/* ����: �ɹ�����һ��ָ���ROI�ڲ�������ROUND��ָ�룬���򷵻�NULL
/************************************************************************/
CWVISIONAPI CIRCLE* cwGetRound( ROI* pRoi );

/************************************************************************/
/* ����: ����Ļ����ʾһ��ͼ��
/* ����: showWnd ��ʾ�豸�Ļ�ͼ���ھ��
/* ����: pImage ��Ҫ��ʾ��ͼ������ָ��
/* ����: pZoom ʹ�øò������Խ���ƽ�ƺ�������ʾ���壬����Ҫ����ΪNULL Ĭ��ΪNULL
/************************************************************************/
CWVISIONAPI void cwDisplayImage( HWND showWnd, const IMAGE* pImage , const OFFSETZOOM* pZoom = NULL);

/************************************************************************/
/* ����: ����Ļ����ʾ���һ��ƥ����
/* ����: showWnd ��ʾ�豸�Ļ�ͼ���ھ��
/* ����: pZoom ʹ�øò������Խ���ƽ�ƺ�������ʾ���� ����Ҫ����ΪNULL Ĭ��ΪNULL
/* ����: showImageWidthΪͼ���ʵ����ʾ��� Ĭ��Ϊ640
/* ����: showImageHeigthΪͼ���ʵ����ʾ�߶� Ĭ��Ϊ480
/************************************************************************/
CWVISIONAPI int cwDisplayLastMatchFeature( HWND showWnd,OFFSETZOOM* pZoom=NULL,int showImageWidth=640,int showImageHeigth=480, bool bShowResult = true);
CWVISIONAPI void cwSetDisplayMatchFeature(bool disp);//�����Ƿ���ʾƥ����
CWVISIONAPI void cwResultRectSeqSet(int seq);//����ƥ���������
CWVISIONAPI void cwResultRectSeqIncrease();//����ƥ���������
CWVISIONAPI void cwResultRectSeqDecrease();//����ƥ���������
CWVISIONAPI void cwResultRectSeqReset();//����ƥ���������

//���ڵ���ģ��ĺ���
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

//�궨
CWVISIONAPI void cwCalibration(IMAGE* pImage);

#endif