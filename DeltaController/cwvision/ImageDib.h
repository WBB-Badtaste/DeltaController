// ImageDib.h: interface for the ImageDib class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMAGEDIB_H__C1757D35_526F_41FE_BC73_54140B0E16DB__INCLUDED_)
#define AFX_IMAGEDIB_H__C1757D35_526F_41FE_BC73_54140B0E16DB__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <FSTREAM>
#include <MATH.H>
//#include <WINDOWS.H>
#include <afxwin.h>
//#include "atltypes.h"

class __declspec(dllexport) ImageDib  
{
public:
	unsigned char * m_pImgData; //图像数据指针
	LPRGBQUAD m_lpColorTable; //颜色指针
	int m_nBitCount;//每个像素占用位数
private:
	LPBYTE m_lpDib; // 指向DIB的指针
	HPALETTE m_hPalette; //逻辑调色板句柄
	int m_nColorTableLength;//颜色表长度

public:
	int m_imgWidth;
	int m_imgHeight;
	LPBITMAPINFOHEADER m_lpBmpInfoHead;//图像信息头指针

public:
	void FillLine(BYTE* src,int s_x, int e_x, int s_y, int e_y);
	void DrawLine(BYTE color,int s_x, int e_x, int s_y, int e_y);

	int GetGrayValue(CPoint p);//获取一个点的灰度值
	void GetGrayValues(char data[][640],char path[],int x1,int x2, int y1,int y2);//获取一个区域的灰度值
	ImageDib();
	ImageDib(CSize size, int nBitCount, LPRGBQUAD lpColorTable, 
		unsigned char *pImgData);           //带参数的构造函数
	virtual ~ImageDib();

	BOOL Read(LPCTSTR lpszPathName);//读取DIB
	BOOL Write(LPCTSTR lpszPathName);//写DIB
	int ComputeColorTableLength(int nBitCount);//计算颜色表长度
	BOOL Draw(CDC* pDC, CPoint origin, CSize size);//图像绘制
	CSize GetDimensions();//获取图像维数
	void ReplaceDib(CSize size, int nBitCount, LPRGBQUAD lpColorTable,unsigned char* pImageData);//用新的数据替代DIB
	
	void Empty();//清理空间
private:
	void MakePalette(); //创建逻辑调色板
	
};

typedef ImageDib IMAGE;

#endif // !defined(AFX_IMAGEDIB_H__C1757D35_526F_41FE_BC73_54140B0E16DB__INCLUDED_)
