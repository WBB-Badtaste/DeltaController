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
	unsigned char * m_pImgData; //ͼ������ָ��
	LPRGBQUAD m_lpColorTable; //��ɫָ��
	int m_nBitCount;//ÿ������ռ��λ��
private:
	LPBYTE m_lpDib; // ָ��DIB��ָ��
	HPALETTE m_hPalette; //�߼���ɫ����
	int m_nColorTableLength;//��ɫ����

public:
	int m_imgWidth;
	int m_imgHeight;
	LPBITMAPINFOHEADER m_lpBmpInfoHead;//ͼ����Ϣͷָ��

public:
	void FillLine(BYTE* src,int s_x, int e_x, int s_y, int e_y);
	void DrawLine(BYTE color,int s_x, int e_x, int s_y, int e_y);

	int GetGrayValue(CPoint p);//��ȡһ����ĻҶ�ֵ
	void GetGrayValues(char data[][640],char path[],int x1,int x2, int y1,int y2);//��ȡһ������ĻҶ�ֵ
	ImageDib();
	ImageDib(CSize size, int nBitCount, LPRGBQUAD lpColorTable, 
		unsigned char *pImgData);           //�������Ĺ��캯��
	virtual ~ImageDib();

	BOOL Read(LPCTSTR lpszPathName);//��ȡDIB
	BOOL Write(LPCTSTR lpszPathName);//дDIB
	int ComputeColorTableLength(int nBitCount);//������ɫ����
	BOOL Draw(CDC* pDC, CPoint origin, CSize size);//ͼ�����
	CSize GetDimensions();//��ȡͼ��ά��
	void ReplaceDib(CSize size, int nBitCount, LPRGBQUAD lpColorTable,unsigned char* pImageData);//���µ��������DIB
	
	void Empty();//����ռ�
private:
	void MakePalette(); //�����߼���ɫ��
	
};

typedef ImageDib IMAGE;

#endif // !defined(AFX_IMAGEDIB_H__C1757D35_526F_41FE_BC73_54140B0E16DB__INCLUDED_)
