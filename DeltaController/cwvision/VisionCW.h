// VisionCW.h: interface for the CVisionCW class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VISIONCW_H__F15806A9_27BC_40F1_B946_E5A521C75CE6__INCLUDED_)
#define AFX_VISIONCW_H__F15806A9_27BC_40F1_B946_E5A521C75CE6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include <afxwin.h>
#include "CWVision.h"

//#include "atltypes.h"

class __declspec(dllexport) CVisionCW  
{
public:
	IMAGE* m_pImage;//显示在控件上的图片

	bool InitParameters();

	void CreateRoi( int RoiType );//创建一个ROI
	void DestroyRoi();//清除ROI

	void SetImageInf( BYTE* pData = NULL, int bitCount = 8, int width = 640, int height = 480);//设置图像的属性
	void SetMainWnd(HWND hWnd);//设置主窗口
	void SetDisplayWnd(HWND hWnd);//设置显示窗口
	void SetDisplayRect(CRect rect);//设置图像显示框相对对话框的区域
	void SetDisplayOffset(OFFSETZOOM zoom);//设置图像显示的位置
	void SetSerachArea(bool showInHwnd,int off = 20);//显示或不显示搜索区域
	void SetFullScreen();//自适应窗口显示图片

	void DispalyImage();//显示图片
	void DisplayROI();//显示调节区域和调节点
	void DisplayMatchResult(bool bShowResultText = true);//显示匹配结果

	//获取信息
	FRRECT* GetRoiRect();//获取任意图形的区域
	bool GetRoiRects(FRRECT rect[4]);//获取四个矩形区域 依次为上右下左

	//调节卡尺信息
	void SetCaliperInf(int caliperNum,double caliperWidth,double caliperLength);
	void GetCaliperInf(int* caliperNum,double* caliperWidth,double* caliperLength);

	//获取卡尺的中心位置
	FPOINT* GetROICenter();

	//处理鼠标事件
	void LButtonDown(UINT nFlags, CPoint point,int roiType);//鼠标左键按下
	void LButtonUp(UINT nFlags, CPoint point);//鼠标左键弹起
	void MouseMove(UINT nFlags, CPoint point);//鼠标移动
	void RButtonDown(UINT nFlags, CPoint point);//鼠标右键按下
//	void RButtonUp(UINT nFlags, CPoint point);//鼠标右键弹起
	void MouseWheel(UINT nFlags, short zDelta, CPoint pt);//鼠标中键滚动
	void RButtonDblClk(UINT nFlags, CPoint point);//鼠标右键双击
	void InvalidateImageRect();

	//图像和模板操作 操作成功返回true操作失败返回false
	bool SaveImage(const char path[]);
	bool LoadImage(const char path[]);
	bool SaveMod(const char path[],bool writeWithouWarn = false);
	bool LoadMod(const char path[]);
	bool ModExist();//返回模板是否训练

	//处理模板
	bool LearnModel(SHAP shap,LEARNOPTION* option,bool showEditModelDlg = false);
	bool LearnModelGrey(LEARNOPTIONGREY* option,bool showEditModelDlg = false);
	MATCH* FindModel(FINDOPTION* option,int* retCount,double angle = 0,double cx = 320,double cy = 240 );
	MATCH* FindModel(MODEL* mod,FRRECT* searchRegion,FINDOPTION* option,int* retCount,
		double angle = 0,double cx = 320,double cy = 240 );
	LINE* FindLine();
	FPOINT* FindCorner();
private:
	MODEL* m_pMod;//模板
	bool m_showDisp;//记录是否显示更新

	CRect m_imageRect;//记录源图控件的起始位置(相对对话框)
	OFFSETZOOM m_dispOffset;//图像偏移参数 用于显示图片（需要ScreenToClient）


	//cwVision 显示图像相关
	HWND m_showWnd;//显示图像的窗体
	bool m_showROi;//控制是否显示ROI
	HWND m_mainWnd;//主窗体
	//	bool m_lButtonDown; //记录左键是否按下
	//	bool m_rButtonDown; //记录右键是否按下
	CPoint m_rButtonDownLoc;//右键按下时的坐标
	FPOINT m_preOffset;//记录之前的偏移值
	bool m_inImageRect;//记录是否在图像内部
	CPoint m_movePoint;//记录当前移动的位置
	CRect m_selectRect;//用户选择的区域,用于在新建或者重建区域时使用

	int m_trainRoiPosition;//记录用户选择的训练框调节点
	ROI* m_trainRoi;//显示训练框区域
	int m_trainRoiType;//显示区域框的类型
	
	int m_searchRoiPosition;//记录用户选择的搜索区域调节点
	ROI* m_searchRoi;//显示搜索区域

public:
	CVisionCW();
	virtual ~CVisionCW();

};

#endif // !defined(AFX_VISIONCW_H__F15806A9_27BC_40F1_B946_E5A521C75CE6__INCLUDED_)
