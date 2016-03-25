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
	IMAGE* m_pImage;//��ʾ�ڿؼ��ϵ�ͼƬ

	bool InitParameters();

	void CreateRoi( int RoiType );//����һ��ROI
	void DestroyRoi();//���ROI

	void SetImageInf( BYTE* pData = NULL, int bitCount = 8, int width = 640, int height = 480);//����ͼ�������
	void SetMainWnd(HWND hWnd);//����������
	void SetDisplayWnd(HWND hWnd);//������ʾ����
	void SetDisplayRect(CRect rect);//����ͼ����ʾ����ԶԻ��������
	void SetDisplayOffset(OFFSETZOOM zoom);//����ͼ����ʾ��λ��
	void SetSerachArea(bool showInHwnd,int off = 20);//��ʾ����ʾ��������
	void SetFullScreen();//����Ӧ������ʾͼƬ

	void DispalyImage();//��ʾͼƬ
	void DisplayROI();//��ʾ��������͵��ڵ�
	void DisplayMatchResult(bool bShowResultText = true);//��ʾƥ����

	//��ȡ��Ϣ
	FRRECT* GetRoiRect();//��ȡ����ͼ�ε�����
	bool GetRoiRects(FRRECT rect[4]);//��ȡ�ĸ��������� ����Ϊ��������

	//���ڿ�����Ϣ
	void SetCaliperInf(int caliperNum,double caliperWidth,double caliperLength);
	void GetCaliperInf(int* caliperNum,double* caliperWidth,double* caliperLength);

	//��ȡ���ߵ�����λ��
	FPOINT* GetROICenter();

	//��������¼�
	void LButtonDown(UINT nFlags, CPoint point,int roiType);//����������
	void LButtonUp(UINT nFlags, CPoint point);//����������
	void MouseMove(UINT nFlags, CPoint point);//����ƶ�
	void RButtonDown(UINT nFlags, CPoint point);//����Ҽ�����
//	void RButtonUp(UINT nFlags, CPoint point);//����Ҽ�����
	void MouseWheel(UINT nFlags, short zDelta, CPoint pt);//����м�����
	void RButtonDblClk(UINT nFlags, CPoint point);//����Ҽ�˫��
	void InvalidateImageRect();

	//ͼ���ģ����� �����ɹ�����true����ʧ�ܷ���false
	bool SaveImage(const char path[]);
	bool LoadImage(const char path[]);
	bool SaveMod(const char path[],bool writeWithouWarn = false);
	bool LoadMod(const char path[]);
	bool ModExist();//����ģ���Ƿ�ѵ��

	//����ģ��
	bool LearnModel(SHAP shap,LEARNOPTION* option,bool showEditModelDlg = false);
	bool LearnModelGrey(LEARNOPTIONGREY* option,bool showEditModelDlg = false);
	MATCH* FindModel(FINDOPTION* option,int* retCount,double angle = 0,double cx = 320,double cy = 240 );
	MATCH* FindModel(MODEL* mod,FRRECT* searchRegion,FINDOPTION* option,int* retCount,
		double angle = 0,double cx = 320,double cy = 240 );
	LINE* FindLine();
	FPOINT* FindCorner();
private:
	MODEL* m_pMod;//ģ��
	bool m_showDisp;//��¼�Ƿ���ʾ����

	CRect m_imageRect;//��¼Դͼ�ؼ�����ʼλ��(��ԶԻ���)
	OFFSETZOOM m_dispOffset;//ͼ��ƫ�Ʋ��� ������ʾͼƬ����ҪScreenToClient��


	//cwVision ��ʾͼ�����
	HWND m_showWnd;//��ʾͼ��Ĵ���
	bool m_showROi;//�����Ƿ���ʾROI
	HWND m_mainWnd;//������
	//	bool m_lButtonDown; //��¼����Ƿ���
	//	bool m_rButtonDown; //��¼�Ҽ��Ƿ���
	CPoint m_rButtonDownLoc;//�Ҽ�����ʱ������
	FPOINT m_preOffset;//��¼֮ǰ��ƫ��ֵ
	bool m_inImageRect;//��¼�Ƿ���ͼ���ڲ�
	CPoint m_movePoint;//��¼��ǰ�ƶ���λ��
	CRect m_selectRect;//�û�ѡ�������,�������½������ؽ�����ʱʹ��

	int m_trainRoiPosition;//��¼�û�ѡ���ѵ������ڵ�
	ROI* m_trainRoi;//��ʾѵ��������
	int m_trainRoiType;//��ʾ����������
	
	int m_searchRoiPosition;//��¼�û�ѡ�������������ڵ�
	ROI* m_searchRoi;//��ʾ��������

public:
	CVisionCW();
	virtual ~CVisionCW();

};

#endif // !defined(AFX_VISIONCW_H__F15806A9_27BC_40F1_B946_E5A521C75CE6__INCLUDED_)
