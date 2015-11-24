
// DeltaControllerDlg.h : ͷ�ļ�
//

#pragma once

#include "nyceapi.h"
#include "afxwin.h"


// CDeltaControllerDlg �Ի���
class CDeltaControllerDlg : public CDialogEx
{
// ����
public:
	CDeltaControllerDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_DELTACONTROLLER_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedDoor();
	afx_msg void OnBnClickedPtp();
	afx_msg void OnBnClickedInit();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedCheck1();

private:
	void StatusHandle(NYCE_STATUS &nyceStatus);
public:
	afx_msg void OnBnClickedHome();
	afx_msg void OnBnClickedBrake();
	afx_msg void OnBnClickedShowPos();
	CListBox m_listBox;
	double m_motion_par_x;
	double m_motion_par_y;
	double m_motion_par_z;
	double m_motion_par_vel;
	int m_motion_par_direc;
	afx_msg void OnBnClickedButton10();
	afx_msg void OnBnClickedMove1();
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedMove10();
	afx_msg void OnBnClickedPump();
	afx_msg void OnBnClickedSwitch();
	afx_msg void OnBnClickedCirlce();
};
