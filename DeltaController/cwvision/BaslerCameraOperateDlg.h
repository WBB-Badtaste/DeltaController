// BaslerCameraOperateDlg.h : header file
//

#if !defined(AFX_BASLERCAMERAOPERATEDLG_H__A221B74E_D9EA_404C_AD4B_5B64292A5C79__INCLUDED_)
#define AFX_BASLERCAMERAOPERATEDLG_H__A221B74E_D9EA_404C_AD4B_5B64292A5C79__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CBaslerCameraOperateDlg dialog
#include "VisionCW.h"

class CBaslerCameraOperateDlg : public CDialog
{
// Construction
public:
	void UpdateImage();
	CBaslerCameraOperateDlg(CWnd* pParent = NULL);	// standard constructor
	~CBaslerCameraOperateDlg();	// 
	CVisionCW m_visionCW;

// Dialog Data
	//{{AFX_DATA(CBaslerCameraOperateDlg)
	enum { IDD = IDD_BASLERCAMERAOPERATE_DIALOG };
	CComboBox	m_combctr_markShape;
	CComboBox	m_combctr_cameraName;
	double	m_editCostTime;
	BOOL	m_check_matchContinues;
	int		m_edit_edge_threshold;
	double	m_edit_match_time;
	CString	m_edit_locate_result;
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CBaslerCameraOperateDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	//{{AFX_MSG(CBaslerCameraOperateDlg)
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnBtnOpenClose();
	afx_msg void OnBtnGetImage();
	afx_msg void OnBtnProperty();
	afx_msg void OnBtnSaveImage();
	afx_msg void OnBtnLearnModel();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDblClk(UINT nFlags, CPoint point);
	afx_msg void OnBtnMatchModel();
	afx_msg void OnCheckSearcharea();
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnBtnOpenHardwareTrigger();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_BASLERCAMERAOPERATEDLG_H__A221B74E_D9EA_404C_AD4B_5B64292A5C79__INCLUDED_)
