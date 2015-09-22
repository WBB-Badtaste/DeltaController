
// DeltaControllerDlg.h : 头文件
//

#pragma once

#include "nyceapi.h"


// CDeltaControllerDlg 对话框
class CDeltaControllerDlg : public CDialogEx
{
// 构造
public:
	CDeltaControllerDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_DELTACONTROLLER_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton6();
	afx_msg void OnBnClickedButton7();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnDestroy();
	afx_msg void OnBnClickedCheck1();

private:
	void StatusHandle(NYCE_STATUS &nyceStatus);
public:
	afx_msg void OnBnClickedButton8();
	afx_msg void OnBnClickedButton2();
};
