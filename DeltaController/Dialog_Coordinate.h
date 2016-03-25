#pragma once


// CDialog_Coordinate dialog

class CDialog_Coordinate : public CDialogEx
{
	DECLARE_DYNAMIC(CDialog_Coordinate)

public:
	CDialog_Coordinate(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDialog_Coordinate();

// Dialog Data
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

public:
	CTabCtrl m_tab;
};
