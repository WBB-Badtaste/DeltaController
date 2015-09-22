
// DeltaControllerDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "DeltaController.h"
#include "DeltaControllerDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "AxisControl.h"
#include "RocksControl.h"
#include "Defines.h"

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CDeltaControllerDlg 对话框




CDeltaControllerDlg::CDeltaControllerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDeltaControllerDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}


void CDeltaControllerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDeltaControllerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON6, &CDeltaControllerDlg::OnBnClickedButton6)
	ON_BN_CLICKED(IDC_BUTTON7, &CDeltaControllerDlg::OnBnClickedButton7)
	ON_BN_CLICKED(IDC_BUTTON3, &CDeltaControllerDlg::OnBnClickedButton3)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_CHECK1, &CDeltaControllerDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BUTTON8, &CDeltaControllerDlg::OnBnClickedButton8)
	ON_BN_CLICKED(IDC_BUTTON2, &CDeltaControllerDlg::OnBnClickedButton2)
END_MESSAGE_MAP()


// CDeltaControllerDlg 消息处理程序
BOOL CDeltaControllerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}


void CDeltaControllerDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}


// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CDeltaControllerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CDeltaControllerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CDeltaControllerDlg::OnBnClickedButton6()
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = RocksDoorDelta();

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton7()
{
	// TODO: Add your control notification handler code here

}


void CDeltaControllerDlg::OnBnClickedButton3()
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (((CButton *)GetDlgItem(IDC_CHECK1))->GetCheck())
		nyceStatus = NyceInit(NYCE_SIM);
	else
		nyceStatus = NyceInit(NYCE_ETH);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiConnect(noName, noId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: Add your message handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = TermAxis(NUM_AXES, axId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiDisconnect(noId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NyceTerm();

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedCheck1()
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);
}


void CDeltaControllerDlg::StatusHandle(NYCE_STATUS &nyceStatus)
{
	SYSTEMTIME time;
	GetSystemTime(&time);
	CString timeStr("");
	timeStr.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
	CListBox *pListBox = (CListBox *)GetDlgItem(IDC_LIST1);
	pListBox->AddString((LPCTSTR)timeStr);
	pListBox->AddString((LPCTSTR)NyceGetStatusString(nyceStatus));
	pListBox->AddString("");
}

void CDeltaControllerDlg::OnBnClickedButton8()
{
	// TODO: Add your control notification handler code here
}


void CDeltaControllerDlg::OnBnClickedButton2()//刹车控制
{
	// TODO: Add your control notification handler code here

}
