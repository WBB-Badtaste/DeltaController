
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
#include "NyceExErrorHandle.h"
#include "Drawer.h"

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
	, m_motion_par_x(0)
	, m_motion_par_y(0)
	, m_motion_par_z(0)
	, m_motion_par_vel(0)
	, m_motion_par_direc(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}


void CDeltaControllerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST1, m_listBox);
	DDX_Text(pDX, IDC_EDIT1, m_motion_par_x);
	DDX_Text(pDX, IDC_EDIT2, m_motion_par_y);
	DDX_Text(pDX, IDC_EDIT3, m_motion_par_z);
	DDX_Text(pDX, IDC_EDIT4, m_motion_par_vel);
	DDX_Radio(pDX, IDC_RADIO1, m_motion_par_direc);
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
	ON_BN_CLICKED(IDC_BUTTON5, &CDeltaControllerDlg::OnBnClickedButton5)
	ON_BN_CLICKED(IDC_BUTTON10, &CDeltaControllerDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON13, &CDeltaControllerDlg::OnBnClickedButton13)
	ON_BN_CLICKED(IDC_BUTTON11, &CDeltaControllerDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CDeltaControllerDlg::OnBnClickedButton12)
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


void CDeltaControllerDlg::OnBnClickedButton6()//运动
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	UpdateData(TRUE);

	DOOR_TRAJ_PARS doorPars1, doorPars2;
	doorPars1.startPos.x = -152.5;
	doorPars1.startPos.y = 0.0;
	doorPars1.startPos.z = -650.0;
	doorPars1.endPos.x = 152.5;
	doorPars1.endPos.y = 0.0;
	doorPars1.endPos.z = -650.0;
	doorPars1.hight = 25.0;
	doorPars1.radius = 8.0;
	doorPars1.trajPars.velocity = m_motion_par_vel;
	doorPars1.trajPars.acceleration = doorPars1.trajPars.velocity * 100;
	doorPars1.trajPars.splineTime = 0.01;

	doorPars2.startPos.x = 152.5;
	doorPars2.startPos.y = 0.0;
	doorPars2.startPos.z = -650.0;
	doorPars2.endPos.x = -152.5;
	doorPars2.endPos.y = 0.0;
	doorPars2.endPos.z = -650.0;
	doorPars2.hight = 25.0;
	doorPars2.radius = 8.0;
	doorPars2.trajPars.velocity = m_motion_par_vel;
	doorPars2.trajPars.acceleration = doorPars2.trajPars.velocity * 100;
	doorPars2.trajPars.splineTime = 0.01;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(doorPars1.startPos, doorPars1.trajPars);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars1);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars2);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton7()//移动到指定点
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	UpdateData(TRUE);

	CARTESIAN_COORD pos;
	pos.x = m_motion_par_x;
	pos.y = m_motion_par_y;
	pos.z = m_motion_par_z;

	TRAJ_PARS trajPars;
	trajPars.velocity = m_motion_par_vel;
	trajPars.acceleration = m_motion_par_vel * 100;
	trajPars.splineTime = 0.01;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars);

	StatusHandle(nyceStatus);
}


HANDLE hThreadReadPos;
HANDLE hEvReadPos;


unsigned __stdcall ThreadReadPosLoop(void* lpParam)
{
	NYCE_STATUS nyceStatus(NYCE_OK);

	double position[3];

	while (WaitForSingleObject(hEvReadPos, INFINITE) == WAIT_OBJECT_0)
	{
		
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksReadPosDelta(position);

	}
	return 0;
}

void CDeltaControllerDlg::OnBnClickedButton3()//初始化
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (((CButton *)GetDlgItem(IDC_CHECK1))->GetCheck())
	{
		nyceStatus = NyceInit(NYCE_SIM);

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksExExportSplineDatas(TRUE);
	}
	else
	{
		nyceStatus = NyceInit(NYCE_ETH);

		nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksExExportSplineDatas(TRUE);
	}

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiConnect(noName[0], &noId[0]);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : InitAxisRexroth(NUM_AXES, axId, axName);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksInitDelta(NUM_AXES, axId);

	StatusHandle(nyceStatus);

// 	uint32_t uThreadReadPos;
// 	hThreadReadPos = (HANDLE)_beginthreadex(NULL, NULL, ThreadReadPosLoop, NULL, 0,&uThreadReadPos);
}


void CDeltaControllerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: Add your message handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksTerm();

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : TermAxis(NUM_AXES, axId);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiDisconnect(noId[0]);

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
	m_listBox.AddString((LPCTSTR)timeStr);
	m_listBox.AddString((LPCTSTR)NyceGetStatusStringEx(nyceStatus));
	m_listBox.AddString("");
}

void CDeltaControllerDlg::OnBnClickedButton8()//回零位
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	UpdateData(TRUE);
	TRAJ_PARS trajPars;
	trajPars.velocity = m_motion_par_vel;
	trajPars.acceleration = m_motion_par_vel * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksHomeDelta(trajPars);

// 	CARTESIAN_COORD homePos;
// 	homePos.x = -65;
// 	homePos.y = 0;
// 	homePos.z = -220;

// 	TRAJ_PARS homeTrajPars;
// 	homeTrajPars.velocity = 100;
// 	homeTrajPars.acceleration = 1000;
// 	homeTrajPars.splineTime = 0.001;

// 	NYCE_STATUS nyceStatus(RocksPtpDelta(homePos, homeTrajPars));

	StatusHandle(nyceStatus);
}


BOOL bOpenBrake = FALSE;
void CDeltaControllerDlg::OnBnClickedButton2()//刹车控制
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	NYCE_DIGITAL_IO_ID io;
	io.slotId = NYCE_SLOT0;

	if (bOpenBrake)
	{
		io.digIONr = NYCE_DIGOUT0;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT1;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT2;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);

		bOpenBrake = FALSE;
	}
	else
	{
		io.digIONr = NYCE_DIGOUT0;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT1;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT2;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

		bOpenBrake = TRUE;
	}

		StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton5()//打开实时位置读取
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	double position[6];
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksReadPosDelta(position);

	SYSTEMTIME time;
	GetSystemTime(&time);
	CString timeStr("");
	timeStr.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
	m_listBox.AddString((LPCTSTR)timeStr);
	CString posStr("");
	posStr.Format(_T("x:%0.2f y:%0.2f z:%0.2f"), position[0], position[1], position[2]);
	m_listBox.AddString((LPCTSTR)posStr);
	m_listBox.AddString("");
}




void CDeltaControllerDlg::OnBnClickedButton10()//-1
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	CARTESIAN_COORD pos;
	pos.x = 0;
	pos.y = 0;
	pos.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.x = -1;
		break;
	case 1:
		pos.y = -1;
		break;
	case 2:
		pos.z = -1;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 5;
	trajPars.acceleration = 5 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton13()//+1
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	CARTESIAN_COORD pos;
	pos.x = 0;
	pos.y = 0;
	pos.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.x = 1;
		break;
	case 1:
		pos.y = 1;
		break;
	case 2:
		pos.z = 1;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 5;
	trajPars.acceleration = 5 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton11()//-10
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	CARTESIAN_COORD pos;
	pos.x = 0;
	pos.y = 0;
	pos.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.x = -10;
		break;
	case 1:
		pos.y = -10;
		break;
	case 2:
		pos.z = -10;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 50;
	trajPars.acceleration = 50 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton12()//+10
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	CARTESIAN_COORD pos;
	pos.x = 0;
	pos.y = 0;
	pos.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.x = 10;
		break;
	case 1:
		pos.y = 10;
		break;
	case 2:
		pos.z = 10;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 50;
	trajPars.acceleration = 50 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}
