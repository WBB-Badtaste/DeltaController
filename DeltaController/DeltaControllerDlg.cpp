
// DeltaControllerDlg.cpp : ʵ���ļ�
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
#include "NyceExErrorHandle.h"
#include "Drawer.h"

// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
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


// CDeltaControllerDlg �Ի���




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
	ON_BN_CLICKED(IDC_BUTTON6, &CDeltaControllerDlg::OnBnClickedDoor)
	ON_BN_CLICKED(IDC_BUTTON7, &CDeltaControllerDlg::OnBnClickedPtp)
	ON_BN_CLICKED(IDC_BUTTON3, &CDeltaControllerDlg::OnBnClickedInit)
	ON_WM_DESTROY()
	ON_BN_CLICKED(IDC_CHECK1, &CDeltaControllerDlg::OnBnClickedCheck1)
	ON_BN_CLICKED(IDC_BUTTON8, &CDeltaControllerDlg::OnBnClickedHome)
	ON_BN_CLICKED(IDC_BUTTON2, &CDeltaControllerDlg::OnBnClickedBrake)
	ON_BN_CLICKED(IDC_BUTTON5, &CDeltaControllerDlg::OnBnClickedShowPos)
	ON_BN_CLICKED(IDC_BUTTON10, &CDeltaControllerDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON13, &CDeltaControllerDlg::OnBnClickedMove1)
	ON_BN_CLICKED(IDC_BUTTON11, &CDeltaControllerDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CDeltaControllerDlg::OnBnClickedMove10)
	ON_BN_CLICKED(IDC_BUTTON1, &CDeltaControllerDlg::OnBnClickedPump)
	ON_BN_CLICKED(IDC_BUTTON4, &CDeltaControllerDlg::OnBnClickedSwitch)
	ON_BN_CLICKED(IDC_BUTTON9, &CDeltaControllerDlg::OnBnClickedCirlce)
END_MESSAGE_MAP()


// CDeltaControllerDlg ��Ϣ�������
BOOL CDeltaControllerDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
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

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
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


// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CDeltaControllerDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}


//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CDeltaControllerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


#define DOOR_HIGHT1 -650
#define DOOR_HIGHT2 -650
void CDeltaControllerDlg::OnBnClickedDoor()//�����˶�
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	UpdateData(TRUE);

	DOOR_TRAJ_PARS doorPars1, doorPars2;
	doorPars1.startPos.position.x = -152.5;
	doorPars1.startPos.position.y = 0.0;
	doorPars1.startPos.position.z = DOOR_HIGHT1;
	doorPars1.endPos.position.x = 152.5;
	doorPars1.endPos.position.y = 0.0;
	doorPars1.endPos.position.z = DOOR_HIGHT2;
	doorPars1.riseHeight = 25.0;
	doorPars1.radius = 8.0;
	doorPars1.trajPars.velocity = m_motion_par_vel;
	doorPars1.trajPars.acceleration = doorPars1.trajPars.velocity * 100;
	doorPars1.trajPars.splineTime = 0.01;

	doorPars2.startPos.position.x = 152.5;
	doorPars2.startPos.position.y = 0.0;
	doorPars2.startPos.position.z = DOOR_HIGHT1;
	doorPars2.endPos.position.x = -152.5;
	doorPars2.endPos.position.y = 0.0;
	doorPars2.endPos.position.z = DOOR_HIGHT2;
	doorPars2.riseHeight = 25.0 + DOOR_HIGHT1 - DOOR_HIGHT2;
	doorPars2.radius = 8.0;
	doorPars2.trajPars.velocity = m_motion_par_vel;
	doorPars2.trajPars.acceleration = doorPars2.trajPars.velocity * 100;
	doorPars2.trajPars.splineTime = 0.01;


	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(doorPars1.startPos, doorPars1.trajPars);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars1);

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksDoorDelta(doorPars2);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedPtp()//�ƶ���ָ����
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	UpdateData(TRUE);

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = m_motion_par_x;
	pos.position.y = m_motion_par_y;
	pos.position.z = m_motion_par_z;

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

void CDeltaControllerDlg::OnBnClickedInit()//��ʼ��
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	if (((CButton *)GetDlgItem(IDC_CHECK1))->GetCheck())
	{
		nyceStatus = NyceInit(NYCE_SIM);
	}
	else
	{
		nyceStatus = NyceInit(NYCE_ETH);
	}

	if (((CButton *)GetDlgItem(IDC_CHECK2))->GetCheck())
	{
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

void CDeltaControllerDlg::OnBnClickedHome()//����λ
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


void CDeltaControllerDlg::OnBnClickedBrake()//ɲ������
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	NYCE_DIGITAL_IO_ID io;
	io.slotId = NYCE_SLOT0;

	uint32_t ioStatus1(0), ioStatus2(0), ioStatus3(0);
	io.digIONr = NYCE_DIGOUT0;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus1);
	io.digIONr = NYCE_DIGOUT1;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus2);
	io.digIONr = NYCE_DIGOUT2;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus3);

	if (ioStatus1 && ioStatus2 && ioStatus3)
	{
		io.digIONr = NYCE_DIGOUT0;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT1;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT2;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
	}
	else
	{
		io.digIONr = NYCE_DIGOUT0;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT1;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

		io.digIONr = NYCE_DIGOUT2;
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
	}

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedShowPos()//��ʵʱλ�ö�ȡ
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

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedButton10()//-1
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = 0;
	pos.position.y = 0;
	pos.position.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)  
	{
	case 0:
		pos.position.x = -1;
		break;
	case 1:
		pos.position.y = -1;
		break;
	case 2:
		pos.position.z = -1;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 5;
	trajPars.acceleration = 5 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedMove1()//+1
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = 0;
	pos.position.y = 0;
	pos.position.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.position.x = 1;
		break;
	case 1:
		pos.position.y = 1;
		break;
	case 2:
		pos.position.z = 1;
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

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = 0;
	pos.position.y = 0;
	pos.position.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.position.x = -10;
		break;
	case 1:
		pos.position.y = -10;
		break;
	case 2:
		pos.position.z = -10;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 50;
	trajPars.acceleration = 50 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedMove10()//+10
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_COORD pos;
	pos.type = KIN_COORD;
	pos.position.x = 0;
	pos.position.y = 0;
	pos.position.z = 0;
	UpdateData(TRUE);
	switch (m_motion_par_direc)
	{
	case 0:
		pos.position.x = 10;
		break;
	case 1:
		pos.position.y = 10;
		break;
	case 2:
		pos.position.z = 10;
		break;
	}

	TRAJ_PARS trajPars;
	trajPars.velocity = 50;
	trajPars.acceleration = 50 * 100;
	trajPars.splineTime = 0.01;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(pos, trajPars, TRUE);

	StatusHandle(nyceStatus);
}

void CDeltaControllerDlg::OnBnClickedPump()//������ձ�
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	NYCE_DIGITAL_IO_ID io;
	io.slotId = NYCE_SLOT0;
	io.digIONr = NYCE_DIGOUT3;

	uint32_t ioStatus(0);
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);

	if (ioStatus)
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
	else
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedSwitch()//���Ƶ�ŷ�
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	NYCE_DIGITAL_IO_ID io;
	io.slotId = NYCE_SLOT1;
	io.digIONr = NYCE_DIGOUT2;

	uint32_t ioStatus(0);
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);

	if (ioStatus)
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
	else
		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);

	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedCirlce()//Բ�ι켣
{
	// TODO: Add your control notification handler code here
	NYCE_STATUS nyceStatus(NYCE_OK);

	ROCKS_COORD readyPos;
	readyPos.type = KIN_COORD;
	readyPos.position.x = -100;
	readyPos.position.y = 0; 
	readyPos.position.z = -550;

	TRAJ_PARS trajPars;
	trajPars.velocity = m_motion_par_vel;
	trajPars.acceleration = m_motion_par_vel * 100;
	trajPars.splineTime = 0.005;

	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksPtpDelta(readyPos, trajPars);

	CARTESIAN_COORD centerOffset;
	centerOffset.x = 100;
	centerOffset.y = 0;
	centerOffset.z = 0;
	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksCricleDelta(centerOffset, -M_PI * 8, trajPars);

	StatusHandle(nyceStatus);
}
