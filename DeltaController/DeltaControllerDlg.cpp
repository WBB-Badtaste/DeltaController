
// DeltaControllerDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "DeltaController.h"
#include "DeltaControllerDlg.h"
#include "afxdialogex.h"

#include "Defines.h"
//NYCE 
//不能放置于H文件中 出现LINK2005ERROR
// #include "AxisControl.h"
// #include "RocksControl.h"
// #include "NyceExErrorHandle.h"

#include "MotionStateMach.h"

#ifdef _DEBUGF
#define new DEBUG_NEW
#endif


const int g_imgWidth = 640 * 2;
const int g_imgHeight = 480 * 2;


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
	, m_edit_locate_result(_T(""))
	, m_edit_match_time(0.000)
	, m_tTime(0.0)
	//, m_msm(this->m_hWnd)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_tTime = 0.0;
	m_turn_angle = 0.0;


	//  m_dRobotPos_x = 0.0;
	//  m_dRobotPos_y = 0.0;
	//  m_dRobotPos_z = 0.0;
}


CDeltaControllerDlg::~CDeltaControllerDlg()
{
	for(int i=0;i<CAMERAS;i++)
	{
		gCameras.CloseCamera(i);
	}
	
	//testing
	delete m_pMsm;
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
	DDX_Control(pDX, IDC_COMB_MODEL_SHAP, m_combctr_markShape);
	DDX_Text(pDX, IDC_EDIT_RESULT, m_edit_locate_result);
	DDX_Text(pDX, IDC_EDIT_COSTTIME, m_edit_match_time);
	//  DDX_Text(pDX, IDC_EDIT5, m_tTime);
	//	DDX_Text(pDX, IDC_EDIT5, m_tTime);
	//	DDV_MinMaxDouble(pDX, m_tTime, 0.0, 100);
	DDX_Text(pDX, IDC_EDIT6, m_turn_angle);
	DDV_MinMaxDouble(pDX, m_turn_angle, -360, 360);
	//  DDX_Text(pDX, IDC_EDIT_POS_X, m_dRobotPos_x);
	//  DDX_Text(pDX, IDC_EDIT_POS_Y, m_dRobotPos_y);
	//  DDX_Text(pDX, IDC_EDIT_POS_Z, m_dRobotPos_z);
	DDX_Control(pDX, IDC_EDIT_POS_X, m_bobotPos_x);
	DDX_Control(pDX, IDC_EDIT_POS_Y, m_bobotPos_y);
	DDX_Control(pDX, IDC_EDIT_POS_Z, m_bobotPos_z);
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
//	ON_BN_CLICKED(IDC_BUTTON5, &CDeltaControllerDlg::OnBnClickedShowPos)
	ON_BN_CLICKED(IDC_BUTTON10, &CDeltaControllerDlg::OnBnClickedButton10)
	ON_BN_CLICKED(IDC_BUTTON13, &CDeltaControllerDlg::OnBnClickedMove1)
	ON_BN_CLICKED(IDC_BUTTON11, &CDeltaControllerDlg::OnBnClickedButton11)
	ON_BN_CLICKED(IDC_BUTTON12, &CDeltaControllerDlg::OnBnClickedMove10)
	ON_BN_CLICKED(IDC_BUTTON1, &CDeltaControllerDlg::OnBnClickedPump)
	ON_BN_CLICKED(IDC_BUTTON4, &CDeltaControllerDlg::OnBnClickedSwitch)
	ON_BN_CLICKED(IDC_BUTTON9, &CDeltaControllerDlg::OnBnClickedCirlce)
	ON_BN_CLICKED(IDC_BUTTON14, &CDeltaControllerDlg::OnBnClickedCatch)
	ON_BN_CLICKED(IDC_BUTTON16, &CDeltaControllerDlg::OnBnClickedBelt)
	ON_BN_CLICKED(IDC_BTN_OPENCAMERA, &CDeltaControllerDlg::OnBnClickedBtnOpencamera)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BTN_ATTRIBUTE, &CDeltaControllerDlg::OnBnClickedBtnAttribute)
	ON_BN_CLICKED(IDC_CHECK_SEARCHAREA, &CDeltaControllerDlg::OnBnClickedCheckSearcharea)
	ON_BN_CLICKED(IDC_BTN_LEARN_MODEL, &CDeltaControllerDlg::OnBnClickedBtnLearnModel)
	ON_BN_CLICKED(IDC_BTN_MATCH_MODEL, &CDeltaControllerDlg::OnBnClickedBtnMatchModel)
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MOUSEMOVE()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONDBLCLK()

	//----JoMar,20160116
	ON_MESSAGE(WM_MATCH_MODEL, OnMatchModel) 
	ON_MESSAGE(WM_HANDLE_NYCESTATUS, OnHanderNyceStatus) 
	ON_MESSAGE(WM_UPDATE_ROBOT_POS, OnUpdateRobotPos) 
	ON_EN_CHANGE(IDC_EDIT5, &CDeltaControllerDlg::OnEnChangeEdit5)
	ON_BN_CLICKED(IDC_BUTTON19, &CDeltaControllerDlg::OnBnClickedButton19)
	ON_BN_CLICKED(IDC_BUTTON_TURN, &CDeltaControllerDlg::OnBnClickedButtonTurn)
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
	//初始化相机
	CSMT1408CameraOperate::InitCameraEnvirionment();//初始化相机 仅能调用一次
	if(gCameras.InitCameras() == false) 
		return FALSE;//初始化相机 仅能调用一次
// 	for(int i=0;i<CAMERAS;i++){
// 		m_combctr_cameraName.AddString(gCameras.mCameraShowName[i]);
// 	}
// 	if(m_combctr_cameraName.GetCount()>0) m_combctr_cameraName.SetCurSel(0);

	cwSetReversePara(FALSE);//

	//cwVision 显示图像相关
	m_visionCW.InitParameters();
	m_visionCW.SetImageInf(NULL,8, g_imgWidth,g_imgHeight);
	m_visionCW.SetDisplayWnd(((CStatic*)GetDlgItem(IDC_IMAGE_SHOW))->m_hWnd);
	m_visionCW.SetMainWnd(m_hWnd);

	//获取源图控件的位置
	CRect rect;
	GetDlgItem( IDC_IMAGE_SHOW )->GetWindowRect(&rect);
	ScreenToClient(rect);//转化为对话框上的相对位置
	m_visionCW.SetDisplayRect(rect);

	// 	m_combctr_cameraName.AddString("DMK 23G274 [1]");
	// 	m_combctr_cameraName.SetCurSel(0);	

	//初始化mark标定参数
	m_combctr_markShape.AddString("圆形");
//	m_combctr_markShape.AddString("正方形");
	m_combctr_markShape.AddString("矩形");
// 	m_combctr_markShape.AddString("带拟合的矩形");
// 	m_combctr_markShape.AddString("角点");
	m_combctr_markShape.AddString("任意图像");
	m_combctr_markShape.SetCurSel(2);
	//m_combctr_markShape

	//	m_visionCW.SetFullScreen();

	OFFSETZOOM zoom;
	zoom.x = 0;
	zoom.y = 0;
	zoom.scale = 0.5;

	m_visionCW.SetDisplayOffset(zoom);

	//testing
	m_pMsm = new CMotionStateMach(this->m_hWnd);

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
	HWND showWnd = GetDlgItem(IDC_IMAGE_SHOW)->m_hWnd;
	m_visionCW.SetDisplayWnd(showWnd);
	m_visionCW.DispalyImage();
 	m_visionCW.DisplayROI();
	m_visionCW.DisplayMatchResult();

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



void CDeltaControllerDlg::OnBnClickedDoor()//门型运动
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);

	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if(!m_pMsm->SwitchToDoorState(m_motion_par_vel))
		PrintStr("Robot is not ready.");
}


void CDeltaControllerDlg::OnBnClickedPtp()//移动到指定点
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);

	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if (m_motion_par_x == 0 || m_motion_par_y == 0 || m_motion_par_z == 0)
	{
		PrintStr("Please input a position.");
		return;
	}


	if(!m_pMsm->SwitchToPtpState(m_motion_par_x, m_motion_par_y, m_motion_par_z, m_motion_par_vel))
		PrintStr("Robot is not ready.");

}

void CDeltaControllerDlg::OnBnClickedInit()//初始化
{
	// TODO: Add your control notification handler code here
	if(!m_pMsm->SwitchToInitState())
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnDestroy()
{
	CDialogEx::OnDestroy();

	// TODO: Add your message handler code here

}


void CDeltaControllerDlg::OnBnClickedCheck1()
{
	// TODO: Add your control notification handler code here
/*	NYCE_STATUS nyceStatus(NYCE_OK);*/
}

void CDeltaControllerDlg::OnBnClickedHome()//回零位
{
	// TODO: Add your control notification handler code here
	if(!m_pMsm->SwitchToHomeState())
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnBnClickedBrake()//刹车控制
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT0;
// 
// 	uint32_t ioStatus1(0), ioStatus2(0), ioStatus3(0);
// 	io.digIONr = NYCE_DIGOUT0;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus1);
// 	io.digIONr = NYCE_DIGOUT1;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus2);
// 	io.digIONr = NYCE_DIGOUT2;
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus3);
// 
// 	if (ioStatus1 && ioStatus2 && ioStatus3)
// 	{
// 		io.digIONr = NYCE_DIGOUT0;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT1;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT2;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	}
// 	else
// 	{
// 		io.digIONr = NYCE_DIGOUT0;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT1;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 		io.digIONr = NYCE_DIGOUT2;
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 	}
// 
// 	StatusHandle(nyceStatus);
}


//void CDeltaControllerDlg::OnBnClickedShowPos()//打开实时位置读取
//{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	double position[6];
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : RocksReadPosDelta(position);
// 
// 	SYSTEMTIME time;
// 	GetSystemTime(&time);
// 	CString timeStr("");
// 	timeStr.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
// 	m_listBox.AddString((LPCTSTR)timeStr);
// 	CString posStr("");
// 	posStr.Format(_T("x:%0.2f y:%0.2f z:%0.2f"), position[0], position[1], position[2]);
// 	m_listBox.AddString((LPCTSTR)posStr);
// 	m_listBox.AddString("");
// 
// 	StatusHandle(nyceStatus);
//}


void CDeltaControllerDlg::OnBnClickedButton10()//-1
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(-1, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedMove1()//+1
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(1, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedButton11()//-10
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(-10, m_motion_par_direc);
}


void CDeltaControllerDlg::OnBnClickedMove10()//+10
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	m_pMsm->SwitchToJogState(10, m_motion_par_direc);
}

void CDeltaControllerDlg::OnBnClickedPump()//控制真空泵
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT0;
// 	io.digIONr = NYCE_DIGOUT3;
// 
// 	uint32_t ioStatus(0);
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);
// 
// 	if (ioStatus)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedSwitch()//控制电磁阀
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	NYCE_DIGITAL_IO_ID io;
// 	io.slotId = NYCE_SLOT3;
// 	io.digIONr = NYCE_DIGOUT2;
// 
// 	uint32_t ioStatus(0);
// 	nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiReadDigitalIO(noId[0], io, &ioStatus);
// 
// 	if (ioStatus)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiClearDigitalOutput(noId[0], io);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : NhiSetDigitalOutput(noId[0], io);
// 
// 	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedCirlce()//圆形轨迹
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	if (m_motion_par_vel == 0)
	{
		PrintStr("Please input a velocity.");
		return;
	}

	if(!m_pMsm->SwitchToCircState(m_motion_par_vel))
	{
		PrintStr("Robot is not ready.");
	}
}


void CDeltaControllerDlg::OnBnClickedCatch()
{

	if(!m_pMsm->SwitchToCatchState())
	{
		PrintStr("Robot is not ready.");
	}

}


void CDeltaControllerDlg::OnBnClickedBelt()//传送带控制
{
	// TODO: Add your control notification handler code here
// 	NYCE_STATUS nyceStatus(NYCE_OK);
// 
// 	SAC_STATE state;
// 	SAC_SPG_STATE spgState;
// 	SacReadState(beltId[0], &state, &spgState);
// 
// 	SAC_JOG_PARS jogPars;
// 	jogPars.velocity = BELT_VEL * BELT_BASE_RATE;
// 	jogPars.acceleration = jogPars.velocity * 10;
// 	jogPars.jerk = jogPars.velocity * 100;
// 
// 	if (state == SAC_READY)
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStartJog(beltId[0], &jogPars);
// 	else
// 		nyceStatus = NyceError(nyceStatus) ? nyceStatus : SacStopJog(beltId[0], &jogPars);
// 
// 	StatusHandle(nyceStatus);
}


void CDeltaControllerDlg::OnBnClickedBtnOpencamera()
{
	UpdateData(TRUE);

	CString btn_name;
	CString cameraName= CAMERA_DISP_NAME[0];
	((CButton*)GetDlgItem(IDC_BTN_OPENCAMERA))->GetWindowText(btn_name);
	if(  btn_name == _T("打开相机")){

		bool rt = gCameras.OpenCamera(cameraName, 1280, 960);
		if(FALSE == rt ) return;//618

		btn_name.Format("%s","关闭相机");
		SetTimer(TIMER_CAPTURE_BASLER_IMAGE,200,NULL);
	}else{
		btn_name.Format("%s","打开相机");
		KillTimer(TIMER_CAPTURE_BASLER_IMAGE);
		gCameras.CloseCamera(cameraName);
	}
	((CButton*)GetDlgItem(IDC_BTN_OPENCAMERA))->SetWindowText(btn_name);

	UpdateData(FALSE);
}

//定时器消息响应函数 
bool captureComplate = true;
void CDeltaControllerDlg::OnTimer(UINT_PTR nIDEvent)
{

	switch(nIDEvent){
	case TIMER_CAPTURE_BASLER_IMAGE:
		{
			if(captureComplate == true) { //防止曝光值过大导致捕获次数过多
					captureComplate = false;
					//UpdateData();
					//CString cameraName;
					//m_combctr_cameraName.GetLBText(m_combctr_cameraName.GetCurSel(),cameraName);
					//clock_t start=clock();
					if( FALSE == gCameras.CaptureOne(CAMERA_DISP_NAME[0],m_visionCW.m_pImage)){
						KillTimer(TIMER_CAPTURE_BASLER_IMAGE);
					}

// 					clock_t finished = clock();
// 					m_editCostTime = (double)(finished-start)/CLOCKS_PER_SEC;
// 					if(m_check_matchContinues){ //持续运行
// 						OnBtnMatchModel();
// 					}

					//UpdateData(FALSE);
					OnPaint();
					captureComplate = true;
				}
			}
		}
	CDialogEx::OnTimer(nIDEvent);
}


void CDeltaControllerDlg::OnBnClickedBtnAttribute()
{
	CString cameraName  = CAMERA_DISP_NAME[0];
	gCameras.ShowPropertyDlg(cameraName);
}


//是否使用搜索区域
void CDeltaControllerDlg::OnBnClickedCheckSearcharea() 
{
	bool ck = ((CButton*)GetDlgItem(IDC_CHECK_SEARCHAREA))->GetCheck();
	m_visionCW.SetSerachArea(ck,20);
	OnPaint();
}


void CDeltaControllerDlg::OnBnClickedBtnLearnModel()
{
	UpdateData(TRUE);
	LEARNOPTION Option;

	Option.minScore = 600;			// 搜索匹配对象的最小分数(0~1000)；
	Option.maxAngle = 180;			// 最大角度（-180~180）；
	Option.minAngle = -180;			// 最小角度（-180~180）；
	Option.maxScale = 102;			// 最小比例（按照百分比，100为1倍）；
	Option.minScale = 98;			// 最大比例（按照百分比，100为1倍）；
	
	Option.color = 0; //颜色
	Option.polarity = 0; //极性
	Option.edgeThreashold = 30;
	Option.intersectDiff = 5;

	OnPaint();

	SHAP shap = SHAP_NULL;
	int index = m_combctr_markShape.GetCurSel();
	if(index == 0) shap = SHAP_CIRCLE;//圆模板
//	else if(index == 1) shap = SHAP_SQUARE;//正方形模板
	else if(index == 1) shap = SHAP_RECTANGLE;//矩形模板
//	else if(index == 3) shap = SHAP_SRECTANGLE;//拟合矩形
//	else if(index == 4) shap = SHAP_CORNER;//角点
	else if(index == 2) shap = SHAP_OTHER;//任意图形模板

	if(m_visionCW.LearnModel(shap,&Option,true) == true){
		//	cwFreeMemory(m_dispRoi);
		//	m_dispRoi = NULL;
	}
	else MessageBox("创建失败");

	OnPaint();
}


void CDeltaControllerDlg::OnBnClickedBtnMatchModel()
{
	UpdateData();

	OnPaint();
	FINDOPTION Option;	
	Option.minScore =  600;					// 搜索匹配对象的最小分数(0~1000)；
	Option.maxAngle =  180;					// 最大角度（-180~180）；
	Option.minAngle = -180;					// 最小角度（-180~180）；
	Option.maxScale =  102;					// 最小比例（按照百分比，100为1倍）；
	Option.minScale =	98;					// 最大比例（按照百分比，100为1倍）；
	Option.needEdge_or_level = 20;			// 最短的边缘，用以限制那些长度小于该值的边缘，以便减少噪音；
	Option.threshold_or_maxError = 30;		// 边缘梯度阈值，只有那些梯度大于该阈值的边缘点才被检测出来；
	Option.findCount = 30;					// 需要搜索的个数；
	//	Option.findLevel = 1;				// 搜索水平(1~10)，搜索水平越高，能搜索到的匹配对象越多，但速度会变慢；
	//	Option.matchMode = 0;				// 0为正常，1为反转，2任意；

	clock_t start,finish;
	double totaltime;
	start=clock();

	int c;
	m_match_result = m_visionCW.FindModel(&Option,&c);

// 	MODEL* pModel = cwLoadModel( "C:/mod.mod" );
// 	m_match_result = m_visionCW.FindModel(pModel, 0, &Option, &c);

	//	g_img_dib ->ReplaceDib(mPImage->GetDimensions(),mPImage->m_nBitCount,mPImage->m_lpColorTable, mPImage->m_pImgData);	

	finish=clock();
	totaltime =(double)(finish-start)/CLOCKS_PER_SEC;
	m_edit_match_time = totaltime;


	if(m_match_result==NULL) {
		m_edit_locate_result = "匹配错误";
		UpdateData(FALSE);

		//让状态机在等待匹配结果，现在恢复线程
		m_pMsm->FinlishMatch(m_match_result->position.x, m_match_result->position.y, m_match_result->angle, false);

		return;
	}

	m_edit_locate_result = "匹配成功";
	m_visionCW.DestroyRoi();
	
	UpdateData(FALSE);
	OnPaint();

	//让状态机在等待匹配结果，现在恢复线程
	m_pMsm->FinlishMatch(m_match_result->position.x, m_match_result->position.y, m_match_result->angle, true);
}

//左键按下事件
void CDeltaControllerDlg::OnLButtonDown(UINT nFlags, CPoint point) 
{
	int roiType;
	int index = m_combctr_markShape.GetCurSel();
	if(index == 0)	roiType = ROI_CIRCLE;		//创建圆形的roi
//	else if(index == 1) roiType = ROI_FSQUARE;	//创建带旋转的正方形ROI
	else if(index == 1) roiType = ROI_FRECT;	//创建矩形ROI
//	else if(index == 3) roiType = ROI_RECT_FIT;	//创建拟合矩形ROI
//	else if(index == 4) roiType = ROI_CORNER;	//创建角点
	else  roiType = ROI_FRECT; //创建矩形roi	//CreateRoi(ROI_RRECT);//创建带旋转的矩形ROI
	m_visionCW.LButtonDown(nFlags,point,roiType);

	CDialog::OnLButtonDown(nFlags, point);
}

void CDeltaControllerDlg::OnLButtonUp(UINT nFlags, CPoint point) 
{
	// TODO: Add your message handler code here and/or call default
	m_visionCW.LButtonUp(nFlags,point);

	CDialog::OnLButtonUp(nFlags, point);
}

//鼠标移动事件
void CDeltaControllerDlg::OnMouseMove(UINT nFlags, CPoint point) 
{
	m_visionCW.MouseMove(nFlags,point);

	CDialog::OnMouseMove(nFlags, point);
}

//鼠标滚动事件
BOOL CDeltaControllerDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt) 
{
	m_visionCW.MouseWheel(nFlags,zDelta,pt);

	return CDialog::OnMouseWheel(nFlags, zDelta, pt);
}

//右键按下
void CDeltaControllerDlg::OnRButtonDown(UINT nFlags, CPoint point) 
{
	m_visionCW.RButtonDown(nFlags,point);

	CDialog::OnRButtonDown(nFlags, point);
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
//右键双击事件
void CDeltaControllerDlg::OnRButtonDblClk(UINT nFlags, CPoint point) 
{
	m_visionCW.RButtonDblClk(nFlags,point);

	CDialog::OnRButtonDblClk(nFlags, point);
}

//----JoMar,20160116
LRESULT CDeltaControllerDlg::OnMatchModel(WPARAM wParam, LPARAM lParam) 
{
	OnBnClickedBtnMatchModel();

	return 0;
}  

LRESULT CDeltaControllerDlg::OnUpdateRobotPos(WPARAM wParam, LPARAM lParam)
{

	double *pos = (double *)lParam;
	CString str("");

	str.Format(_T("%f"), pos[0]);
	m_bobotPos_x.SetWindowText(str);

	str.Format(_T("%f"), pos[1]);
	m_bobotPos_y.SetWindowText(str);
	
	str.Format(_T("%f"), pos[2]);
	m_bobotPos_z.SetWindowText(str);
	
	return 0;
}

LRESULT CDeltaControllerDlg::OnHanderNyceStatus(WPARAM wParam, LPARAM lParam)
{
	m_listBox.AddString(*(CString*)lParam);
	return 0;
}

void CDeltaControllerDlg::OnEnChangeEdit5()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}


void CDeltaControllerDlg::OnBnClickedButton19()
{
	// TODO: Add your control notification handler code here
	m_visionCW.SaveMod("C:/mod.mod");
}


void CDeltaControllerDlg::OnBnClickedButtonTurn()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
//	RocksRotateAngle(m_turn_angle);
	UpdateData(FALSE);
}


void CDeltaControllerDlg::PrintStr(const CString &str)
{
	CString string;
	SYSTEMTIME time;
	GetSystemTime(&time);
	string = "";
	string.Format(_T("%04u/%02u/%02u %02u:%02u:%02u"), time.wYear, time.wMonth, time.wDay, time.wHour + 8, time.wMinute, time.wSecond);
	string += ": ";
	string += str;
	string += "\n";
	m_listBox.AddString(string);
}