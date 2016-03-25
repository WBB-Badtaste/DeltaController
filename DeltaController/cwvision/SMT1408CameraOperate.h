// VisionCW.h: interface for the CVisionCW class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_)
#define AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//用于去掉使用map的警告
#ifdef WIN32 
#pragma warning (disable: 4514 4786)
#endif

#include <MAP>
#include "BaslerCamera.h"
#include "CWVision.h"

const int  CAMERAS =  1;//可用相机数
const int MAX_CAMERAS = 5;//相机总数
enum CAMERA_INDEX{
	MARK_LEFT=0, //左MARK相机
	LED_LEFT,//左LED相机
	MARK_RIGHT,//右MARK相机
	LED_RIGHT, //右LED相机
	LED_CHECK//LED检查相机
};//相机的编号

//const CString CAMERA_NAME[MAX_CAMERAS] = {"CIC-300_2","CIC-300_0","CIC-300_1","CIC-300_3","CIC-300_4"};//相机的名称_序号
const CString CAMERA_NAME[MAX_CAMERAS] = {_T("acA1300-30gm_0"),	_T("acA1300-30gm_2"),_T("acA640-120gm_1"),
_T("acA640-120gm_3"),_T("acA640-120gm_4")};//相机的名称_序号
const CString CAMERA_DISP_NAME[MAX_CAMERAS] = {_T("左MARK相机"),_T("左LED相机"),_T("右MARK相机"),
	_T("右LED相机"),_T("LED检查相机")};//相机的显示名称

#define TIMER_CAPTURE_BASLER_IMAGE 1526 //basler相机捕获图像定时器
#define TIMER_CAPTURE_BASLER_IMAGE_HD 1527 //basler相机捕获图像定时器（硬触发）

typedef enum{
	SOFTWARE_MODEL = 1,
	HARWARE_MODEL = 2
}TriggerModel;
class CSMT1408CameraOperate  
{
public:
	CSMT1408CameraOperate();
	virtual ~CSMT1408CameraOperate();

public:
	
	static void InitCameraEnvirionment();//初始化环境 只能调用一次
	bool InitCameras();//初始化相机 只需要调用一次	

	bool OpenCamera(int seq,int imgWidth = 640,int imgHeight = 480);//通过相机索引 打开相机
	void CloseCamera(int seq);//通过相机索引 关闭相机
	bool OpenCamera(CString cameraDispName,int imgWidth = 640,int imgHeight = 480);//通过相机显示名 打开相机
	void CloseCamera(CString cameraDispName);//通过相机显示名 关闭相机

	bool SetCaptureModel(int seq,TriggerModel model);//设置触发模式
	bool SetCaptureModel(CString cameraDispName,TriggerModel model);//设置触发模式

	bool CaptureOne(CString cameraDispName,IMAGE* pImage,bool software = true,double timeOut = INFINITE);//软触发捕获一张/硬触发等待一张
	bool CaptureOne(int seq,IMAGE* pImage,bool software = true,double timeOut = INFINITE);//软触发捕获一张/硬触发等待一张

	void ShowPropertyDlg(CString cameraDispName,bool showSys = false);//显示属性对话框(参数为是否显示系统自带的)
	void ShowPropertyDlg(int seq,bool showSys = false);//显示属性对话框(参数为是否显示系统自带的)

// 	double SetExposure(int seq,double exposureValue);//设置曝光时间单位为ms返回实际的设置值
// 	double GetExposure(int seq);//获取曝光时间单位为ms
// 
// 	__int64 SetGain(int seq,__int64 gainValue);//设置增益值单位为db返回实际的设置值
// 	__int64 GetGain(int seq);//获取增益值单位为db
// 
// 	
// 	bool SaveExposureToFile(int seq,double exposureValue);//保存曝光时间单位为ms
// 	double LoadExposureFromFile(int seq);//从文件获取曝光时间 单位为ms 不存在返回0
// 	   
// 	bool SaveGainToFile(int seq,__int64 gainValue);//保存增益值单位为db
// 	__int64 LoadGainFromFile(int seq);//获取增益值单位为db 不存在返回0

public:
	CString mCameraShowName[CAMERAS];//相机下标对应的实际名称
	
	std::map<CString,CAMERA_INDEX> mCameraMapDispNameToID;//相机的显示名称和索引对应
	std::map<CString,int> mCameraMapDispNameToSeq;//相机的显示名称和扫描顺序对应
private:
	CBaslerCamera mCamera[CAMERAS];//存放相机
};
extern CSMT1408CameraOperate gCameras;
extern int gCaptureTimerIndex;
const int CPI_NULL = -1;
const int CPI_MARK = 1;
const int CPI_MANAGER = 2;

#endif // !defined(AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_)
