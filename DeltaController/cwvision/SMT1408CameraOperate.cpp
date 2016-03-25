// VisionCW.cpp: implementation of the CVisionCW class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SMT1408CameraOperate.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

CSMT1408CameraOperate gCameras;//全局公用的相机
int gCaptureTimerIndex = -1;


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSMT1408CameraOperate::CSMT1408CameraOperate()
{

}

CSMT1408CameraOperate::~CSMT1408CameraOperate(){
	
}

//函数返回应用程序所在的路径  
static CString GetCurrentPath()  
{   
	CString    sPath;   
	GetModuleFileName(NULL,sPath.GetBufferSetLength(MAX_PATH+1),MAX_PATH);   
	sPath.ReleaseBuffer();   
	int    nPos;   
	nPos=sPath.ReverseFind('\\');   
	sPath=sPath.Left(nPos);   
	return    sPath;   
}

//初始化 仅调用一次 初始化失败返回false
//int gDeviceNum = 0;
void CSMT1408CameraOperate::InitCameraEnvirionment(){
	static bool InitCameraEn = false;
	if(InitCameraEn == TRUE) return;
	InitCameraEn = true;
	CBaslerCamera::InitAll();//初始化相机 仅能调用一次
 }
//初始化相机
int gDeviceNum = -1;
bool CSMT1408CameraOperate::InitCameras(){
	static bool InitCamera = false;
	if(InitCamera == TRUE) return false;
	InitCamera = true;	 

	gDeviceNum = CBaslerCamera::EnumerateDevices();//枚举相机 仅能调用一次
	int i =0;
	std::map<CString,CAMERA_INDEX> cameraMapNameToID;//相机的名称和索引对应
	for(i=0;i<MAX_CAMERAS;i++){
		cameraMapNameToID[CAMERA_NAME[i]] = CAMERA_INDEX(i+1);
		mCameraMapDispNameToID[CAMERA_DISP_NAME[i]] = CAMERA_INDEX(i);
		//g_bCameraRemoved[i] = FALSE;
	}
	if(gDeviceNum != CAMERAS){
		//CString msg;
		//msg.Format("%s找不到，请检查相机线路",CAMERA_DISP_NAME[nDeviceNum]);
		AfxMessageBox("有相机找不到，请检查相机线路");
		exit(0);
		return FALSE;//相机数目不对
		//return TRUE ;
	}
	i = 0;
	char deviceName[256];
	char deviceID[256];
	char name[512];
	while(i<gDeviceNum){
		mCamera[i].SetDeviceIndex(i);
		mCamera[i].OpenDevice();
		mCamera[i].GetDeviceModelName(deviceName);
		mCamera[i].GetDeviceUserID(deviceID);
		sprintf(name,"%s_%s",deviceName,deviceID);
		int index = cameraMapNameToID[name];
		if(index <= 0) {
			AfxMessageBox("相机编号(" + CString(name) + ")错误");
			mCamera[i].CloseDevice();//关闭相机 防止被占用
			exit(0);
		}
		mCameraShowName[i] = CAMERA_DISP_NAME[index-1];
		mCameraMapDispNameToSeq[mCameraShowName[i]] = i + 1;
		
		//加载默认曝光值和增益值
		mCamera[i].LoadParameters();
		//CAMERA_INDEX id = CAMERA_INDEX(i);
// 		double exposure = LoadExposureFromFile(i);
// 		if(exposure>0) SetExposure(i,exposure);
// 		__int64 gain = LoadGainFromFile(i);
// 		if(gain>0) SetGain(i,gain);

				
		mCamera[i].CloseDevice();
		i++;
	}
	return true;
}
/************************************************************************/
/* 通过相机索引 打开相机(设置软触发)                                    */
/************************************************************************/
bool CSMT1408CameraOperate::OpenCamera(int seq,int imgWidth/* = 640*/,int imgHeight/* = 480*/){
	if(seq < 0 || seq >= gDeviceNum) return false;
	mCamera[seq].AcquisitionStop();
	mCamera[seq].CloseDevice();//不管是有有相机 先关闭
	mCamera[seq].SetDeviceIndex(seq);
	mCamera[seq].OpenDevice();
	int Width = mCamera[seq].GetWidthMax();
	int Height = mCamera[seq].GetHeightMax();
	int offx = (Width - imgWidth)/2;
	int offy = (Height - imgHeight)/2;
	Width = imgWidth;
	Height = imgHeight;
	// Beware : Windows Bitmaps have to have a DWORD alligned width :-(
	mCamera[seq].SetWidth(Width);
	mCamera[seq].SetHeight(Height);
	mCamera[seq].SetOffsetX(offx);
	mCamera[seq].SetOffsetY(offy);
	//mCamera.SetFreerunMode();
	mCamera[seq].SetSoftTriggermode();
	
	mCamera[seq].AcquisitionPrepare();
	mCamera[seq].AcquisitionStart();
	return true;
}
//通过相机索引 关闭相机
void CSMT1408CameraOperate::CloseCamera(int seq){
	if(seq < 0 || seq >= gDeviceNum) return ;
	mCamera[seq].AcquisitionStop();
	mCamera[seq].CloseDevice();//关闭相机
}
//通过相机显示名 打开相机
bool CSMT1408CameraOperate::OpenCamera(CString cameraDispName,int imgWidth/* = 640*/,int imgHeight/* = 480*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return OpenCamera(seq,imgWidth,imgHeight);
}
//通过相机显示名 关闭相机
void CSMT1408CameraOperate::CloseCamera(CString cameraDispName){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	CloseCamera(seq);
}

//设置触发模式
bool CSMT1408CameraOperate::SetCaptureModel(int seq,TriggerModel model){
	bool val = false;
	if(model == SOFTWARE_MODEL) val = mCamera[seq].SetSoftTriggermode();
	else if (model == HARWARE_MODEL) val = mCamera[seq].SetExternalTrigger();
	return val;
}
//设置触发模式
bool CSMT1408CameraOperate::SetCaptureModel(CString cameraDispName,TriggerModel model){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return SetCaptureModel(seq,model);
}

/************************************************************************/
/* 软触发捕获一张                                                       */
/* 参数timeOut为捕获超时时间单位ms，默认为无限等待
/* 捕获成功返回true 捕获失败返回false
/************************************************************************/
bool CSMT1408CameraOperate::CaptureOne(int seq,IMAGE* pImage,bool software/* = true*/,double timeOut/* = INFINITE*/){
	if(seq < 0 || seq >= gDeviceNum) return false;
	bool result = false;
	if(software) mCamera[seq].SendSoftTriggerCommand();
	mCamera[seq].GrabImage(&result, pImage->m_pImgData,timeOut);
	return result;
}
/************************************************************************/
/* 软触发捕获一张                                                       */
/* 参数timeOut为捕获超时时间单位ms，默认为无限等待
/* 捕获成功返回true 捕获失败返回false
/************************************************************************/
bool CSMT1408CameraOperate::CaptureOne(CString cameraDispName,IMAGE* pImage,bool software/* = true*/,double timeOut/* = INFINITE*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return CaptureOne(seq,pImage,software,timeOut);
}

//显示属性对话框(参数为是否显示系统自带的)
void CSMT1408CameraOperate::ShowPropertyDlg(int seq,bool showSys/* = false*/){
	if(seq < 0 || seq >= gDeviceNum) return ;
	mCamera[seq].ShowPropertyDlg(showSys);
}
//显示属性对话框(参数为是否显示系统自带的)
void CSMT1408CameraOperate::ShowPropertyDlg(CString cameraDispName,bool showSys/* = false*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	ShowPropertyDlg(seq,showSys);
}

// 设置曝光时间单位为ms 返回实际的设置值
// double CSMT1408CameraOperate::SetExposure(int seq,double exposureValue){
// 	return mCamera[seq].SetExposureTimeAbs(exposureValue*1000);
// }
// 获取曝光时间单位为ms
// double CSMT1408CameraOperate::GetExposure(int seq){
// 	return mCamera[seq].GetExposureTimeAbs()/1000.0;
// }
// 设置增益值单位为db 返回实际的设置值
// __int64 CSMT1408CameraOperate::SetGain(int seq,__int64 gainValue){
// 	return mCamera[seq].SetGainRaw(gainValue);
// }
// 获取增益值单位为db
// __int64 CSMT1408CameraOperate::GetGain(int seq){
// 	return mCamera[seq].GetGainRaw();
// }
// 
// 保存相机参数的绝对路径
// const CString paraCameraFilePath = GetCurrentPath() + "\\cameras.para";
// 保存曝光时间单位为ms
// bool CSMT1408CameraOperate::SaveExposureToFile(int seq,double exposureValue){
// 	CString saveName = mCameraShowName[seq];//得到相机的显示名 左MARK相机...
// 	CString writeValue;
// 	writeValue.Format("%.3f",exposureValue);
// 	::WritePrivateProfileString("Exposure",saveName,writeValue,paraCameraFilePath);//段名，Key名，要写入Key的字符串
// 	return true;
// }
// 从文件获取曝光时间 单位为ms 不存在返回0
// double CSMT1408CameraOperate::LoadExposureFromFile(int seq){
// 	CString saveName = mCameraShowName[seq];//得到相机的显示名 左MARK相机...
// 	char readValue[20];
// 	::GetPrivateProfileString("Exposure",saveName,"0",readValue,20,paraCameraFilePath);//
// 	double rt = atof(readValue);
// 	return rt;
// }
// 保存增益值单位为db   
// bool CSMT1408CameraOperate::SaveGainToFile(int seq,__int64 gainValue){
// 	CString saveName = mCameraShowName[seq];//得到相机的显示名 左MARK相机...
// 	CString writeValue;
// 	writeValue.Format("%d",gainValue);
// 	::WritePrivateProfileString("Gain",saveName,writeValue,paraCameraFilePath);//段名，Key名，要写入Key的字符串
// 	return true;
// }
// 获取增益值单位为db 不存在返回0
// __int64 CSMT1408CameraOperate::LoadGainFromFile(int seq){
// 	CString saveName = mCameraShowName[seq];//得到相机的显示名 左MARK相机...
// 	char readValue[20];
// 	__int64 rt = ::GetPrivateProfileInt("Gain",saveName,0,paraCameraFilePath);//
// 	return rt;
// }
