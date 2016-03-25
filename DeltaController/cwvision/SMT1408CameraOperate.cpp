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

CSMT1408CameraOperate gCameras;//ȫ�ֹ��õ����
int gCaptureTimerIndex = -1;


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSMT1408CameraOperate::CSMT1408CameraOperate()
{

}

CSMT1408CameraOperate::~CSMT1408CameraOperate(){
	
}

//��������Ӧ�ó������ڵ�·��  
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

//��ʼ�� ������һ�� ��ʼ��ʧ�ܷ���false
//int gDeviceNum = 0;
void CSMT1408CameraOperate::InitCameraEnvirionment(){
	static bool InitCameraEn = false;
	if(InitCameraEn == TRUE) return;
	InitCameraEn = true;
	CBaslerCamera::InitAll();//��ʼ����� ���ܵ���һ��
 }
//��ʼ�����
int gDeviceNum = -1;
bool CSMT1408CameraOperate::InitCameras(){
	static bool InitCamera = false;
	if(InitCamera == TRUE) return false;
	InitCamera = true;	 

	gDeviceNum = CBaslerCamera::EnumerateDevices();//ö����� ���ܵ���һ��
	int i =0;
	std::map<CString,CAMERA_INDEX> cameraMapNameToID;//��������ƺ�������Ӧ
	for(i=0;i<MAX_CAMERAS;i++){
		cameraMapNameToID[CAMERA_NAME[i]] = CAMERA_INDEX(i+1);
		mCameraMapDispNameToID[CAMERA_DISP_NAME[i]] = CAMERA_INDEX(i);
		//g_bCameraRemoved[i] = FALSE;
	}
	if(gDeviceNum != CAMERAS){
		//CString msg;
		//msg.Format("%s�Ҳ��������������·",CAMERA_DISP_NAME[nDeviceNum]);
		AfxMessageBox("������Ҳ��������������·");
		exit(0);
		return FALSE;//�����Ŀ����
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
			AfxMessageBox("������(" + CString(name) + ")����");
			mCamera[i].CloseDevice();//�ر���� ��ֹ��ռ��
			exit(0);
		}
		mCameraShowName[i] = CAMERA_DISP_NAME[index-1];
		mCameraMapDispNameToSeq[mCameraShowName[i]] = i + 1;
		
		//����Ĭ���ع�ֵ������ֵ
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
/* ͨ��������� �����(��������)                                    */
/************************************************************************/
bool CSMT1408CameraOperate::OpenCamera(int seq,int imgWidth/* = 640*/,int imgHeight/* = 480*/){
	if(seq < 0 || seq >= gDeviceNum) return false;
	mCamera[seq].AcquisitionStop();
	mCamera[seq].CloseDevice();//������������� �ȹر�
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
//ͨ��������� �ر����
void CSMT1408CameraOperate::CloseCamera(int seq){
	if(seq < 0 || seq >= gDeviceNum) return ;
	mCamera[seq].AcquisitionStop();
	mCamera[seq].CloseDevice();//�ر����
}
//ͨ�������ʾ�� �����
bool CSMT1408CameraOperate::OpenCamera(CString cameraDispName,int imgWidth/* = 640*/,int imgHeight/* = 480*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return OpenCamera(seq,imgWidth,imgHeight);
}
//ͨ�������ʾ�� �ر����
void CSMT1408CameraOperate::CloseCamera(CString cameraDispName){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	CloseCamera(seq);
}

//���ô���ģʽ
bool CSMT1408CameraOperate::SetCaptureModel(int seq,TriggerModel model){
	bool val = false;
	if(model == SOFTWARE_MODEL) val = mCamera[seq].SetSoftTriggermode();
	else if (model == HARWARE_MODEL) val = mCamera[seq].SetExternalTrigger();
	return val;
}
//���ô���ģʽ
bool CSMT1408CameraOperate::SetCaptureModel(CString cameraDispName,TriggerModel model){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return SetCaptureModel(seq,model);
}

/************************************************************************/
/* ��������һ��                                                       */
/* ����timeOutΪ����ʱʱ�䵥λms��Ĭ��Ϊ���޵ȴ�
/* ����ɹ�����true ����ʧ�ܷ���false
/************************************************************************/
bool CSMT1408CameraOperate::CaptureOne(int seq,IMAGE* pImage,bool software/* = true*/,double timeOut/* = INFINITE*/){
	if(seq < 0 || seq >= gDeviceNum) return false;
	bool result = false;
	if(software) mCamera[seq].SendSoftTriggerCommand();
	mCamera[seq].GrabImage(&result, pImage->m_pImgData,timeOut);
	return result;
}
/************************************************************************/
/* ��������һ��                                                       */
/* ����timeOutΪ����ʱʱ�䵥λms��Ĭ��Ϊ���޵ȴ�
/* ����ɹ�����true ����ʧ�ܷ���false
/************************************************************************/
bool CSMT1408CameraOperate::CaptureOne(CString cameraDispName,IMAGE* pImage,bool software/* = true*/,double timeOut/* = INFINITE*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	return CaptureOne(seq,pImage,software,timeOut);
}

//��ʾ���ԶԻ���(����Ϊ�Ƿ���ʾϵͳ�Դ���)
void CSMT1408CameraOperate::ShowPropertyDlg(int seq,bool showSys/* = false*/){
	if(seq < 0 || seq >= gDeviceNum) return ;
	mCamera[seq].ShowPropertyDlg(showSys);
}
//��ʾ���ԶԻ���(����Ϊ�Ƿ���ʾϵͳ�Դ���)
void CSMT1408CameraOperate::ShowPropertyDlg(CString cameraDispName,bool showSys/* = false*/){
	CAMERA_INDEX seq = (CAMERA_INDEX)(gCameras.mCameraMapDispNameToSeq[cameraDispName] - 1);
	ShowPropertyDlg(seq,showSys);
}

// �����ع�ʱ�䵥λΪms ����ʵ�ʵ�����ֵ
// double CSMT1408CameraOperate::SetExposure(int seq,double exposureValue){
// 	return mCamera[seq].SetExposureTimeAbs(exposureValue*1000);
// }
// ��ȡ�ع�ʱ�䵥λΪms
// double CSMT1408CameraOperate::GetExposure(int seq){
// 	return mCamera[seq].GetExposureTimeAbs()/1000.0;
// }
// ��������ֵ��λΪdb ����ʵ�ʵ�����ֵ
// __int64 CSMT1408CameraOperate::SetGain(int seq,__int64 gainValue){
// 	return mCamera[seq].SetGainRaw(gainValue);
// }
// ��ȡ����ֵ��λΪdb
// __int64 CSMT1408CameraOperate::GetGain(int seq){
// 	return mCamera[seq].GetGainRaw();
// }
// 
// ������������ľ���·��
// const CString paraCameraFilePath = GetCurrentPath() + "\\cameras.para";
// �����ع�ʱ�䵥λΪms
// bool CSMT1408CameraOperate::SaveExposureToFile(int seq,double exposureValue){
// 	CString saveName = mCameraShowName[seq];//�õ��������ʾ�� ��MARK���...
// 	CString writeValue;
// 	writeValue.Format("%.3f",exposureValue);
// 	::WritePrivateProfileString("Exposure",saveName,writeValue,paraCameraFilePath);//������Key����Ҫд��Key���ַ���
// 	return true;
// }
// ���ļ���ȡ�ع�ʱ�� ��λΪms �����ڷ���0
// double CSMT1408CameraOperate::LoadExposureFromFile(int seq){
// 	CString saveName = mCameraShowName[seq];//�õ��������ʾ�� ��MARK���...
// 	char readValue[20];
// 	::GetPrivateProfileString("Exposure",saveName,"0",readValue,20,paraCameraFilePath);//
// 	double rt = atof(readValue);
// 	return rt;
// }
// ��������ֵ��λΪdb   
// bool CSMT1408CameraOperate::SaveGainToFile(int seq,__int64 gainValue){
// 	CString saveName = mCameraShowName[seq];//�õ��������ʾ�� ��MARK���...
// 	CString writeValue;
// 	writeValue.Format("%d",gainValue);
// 	::WritePrivateProfileString("Gain",saveName,writeValue,paraCameraFilePath);//������Key����Ҫд��Key���ַ���
// 	return true;
// }
// ��ȡ����ֵ��λΪdb �����ڷ���0
// __int64 CSMT1408CameraOperate::LoadGainFromFile(int seq){
// 	CString saveName = mCameraShowName[seq];//�õ��������ʾ�� ��MARK���...
// 	char readValue[20];
// 	__int64 rt = ::GetPrivateProfileInt("Gain",saveName,0,paraCameraFilePath);//
// 	return rt;
// }
