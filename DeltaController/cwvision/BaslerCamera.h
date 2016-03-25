// Basler.h : header file
//

#if !defined(AFX_PYLON_H__41AA6513_E0F8_4C97_8540_11F569B6FD24__INCLUDED_)
#define AFX_PYLON_H__41AA6513_E0F8_4C97_8540_11F569B6FD24__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include <AFXWIN.H>
//#include "windows.h"                 // 前面加上这个

#include <malloc.h>
#include <pylonc/PylonC.h>

enum e_ExposureAuto {ExposureAuto_Off, ExposureAuto_Once, ExposureAuto_Continuous}; 
enum e_GainAuto {GainAuto_Off, GainAuto_Once, GainAuto_Continuous}; 

#define CAMERA_USED_NUM	5 // Number of camera used in the system
#define NUM_BUFFERS		40         /* Number of buffers used for grabbing. actually, this can equal to half of frames rate */

#define CHECK(errc) 

class __declspec(dllexport) CBaslerCamera
{
public:
	CBaslerCamera();
	virtual ~CBaslerCamera();

public:

	// 三个static函数，不管使用多少相机，也只能被调用一次
	static void		InitAll();
	static void		PylonTerminateAll();
	//返回相机数
	static int EnumerateDevices();	

	//根据自定义的相机的UserID得到枚举的相机列表中的相机序号，返回相机的序列号
	int  GetDeviceIndexFromUserID(char UserID[]);
	bool SetDeviceIndex(int ndeviceIndex);

	bool GetDeviceSN(char name[]);//获取相机唯一序列号
	bool GetDeviceUserID(char name[]);//获取相机ID 用户自定义名
	bool GetDeviceModelName(char name[]);//获取相机类型名

	BOOL OpenDevice();
	BOOL CloseDevice();
	void DestroyDevice();

	void ShowPropertyDlg(bool showSys = false);//显示属性对话框(参数为是否显示系统自带的)
	bool SaveParameters();//保存参数(自动调用)
	bool LoadParameters();//加载参数(自动调用)
	
	__int64 GetGainValue();//获取增益值
	bool GetGainRange(__int64* minVal,__int64* maxVal);//获取增益值范围
	bool SetGainValue(__int64 val);//设置增益值
	
	__int64 GetExposureValue();//获取曝光值
	bool GetExposureRange(__int64* minVal,__int64* maxVal);//获取曝光值范围
	bool SetExposureValue(__int64 val);//设置曝光值
	
	__int64 GetContrastValue();//获取对比度值
	bool GetContrastRange(__int64* minVal,__int64* maxVal);//获取对比度值范围
	bool SetContrastValue(__int64 val);//设置对比度值
	
	__int64 GetBrightnessValue();//获取亮度值
	bool GetBrightnessRange(__int64* minVal,__int64* maxVal);//获取亮度值范围
	bool SetBrightnessValue(__int64 val);//设置亮度值


	BOOL IsDeviceRemoved();
	BOOL InitialSetup();

	// set Image format
	BOOL SetImageFormatToMono8();

	BOOL SetSoftTriggermode();
	BOOL SendSoftTriggerCommand();

	BOOL SetFreerunMode();

	BOOL SetExternalTrigger();
	BOOL SetTriggerDelayAbs(double TriggerDelayAbs_us);
	BOOL SetDebouncerTime(double DebouncerTime_us);

	BOOL SetExposureMode();
	double SetExposureTimeAbs(double ExposureTime_us);
	double GetExposureTimeAbs();
	__int64 SetExposureTimeRaw(__int64 ExposureTime_us);
	__int64 GetExposureTimeRaw();
	
	BOOL SetGainMode(e_GainAuto eMode);
	int64_t SetGainRaw(int64_t nSetVaule);
	int64_t GetGainRaw();

	BOOL SetAOI(__int32 nWidth, __int32 nHeight);
	
	BOOL SetWidth( int nWidth);	// 设置指定相机的图像宽和高
	BOOL SetHeight(int nHeight);	// 设置指定相机的图像宽和高

	int GetWidth();	//获得指定相机的宽
	int GetHeight();// 获得指定相机的高

	int GetWidthMax();	//获得指定相机的宽
	int GetHeightMax();// 获得指定相机的高
	
	BOOL SetOffsetX(int nOffsetX);	//设置X方向偏移
	BOOL SetOffsetY(int OffsetY);  // 设置Y方向偏移
	
	BOOL SetCenterX(_Bool bSetValue); //设置X方向中心对齐
	BOOL SetCenterY(_Bool bSetValue); //设置Y方向中心对齐

	BOOL AcquisitionStart();
	BOOL AcquisitionStop();

	BOOL AcquisitionPrepare();
	BOOL AcquisitionCleanup();

	BOOL GrabImage();
	BOOL GrabImage(bool* bSuccess, unsigned char* pImageBuffer,int timout=INFINITE);	//采集获取图像并显示
	

	//加载和设置用户userSetConfiguration
	BOOL SetUserConfiguration();
	BOOL LoadUserConfiguration();

	// 从文件加载相机设置
	BOOL SaveCameraSetToFile();
	BOOL LoadCameraSetFromFile();

private:	
	__int32 adjust__int32(__int32 min, __int32 max, __int32 incr, __int32 nSetVaule);
	__int32 GetIntegerFeature(char* featureName);
	__int32 GetIntegerFeatureMax(char* featureName);
	BOOL SetIntegerFeature(char* featureName, __int32 nSetVaule);
	BOOL GetBooleanFeature( char* featureName);
	BOOL SetBooleanFeature( char* featureName, _Bool bSetValue);

	bool GetIntegerFeatureRange(__int64* minVal,__int64* maxVal,const char featureName[]);//获取属性的范围
	__int64 SetIntegerFeatureValue(__int64 val,const char featureName[]);//设置属性的值
	__int64 GetIntegerFeatureValue(const char featureName[]);//获取属性的值
	//as reference

	/* Sets the heartbeat timeout. */
	int64_t setHeartbeatTimeout( PYLON_DEVICE_HANDLE hDevice, int64_t timeout_ms );

public:	

	static int							g_nTotalDeviceNum;
	PYLON_DEVICE_HANDLE					g_hDev;

	__int32								payloadSize;					/* Size of an image frame in bytes. */
	unsigned char						*buffers[NUM_BUFFERS];			/* Buffers used for grabbing. */
	PYLON_STREAMBUFFER_HANDLE			hBufHandles[NUM_BUFFERS];		/* Handles for the buffers. */
	PylonGrabResult_t					grabResult;						/* Stores the result of a grab operation. */
	PYLON_STREAMGRABBER_HANDLE			hGrabber;						/* Handle for the pylon stream grabber. */
	PYLON_WAITOBJECT_HANDLE				hWait;							/* Handle used for waiting for a grab to be finished. */
	PYLON_WAITOBJECTS_HANDLE			wos;							/* Wait objects. */
	PYLON_WAITOBJECT_HANDLE				wo_hCancelGrab;					/* CancelGrab wait object. */
	PYLON_DEVICECALLBACK_HANDLE			hCbRemoved;           /* Required for deregistering the callback. */

	const char*							triggerSelectorValue; /* Preselect the trigger for image acquisition. */
    _Bool								isAvailFrameStart;          /* Used for checking feature availability. */
    _Bool								isAvailAcquisitionStart;    /* Used for checking feature availability. */
//	CMyImage m_pImage;
//	ImageDib* m_pImage;

private:
	int									nCurrentIndex;

#ifdef _DEBUG
    int									isGigECamera;       /* 1 if the device is a GigE device. */
#endif
	
};



#endif