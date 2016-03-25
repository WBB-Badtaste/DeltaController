// Basler.h : header file
//

#if !defined(AFX_PYLON_H__41AA6513_E0F8_4C97_8540_11F569B6FD24__INCLUDED_)
#define AFX_PYLON_H__41AA6513_E0F8_4C97_8540_11F569B6FD24__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include <AFXWIN.H>
//#include "windows.h"                 // ǰ��������

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

	// ����static����������ʹ�ö��������Ҳֻ�ܱ�����һ��
	static void		InitAll();
	static void		PylonTerminateAll();
	//���������
	static int EnumerateDevices();	

	//�����Զ���������UserID�õ�ö�ٵ�����б��е������ţ�������������к�
	int  GetDeviceIndexFromUserID(char UserID[]);
	bool SetDeviceIndex(int ndeviceIndex);

	bool GetDeviceSN(char name[]);//��ȡ���Ψһ���к�
	bool GetDeviceUserID(char name[]);//��ȡ���ID �û��Զ�����
	bool GetDeviceModelName(char name[]);//��ȡ���������

	BOOL OpenDevice();
	BOOL CloseDevice();
	void DestroyDevice();

	void ShowPropertyDlg(bool showSys = false);//��ʾ���ԶԻ���(����Ϊ�Ƿ���ʾϵͳ�Դ���)
	bool SaveParameters();//�������(�Զ�����)
	bool LoadParameters();//���ز���(�Զ�����)
	
	__int64 GetGainValue();//��ȡ����ֵ
	bool GetGainRange(__int64* minVal,__int64* maxVal);//��ȡ����ֵ��Χ
	bool SetGainValue(__int64 val);//��������ֵ
	
	__int64 GetExposureValue();//��ȡ�ع�ֵ
	bool GetExposureRange(__int64* minVal,__int64* maxVal);//��ȡ�ع�ֵ��Χ
	bool SetExposureValue(__int64 val);//�����ع�ֵ
	
	__int64 GetContrastValue();//��ȡ�Աȶ�ֵ
	bool GetContrastRange(__int64* minVal,__int64* maxVal);//��ȡ�Աȶ�ֵ��Χ
	bool SetContrastValue(__int64 val);//���öԱȶ�ֵ
	
	__int64 GetBrightnessValue();//��ȡ����ֵ
	bool GetBrightnessRange(__int64* minVal,__int64* maxVal);//��ȡ����ֵ��Χ
	bool SetBrightnessValue(__int64 val);//��������ֵ


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
	
	BOOL SetWidth( int nWidth);	// ����ָ�������ͼ���͸�
	BOOL SetHeight(int nHeight);	// ����ָ�������ͼ���͸�

	int GetWidth();	//���ָ������Ŀ�
	int GetHeight();// ���ָ������ĸ�

	int GetWidthMax();	//���ָ������Ŀ�
	int GetHeightMax();// ���ָ������ĸ�
	
	BOOL SetOffsetX(int nOffsetX);	//����X����ƫ��
	BOOL SetOffsetY(int OffsetY);  // ����Y����ƫ��
	
	BOOL SetCenterX(_Bool bSetValue); //����X�������Ķ���
	BOOL SetCenterY(_Bool bSetValue); //����Y�������Ķ���

	BOOL AcquisitionStart();
	BOOL AcquisitionStop();

	BOOL AcquisitionPrepare();
	BOOL AcquisitionCleanup();

	BOOL GrabImage();
	BOOL GrabImage(bool* bSuccess, unsigned char* pImageBuffer,int timout=INFINITE);	//�ɼ���ȡͼ����ʾ
	

	//���غ������û�userSetConfiguration
	BOOL SetUserConfiguration();
	BOOL LoadUserConfiguration();

	// ���ļ������������
	BOOL SaveCameraSetToFile();
	BOOL LoadCameraSetFromFile();

private:	
	__int32 adjust__int32(__int32 min, __int32 max, __int32 incr, __int32 nSetVaule);
	__int32 GetIntegerFeature(char* featureName);
	__int32 GetIntegerFeatureMax(char* featureName);
	BOOL SetIntegerFeature(char* featureName, __int32 nSetVaule);
	BOOL GetBooleanFeature( char* featureName);
	BOOL SetBooleanFeature( char* featureName, _Bool bSetValue);

	bool GetIntegerFeatureRange(__int64* minVal,__int64* maxVal,const char featureName[]);//��ȡ���Եķ�Χ
	__int64 SetIntegerFeatureValue(__int64 val,const char featureName[]);//�������Ե�ֵ
	__int64 GetIntegerFeatureValue(const char featureName[]);//��ȡ���Ե�ֵ
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