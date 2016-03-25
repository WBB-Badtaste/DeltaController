// VisionCW.h: interface for the CVisionCW class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_)
#define AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//����ȥ��ʹ��map�ľ���
#ifdef WIN32 
#pragma warning (disable: 4514 4786)
#endif

#include <MAP>
#include "BaslerCamera.h"
#include "CWVision.h"

const int  CAMERAS =  1;//���������
const int MAX_CAMERAS = 5;//�������
enum CAMERA_INDEX{
	MARK_LEFT=0, //��MARK���
	LED_LEFT,//��LED���
	MARK_RIGHT,//��MARK���
	LED_RIGHT, //��LED���
	LED_CHECK//LED������
};//����ı��

//const CString CAMERA_NAME[MAX_CAMERAS] = {"CIC-300_2","CIC-300_0","CIC-300_1","CIC-300_3","CIC-300_4"};//���������_���
const CString CAMERA_NAME[MAX_CAMERAS] = {_T("acA1300-30gm_0"),	_T("acA1300-30gm_2"),_T("acA640-120gm_1"),
_T("acA640-120gm_3"),_T("acA640-120gm_4")};//���������_���
const CString CAMERA_DISP_NAME[MAX_CAMERAS] = {_T("��MARK���"),_T("��LED���"),_T("��MARK���"),
	_T("��LED���"),_T("LED������")};//�������ʾ����

#define TIMER_CAPTURE_BASLER_IMAGE 1526 //basler�������ͼ��ʱ��
#define TIMER_CAPTURE_BASLER_IMAGE_HD 1527 //basler�������ͼ��ʱ����Ӳ������

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
	
	static void InitCameraEnvirionment();//��ʼ������ ֻ�ܵ���һ��
	bool InitCameras();//��ʼ����� ֻ��Ҫ����һ��	

	bool OpenCamera(int seq,int imgWidth = 640,int imgHeight = 480);//ͨ��������� �����
	void CloseCamera(int seq);//ͨ��������� �ر����
	bool OpenCamera(CString cameraDispName,int imgWidth = 640,int imgHeight = 480);//ͨ�������ʾ�� �����
	void CloseCamera(CString cameraDispName);//ͨ�������ʾ�� �ر����

	bool SetCaptureModel(int seq,TriggerModel model);//���ô���ģʽ
	bool SetCaptureModel(CString cameraDispName,TriggerModel model);//���ô���ģʽ

	bool CaptureOne(CString cameraDispName,IMAGE* pImage,bool software = true,double timeOut = INFINITE);//��������һ��/Ӳ�����ȴ�һ��
	bool CaptureOne(int seq,IMAGE* pImage,bool software = true,double timeOut = INFINITE);//��������һ��/Ӳ�����ȴ�һ��

	void ShowPropertyDlg(CString cameraDispName,bool showSys = false);//��ʾ���ԶԻ���(����Ϊ�Ƿ���ʾϵͳ�Դ���)
	void ShowPropertyDlg(int seq,bool showSys = false);//��ʾ���ԶԻ���(����Ϊ�Ƿ���ʾϵͳ�Դ���)

// 	double SetExposure(int seq,double exposureValue);//�����ع�ʱ�䵥λΪms����ʵ�ʵ�����ֵ
// 	double GetExposure(int seq);//��ȡ�ع�ʱ�䵥λΪms
// 
// 	__int64 SetGain(int seq,__int64 gainValue);//��������ֵ��λΪdb����ʵ�ʵ�����ֵ
// 	__int64 GetGain(int seq);//��ȡ����ֵ��λΪdb
// 
// 	
// 	bool SaveExposureToFile(int seq,double exposureValue);//�����ع�ʱ�䵥λΪms
// 	double LoadExposureFromFile(int seq);//���ļ���ȡ�ع�ʱ�� ��λΪms �����ڷ���0
// 	   
// 	bool SaveGainToFile(int seq,__int64 gainValue);//��������ֵ��λΪdb
// 	__int64 LoadGainFromFile(int seq);//��ȡ����ֵ��λΪdb �����ڷ���0

public:
	CString mCameraShowName[CAMERAS];//����±��Ӧ��ʵ������
	
	std::map<CString,CAMERA_INDEX> mCameraMapDispNameToID;//�������ʾ���ƺ�������Ӧ
	std::map<CString,int> mCameraMapDispNameToSeq;//�������ʾ���ƺ�ɨ��˳���Ӧ
private:
	CBaslerCamera mCamera[CAMERAS];//������
};
extern CSMT1408CameraOperate gCameras;
extern int gCaptureTimerIndex;
const int CPI_NULL = -1;
const int CPI_MARK = 1;
const int CPI_MANAGER = 2;

#endif // !defined(AFX_VISIONCW_H__FE7D1DD2_285D_4145_AA0A_864D910B1F29__INCLUDED_)
