// VisionCW.cpp: implementation of the CVisionCW class.
//
//////////////////////////////////////////////////////////////////////

//#include "stdafx.h"
//#include "pcb_mark.h"
#include "VisionCW.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CVisionCW::CVisionCW()
{
	m_showWnd = NULL;
	m_mainWnd = NULL;
	m_pImage = NULL;
	m_pMod = NULL;
}

CVisionCW::~CVisionCW()
{
	if(m_pImage) cwFreeImage(m_pImage);
	if(m_pMod) cwFreeModel(m_pMod);
}
/************************************************************************/
/* ��ʼ������
/************************************************************************/
bool CVisionCW::InitParameters(){
	SetImageInf(NULL,8,640,480);
	
	//cwVision ��ʾͼ�����
	m_trainRoiPosition = 0;//��¼�û�ѡ��ĵ��ڵ�
	m_trainRoi = NULL;//��ʾ����
	
	m_searchRoiPosition = 0;
	m_searchRoi = NULL;//��������
	
	m_trainRoiType = -1;//��ʾ����������
	m_dispOffset.x = 0;//mPic_rect.left;
	m_dispOffset.y = 0;//mPic_rect.top;
	m_dispOffset.scale = 1;//0.5;
	
//	m_lButtonDown = false;
//	m_rButtonDown = false;
	m_showDsip = true;
	return true;
}

//����һ��ROI
void CVisionCW::CreateRoi( int RoiType ){
	if( m_trainRoiType == RoiType && m_trainRoi!=NULL ) //��ͬ��roi�Ѿ��������ٴ���
		return;
	
	if( RoiType != m_trainRoiType && m_trainRoi!=NULL ){ //����ͬ���Ѿ����������ͷ�
		cwFreeRoi( m_trainRoi);
		m_trainRoi = NULL;
	}
	
	m_trainRoiType = RoiType;
	if( m_trainRoi == NULL ){ //ROI�������򴴽�
		m_trainRoi = cwCreateRoi( m_trainRoiType );
	}	
}
//���ROI
void CVisionCW::DestroyRoi(){
	if(m_trainRoi) cwFreeRoi(m_trainRoi);
	m_trainRoi = NULL;
}

//����ͼ�������
void CVisionCW::SetImageInf(  BYTE* pData/* = NULL*/, int bitCount/* = 8*/, int width/* = 640*/, int height/* = 480*/){
	if(m_pImage != NULL) cwFreeImage(m_pImage);
	m_pImage = cwCreateImage( pData, bitCount, width, height );
}

//����������
void CVisionCW::SetMainWnd(HWND hWnd){
	m_mainWnd = hWnd;
}

//������ʾ����
void CVisionCW::SetDisplayWnd(HWND hWnd){
	if(!m_showDsip) return;
	m_showWnd = hWnd;
}

//����ͼ����ʾ����ԶԻ��������
void CVisionCW::SetDisplayRect(CRect rect){
	m_imageRect = rect;
}

//����ͼ����ʾ��λ��
void CVisionCW::SetDisplayOffset(OFFSETZOOM zoom){
	m_dispOffset = zoom;
}

//��ʾ����ʾ��������
void CVisionCW::SetSerachArea(bool showInHwnd,int off/* = 20*/){
	if(showInHwnd){
		if( m_searchRoi != NULL ) cwFreeRoi(m_searchRoi);
		
		m_searchRoi = cwCreateRoi( ROI_RRECT );
		cwSetRoi(m_searchRoi,NULL,off,off,m_pImage->m_imgWidth-off,m_pImage->m_imgHeight-off);
	}else{
		if( m_searchRoi != NULL ) cwFreeRoi(m_searchRoi);
		m_searchRoi = NULL;
	}
}
//����Ӧ������ʾͼƬ
void CVisionCW::SetFullScreen(){
	m_dispOffset.x = 0;
	m_dispOffset.y = 0;
	m_dispOffset.scale = (m_imageRect.right - m_imageRect.left + 1)*1.0 / m_pImage->m_imgWidth;
	Sleep(200);//��ֹ�϶�Ӱ��
	InvalidateImageRect();
}

//��ʾͼƬ
void CVisionCW::DispalyImage(){
	if(!m_showDsip) return;
	if(m_showWnd && m_pImage) cwDisplayImage(m_showWnd,m_pImage,&m_dispOffset);
}

//��ʾ��������͵��ڵ�
void CVisionCW::DisplayROI(){
	if(!m_showDsip) return;
	if(m_showWnd){
		//��ʾѵ����
		cwDisplayRoi(m_showWnd, &m_dispOffset, m_trainRoi, CCOLOR_RED );
		cwDisplayRoiNode(m_showWnd, &m_dispOffset, m_trainRoi,CCOLOR_RED);
		//��ʾ���������
		cwDisplayRoi(m_showWnd, &m_dispOffset, m_searchRoi, CCOLOR_YELLOW ,false);
		cwDisplayRoiNode(m_showWnd, &m_dispOffset, m_searchRoi, CCOLOR_YELLOW );
	}
}

//��ʾƥ����
void CVisionCW::DisplayMatchResult(){
	if(!m_showDsip) return;
	if(m_showWnd) cwDisplayLastMatchFeature(m_showWnd,&m_dispOffset);
}

//���ڿ�����Ϣ
void CVisionCW::SetCaliperInf(int caliperNum,double caliperWidth,double caliperLength){
	if(m_trainRoi!=NULL){
		cwSetCaliperLength(m_trainRoi,caliperLength);
		cwSetCaliperNum(m_trainRoi,caliperNum);
		cwSetCaliperWidth(m_trainRoi,caliperWidth);
		InvalidateImageRect();
	}
}
void CVisionCW::GetCaliperInf(int* caliperNum,double* caliperWidth,double* caliperLength){
	if(m_trainRoi!=NULL){
		*caliperNum = cwGetCaliperNum(m_trainRoi);
		*caliperWidth = cwGetCaliperWidth(m_trainRoi);
		*caliperLength = cwGetCaliperLength(m_trainRoi);
		InvalidateImageRect();
	}
}

//��������¼�
//����������
void CVisionCW::LButtonDown(UINT nFlags, CPoint point,int roiType){
//	m_lButtonDown = true;	
	if( point.x>=m_imageRect.left&&point.x<=m_imageRect.right){//��ͼ��������
		if(point.y>=m_imageRect.top&&point.y<=m_imageRect.bottom){
			CreateRoi(roiType);//����roi
			//ת��Ϊ�ֲ�����
			point.x -= m_imageRect.left;
			point.y -= m_imageRect.top;
			//��¼����λ��
			m_selectRect.left = point.x ;
			m_selectRect.top = point.y ;			
			if(m_searchRoi != NULL){
				m_searchRoiPosition = cwGetRoiNode( m_searchRoi, &m_dispOffset, point.x, point.y,true);//��ȡ���ڵ�
				cwRoiCursor( m_searchRoi, m_searchRoiPosition );//��ʾ���ڵ�����ͼ��				
			}			
			if(m_trainRoi != NULL && m_searchRoiPosition == 0){				
				m_trainRoiPosition = cwGetRoiNode( m_trainRoi, &m_dispOffset, point.x, point.y,true);//��ȡ���ڵ�
				cwRoiCursor( m_trainRoi, m_trainRoiPosition );//��ʾ���ڵ�����ͼ��
			}
		}
	}
}
//����������
// void CVisionCW::LButtonUp(UINT nFlags, CPoint point){
// 	m_lButtonDown = false;
// }
//����ƶ�
clock_t preTime = clock();
void CVisionCW::MouseMove(UINT nFlags, CPoint point){
	m_movePoint = point;
	m_inImageRect = false;
	if(/*MK_LBUTTON == nFlags &&*/ point.x>=m_imageRect.left&&point.x<=m_imageRect.right){//��ͼ��������
		if(point.y>=m_imageRect.top&&point.y<=m_imageRect.bottom){
			m_selectRect.right = point.x-m_imageRect.left;
			m_selectRect.bottom = point.y- m_imageRect.top;
			
			int tmpPosition = cwGetRoiNode( m_searchRoi, &m_dispOffset, point.x -m_imageRect.left , point.y - m_imageRect.top );
			cwRoiCursor( m_searchRoi,/* m_Position);//*/ tmpPosition);
			if(m_searchRoi!=NULL && MK_LBUTTON == nFlags  && 0 != m_searchRoiPosition){
				if( 0 == m_searchRoiPosition ){
					//	cwSetRoi( m_searchRoi, &m_dispOffset, m_selectRect.left, m_selectRect.top, m_selectRect.right, m_selectRect.bottom );
				}else{					
					cwAdjustRoi( m_searchRoi, &m_dispOffset, m_searchRoiPosition, point.x -m_imageRect.left , point.y - m_imageRect.top );
				}
			}else{
				if(tmpPosition ==0 ) cwRoiCursor( m_trainRoi,/* m_Position);//*/cwGetRoiNode( m_trainRoi, &m_dispOffset, point.x -m_imageRect.left , point.y - m_imageRect.top ) );
				if(m_trainRoi!=NULL && MK_LBUTTON == nFlags ){
					if( 0 == m_trainRoiPosition ){
						cwSetRoi( m_trainRoi, &m_dispOffset, m_selectRect.left, m_selectRect.top, m_selectRect.right, m_selectRect.bottom );
					}else{					
						cwAdjustRoi( m_trainRoi, &m_dispOffset, m_trainRoiPosition, point.x -m_imageRect.left , point.y - m_imageRect.top );
					}
				}
			}
			
			if(MK_LBUTTON == nFlags/*m_lButtonDown*/) InvalidateImageRect();
			
			if(point.x >= m_imageRect.left + m_dispOffset.x && point.x < m_imageRect.left + m_dispOffset.x + m_pImage->m_imgWidth * m_dispOffset.scale){
				if(point.y >= m_imageRect.top + m_dispOffset.y && point.y < m_imageRect.top + m_dispOffset.y + m_pImage->m_imgHeight * m_dispOffset.scale){
					m_inImageRect = true;
				}
			}
		}
	}
	if(m_inImageRect && MK_RBUTTON == nFlags/*m_rButtonDown*/){
		m_dispOffset.x = m_preOffset.x + (point.x - m_rButtonDownLoc.x);
		m_dispOffset.y = m_preOffset.y + (point.y - m_rButtonDownLoc.y);
		
		clock_t t = clock();
		if(clock() - preTime >= 100){
			InvalidateImageRect();
			preTime = clock();
		}
		
	}
}
//����Ҽ�����
void CVisionCW::RButtonDown(UINT nFlags, CPoint point){
	if(m_inImageRect){
//		m_rButtonDown = true;
		m_rButtonDownLoc = point;
		m_preOffset.x = m_dispOffset.x;
		m_preOffset.y = m_dispOffset.y;
	}
}
//����Ҽ�����
// void CVisionCW::RButtonUp(UINT nFlags, CPoint point){
// 	m_rButtonDown = false;
// }
//����м�����
void CVisionCW::MouseWheel(UINT nFlags, short zDelta, CPoint pt){
	if(m_inImageRect){
		//ת��Ϊͼ������
		CPoint point = m_movePoint;
		double preScale = m_dispOffset.scale;
		point.x -= (m_imageRect.left + m_dispOffset.x);
		//point.x *= preScale;
		point.y -= (m_imageRect.top + m_dispOffset.y);
		//point.y *= preScale;
		
		double step;
		if(preScale >= 1) step = 0.2;
		else step = 0.04;
		if(zDelta >=0) m_dispOffset.scale +=step;
		else m_dispOffset.scale -=step;
		
		if(m_dispOffset.scale < 0.1) m_dispOffset.scale = 0.1;
		if(m_dispOffset.scale > 5) m_dispOffset.scale = 5;
		double scale = (m_dispOffset.scale - preScale)/preScale;//������������
		m_dispOffset.x -= point.x * scale;
		m_dispOffset.y -= point.y * scale;
		
		InvalidateImageRect();
	}
}
//����Ҽ�˫��
void CVisionCW::RButtonDblClk(UINT nFlags, CPoint point){
	if(m_inImageRect){
		SetFullScreen();
	}
}

//����ͼ������
void CVisionCW::InvalidateImageRect(){ 
	if(m_mainWnd) InvalidateRect(m_mainWnd,m_imageRect,true);
}

//ͼ���ģ����� �����ɹ�����true����ʧ�ܷ���false
bool CVisionCW::SaveImage(char path[]){
	if(m_pImage == NULL) return false;
	return m_pImage->Write(path);
}
bool CVisionCW::LoadImage(char path[]){
	return m_pImage->Read(path);
}
bool CVisionCW::SaveMod(char path[],bool writeWithouWarn/* = false*/){
	if(m_pMod == NULL )return false;
	return cwSaveModel(path,m_pMod,writeWithouWarn);
}
bool CVisionCW::LoadMod(char path[]){
	if(m_pMod) cwFreeModel(m_pMod);
	m_pMod = cwLoadModel(path);
	if(m_pMod == NULL) return false;
	return true;
}
bool CVisionCW::ModExist(){//����ģ���Ƿ�ѵ��
	return NULL != m_pMod;
}

//ѧϰģ��
bool CVisionCW::LearnModel(SHAP shap,LEARNOPTION* option,bool showEditModelDlg/* = false*/){
	//�쳣�ж�
	if(m_pImage->m_nBitCount!=8 || m_pImage->m_nBitCount!=8){
		::MessageBox(0,"ֻ����Ҷ�ͼ��",MB_OK,0);
		return false;
	}
	if (m_trainRoi == NULL) return false;//����Ϊ��
	if(m_pMod!=NULL) cwFreeModel(m_pMod);	
	m_showDsip = false;//ѧϰģ��༭ʱ���������ؼ�Ӱ��
	FRRECT* rect = cwGetFRRect(m_searchRoi);
	m_pMod = cwLearnModel(m_pImage,m_trainRoi,&m_dispOffset,shap,option,showEditModelDlg,rect);
	if(rect) cwFreeMemory(rect);
	m_showDsip = true;
	if(m_pMod == NULL) return false;
	return true;
}

//ƥ��ģ��
MATCH* CVisionCW::FindModel(FINDOPTION* option,int* retCount){
	if(m_pMod==NULL) 
		return NULL;
	FRRECT* rect = cwGetFRRect(m_searchRoi);
	MATCH* rt = cwFindModel(m_pImage,rect,m_pMod,option,retCount);
	if(rect) cwFreeMemory(rect);
	return rt;
}
//ƥ��ģ��
MATCH* CVisionCW::FindModel(MODEL* mod,FRRECT* searchRegion,FINDOPTION* option,int* retCount){
	if(m_pMod==NULL) 
		return NULL;
	return cwFindModel(m_pImage,searchRegion,mod,option,retCount);
}
//����
LINE* CVisionCW::FindLine(){
	return cwFindLine(m_pImage,m_trainRoi,&m_dispOffset);
}
//�ҽǵ�
FPOINT* CVisionCW::FindCorner(){
	return cwFindCorner(m_pImage,m_trainRoi,&m_dispOffset);
}