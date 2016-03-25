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
/* 初始化参数
/************************************************************************/
bool CVisionCW::InitParameters(){
	SetImageInf(NULL,8,640,480);
	
	//cwVision 显示图像相关
	m_trainRoiPosition = 0;//记录用户选择的调节点
	m_trainRoi = NULL;//显示区域
	
	m_searchRoiPosition = 0;
	m_searchRoi = NULL;//搜索区域
	
	m_trainRoiType = -1;//显示区域框的类型
	m_dispOffset.x = 0;//mPic_rect.left;
	m_dispOffset.y = 0;//mPic_rect.top;
	m_dispOffset.scale = 1;//0.5;
	
//	m_lButtonDown = false;
//	m_rButtonDown = false;
	m_showDsip = true;
	return true;
}

//创建一个ROI
void CVisionCW::CreateRoi( int RoiType ){
	if( m_trainRoiType == RoiType && m_trainRoi!=NULL ) //相同的roi已经创建则不再创建
		return;
	
	if( RoiType != m_trainRoiType && m_trainRoi!=NULL ){ //若不同并已经创建则先释放
		cwFreeRoi( m_trainRoi);
		m_trainRoi = NULL;
	}
	
	m_trainRoiType = RoiType;
	if( m_trainRoi == NULL ){ //ROI不存在则创建
		m_trainRoi = cwCreateRoi( m_trainRoiType );
	}	
}
//清除ROI
void CVisionCW::DestroyRoi(){
	if(m_trainRoi) cwFreeRoi(m_trainRoi);
	m_trainRoi = NULL;
}

//设置图像的属性
void CVisionCW::SetImageInf(  BYTE* pData/* = NULL*/, int bitCount/* = 8*/, int width/* = 640*/, int height/* = 480*/){
	if(m_pImage != NULL) cwFreeImage(m_pImage);
	m_pImage = cwCreateImage( pData, bitCount, width, height );
}

//设置主窗口
void CVisionCW::SetMainWnd(HWND hWnd){
	m_mainWnd = hWnd;
}

//设置显示窗口
void CVisionCW::SetDisplayWnd(HWND hWnd){
	if(!m_showDsip) return;
	m_showWnd = hWnd;
}

//设置图像显示框相对对话框的区域
void CVisionCW::SetDisplayRect(CRect rect){
	m_imageRect = rect;
}

//设置图像显示的位置
void CVisionCW::SetDisplayOffset(OFFSETZOOM zoom){
	m_dispOffset = zoom;
}

//显示或不显示搜索区域
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
//自适应窗口显示图片
void CVisionCW::SetFullScreen(){
	m_dispOffset.x = 0;
	m_dispOffset.y = 0;
	m_dispOffset.scale = (m_imageRect.right - m_imageRect.left + 1)*1.0 / m_pImage->m_imgWidth;
	Sleep(200);//防止拖动影响
	InvalidateImageRect();
}

//显示图片
void CVisionCW::DispalyImage(){
	if(!m_showDsip) return;
	if(m_showWnd && m_pImage) cwDisplayImage(m_showWnd,m_pImage,&m_dispOffset);
}

//显示调节区域和调节点
void CVisionCW::DisplayROI(){
	if(!m_showDsip) return;
	if(m_showWnd){
		//显示训练框
		cwDisplayRoi(m_showWnd, &m_dispOffset, m_trainRoi, CCOLOR_RED );
		cwDisplayRoiNode(m_showWnd, &m_dispOffset, m_trainRoi,CCOLOR_RED);
		//显示搜索区域框
		cwDisplayRoi(m_showWnd, &m_dispOffset, m_searchRoi, CCOLOR_YELLOW ,false);
		cwDisplayRoiNode(m_showWnd, &m_dispOffset, m_searchRoi, CCOLOR_YELLOW );
	}
}

//显示匹配结果
void CVisionCW::DisplayMatchResult(){
	if(!m_showDsip) return;
	if(m_showWnd) cwDisplayLastMatchFeature(m_showWnd,&m_dispOffset);
}

//调节卡尺信息
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

//处理鼠标事件
//鼠标左键按下
void CVisionCW::LButtonDown(UINT nFlags, CPoint point,int roiType){
//	m_lButtonDown = true;	
	if( point.x>=m_imageRect.left&&point.x<=m_imageRect.right){//在图像区域内
		if(point.y>=m_imageRect.top&&point.y<=m_imageRect.bottom){
			CreateRoi(roiType);//创建roi
			//转换为局部坐标
			point.x -= m_imageRect.left;
			point.y -= m_imageRect.top;
			//记录按下位置
			m_selectRect.left = point.x ;
			m_selectRect.top = point.y ;			
			if(m_searchRoi != NULL){
				m_searchRoiPosition = cwGetRoiNode( m_searchRoi, &m_dispOffset, point.x, point.y,true);//获取调节点
				cwRoiCursor( m_searchRoi, m_searchRoiPosition );//显示调节点的鼠标图像				
			}			
			if(m_trainRoi != NULL && m_searchRoiPosition == 0){				
				m_trainRoiPosition = cwGetRoiNode( m_trainRoi, &m_dispOffset, point.x, point.y,true);//获取调节点
				cwRoiCursor( m_trainRoi, m_trainRoiPosition );//显示调节点的鼠标图像
			}
		}
	}
}
//鼠标左键弹起
// void CVisionCW::LButtonUp(UINT nFlags, CPoint point){
// 	m_lButtonDown = false;
// }
//鼠标移动
clock_t preTime = clock();
void CVisionCW::MouseMove(UINT nFlags, CPoint point){
	m_movePoint = point;
	m_inImageRect = false;
	if(/*MK_LBUTTON == nFlags &&*/ point.x>=m_imageRect.left&&point.x<=m_imageRect.right){//在图像区域内
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
//鼠标右键按下
void CVisionCW::RButtonDown(UINT nFlags, CPoint point){
	if(m_inImageRect){
//		m_rButtonDown = true;
		m_rButtonDownLoc = point;
		m_preOffset.x = m_dispOffset.x;
		m_preOffset.y = m_dispOffset.y;
	}
}
//鼠标右键弹起
// void CVisionCW::RButtonUp(UINT nFlags, CPoint point){
// 	m_rButtonDown = false;
// }
//鼠标中键滚动
void CVisionCW::MouseWheel(UINT nFlags, short zDelta, CPoint pt){
	if(m_inImageRect){
		//转换为图像坐标
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
		double scale = (m_dispOffset.scale - preScale)/preScale;//计算扩缩倍数
		m_dispOffset.x -= point.x * scale;
		m_dispOffset.y -= point.y * scale;
		
		InvalidateImageRect();
	}
}
//鼠标右键双击
void CVisionCW::RButtonDblClk(UINT nFlags, CPoint point){
	if(m_inImageRect){
		SetFullScreen();
	}
}

//更新图像区域
void CVisionCW::InvalidateImageRect(){ 
	if(m_mainWnd) InvalidateRect(m_mainWnd,m_imageRect,true);
}

//图像和模板操作 操作成功返回true操作失败返回false
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
bool CVisionCW::ModExist(){//返回模板是否训练
	return NULL != m_pMod;
}

//学习模板
bool CVisionCW::LearnModel(SHAP shap,LEARNOPTION* option,bool showEditModelDlg/* = false*/){
	//异常判断
	if(m_pImage->m_nBitCount!=8 || m_pImage->m_nBitCount!=8){
		::MessageBox(0,"只处理灰度图像",MB_OK,0);
		return false;
	}
	if (m_trainRoi == NULL) return false;//区域为空
	if(m_pMod!=NULL) cwFreeModel(m_pMod);	
	m_showDsip = false;//学习模板编辑时不受其它控件影响
	FRRECT* rect = cwGetFRRect(m_searchRoi);
	m_pMod = cwLearnModel(m_pImage,m_trainRoi,&m_dispOffset,shap,option,showEditModelDlg,rect);
	if(rect) cwFreeMemory(rect);
	m_showDsip = true;
	if(m_pMod == NULL) return false;
	return true;
}

//匹配模板
MATCH* CVisionCW::FindModel(FINDOPTION* option,int* retCount){
	if(m_pMod==NULL) 
		return NULL;
	FRRECT* rect = cwGetFRRect(m_searchRoi);
	MATCH* rt = cwFindModel(m_pImage,rect,m_pMod,option,retCount);
	if(rect) cwFreeMemory(rect);
	return rt;
}
//匹配模板
MATCH* CVisionCW::FindModel(MODEL* mod,FRRECT* searchRegion,FINDOPTION* option,int* retCount){
	if(m_pMod==NULL) 
		return NULL;
	return cwFindModel(m_pImage,searchRegion,mod,option,retCount);
}
//找线
LINE* CVisionCW::FindLine(){
	return cwFindLine(m_pImage,m_trainRoi,&m_dispOffset);
}
//找角点
FPOINT* CVisionCW::FindCorner(){
	return cwFindCorner(m_pImage,m_trainRoi,&m_dispOffset);
}