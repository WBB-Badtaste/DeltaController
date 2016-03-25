// Dialog_Coordinate.cpp : implementation file
//

#include "stdafx.h"
#include "DeltaController.h"
#include "Dialog_Coordinate.h"
#include "afxdialogex.h"


// CDialog_Coordinate dialog

IMPLEMENT_DYNAMIC(CDialog_Coordinate, CDialogEx)

CDialog_Coordinate::CDialog_Coordinate(CWnd* pParent /*=NULL*/)
	: CDialogEx(CDialog_Coordinate::IDD, pParent)
{

}

CDialog_Coordinate::~CDialog_Coordinate()
{
}


void CDialog_Coordinate::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB, m_tab);
}


BEGIN_MESSAGE_MAP(CDialog_Coordinate, CDialogEx)
END_MESSAGE_MAP()


// CDialog_Coordinate message handlers
