
// Robotics_2Dlg.h : header file
//

#pragma once
#include "afxwin.h"


// CRobotics_2Dlg dialog
class CRobotics_2Dlg : public CDialogEx
{
	// Construction
public:
	CRobotics_2Dlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ROBOTICS_2_DIALOG };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	//	CButton m_torque;
	afx_msg void OnClickedButtonStart();
	afx_msg void OnClickedButtonStop();
//
	
	afx_msg void OnClickedButtonConnect();
	
	BOOL m_torque;
	
	
	BOOL m_torque1;
	afx_msg void OnBnClickedChecktorque();
//	
	int m_intcombo;
	double SD_Inverse_KIN(double x, double y, double z);
	
	afx_msg void OnBnClickedButKin();
	
	double  point_th[4], send_point[4];
	

	afx_msg void OnTimer(UINT_PTR nIDEvent);




	void Set_pos_zero(double deg[4], double val[4]);

	void Read_position(int i);
//	
	void bajar(double a, double b);
	void rect_inverse(double rect[2][5]);

	int number = 0;
	
	afx_msg void OnBnClickedButtRec();
	CString side_a;
	CString side_b;
	afx_msg void OnBnClickedButtMove();
	CEdit x_value;
	CEdit y_value;
	CEdit z_value;
	CEdit simple;
	afx_msg void OnBnClickedButtsimple();
};
