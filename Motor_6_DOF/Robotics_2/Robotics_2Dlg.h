
// Robotics_2Dlg.h : header file
//

#pragma once
#include "afxwin.h"
#include "Comm.h"
#include <SDKDDKVer.h>

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
	int number = 0;
	
	BOOL m_torque1;
	
	int m_intcombo;
	double SD_Inverse_KIN(double x, double y, double z);
	
	
	
	double  point_th[4], send_point[4];
	






	void Set_pos_zero(double deg[7]);

	void Read_position(int i);
	
	
	void rect_inverse(double rect[2][5]);
	

	//void arm_control(double angle[6], double kinematic[3]);
	void Forward_Kin(double q0_7[7]);
	void mult_matrix_4x4(double aa[4][4], double bb[4][4], double cc[4][4]);
	
	void Manage_Arm(double deg[7], double back_inv_jac[3][3]);
	void math_jacobian_calc(double q_rad[7], double inv_jac[3][3]);
	void inverse_jacobian(double inv_jac[3][3], double send_inv_jac[3][3]);
	
	void sycnWrite_joint_Velocity_123(int motor[3]);
	int syncRead_joint_Position(int pos_data[7]);

	afx_msg void OnBnClickedButtRec();
	CString side_a;
	CString side_b;
	afx_msg void OnBnClickedButtMove();

	CEdit simple;
	afx_msg void OnBnClickedButtsimple();
	CButton strCEdit;
	CString strCstring;
	CEdit CEditstr;
	//afx_msg void OnStnClickedStaticText();
	CString m_zupdown;
	CString m_yupdown;
	CString m_xupdown;
	
	afx_msg void OnClickedButtZminus();
	afx_msg void OnBnClickedButtZplus();
	afx_msg void OnBnClickedButtOnall();
	
	afx_msg void OnBnClickedButtOffall();
	afx_msg void OnBnClickedButtOneon();
	afx_msg void OnBnClickedButtOff();
	afx_msg void OnBnClickedButtHomeadd();
	afx_msg void OnBnClickedButtSethomeadd();
	afx_msg void OnBnClickedButtXplus();
	afx_msg void OnBnClickedButtXminus();
	afx_msg void OnBnClickedButtYminus();
	afx_msg void OnBnClickedButtYplus();
//	BOOL mode_change;
	
	CEdit m_set_time_vel_cont;
	afx_msg void OnBnClickedButtSetTimevel();
	afx_msg void OnBnClickedCheckmodechange();





	BOOL m_mode_change;

	afx_msg void OnTimer(UINT_PTR nIDEvent);

	afx_msg void OnBnClickedButtonMovXyz();
	afx_msg void OnBnClickedButtchangevariable();
	afx_msg void OnBnClickedButtYmin();
};
