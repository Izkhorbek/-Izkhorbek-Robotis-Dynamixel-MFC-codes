
// Robotics_2Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "Robotics_2.h"
#include "Robotics_2Dlg.h"
#include "afxdialogex.h"
#include "dynamixel_sdk.h"

#define new DEBUG_NEW
//#endif

#include <corecrt.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include <WinBase.h>
#include < wchar.h >
#include <Thread>


// Control table address

       
#define ADDR_PRO_TORQUE_ENABLE4         64    // 562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION4         116   // 596
#define ADDR_PRO_GOAL_VELOCITY4         104   // 596

#define ADDR_PROFILE_VELOCITY4          112   // 596
#define ADDR_PROFILE_ACCELATION4        108   // 596


#define ADDR_PRO_PRESENT_POSITION4       132   // 611
#define ADDR_PRO_PRESENT_VELOCITY4       128   // 611
#define ADDR_PRO_LED_ENABLE4             65
#define ADDR_PRO_OPERATING_MODE4         11

#define ADDR_PRO_ADDR_PRO_ID_SET_MODE   7
#define ADDR_PRO_HOMMING_OFFSET_MODE    20
#define ADDR_PRO_MOVING_THRESHOLD_MODE  17    // 4B
#define ADDR_PRO_MOVING_THRESHOLD		24
#define ADDR_PRO_MOVING					122
#define ADDR_PRO_MAX_POS_LIMIT          36
#define ADDR_PRO_MIN_POS_LIMIT          40
#define ADDR_PRO_TORQUE_LIMIT           30
#define ADDR_PRO_ACCEL_LIMIT            26

#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4


#define VELOCITY_MODE4                    1
#define POSITION_MODE4                    3
#define EXTENDED_POSITION_MODE4           4
#define PWM_MODE4                         5 
// for number of motors
#define MEM_AXIS                         4  

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          1                   // Dynamixel ID: 1
#define DXL_ID2                          2
#define DXL_ID3                          4


#define BAUDRATE                        57600       // 57600
#define DEVICENAME1                      "COM3"   //  "/dev/ttyUSB0"      // Check which port is being used on your controller

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      1             // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4095              // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold


#define ESC_ASCII_VALUE                 0x1b


#define PI                            3.14159
// lenth mm
#define  L0  8.5
#define  L1  18.0
#define  L2  24.0

#define  Lh  0.0
#define q1_center   1875 
#define q2_center	2835
#define q3_center	2492


#define TimerCount  100

#define basic_q     0.088



//////////////////////////////////////////

dynamixel::PortHandler *portHandler1 = dynamixel::PortHandler::getPortHandler(DEVICENAME1);
dynamixel::PacketHandler *packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
dynamixel::GroupSyncWrite groupsyncWrite(portHandler1, packetHandler1, ADDR_PRO_GOAL_POSITION4, LEN_PRO_GOAL_POSITION);
dynamixel::GroupSyncRead groupsyncRead(portHandler1,packetHandler1,ADDR_PRO_GOAL_POSITION4, LEN_PRO_GOAL_POSITION);
int index = 0;
int dxl_comm_result = COMM_TX_FAIL;             // Communication result
int timer_wait = COMM_RX_WAITING;               // waiting for 
int busy_in[4] = {0};


int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

uint8_t dxl_error = 0;                          // Dynamixel error
int32_t dxl_present_position[MEM_AXIS] = { 0 };               // Present position
int32_t dxl_present_velocity[MEM_AXIS] = { 0 };              // Present velocity
int32_t dxl_profile_velocity = 0;
int32_t dxl_threshold[MEM_AXIS] = { 0 };
int32_t moving = 0;
 
int arm[MEM_AXIS] = { 0, DXL_ID1, DXL_ID2, DXL_ID3 };
int edit_presp[MEM_AXIS] = { 0, IDC_EDIT_Presp1, IDC_EDIT_Presp2,IDC_EDIT_Presp4 };
int edit_presv[MEM_AXIS] = { 0, IDC_EDIT_Presv1, IDC_EDIT_Presv2, IDC_EDIT_Presv4 };
int edit_goalp[MEM_AXIS] = { 0, IDC_EDIT_Goalp1, IDC_EDIT_Goalp2, IDC_EDIT_Goalp4 };
int edit_goalv[MEM_AXIS] = { 0, IDC_EDIT_Goalv1, IDC_EDIT_Goalv2,IDC_EDIT_Goalv4 };
// new variable for goal position and velocity

bool dxl_addparam = false;
bool dxl_getdata_result = false;

int goalp1, goalp2, goalp4;
int goalv1, goalv2,  goalv4;
int goal_position[MEM_AXIS] = { 0, goalp1, goalp2,goalp4 };
int goal_velocity[MEM_AXIS] = { 0, goalv1, goalv2,goalv4 };

int q_current_position[MEM_AXIS] = {0, q1_center, q2_center, q3_center};

// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CRobotics_2Dlg dialog



CRobotics_2Dlg::CRobotics_2Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTICS_2_DIALOG, pParent)
	, m_torque(FALSE)
	, m_intcombo(0)
	, side_a(_T(""))
	, side_b(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotics_2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);


	DDX_Check(pDX, IDC_CHECK_torque, m_torque);

	DDX_Text(pDX, IDC_EDIT11, side_a);
	DDX_Text(pDX, IDC_EDIT12, side_b);
	DDX_Control(pDX, IDC_EDIT_X1, x_value);
	DDX_Control(pDX, IDC_EDIT_Y1, y_value);
	DDX_Control(pDX, IDC_EDIT_Z1, z_value);
	DDX_Control(pDX, IDC_EDIT_SMP, simple);
}

BEGIN_MESSAGE_MAP(CRobotics_2Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_start, &CRobotics_2Dlg::OnClickedButtonStart)
	ON_BN_CLICKED(IDC_BUTTON_stop, &CRobotics_2Dlg::OnClickedButtonStop)
//	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_connect, &CRobotics_2Dlg::OnClickedButtonConnect)

ON_BN_CLICKED(IDC_CHECK_torque, &CRobotics_2Dlg::OnBnClickedChecktorque)
ON_WM_TIMER()
ON_BN_CLICKED(IDC_BUTT_REC, &CRobotics_2Dlg::OnBnClickedButtRec)
ON_BN_CLICKED(IDC_BUTT_Move, &CRobotics_2Dlg::OnBnClickedButtMove)
ON_BN_CLICKED(IDC_BUTT_simple, &CRobotics_2Dlg::OnBnClickedButtsimple)
END_MESSAGE_MAP()


// CRobotics_2Dlg message handlers

BOOL CRobotics_2Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	SetDlgItemText(IDC_EDIT_Goalp1, _T("2048"));
	SetDlgItemText(IDC_EDIT_Goalp2, _T("2048"));
	SetDlgItemText(IDC_EDIT_Goalp4, _T("2048"));

	SetDlgItemText(IDC_EDIT_Goalv1, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv2, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv4, _T("20"));


	SetDlgItemText(IDC_EDIT_vel_prof, _T("20"));
	SetDlgItemText(IDC_EDIT_acc_prof, _T("0"));
	

	
	//mode_change(POSITION_MODE);
	//SetTimer(1, 100, NULL);

	// TODO: Add extra initialization here

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CRobotics_2Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CRobotics_2Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CRobotics_2Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CRobotics_2Dlg::OnClickedButtonStart()
{
	// TODO: Add your control notification handler code here
	CString str,str1;
	int  acc_profile, vel_profile;
	
	
	for (int i = 1; i < 4; i++)
	{
		GetDlgItemText(edit_goalp[i], str);
		goal_position[i] = _ttoi(str);


		GetDlgItemText(edit_goalv[i], str1);
		goal_velocity[i] = _ttoi(str1);

		GetDlgItemText(IDC_EDIT_vel_prof, str1);
		vel_profile = _ttoi(str1);

		GetDlgItemText(IDC_EDIT_acc_prof, str1);
		acc_profile = _ttoi(str1);


		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PROFILE_VELOCITY4, vel_profile, &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PRO_GOAL_POSITION4, goal_position[i], &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PRO_GOAL_VELOCITY4, goal_velocity[i], &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PROFILE_ACCELATION4, acc_profile, &dxl_error);

	}
				

}


void CRobotics_2Dlg::OnClickedButtonStop()
{
	// TODO: Add your control notification handler code here
	portHandler1->clearPort();
	//m_torque == FALSE;
	KillTimer(0);
	OnOK();
}


void CRobotics_2Dlg::OnClickedButtonConnect()
{
	CString str;
	
	if (portHandler1->openPort())
	{
		//
	}
	else
	{
	//
	}

	// SetBaudrate
	if (portHandler1->setBaudRate(BAUDRATE))
	{
		
	}
	
	Sleep(50);
	

		if (dxl_comm_result != COMM_SUCCESS)
		{
			
			str.Format(_T("1:%d %s"), packetHandler1->getTxRxResult(dxl_comm_result), packetHandler1->getTxRxResult(dxl_comm_result));
		
		}
		else if (dxl_error != 0)
		{
			
			str.Format(_T("2:%d %s"), packetHandler1->getRxPacketError(dxl_error), packetHandler1->getRxPacketError(dxl_error));
			AfxMessageBox(str); //  _T("%s\n", packetHandler1->getRxPacketError(dxl_error)) );
		}
		else
		{
		
		}
	
		
		Sleep(10);
		
		SetTimer(1, 100, NULL);
	
}

void CRobotics_2Dlg::Read_position(int i)
{
	CString str;
	for (int i = 1; i < 4; i++)
	{
		dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_PRESENT_POSITION4, (uint32_t*)&dxl_present_position[i], &dxl_error);
		//dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_MOVING_THRESHOLD, (uint32_t*)&dxl_threshold [i] , &dxl_error);

		//SD_Forward_KIN(double deg, double deg1, double deg2)

		dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_PRESENT_VELOCITY4, (uint32_t*)&dxl_present_velocity[i], &dxl_error);
		
	}
	SetTimer(1, 100, NULL);
}

void CRobotics_2Dlg::OnBnClickedChecktorque()
{
	// TODO: Add your control notification handler code here
	UpdateData(TRUE);
	int i;
	if (m_torque == TRUE)
	{
		for (i = 1; i < 4; i++)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 1, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_OPERATING_MODE4, EXTENDED_POSITION_MODE4, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 1, &dxl_error);
		}
	}
	else if (m_torque == FALSE)
	{
		for (i = 1; i < 4; i++)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_OPERATING_MODE4, EXTENDED_POSITION_MODE4, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 0, &dxl_error);
		}
	};
	Sleep(50);
	UpdateData(FALSE);
	
}


double CRobotics_2Dlg::SD_Inverse_KIN(double x, double y, double z)
{
	CString str,str2;
	double xd, yd, zd;
	double d2, d, qsi, alpha, beta;
	double th1, th2, th3, th[4] = { 0 }, get_th[4] = {0};
	int i;

	xd = x; yd = y; zd = z;

	if (zd > L0)
	{
		d2 = xd * xd + yd * yd + (zd - L0) * (zd - L0);
		d = sqrt(d2);
		qsi = acos((L1 * L1 + L2 * L2 - d2) / (2.0 * L1 * L2));
		th3 = qsi - PI;
		alpha = atan2((zd - L0), sqrt(xd * xd + yd * yd));
		beta = acos((L2 * L2 + d2 - L2 * L2) / (2.0 * L1 * d));
		th2 = alpha + beta;
	}
	else
	{
		d2 = xd * xd + yd * yd + (L0 - zd)* (L0 - zd);
		d = sqrt(d2);
		qsi = acos((L1 * L1 + L2 * L2 - d2) / (2.0 * L1 * L2));
		th3 = qsi - PI;
		alpha = atan2((L0 - zd), sqrt(xd * xd + yd * yd));
		beta = acos((L1 * L1 + d2 - L2 * L2) / (2.0 * L1 * d));
		th2 = beta - alpha;
	}

	th1 = atan2(yd, xd);

	

	th[1] = th1*180.0/ (PI*0.088);  // degrees
	th[2] = th2*180.0/ (PI*0.088);
	th[3] = th3*180.0/ (PI*0.088);
										  

	
	get_th[1] = 1825 + th[1];
	get_th[2] = 1825 + th[2];
	get_th[3] = 2460 + th[3];
	
	
for ( i = 1;  i < 4; i++)
	//for()
	{

		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PROFILE_VELOCITY4, 40, &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PRO_GOAL_POSITION4, get_th[i], &dxl_error);
		
	}

return TRUE;
	
	//SetTimer(TimerCount, 100, NULL); // Returns null

}


void CRobotics_2Dlg::OnBnClickedButKin()
{
	// TODO: Add your control notification handler code here

	CString str,str2;
	double kin_x, kin_y, kin_z;

	GetDlgItemText(IDC_EDIT_X1, str);
	kin_x = _wtoi(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	kin_y = _wtoi(str);

	GetDlgItemText(IDC_EDIT_Z1, str);
	kin_z = _wtoi(str);

	
	SD_Inverse_KIN(kin_x, kin_y, kin_z);
	
}

void CRobotics_2Dlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	CString str;
	double deg[4],son[4];
	Read_position(1);
	Set_pos_zero(deg, son);

	for (int i = 1; i < 4; i++)
	{	
		str.Format(_T("%.2f"), deg[i]);
		SetDlgItemText(edit_presp[i], str);
	}
	CDialogEx::OnTimer(nIDEvent);
}

void CRobotics_2Dlg::Set_pos_zero(double deg[4], double val[4])
{
	CString str;


	for (int i = 1; i < 4; i++)
	{
		deg[i] = (dxl_present_position[i] - q_current_position[i]) * basic_q;
		if (deg[i] >= 180)deg[i] = deg[i] - 360;
		else if (deg[i] <= -180) deg[i] = deg[i] + 360;

		val[i] = deg[i];

	}

}


void CRobotics_2Dlg::bajar(double a, double b)
{
	
		/*{
			do {
				SD_Inverse_KIN(rect[0][0], rect[0][1], z);
			} while (dxl_present_position > dxl_threshold);
		}
		*/
		//rect_inverse(rect);

}

void CRobotics_2Dlg::rect_inverse(double rec[2][5])
{
	CString str, str2;
	int i;
	double xd, yd, zd = 0;
	double d2, d, qsi, alpha, beta;
	double th1, th2, th3,  get_th[4][6] = { 0 };
	for (i =0; i < 5; i++)
	{
		
			
		xd = rec[0][i];
		yd = rec[1][i];
			if (zd > L0)
			{
				d2 = xd * xd + yd * yd + (zd - L0) * (zd - L0);
				d = sqrt(d2);
				qsi = acos((L1 * L1 + L2 * L2 - d2) / (2.0 * L1 * L2));
				th3 = qsi - PI;
				alpha = atan2((zd - L0), sqrt(xd * xd + yd * yd));
				beta = acos((L2 * L2 + d2 - L2 * L2) / (2.0 * L1 * d));
				th2 = alpha + beta;
			}
			else
			{
				d2 = xd * xd + yd * yd + (L0 - zd) * (L0 - zd);
				d = sqrt(d2);
				qsi = acos((L1 * L1 + L2 * L2 - d2) / (2.0 * L1 * L2));
				th3 = qsi - PI;
				alpha = atan2((L0 - zd), sqrt(xd * xd + yd * yd));
				beta = acos((L1 * L1 + d2 - L2 * L2) / (2.0 * L1 * d));
				th2 = beta - alpha;
			}

			th1 = atan2(yd, xd);

			

			get_th[1][i] = 1825 + th1 * 180.0 / (PI * 0.088);
			get_th[2][i] = 1825 + th2 * 180.0 / (PI * 0.088);
			get_th[3][i] = 2460 + th3 * 180.0 / (PI * 0.088);


	}


}

void CRobotics_2Dlg::OnBnClickedButtRec()
{
	CString str;
	double side_a, side_b;
	
	GetDlgItemText(IDC_EDIT11, str);
	side_a = _ttoi(str);
	GetDlgItemText(IDC_EDIT12, str);
	side_b = _ttoi(str);
	
	double  a, b, x, y, z;
	double rect[5][2] = { 0 };
	a = side_a ;
	b = side_b ;
	x = 13;
	y = 13;
	z = 0;


	rect[0][0] = 13;
	rect[0][1] = 13;
	rect[1][0] =(13 + a);
	rect[1][1] =13;
	rect[2][0] =(13 + a);
	rect[2][1] =(13 + b);
	rect[3][0] = 13;
	rect[3][1] = (13 + b);
	rect[4][0] = 13;
	rect[4][1] = 13;

	number = 0;
	switch (number)
	{
	case 0:
		SD_Inverse_KIN(rect[number][0], rect[number][1], z);
		Sleep(5000);
		number += 1;
	case 1:
		for (double i = x; i <= rect[number][0]; i+=0.1)
		{
			SD_Inverse_KIN(i, rect[number][1], z);

			Sleep(10);
		}
		 number += 1;
		
	case 2:
		for (double i = y; i <= rect[number][1]; i+=0.1)
		{
			SD_Inverse_KIN(rect[number][0], i, z);
			Sleep(10);
		}
		
		number += 1;
		
	case 3:
		for (double i = rect[2][0]; i >= rect[number][0]; i-=0.1)
		{
			SD_Inverse_KIN(i, rect[number][1], z);
			Sleep(10);
		}
		number += 1;
		
	case 4:
		for (double i = rect[3][1]; i >= rect[number][1]; i-=0.1)
		{
			SD_Inverse_KIN(rect[number][0], i, z);
			Sleep(10);
		}
		number += 1;
		
	default:
		break;
	}
			
	/*for (int i = 0; i < 5; i++)
	{
		SD_Inverse_KIN(rect[i][0], rect[i][1], z);
		Sleep(10000);
	}
	/*
	str.Format(_T("%.2f"), rect[0][0]);
	x_value.SetWindowTextW(str);
	//x = _ttoi(str);
	
	str.Format(_T("%.2f"), rect[0][1]);
	y_value.SetWindowTextW(str);
	//y = _ttoi(str);
	
	str.Format(_T("%.2f"), z);
	z_value.SetWindowTextW(str);
	//z = _ttoi(str);
	*/
}


void CRobotics_2Dlg::OnBnClickedButtMove()
{
	CString str;
	double x, y, z;


	x_value.GetWindowTextW(str);
	x = _ttoi(str);

	y_value.GetWindowTextW(str);
	y = _ttoi(str);

	z_value.GetWindowTextW(str);
	z  =  _ttoi(str);

	SD_Inverse_KIN(x, y, z);
}


void CRobotics_2Dlg::OnBnClickedButtsimple()
{
	CString str;
	
	
}

