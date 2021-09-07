
// Robotics_2Dlg.cpp : implementation file
//

#include "stdafx.h"
#include "Robotics_2.h"
#include "Robotics_2Dlg.h"
#include "afxdialogex.h"
#include "dynamixel_sdk.h"
#include "Framework_learn.h"
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
#define ADDR_STATE_RETURN				68
#define ADDR_PRO_VEL_P_GAIN             78
#define ADDR_PRO_VEL_I_GAIN				76

#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4


#define VELOCITY_MODE4                    1
#define POSITION_MODE4                    3
#define EXTENDED_POSITION_MODE4           4
#define PWM_MODE4                         5 
// for number of motors
#define MEM_AXIS                         7  

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID1                          1                   // Dynamixel ID: 1
#define DXL_ID2                          2
#define DXL_ID4                          4
#define DXL_ID5                          5                   // Dynamixel ID: 1
#define DXL_ID6                          6
#define DXL_ID7                          7


#define BAUDRATE                        115200       // 57600
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
#define  L2  18.0
#define  L3	 6.0
#define  h	 45.0

#define  Lh  0.0
#define q1_center   1862 //163.87/(0.088) 
#define q2_center	2865 //165.54/(0.088)
#define q3_center	2026 //178.29/(0.088)
#define q4_center	2488 //218.97/(0.088)
#define q5_center	1533 //134.97/(0.088)
#define q6_center	2060 //181.36/(0.088)
#define q7_center	2059 //181.22 //181.22/(0.088)


#define TimerCount  33

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
//sycn read and write
bool dxl_addparam_result = true;

int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };         // Goal position

uint8_t dxl_error = 0;                          // Dynamixel error
int32_t dxl_present_position[MEM_AXIS] = { 0 };               // Present position
int32_t dxl_present_velocity[MEM_AXIS] = { 0 };              // Present velocity
int32_t dxl_profile_velocity = 0;
int32_t dxl_threshold[MEM_AXIS] = { 0 };
int32_t moving = 0;
 


int arm[MEM_AXIS] = { 0, DXL_ID1, DXL_ID2, DXL_ID4, DXL_ID5, DXL_ID6, DXL_ID7 };
int edit_presp[MEM_AXIS] = { 0, IDC_EDIT_Presp1, IDC_EDIT_Presp2, IDC_EDIT_Presp4,IDC_EDIT_Presp5,IDC_EDIT_Presp6,IDC_EDIT_Presp7 };
int edit_presv[MEM_AXIS] = { 0, IDC_EDIT_Presv1, IDC_EDIT_Presv2, IDC_EDIT_Presv4,IDC_EDIT_Presv6 ,IDC_EDIT_Presv7,IDC_EDIT_Presv8 };
int edit_goalp[MEM_AXIS] = { 0, IDC_EDIT_Goalp1, IDC_EDIT_Goalp2, IDC_EDIT_Goalp4,IDC_EDIT_Goalp6,IDC_EDIT_Goalp7,IDC_EDIT_Goalp8 };
int edit_goalv[MEM_AXIS] = { 0, IDC_EDIT_Goalv1, IDC_EDIT_Goalv2,IDC_EDIT_Goalv4,IDC_EDIT_Goalv6,IDC_EDIT_Goalv7,IDC_EDIT_Goalv8 };
// new variable for goal position and velocity

int m_qchecks[MEM_AXIS] = { 0, IDC_CHECK1, IDC_CHECK2, IDC_CHECK4, IDC_CHECK5, IDC_CHECK6,IDC_CHECK7 };

bool dxl_addparam = false;
bool dxl_getdata_result = false;

int goalp1, goalp2, goalp4, goalp5, goalp6, goalp7;
int goalv1, goalv2,  goalv4, goalv5, goalv6, goalv7;

int goal_position[MEM_AXIS] = { 0, goalp1, goalp2,goalp4,goalp5,goalp6,goalp7 };
int goal_velocity[MEM_AXIS] = { 0, goalv1, goalv2,goalv4, goalv5, goalv6, goalv7 };

double q_current_position[MEM_AXIS] = {0, q1_center, q2_center, q4_center,q5_center,q6_center,q7_center }; // qiymatlarini belgilash kerak 

double home_addres1, home_addres2, home_addres4, home_addres5, home_addres6, home_addres7;
double home_adds[MEM_AXIS] = { 0, home_addres1, home_addres2, home_addres4, home_addres5, home_addres6, home_addres7 };

int g_smpl_time;
double r_dx, r_dy, r_dz; // bular velocity mode da vaqt va masofa uchun ishlaydi.


int mode_control_flag = POSITION_MODE4;



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






CRobotics_2Dlg::CRobotics_2Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_ROBOTICS_2_DIALOG, pParent)
	, m_torque(FALSE)
	, m_intcombo(0)
	, side_a(_T(""))
	, side_b(_T(""))
	, strCstring(_T(""))
	, m_zupdown(_T(""))
	, m_yupdown(_T(""))
	, m_xupdown(_T(""))
	
	, m_mode_change(0)

{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CRobotics_2Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);

	DDX_Check(pDX, IDC_CHECK_mode_change, m_mode_change);

}

BEGIN_MESSAGE_MAP(CRobotics_2Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_start, &CRobotics_2Dlg::OnClickedButtonStart)
	ON_BN_CLICKED(IDC_BUTTON_stop, &CRobotics_2Dlg::OnClickedButtonStop)

	ON_BN_CLICKED(IDC_BUTTON_connect, &CRobotics_2Dlg::OnClickedButtonConnect)


ON_BN_CLICKED(IDC_BUTT_REC, &CRobotics_2Dlg::OnBnClickedButtRec)
ON_BN_CLICKED(IDC_BUTT_Move, &CRobotics_2Dlg::OnBnClickedButtMove)
ON_BN_CLICKED(IDC_BUTT_simple, &CRobotics_2Dlg::OnBnClickedButtsimple)
ON_BN_CLICKED(IDC_BUTT_Zminus, &CRobotics_2Dlg::OnClickedButtZminus)
ON_BN_CLICKED(IDC_BUTT_Zplus, &CRobotics_2Dlg::OnBnClickedButtZplus)
ON_BN_CLICKED(IDC_BUTT_On_all, &CRobotics_2Dlg::OnBnClickedButtOnall)
ON_BN_CLICKED(IDC_BUTT_Off_all, &CRobotics_2Dlg::OnBnClickedButtOffall)
ON_BN_CLICKED(IDC_BUTT_One_on, &CRobotics_2Dlg::OnBnClickedButtOneon)
ON_BN_CLICKED(IDC_BUTT_Off, &CRobotics_2Dlg::OnBnClickedButtOff)
ON_BN_CLICKED(IDC_BUTT_Home_add, &CRobotics_2Dlg::OnBnClickedButtHomeadd)
ON_BN_CLICKED(IDC_BUTT_Sethome_add, &CRobotics_2Dlg::OnBnClickedButtSethomeadd)
ON_BN_CLICKED(IDC_BUTT_Xplus, &CRobotics_2Dlg::OnBnClickedButtXplus)
ON_BN_CLICKED(IDC_BUTT_Xminus, &CRobotics_2Dlg::OnBnClickedButtXminus)
ON_BN_CLICKED(IDC_BUTT_Yminus, &CRobotics_2Dlg::OnBnClickedButtYminus)
ON_BN_CLICKED(IDC_BUTT_Yplus, &CRobotics_2Dlg::OnBnClickedButtYplus)

ON_BN_CLICKED(IDC_BUTT_Set_Time_vel, &CRobotics_2Dlg::OnBnClickedButtSetTimevel)
ON_BN_CLICKED(IDC_CHECK_mode_change, &CRobotics_2Dlg::OnBnClickedCheckmodechange)

ON_WM_TIMER()

ON_BN_CLICKED(IDC_BUTTON_MOV_XYZ, &CRobotics_2Dlg::OnBnClickedButtonMovXyz)
ON_BN_CLICKED(IDC_BUTT_change_variable, &CRobotics_2Dlg::OnBnClickedButtchangevariable)
ON_BN_CLICKED(IDC_BUTT_Y_min, &CRobotics_2Dlg::OnBnClickedButtYmin)
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
	SetDlgItemText(IDC_EDIT_Goalp6, _T("2048"));
	SetDlgItemText(IDC_EDIT_Goalp7, _T("2048"));
	SetDlgItemText(IDC_EDIT_Goalp8, _T("2048"));

	SetDlgItemText(IDC_EDIT_Goalv1, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv2, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv4, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv6, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv7, _T("20"));
	SetDlgItemText(IDC_EDIT_Goalv8, _T("20"));


	SetDlgItemText(IDC_EDIT_vel_prof, _T("20"));
	SetDlgItemText(IDC_EDIT_acc_prof, _T("0"));
	

	SetDlgItemText(IDC_EDIT_X1, _T("14"));
	SetDlgItemText(IDC_EDIT_Y1, _T("14"));
	SetDlgItemText(IDC_EDIT_Z1, _T("14"));
	CString str;

	g_smpl_time = 10; //msec
	str.Format(_T("%d"), g_smpl_time);
	SetDlgItemTextW(IDC_EDIT_Chas_Time, str);
	

	r_dx = 0.05; //mmeter
	r_dy = 0.05;
	r_dz = 0.05;
	
	str.Format(_T("%.2f"), r_dx); SetDlgItemText(IDC_EDIT_dx, str); // 10msec sampling rate 
	str.Format(_T("%.2f"), r_dy); SetDlgItemText(IDC_EDIT_dy, str); // 10msec sampling rate 
	str.Format(_T("%.2f"), r_dz); SetDlgItemText(IDC_EDIT_dz, str); // 10msec sampling rate 

	for (int i = 1; i < 7; i++)
	{
		home_adds[i] = q_current_position[i];
	}



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
	
	
	for (int i = 1; i < 7; i++)
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
	for (int i = 1; i < 7; i++)
	{
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 0, &dxl_error);
	}
	portHandler1->clearPort();
	portHandler1->closePort();
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
			
			//str.Format(_T("1:%d %s"), packetHandler1->getTxRxResult(dxl_comm_result), packetHandler1->getTxRxResult(dxl_comm_result));
		
		}
		else if (dxl_error != 0)
		{
			
			//str.Format(_T("2:%d %s"), packetHandler1->getRxPacketError(dxl_error), packetHandler1->getRxPacketError(dxl_error));
			AfxMessageBox(str); //  _T("%s\n", packetHandler1->getRxPacketError(dxl_error)) );
		}
		else
		{
		
		}
		for (int i = 1; i < 7; i++)
		{
			//dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_OPERATING_MODE4, POSITION_MODE4, &dxl_error);
			
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4,1, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 1, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_STATE_RETURN, 2, &dxl_error);
			dxl_addparam_result = groupsyncRead.addParam(arm[i]);
			((CButton*)GetDlgItem(m_qchecks[i]))->SetCheck(TRUE);
		}
	
		
		SetTimer(TimerCount,100,NULL);
	
}

void CRobotics_2Dlg::Read_position(int i)
{
	CString str;
	for (int i = 1; i <7; i++)
	{
		dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_PRESENT_POSITION4, (uint32_t*)&dxl_present_position[i], &dxl_error);
		//dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_MOVING_THRESHOLD, (uint32_t*)&dxl_threshold [i] , &dxl_error);

		//SD_Forward_KIN(double deg, double deg1, double deg2)

		dxl_comm_result = packetHandler1->read4ByteTxRx(portHandler1, arm[i], ADDR_PRO_PRESENT_VELOCITY4, (uint32_t*)&dxl_present_velocity[i], &dxl_error);
		
		str.Format(_T("%d"), dxl_present_velocity[i]);
		SetDlgItemText(edit_presv[i], str);
	}

	
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
		beta = acos((L1 * L1 + d2 - L2 * L2) / (2.0 * L1 * d));  // shu yerda xato ketgansan L2 ni L1 ga otkazib qoy
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
										  

	
	get_th[1] = q1_center + th[1];
	get_th[2] = q2_center - 1024 + th[2];
	get_th[3] = q4_center + th[3];
	
	
for( i = 1;  i < 4; i++)
	
	{

		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PROFILE_VELOCITY4, 40, &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PRO_GOAL_POSITION4, get_th[i], &dxl_error);
		
	}

return TRUE;
	

}

void CRobotics_2Dlg::Forward_Kin(double q1_7[7])
{
	CString str;
	double c1, c2, c3, c4, c5, c6;
	double s1, s2, s3, s4, s5, s6;
	double th1, th2, th3, th4, th5, th6,q1_7_degree[7];
	double A0[4][4], A1[4][4], A2[4][4], A3[4][4], A4[4][4], A5[4][4], A6[4][4];
	double T[4][4] = { {1.0,1.0,1.0,1.0 },{1.0,1.0,1.0,1.0 },{1.0,1.0,1.0,1.0 },{1.0,1.0,1.0,1.0 } }; 
	double R03[3][3], R46[3][3], R06[3][3];
	
	for (int i = 1; i < 7; i++)
	{
		q1_7_degree[i] = (q1_7[i] * PI) / 180;
	}

	th1 = q1_7_degree[1];
	th2 = PI / 2 + q1_7_degree[2];
	th3 = PI / 2 + q1_7_degree[3];
	th4 = q1_7_degree[4];
	th5 = q1_7_degree[5];
	th6 = q1_7_degree[6];

	c1 = cos(th1);
	c2 = cos(th2);
	c3 = cos(th3);
	c4 = cos(th4);
	c5 = cos(th5);
	c6 = cos(th6);

	s1 = sin(th1);
	s2 = sin(th2);
	s3 = sin(th3);
	s4 = sin(th4);
	s5 = sin(th5);
	s6 = sin(th6);

	// T =
	//	ci - si * cos(alpha)    si * sin(alpha)   ci * a
	//	si    ci * cos(alpha) - ci * sin(alpha)  si * a
	//	0     sin(aplha)        cos(alpha)        d
	//	0       0                 0               1
	/*
	A0[0][0] = 1;		A0[0][1] = 0;			A0[0][2] = 0;		A0[0][3] = 0;
	A0[1][0] = 0;		A0[1][1] = 0;			A0[1][2] = 1;		A0[1][3] = 0; 
	A0[2][0] = 0;		A0[2][1] = -1;			A0[2][2] = 0;		A0[2][3] = h; // h ni belgilab qo'y
	A0[3][0] = 0;		A0[3][1] = 0;			A0[3][2] = 0;		A0[3][3] = 1;
	*/
	A1[0][0] = c1;		A1[0][1] = 0;			A1[0][2] = s1;		A1[0][3] = 0;
	A1[1][0] = s1;		A1[1][1] = 0;			A1[1][2] = -c1;		A1[1][3] = 0;
	A1[2][0] = 0;		A1[2][1] = 1;			A1[2][2] = 0;		A1[2][3] = L0;
	A1[3][0] = 0;		A1[3][1] = 0;			A1[3][2] = 0;		A1[3][3] = 1;

	A2[0][0] = c2;		A2[0][1] = -s2;			A2[0][2] = 0;		A2[0][3] = L1*c2;
	A2[1][0] = s2;		A2[1][1] = c2;			A2[1][2] = 0;		A2[1][3] = L1*s2;
	A2[2][0] = 0;		A2[2][1] = 0;			A2[2][2] = 1;		A2[2][3] = 0;
	A2[3][0] = 0;		A2[3][1] = 0;			A2[3][2] = 0;		A2[3][3] = 1;

	A3[0][0] = c3;		A3[0][1] = 0;			A3[0][2] = s3;		A3[0][3] = 0;
	A3[1][0] = s3;		A3[1][1] = 0;			A3[1][2] = -c3;		A3[1][3] = 0;
	A3[2][0] = 0;		A3[2][1] = 1;			A3[2][2] = 0;		A3[2][3] = 0;
	A3[3][0] = 0;		A3[3][1] = 0;			A3[3][2] = 0;		A3[3][3] = 1;

	A4[0][0] = c4;		A4[0][1] = 0;			A4[0][2] = s4;		A4[0][3] = 0;
	A4[1][0] = s4;		A4[1][1] = 0;			A4[1][2] = -c4;		A4[1][3] = 0;
	A4[2][0] = 0;		A4[2][1] = 1;			A4[2][2] = 0;		A4[2][3] = L2;
	A4[3][0] = 0;		A4[3][1] = 0;			A4[3][2] = 0;		A4[3][3] = 1;

	A5[0][0] = c5;		A5[0][1] = 0;			A5[0][2] = -s5;		A5[0][3] = 0;
	A5[1][0] = s5;		A5[1][1] = 0;			A5[1][2] = c5;		A5[1][3] = 0;
	A5[2][0] = 0;		A5[2][1] = -1;			A5[2][2] = 0;		A5[2][3] = 0;
	A5[3][0] = 0;		A5[3][1] = 0;			A5[3][2] = 0;		A5[3][3] = 1;

	A6[0][0] = c6;		A6[0][1] = -s6;			A6[0][2] = 0;		A6[0][3] = 0;
	A6[1][0] = s6;		A6[1][1] = c6;			A6[1][2] = 0;		A6[1][3] = 0;
	A6[2][0] = 0;		A6[2][1] = 0;			A6[2][2] = 1;		A6[2][3] = L3;
	A6[3][0] = 0;		A6[3][1] = 0;			A6[3][2] = 0;		A6[3][3] = 1;


	
	mult_matrix_4x4(A1, A2, T);
	mult_matrix_4x4(T, A3, T);

	
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R03[i][j] = T[i][j]; //Rotation matrix 3 ta burchak uchun... q1, q2, q4;
		}
	}

	mult_matrix_4x4(T, A4, T);
	mult_matrix_4x4(T, A5, T);
	mult_matrix_4x4(T, A6, T);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R06[i][j] = T[i][j]; //Rotation matrixdan R orientationni topish q5, q6, q7...
		}
	}

	str.Format(_T("%.2f,   %.2f,   %.2f"), T[0][3], T[1][3], T[2][3] );
	SetDlgItemText(IDC_EDIT_XYZ, str);


}

void CRobotics_2Dlg::mult_matrix_4x4(double aa[4][4], double bb[4][4], double cc[4][4])
{
	int i, j, k;
	double tt[4][4];

	for (i = 0; i < 4; i++) 
	{
		for (j = 0; j < 4; j++)
		{
			tt[i][j] = 0.0;
			for (k = 0; k < 4; k++)
			{
				tt[i][j] = tt[i][j] + (aa[i][k] * bb[k][j]);
			}
		}
	}

	for (i = 0; i < 4; i++) 
		{
			for (j = 0; j < 4; j++)
			{
				cc[i][j] = tt[i][j];
			}
		}
	


}


void CRobotics_2Dlg::Set_pos_zero(double deg[7])
{
	CString str;
	for (int i = 1; i < 7; i++)
		{
		deg[i] = (dxl_present_position[i] - q_current_position[i]) * basic_q;
		if (deg[i] >= 180)deg[i] = deg[i] - 360;
		else if (deg[i] <= -180) deg[i] = deg[i] + 360;

		str.Format(_T("%.2f,  %.2f,  %.2f "), deg[1], deg[2], deg[3]);
		SetDlgItemText(IDC_EDIT_q13, str);

		str.Format(_T("%.2f,  %.2f,  %.2f "), deg[4], deg[5], deg[6]);
		SetDlgItemText(IDC_EDIT_q57, str);

		}
	for (int i = 1; i < 7; i++)
		{
		str.Format(_T("%.2f"), deg[i]);
		SetDlgItemText(edit_presp[i], str);
		}
	Forward_Kin(deg);
}


void CRobotics_2Dlg::rect_inverse(double rec[2][5]) // ishlatmayapman 
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
	switch (number&&((CButton*)GetDlgItem(IDC_CHECK1))->GetCheck() == TRUE)
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
			

}


void CRobotics_2Dlg::OnBnClickedButtMove()
{
}


void CRobotics_2Dlg::OnBnClickedButtsimple()
{
	uint8_t y[2];
	int x[1];
	x[0] = 2005;
	y[0] = DXL_LOBYTE(DXL_LOWORD(x[0]));
	y[0] = DXL_HIBYTE(DXL_LOWORD(x[0]));
	y[0] = DXL_LOBYTE(DXL_HIWORD(x[0]));
	y[0] = DXL_HIBYTE(DXL_HIWORD(x[0]));

	groupsyncWrite.changeParam(DXL_ID1, (uint8_t*)x);
	groupsyncWrite.changeParam(DXL_ID2, (uint8_t*)x);
	groupsyncWrite.changeParam(DXL_ID4, (uint8_t*)x);

	groupsyncWrite.txPacket();
}


void CRobotics_2Dlg::OnBnClickedButtOnall()
{

	UpdateData(TRUE);
	for (int i = 1; i < 7; i++)
	{
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 1, &dxl_error);
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 1, &dxl_error);
		((CButton*)GetDlgItem(m_qchecks[i]))->SetCheck(TRUE);
		
	}
	
}


void CRobotics_2Dlg::OnBnClickedButtOffall()
{
	UpdateData(TRUE);
	for (int i = 1; i < 7; i++)
	{
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
		dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 0, &dxl_error);
		((CButton*)GetDlgItem(m_qchecks[i]))->SetCheck(FALSE);
		
	}
	
}


void CRobotics_2Dlg::OnBnClickedButtOneon()
{
	UpdateData(TRUE);
	
	for (int i = 1; i < 7; i++)
	{
		if (((CButton*)GetDlgItem(m_qchecks[i]))->GetCheck()==TRUE)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 1, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 1, &dxl_error);
			
		}
	}
}


void CRobotics_2Dlg::OnBnClickedButtOff()
{
	for (int i = 1; i < 7; i++)
	{
		if (((CButton*)GetDlgItem(m_qchecks[i]))->GetCheck() == FALSE)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_LED_ENABLE4, 0, &dxl_error);
			
		}
	}
	
}


void CRobotics_2Dlg::OnBnClickedButtHomeadd()
{

	for (int i = 1; i < 7; i++)
	{
		
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PROFILE_VELOCITY4, 40, &dxl_error);
		dxl_comm_result = packetHandler1->write4ByteTxRx(portHandler1, arm[i], ADDR_PRO_GOAL_POSITION4, home_adds[i], &dxl_error);
		
	}
}


void CRobotics_2Dlg::OnBnClickedButtSethomeadd()
{
	CString str;
	double deg[7];
	Read_position(1);
	for (int i = 1; i < 7; i++)
	{
		home_adds[i] = dxl_present_position[i];
	}
	
}


void CRobotics_2Dlg::OnBnClickedButtXplus()
{
	CString str, str2, str3;
	double x, y, z;

	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str) + 0.388;
	// x ga if berish kerak chunki x ni oraligi kerak 

	str.Format(_T("%.2f"), x);
	SetDlgItemText(IDC_EDIT_X1, str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str);


	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str);


	SD_Inverse_KIN(x, y, z);
	
}


void CRobotics_2Dlg::OnBnClickedButtXminus()
{
	CString str;
	double x, y, z;
	
	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str) - 0.388;

	str.Format(_T("%.2f"), x);
	SetDlgItemText(IDC_EDIT_X1, str);


	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str);


	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str);

	SD_Inverse_KIN(x, y, z);

}

void CRobotics_2Dlg::OnClickedButtZminus()
{
	CString str;
	double x, y, z;

	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str);


	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str);
	if (z >= 0)
	{
		z = z - 0.388;
	}
	else
	{
		z = 0;
	}
	str.Format(_T("%.2f"), z);
	SetDlgItemText(IDC_EDIT_Z1, str);



	SD_Inverse_KIN(x, y, z);
}

void CRobotics_2Dlg::OnBnClickedButtZplus()
{
	CString str, str2, str3;
	double x, y, z;

	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str);

	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str) + 0.388;

	str.Format(_T("%.2f"), z);
	SetDlgItemText(IDC_EDIT_Z1, str);


	SD_Inverse_KIN(x, y, z);
}


void CRobotics_2Dlg::OnBnClickedButtYminus()
{
	CString str, str2, str3;
	double x, y, z;

	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str) - 0.388;
	str.Format(_T("%.2f"), y);
	SetDlgItemText(IDC_EDIT_Y1, str);

	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str);


	SD_Inverse_KIN(x, y, z);
}


void CRobotics_2Dlg::OnBnClickedButtYplus()
{
	CString str, str2, str3;
	double x, y, z;

	GetDlgItemText(IDC_EDIT_X1, str);
	x = _wtof(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	y = _wtof(str);
	y = y + 0.388;
	str.Format(_T("%.2f"), y);
	SetDlgItemText(IDC_EDIT_Y1, str);

	GetDlgItemText(IDC_EDIT_Z1, str);
	z = _wtof(str);


	SD_Inverse_KIN(x, y, z);
}



void CRobotics_2Dlg::OnBnClickedButtSetTimevel()
{
	CString str;
	double set_Timer_VEL;
	GetDlgItemTextW(IDC_EDIT_Chas_Time,str);
	g_smpl_time = _wtof(str);
	
	SetTimer(TimerCount, g_smpl_time, NULL);
}


void CRobotics_2Dlg::OnBnClickedCheckmodechange()
{
	UpdateData(TRUE);
	
	if (m_mode_change == 1)
	{
		for (int i = 1; i < 4; i++)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_OPERATING_MODE4, VELOCITY_MODE4, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 1, &dxl_error);

            dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_VEL_P_GAIN, 100, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_VEL_I_GAIN, 1920, &dxl_error);
		}
		mode_control_flag = VELOCITY_MODE4;
		OnBnClickedButtSetTimevel();
	}
	else
	{
		for (int i = 1; i < 4; i++)
		{
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 0, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_OPERATING_MODE4, POSITION_MODE4, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_TORQUE_ENABLE4, 1, &dxl_error);

			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_VEL_P_GAIN, 100, &dxl_error);
			dxl_comm_result = packetHandler1->write1ByteTxRx(portHandler1, arm[i], ADDR_PRO_VEL_I_GAIN, 1920, &dxl_error);

		}
		mode_control_flag = POSITION_MODE4;
		SetTimer(TimerCount, 100, NULL);
	}
}


void CRobotics_2Dlg::OnTimer(UINT_PTR nIDEvent)
{
	CString str;
	double deg[7],q_deg[7], motor_deg[3][3] = {0};
	Read_position(1);
	Set_pos_zero(deg);
	
	switch (nIDEvent)
	{
		case TimerCount:
			if (mode_control_flag == POSITION_MODE4)
				{
				Read_position(1);
				Set_pos_zero(deg);
			
				}
			if (m_mode_change ==1 && mode_control_flag == VELOCITY_MODE4)
				{
					syncRead_joint_Position(dxl_present_position);

					for (int i = 1; i < 7; i++)
					{
						deg[i] = (dxl_present_position[i] - q_current_position[i]) * basic_q;
						if(deg[i] >= 180)deg[i] = deg[i] - 360;
						else if (deg[i] <= -180) deg[i] = deg[i] + 360;
					}

					for (int i = 1; i < 7; i++)
					{
						q_deg[i] = deg[i] * PI / 180.0;
					}
					Manage_Arm(q_deg, motor_deg);
				}
			break;

		
	}
	
	CDialogEx::OnTimer(nIDEvent);
}


int CRobotics_2Dlg::syncRead_joint_Position(int pos_data[7])
{
	int id;
	dxl_comm_result = groupsyncRead.txRxPacket();
	for (int m = 1; m < 7; m++)
	{
		do {
			
			dxl_getdata_result = groupsyncRead.isAvailable(arm[m], ADDR_PRO_PRESENT_POSITION4, LEN_PRO_PRESENT_POSITION);
		} while (dxl_getdata_result != true);
		if (dxl_getdata_result != true) return 0;
	}

	for (int m = 1; m < 7; m++)
	{
		
		pos_data[m] = groupsyncRead.getData(arm[m], ADDR_PRO_PRESENT_POSITION4, LEN_PRO_PRESENT_POSITION);

	}
	return 1;
}

void CRobotics_2Dlg::Manage_Arm(double deg[7], double back_inv_jac[3][3])
{
	double send_inv_jac[3][3];
	double res_velq[3], back_inv_jacob[3][3];
	double dx_vel, dy_vel, dz_vel, rvel_base;
	int write_motor[3];
	CString str;
	rvel_base = (2.0 * PI * 0.229) / 60.0;

	math_jacobian_calc(deg,send_inv_jac);
	inverse_jacobian(send_inv_jac, back_inv_jacob);


	dx_vel = (r_dx * 1000) / g_smpl_time;
	dy_vel = (r_dy * 1000) / g_smpl_time;
	dz_vel = (r_dz * 1000) / g_smpl_time;
	res_velq[0] = back_inv_jacob[0][0] * dx_vel + back_inv_jacob[0][1] * dy_vel + back_inv_jacob[0][2] * dz_vel;
	res_velq[1] = back_inv_jacob[1][0] * dx_vel + back_inv_jacob[1][1] * dy_vel + back_inv_jacob[1][2] * dz_vel;
	res_velq[2] = back_inv_jacob[2][0] * dx_vel + back_inv_jacob[2][1] * dy_vel + back_inv_jacob[2][2] * dz_vel;

	write_motor[0] = (int)(res_velq[0] / rvel_base);
	write_motor[1] = (int)(res_velq[1] / rvel_base);
	write_motor[2] = (int)(res_velq[2] / rvel_base);

	
	str.Format(_T("%d, %d, %d"), write_motor[0], write_motor[1], write_motor[2]);
	SetDlgItemText(IDC_EDIT9, str);

	sycnWrite_joint_Velocity_123(write_motor);

}

void CRobotics_2Dlg::sycnWrite_joint_Velocity_123(int motor[3])
{
	uint8_t param[4];
	int motor2[4];
	int i,id;
	CString str;

	motor2[0] = 0; motor2[1] = motor[0]; motor2[2] = motor[1]; motor2[3] = motor[2];
	for (i = 1; i < 4; i++)
	{
		param[0] = DXL_LOBYTE(DXL_LOWORD(motor2[i]));
		param[1] = DXL_HIBYTE(DXL_LOWORD(motor2[i]));
		param[2] = DXL_LOBYTE(DXL_HIWORD(motor2[i]));
		param[3] = DXL_HIBYTE(DXL_HIWORD(motor2[i]));

		
		dxl_addparam_result = groupsyncWrite.addParam(arm[i], param);
		if (dxl_addparam_result != true)
		{
			str.Format(_T("%d"), i);
			AfxMessageBox(str);
		}
	}
	dxl_comm_result = groupsyncWrite.txPacket();
	groupsyncWrite.clearParam();
}


void CRobotics_2Dlg::inverse_jacobian(double inv_jac[3][3], double send_inv_jac[3][3]) 
{
	double det, invdet;
	det = inv_jac[0][0] * (inv_jac[1][1] * inv_jac[2][2] - inv_jac[2][1] * inv_jac[1][2])
		- inv_jac[0][1] * (inv_jac[1][0] * inv_jac[2][2] - inv_jac[1][2] * inv_jac[2][0])
		+ inv_jac[0][2] * (inv_jac[1][0] * inv_jac[2][1] - inv_jac[1][1] * inv_jac[2][0]);

	invdet = 1 / det;

	// Matrix33d minv; // inverse of matrix m

	send_inv_jac[0][0] = (inv_jac[1][1] * inv_jac[2][2] - inv_jac[2][1] * inv_jac[1][2]) * invdet;
	send_inv_jac[1][0] = (inv_jac[1][2] * inv_jac[2][0] - inv_jac[1][0] * inv_jac[2][2]) * invdet;
	send_inv_jac[2][0] = (inv_jac[1][0] * inv_jac[2][1] - inv_jac[2][0] * inv_jac[1][1]) * invdet;

	send_inv_jac[0][1] = (inv_jac[0][2] * inv_jac[2][1] - inv_jac[0][1] * inv_jac[2][2]) * invdet;
	send_inv_jac[1][1] = (inv_jac[0][0] * inv_jac[2][2] - inv_jac[0][2] * inv_jac[2][0]) * invdet;
	send_inv_jac[2][1] = (inv_jac[2][0] * inv_jac[0][1] - inv_jac[0][0] * inv_jac[2][1]) * invdet;

	send_inv_jac[0][2] = (inv_jac[0][1] * inv_jac[1][2] - inv_jac[0][2] * inv_jac[1][1]) * invdet;
	send_inv_jac[1][2] = (inv_jac[1][0] * inv_jac[0][2] - inv_jac[0][0] * inv_jac[1][2]) * invdet;
	send_inv_jac[2][2] = (inv_jac[0][0] * inv_jac[1][1] - inv_jac[1][0] * inv_jac[0][1]) * invdet;



}





void CRobotics_2Dlg::math_jacobian_calc(double q_rad[7], double inv_jac[3][3])
{
	double q1, q2, q3, q4, q5, q6;
	double deg_q[7];
	double j11, j12, j13, j21, j22, j23, j41, j42, j43;
	double c1, c2, c3, c4, c5, c6;
	double s1, s2, s3, s4, s5, s6;
	q1 = q_rad[1]; q2 = q_rad[2] + PI/2; q3 = q_rad[3] + PI / 2; q4 = q_rad[4]; q5 = q_rad[5]; q6 = q_rad[6];

	c1 = cos(q1); s1 = sin(q1);
	c2 = cos(q2); s2 = sin(q2);
	c3 = cos(q3); s3 = sin(q3);
	c4 = cos(q4); s4 = sin(q4);
	c5 = cos(q5); s5 = sin(q5);
	c6 = cos(q6); s6 = sin(q6);


	// th1 dan th3 gacha bolgan x,y,z
	//Px = c1* c2* l2;
	//Py = c2*l2*s1;
	// Pz = l1 + l2*s2; this is up to th3;

	// th1 dan th4 gacha bolgan x,y,z
	//px = l3* (c1 * c2 * s3 + c1 * c3 * s2) + c1 * c2 * l2; this up to th4
	//py = l3* (c2 * s1 * s3 + c3 * s1 * s2) + c2 * l2 * s1;
	//pz = l1 + l2 * s2 - l3 * (c2 * c3 - s2 * s3)

	// th1 dan th6 gacha bolgan x,y,z
	//L3*(c5*(c1*c2*s3 + c1*c3*s2) - s5*(s1*s4 + c4*(c1*c2*c3 - c1*s2*s3))) + L2*(c1*c2*s3 + c1*c3*s2) + L1*c1*c2
	//L3*(c5*(c2*s1*s3 + c3*s1*s2) + s5*(c1*s4 - c4*(c2*c3*s1 - s1*s2*s3))) + L2*(c2*s1*s3 + c3*s1*s2) + L1*c2*s1
	//L0 + L1*s2 - L2*(c2*c3 - s2*s3) - L3*(c5*(c2*c3 - s2*s3) + c4*s5*(c2*s3 + c3*s2))
	 // x,y,z 6ta burchakdan chiqqan natija: th1, th2, th3 bo'yicha xosila olamiz/


	j11 = L3 * (c5 * (-s1 * c2 * s3 - s1 * c3 * s2) - s5 * (c1 * s4 + c4 * (-s1 * c2 * c3 + s1 * s2 * s3))) + L2 * (-s1 * c2 * s3 - s1 * c3 * s2) - L1 * s1 * c2;
	j12 = L3 * (c5 * (c2 * c1 * s3 + c3 * c1 * s2) + s5 * (-s1 * s4 - c4 * (c2 * c3 * c1 - c1 * s2 * s3))) + L2 * (c2 * c1 * s3 + c3 * c1 * s2) + L1 * c2 * c1;
	j13 = 0;

	j21 = L3 * (c5 * (c1 * (-s2) * s3 + c1 * c3 * 2) - s5 * (0 + c4 * (c1 * (-s2) * c3 - c1 * c2 * s3))) + L2 * (c1 * (-s2) * s3 + c1 * c3 * c2) - L1 * c1 * s2;
	j22 = L3 * (c5 * ((-s2) * s1 * s3 + c3 * s1 * c2) + s5 * (0 - c4 * ((-s2) * c3 * s1 - s1 * c2 * s3))) + L2 * ((-s2) * s1 * s3 + c3 * s1 * c2) - L1 * s2 * s1;
	j23 = L1 * c2 - L2 * ((-s2) * c3 - c2 * s3) - L3 * (c5 * ((-s2) * c3 - c2 * s3) + c4 * s5 * ((-s2) * s3 + c3 * c2));
	/*
	Bu o'rtada th3 ga tegishli hosila bo`lish kerak edi lekin biz th3 ni bog`lab qo`yganimiz uchun uni hisoblamadik.
	*/

	j41 = L3 * (c5 * (c1 * c2 * c3 - c1 * s3 * s2) - s5 * (0 + c4 * (-c1 * c2 * s3 - c1 * s2 * c3))) + L2 * (c1 * c2 * c3 - c1 * s3 * s2) + 0;
	j42 = L3 * (c5 * (c2 * s1 * s3 + c3 * s1 * s2) + s5 * (c1 * s4 - c4 * (c2 * c3 * s1 - s1 * s2 * s3))) + L2 * (c2 * s1 * s3 + c3 * s1 * s2) + 0;
	j43 = 0 - L2 * (-c2 * s3 - s2 * c3) - L3 * (c5 * (-c2 * s3 - s2 * c3) + c4 * s5 * (c2 * c3 - s3 * s2));

	inv_jac[0][0] = j11;			inv_jac[1][0] = j21;						inv_jac[2][0] = j41;
	inv_jac[0][1] = j12;			inv_jac[1][1] = j22;						inv_jac[2][1] = j42;
	inv_jac[0][2] = j13;			inv_jac[1][2] = j23;						inv_jac[2][2] = j43;


}



void CRobotics_2Dlg::OnBnClickedButtonMovXyz()
{
	CString str;
	double a, b, c;

	GetDlgItemText(IDC_EDIT_X1, str);
	a = _wtof(str);

	GetDlgItemText(IDC_EDIT_Y1, str);
	b = _wtof(str);

	GetDlgItemText(IDC_EDIT_Z1, str);
	c = _wtof(str);

	SD_Inverse_KIN(a, b, c);
}


void CRobotics_2Dlg::OnBnClickedButtchangevariable()
{
	CString str;
	double dx_mmsec, dy_mmsec, dz_mmsec, dist;
	//GetDlgItemText(IDC_EDIT_SAMPLE_RATE, str);
	//smpl = _ttoi(str);

	GetDlgItemText(IDC_EDIT_dx, str);  r_dx = _wtof(str);
	dx_mmsec = r_dx * 1000.0 / g_smpl_time;
	str.Format(_T("%.2f"), dx_mmsec); SetDlgItemText(IDC_EDIT_change_dx, str);

	GetDlgItemText(IDC_EDIT_dy, str);  r_dy = _wtof(str);
	dy_mmsec = r_dy * 1000.0 / g_smpl_time;
	str.Format(_T("%.2f"), dy_mmsec); SetDlgItemText(IDC_EDIT_change_dy, str);

	GetDlgItemText(IDC_EDIT_dx, str);  r_dz = _wtof(str);
	dz_mmsec = r_dz * 1000.0 / g_smpl_time;
	str.Format(_T("%.2f"), dz_mmsec); SetDlgItemText(IDC_EDIT_change_dz, str);

}


void CRobotics_2Dlg::OnBnClickedButtYmin()
{
	CString str;

	GetDlgItemText(IDC_EDIT_dx, str);
	r_dx = _wtof(str);  str = "";  // 30msec 	

	r_dy *= -1.0;
	r_dx = r_dz = 0.0;
	str.Format(_T("Y = %f"), r_dy);
	// AfxMessageBox(str);
	mode_control_flag = VELOCITY_MODE4; // 일단은 OnTimer에서 제어 

}
