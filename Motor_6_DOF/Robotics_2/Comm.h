#pragma once
//---------------------------- ��� ���� --------------------------//
#define	BUFF_SIZE		8192

#define	ASCII_LF			0x0a
#define	ASCII_CR			0x0d
#define	ASCII_XON			0x11
#define	ASCII_XOFF			0x13

#define MODE_CHAR_CHECK		0x00
#define MODE_LENGTH			0x01


#include "stdafx.h"
#include "Robotics_2.h"
#include "Robotics_2Dlg.h"
#include "afxdialogex.h"
#include "dynamixel_sdk.h"
//	��� Ŭ����	CCommThread 

// ��Ʈ ���� : OpenPort("COM1", CBR_9600);
// ��Ʈ���� �б� :
//   ��Ʈ�� �� �Ŀ� ��Ʈ�� �ڷᰡ �����ϸ� WM_COMM_READ �޽����� ���� 
//   �����쿡 ���޵ȴ�. ON_MESSAGE ��ũ�θ� �̿�, �Լ��� �����ϰ�
//   m_ReadData String�� ����� �����͸� �̿� �б�
// ��Ʈ�� ���� : WriteComm(buff, 30)�� ���� ���ۿ� �� ũ�⸦ �ǳ׸� �ȴ�.

class CComm
{
public:
	//--------- ȯ�� ���� -----------------------------------------//
	HANDLE			m_hComm;				// ��� ��Ʈ ���� �ڵ�
	CString			m_sPortName;			// ��Ʈ �̸� (COM1 ..)
	BOOL			m_bConnected;			// ��Ʈ�� ���ȴ��� ������ ��Ÿ��.
	OVERLAPPED		m_osRead, m_osWrite;	// ��Ʈ ���� Overlapped structure
	WORD			m_wPortID;				// WM_COMM_READ�� �Բ� ������ �μ�.
	HANDLE			m_hThreadComm;			// Watch�Լ� Thread �ڵ�.

	unsigned char	m_ucStartChar;
	unsigned char	m_ucEndChar;
	unsigned int	m_udLength;
	unsigned char	m_ucPacketMode;
	char			m_strRcv[2048];
	char			m_strMadePacket[2048];
	unsigned int	m_curCount;

	void (*RcvCallBackFun)(void* pParm);		// �ݹ��Լ��� �����
	void* m_pParm;

	//--------- �ܺ� ��� �Լ� ------------------------------------//
	BOOL	OpenPort(CString sPortName, DWORD dwBaud);
	void	ClosePort();
	DWORD	WriteComm(BYTE* pBuff, DWORD nToWrite);
	void	SetModeLength(unsigned char length);
	void	SetModeCharCheck(unsigned char sChar, unsigned char eChar);
	void	RegCallBackFunction(void (*CallBackFun)(void*), void* pParm);

	//--------- ���� ��� �Լ� ------------------------------------//
	DWORD	ReadComm(BYTE* pBuff, DWORD nToRead);
};

// Thread�� ����� �Լ� 
void ThreadComm(void* pParm);
