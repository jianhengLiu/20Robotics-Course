
#include "stdafx.h"
#include "Robot.h"
#include "QKMLink\qkmlinklib_i.c"
#include

using namespace std;

#define ROBOT_RECV_TIME_OUT     (500)

#pragma region ����/�����ӿ�

Robot::Robot()
{
	//����QKMLink
	// 4.��GUI����ʱ����������:
	// (1)CoInitialize�Ǳ�����õ�API�������ǳ�ʼ�����������COM����
	// (2)CoUninitialize()�Ǳ�����õ�API����CoInitialize���ʹ�ã�������ж
	// �����������COM�������ڹر�GUI��ʱ����ã�ע�⣺���������ط����ã���
	// ����COM�������ʹ�ã����QKMLink�⽫ʧЧ������
	// 
	HRESULT hr = E_FAIL;
	hr = CoInitialize(NULL);
	if (SUCCEEDED(hr))
	{
		m_IsInit = true;

		// CoCreateInstance���ڴ���QKMLink�ӿڣ�����Ŀ2�������Ľӿڶ���Ҫʹ��
		// ��API��������ʹ�á�
		hr = CoCreateInstance(CLSID_QKMLink, NULL, CLSCTX_INPROC_SERVER, IID_IQKMLink, (void**)&g_IQKMLink);
		if (FAILED(hr))
		{
		
			m_IsInit = false;
		}
		hr = CoCreateInstance(CLSID_QKMLinkConfig, NULL, CLSCTX_INPROC_SERVER, IID_IQKMLinkConfig, (void**)&g_IQKMLinkConfig);
		if (FAILED(hr))
		{
		
			m_IsInit = false;
		}
		hr = CoCreateInstance(CLSID_QKMLinkClient, NULL, CLSCTX_INPROC_SERVER, IID_IQKMLinkClient, (void**)&g_IQKMLinkClient);
		if (FAILED(hr))
		{
			
			m_IsInit = false;
		}
	}

}

Robot::~Robot()
{
	g_IQKMLinkClient->Close();
}

#pragma endregion

#pragma region ����


void Robot::AnalysisCmd(CString src, CString & dst) const
{
	src.Remove('[');
	src.Remove(']');
	dst = src;
}

vector<CString> Robot::SplitCString(CString strSource, CString ch) const
{
	vector <CString> vecString;

	int iPos = 0;

	CString strTmp;

	strTmp = strSource.Tokenize(ch, iPos);

	while (strTmp.Trim() != _T(""))

	{
		vecString.push_back(strTmp);

		strTmp = strSource.Tokenize(ch, iPos);

	}

	return vecString;
}

#pragma endregion




#pragma region ͨ�ſ���

bool Robot::Connect(CString ip, unsigned short port, CString id)
{
	if (m_IsInit)
	{

		//��ȡ����״̬
		long linkStatus = -1;
		g_IQKMLinkClient->CurrentState(&linkStatus);

		if (linkStatus == -1)
		{
			//��������������

			// 1.���ÿͻ���
			DWORD dwThreadID;
			HANDLE hClientRecieve;
			long lResult;
			BSTR bsServerAddr = ::SysAllocString(ip);
			BSTR bsSourceID = ::SysAllocString(id);

			// ��1��������Ҫ���ӵķ�����IP��ַ
			g_IQKMLinkClient->put_ServerAddress(bsServerAddr);
			// ��2��������Ҫ���ӵķ�����IP�˿�
			g_IQKMLinkClient->put_ServerPort(port);
			// ��3�����ñ��ͻ��˵�ID��
			g_IQKMLinkClient->put_SourceID(bsSourceID);
			// ��4�����ý���ģʽ��0��ʹ���߳̽��գ�1��ʹ���¼��ص�����
			g_IQKMLinkClient->put_NoticeMode(0);
			// ��5���Ƿ�ʹ��debugģʽ
			g_IQKMLinkClient->put_DebugMode(VARIANT_FALSE);
			// ��6�����ӷ�����
			g_IQKMLinkClient->Connect(&lResult);
			if (lResult < 0)
			{
				IsConnected = false;
				return false;
			}

			// 2.�����ͻ��˽����̣߳����ͷ��߳̾��
			hClientRecieve = CreateThread(NULL, 0, ClientRecieveData, PVOID(this), 0, &dwThreadID);
			CloseHandle(hClientRecieve);

			// 3.�ͷ�BSTR��Դ
			::SysFreeString(bsServerAddr);
			::SysFreeString(bsSourceID);
			IsConnected = true;

			return true;
		}
	}

	return false;
}

void Robot::Disconnect()
{
	IsConnected = false;
	g_IQKMLinkClient->Close();
}

bool Robot::Send(CString sendData)
{
	std::unique_lock<std::mutex> lock(ComLock);

	try
	{

		if (IsConnected)
		{
			bool result = false;

			BSTR bsSend = NULL, bsCommand = NULL, bsState = NULL, bsSourceID = NULL, bsDestinationID = NULL;
			long lPacketID, lSequenceNum, lTimeOut, lCode;

			//ָ���װ
			CString cmd = L"[" + sendData + L"]";
			bsSend = cmd.AllocSysString();	// ָ������

			SendMessageProcess(cmd);

			//��������
			IQKMLinkResult* pIResultClientSend = nullptr;
			g_IQKMLinkClient->Send(bsSend, L"ACKOFF", 10, &pIResultClientSend);

			if (pIResultClientSend != nullptr)
			{
				pIResultClientSend->get_Command(&bsCommand);
				pIResultClientSend->get_SourceID(&bsSourceID);
				pIResultClientSend->get_DestinationID(&bsDestinationID);
				pIResultClientSend->get_PacketID(&lPacketID);
				pIResultClientSend->get_SequenceNum(&lSequenceNum);
				pIResultClientSend->get_Timeout(&lTimeOut);
				pIResultClientSend->get_State(&bsState);
				pIResultClientSend->get_Code(&lCode);
				//sResult.Format(_T("Client feedback result %s %s %s %d %d %d %s %d"), bsCommand, bsSourceID, bsDestinationID, lPacketID, lSequenceNum, lTimeOut, bsState, lCode);
				result = true;
			}

			// 4.�ͷ�BSTR��Դ
			::SysFreeString(bsSend);
			::SysFreeString(bsCommand);
			::SysFreeString(bsSourceID);
			::SysFreeString(bsDestinationID);
			::SysFreeString(bsState);

			// 5.�ͷ�QKMLinkResult
			if (pIResultClientSend != nullptr)
			{
				pIResultClientSend->Release();
			}

			return result;
		}
	}
	catch (const std::exception&)
	{

	}

	return false;
}

//�ͻ��˽����̡߳�
static DWORD WINAPI ClientRecieveData(PVOID lParam)
{
	Robot *dlgLinkTest = (Robot*)lParam;

	//std::unique_lock<std::mutex> lock(dlgLinkTest->ComLock);

	// 1.����COM���������߳���ʹ��QKMLink��Ҫʹ��CoInitialize���߳��˳�ʱ����
	// CoUninitialize()�ͷ�COM����
	HRESULT hr = E_FAIL;
	hr = CoInitialize(NULL);	//�����߳�ʹ��COM��������ô˺�����ʼ��COM��
	if (SUCCEEDED(hr))
	{
		while (true)
		{
			// 2.���մӻ����˶˷��͹�������Ϣ��������������Ϣ����������Ϣ
			BSTR bsMessage = NULL, bsCommand = NULL;
			CString sMessage, sTemp;
			long lErrorCode;
			IQKMLinkResult* pIResultClientRcv = NULL;
			HRESULT hr = NULL;
			hr = dlgLinkTest->g_IQKMLinkClient->Receive(&bsMessage, &pIResultClientRcv, &lErrorCode);
			sMessage = bsMessage;

			// ����error code = 0������������������Ҫ����
			if (lErrorCode == 0)
			{
				if (hr == S_OK && pIResultClientRcv != NULL)
				{
					// 3.��ȡResult�ӿڵ����ԣ�����ʾ
					CString sShowResult;
					long lPacketID, lSequenceNum, lResultCode;
					pIResultClientRcv->get_Code(&lResultCode);
					pIResultClientRcv->get_Command(&bsCommand);
					pIResultClientRcv->get_PacketID(&lPacketID);
					pIResultClientRcv->get_SequenceNum(&lSequenceNum);
					sShowResult.Format(_T("thread: %s,%s,%d,%d\r\n"), sMessage, bsCommand, lPacketID, lSequenceNum);

					//���ݽ���
					if (lResultCode == 0)
					{
						dlgLinkTest->RecvMessageProcess(sMessage);

						dlgLinkTest->RecvLock.lock();
						dlgLinkTest->Message = sMessage;
						dlgLinkTest->RecvStatus = 1;
						dlgLinkTest->RecvLock.unlock();
					}
					else
					{
						dlgLinkTest->RecvLock.lock();
						dlgLinkTest->Message = sMessage;
						dlgLinkTest->RecvStatus = 2;
						dlgLinkTest->RecvLock.unlock();
					}

					// 4.�ͷ���Ϣ��
					hr = dlgLinkTest->g_IQKMLinkClient->Stop(pIResultClientRcv, &lErrorCode);
					if (hr != S_OK && lErrorCode != 0)
					{
						MessageBox(NULL, _T("Client stop result fail!"), _T("Warn"), MB_OK);
					}
				}
			}

			// 5.�ͷ�BSTR��Դ
			::SysFreeString(bsMessage);
			::SysFreeString(bsCommand);

			// 6.�ͷ�QKMLinkResult
			pIResultClientRcv->Release();
		}

		// 7.�߳��˳�ʱ���ͷ��̵߳�COM����
		CoUninitialize();
	}
	return 0;
}

#pragma endregion

#pragma region ϵͳ����

bool Robot::Login()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("System.Login 0"));

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Logout()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("System.Logout"));

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Auto(bool isAuto)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("System.Auto %d"), isAuto);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Start()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("System.Start"));

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Retry()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("System.Retry %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

#pragma endregion

#pragma region �����˿���

bool Robot::PowerOn()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData = L"Robot.PowerEnable 1,1";

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::PowerOff()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData = L"Robot.PowerEnable 1,0";

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Home()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.Home %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Here(double & x, double & y, double & z, double & roll)
{
	double yaw, pitch;

	return Here(x, y, z, yaw, pitch, roll);
}

bool Robot::Here(double & x, double & y, double & z, double & yaw, double & pitch, double & roll)
{
	int config;
	return Here(x, y, z, yaw, pitch, roll, config);
}

bool Robot::Here(double & x, double & y, double & z, double & yaw, double & pitch, double & roll, int & config)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		bool result = false;
		CString sendData;
		sendData.Format(_T("Robot.Where %d"), RobotIndex);

		//��������
		result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);

				//cmd: 0 xxx,xxx,xxx,xxx...
				vector<CString> cmdString = SplitCString(cmd, L" ");
				if (cmdString.size() >= 2)
				{
					if (cmdString[0] == L"0")
					{
						vector<CString> pointString = SplitCString(cmdString[1], L",");

						if (pointString.size() >= 6)
						{
							x = _ttof(pointString[0]);
							y = _ttof(pointString[1]);
							z = _ttof(pointString[2]);
							yaw = _ttof(pointString[3]);
							pitch = _ttof(pointString[4]);
							roll = _ttof(pointString[5]);
							config = _ttoi(pointString[6]);

							RecvLock.unlock();
							return true;
						}
					}
				}
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}

	return false;
}

bool Robot::Joint(double & j1, double & j2, double & j3, double & j4)
{
	double j5, j6;

	return Joint(j1, j2, j3, j4, j5, j6);
}

bool Robot::Joint(double & j1, double & j2, double & j3, double & j4, double & j5, double & j6)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		bool result = false;
		CString sendData;
		sendData.Format(_T("Robot.WhereAngle %d"), RobotIndex);

		//��������
		result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);

				//cmd: 0 xxx,xxx,xxx,xxx...
				vector<CString> cmdString = SplitCString(cmd, L" ");
				if (cmdString.size() >= 2)
				{
					if (cmdString[0] == L"0")
					{
						vector<CString> pointString = SplitCString(cmdString[1], L",");

						if (pointString.size() >= 6)
						{
							j1 = _ttof(pointString[0]);
							j2 = _ttof(pointString[1]);
							j3 = _ttof(pointString[2]);
							j4 = _ttof(pointString[3]);
							j5 = _ttof(pointString[4]);
							j6 = _ttof(pointString[5]);

							RecvLock.unlock();
							return true;
						}
						else if (pointString.size() >= 4)
						{
							j1 = _ttof(pointString[0]);
							j2 = _ttof(pointString[1]);
							j3 = _ttof(pointString[2]);
							j4 = _ttof(pointString[3]);
							j5 = -1;
							j6 = -1;

							RecvLock.unlock();
							return true;
						}
					}
				}

			}
			RecvLock.unlock();

			Sleep(1);
		}
	}

	return false;
}

bool Robot::Brake()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.Brake %d,ALL,0"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::LooseBrake()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.Brake %d,ALL,1"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::GetState(int &powerState, int &homeState, int &runState)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.State %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");

				if ((cmdString.size() == 2) && (cmdString[0] == L"0"))
				{
					vector<CString> status = SplitCString(cmdString[1], L",");
					if (status.size() == 3)
					{
						powerState = _ttoi(status[0]);
						homeState = _ttoi(status[1]);
						runState = _ttoi(status[2]);

						RecvLock.unlock();
						return true;
					}
				}


				RecvLock.unlock();
				return false;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::SetJogMode(bool jogMode, bool jogSwitch)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.JogMode %d,%d,%d"), RobotIndex, jogMode, jogSwitch);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::SetJogDistance(double distance)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.JogDistance %d,%f"), RobotIndex, distance);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::Jog(CString axisName, double jogSpeed)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Move.Jog %s,%f"), axisName, jogSpeed);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::StopJog()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Move.StopJog %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

//��������ϵ
//1: �ؽ�����
//2: ��������
//3: ��������
//4: ��������
bool Robot::SetFrmae(int mode)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("Robot.Frame %d,%d"), RobotIndex, mode);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

#pragma endregion

#pragma region PPB����

bool Robot::EnablePPB(bool isEnable)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("PPB.Enable %d,%d"), RobotIndex, isEnable);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::ReadFilePPB(CString file)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("PPB.ReadFile %d,%s"), RobotIndex, file);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::StartPositionPPB()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("PPB.J2StartPoint %d,0,%d"), RobotIndex, 1);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::RunPPB()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("PPB.Run %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::StopPPB()
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("PPB.Run %d"), RobotIndex);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				vector<CString> cmdString = SplitCString(cmd, L" ");
				RecvLock.unlock();
				return cmdString[0] == L"0";
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

#pragma endregion

#pragma region IO����

bool Robot::SetDOUT(int index, bool isEnable)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("IO.Set DOUT(%d),%d"), index, isEnable);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::SetAOUT(int index, bool isEnable)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("IO.Set AOUT(%d),%d"), index, isEnable);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::GetDOUT(int index, bool &isEnable)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("IO.Get DOUT(%d)"), index);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				isEnable = _ttoi(cmd);
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

bool Robot::GetAOUT(int index, bool &isEnable)
{
	if (IsConnected)
	{
		//��λ���ձ�־
		RecvLock.lock();
		RecvStatus = 0;
		Message = L"";
		RecvLock.unlock();

		CString sendData;
		sendData.Format(_T("IO.Get AOUT(%d)"), index);

		//��������
		bool result = Send(sendData);
		if (!result)
		{
			return false;
		}

		//�ȴ����ݷ���
		for (size_t i = 0; i < ROBOT_RECV_TIME_OUT; i++)
		{
			RecvLock.lock();
			if (RecvStatus != 0)
			{
				CString cmd;
				AnalysisCmd(Message, cmd);
				isEnable = _ttoi(cmd);
				RecvLock.unlock();
				return true;
			}
			RecvLock.unlock();

			Sleep(1);
		}
	}
	return false;
}

#pragma endregion
