// ConsoleApplication1.cpp : 定义控制台应用程序的入口点。


#include<iostream>
#include <fstream>
#include <sstream>
#include<winsock.h>
#include <conio.h>
#pragma comment(lib,"ws2_32.lib")
using namespace std;
#include "FtpControl.h"

#include "include/HLrobotconfig.h"
#include "include/MotionPlan.h"

void initialization();
/*函数定义开始*/
vector<PosStruct> userinterface(int functionOption);
PosStruct getPos(int posOption);
void trackingTraj(SOCKET s_server, int i);
void catchObject(SOCKET s_server);
void releaseObject(SOCKET s_server);
int PileUpFunction(vector<PosStruct> vect_desiredPos);
int PileRemoveFunction(vector<PosStruct> DesirePlacePos);
/*函数定义结束*/
#pragma comment(lib, "WS2_32.lib")
int main()
{
    /***********************
    *
    *该部分为与机器人之间的通讯，不需要更改
    *
    ************************/

    //定义长度变量
    int send_len = 0;
    int recv_len = 0;
    //定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
    char recv_buf[200] = {};
    string recvstr;
    //定义服务端套接字，接受请求套接字
    SOCKET s_server;
    //服务端地址客户端地址
    SOCKADDR_IN server_addr;
    initialization();
    //填充服务端信息
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.S_un.S_addr = inet_addr("192.168.10.120");
    server_addr.sin_port = htons(2090);
    //创建套接字
    s_server = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(s_server, (SOCKADDR *)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
        cout << "服务器连接失败！" << endl;
        WSACleanup();
    }
    else {
        cout << "服务器连接成功！" << endl;
    }

    /***********************
    *
    *该部分是机器人宏指令的使用方法
    *
    ************************/

    send_len = send(s_server, "[1# System.Login 0]", 100, 0);        //登录  
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[2# Robot.PowerEnable 1,1]", 100, 0);  //使能
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[3# Robot.Home 1]", 100, 0);      //回零
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[4# System.Auto 1]", 100, 0);    //设置为自动模式
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[5# System.Abort 1]", 100, 0);    //停止   
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[6# System.Start 1]", 100, 0);     //启动
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[7# System.Speed 10]", 100, 0);    //机器人速度设置为10
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[8# IO.Set DOUT(20104),0]", 100, 0);    //关闭放气
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    send_len = send(s_server, "[9# IO.Set DOUT(20103),0]", 100, 0);   //关闭吸气
    recv_len = recv(s_server, recv_buf, 100, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);

    cout << "Sys:功能选择：码垛【1】 or 拆垛【2】" << endl;                           //码垛：将不同位置的垛放置到同一位置；拆垛：把同一位置的垛拆到不同位置
    int functionOption;
    cin >> functionOption;
    
    int num_circle = 0;
    if (functionOption == 1)
    {
        vector<PosStruct> vect_desiredPos = userinterface(functionOption);                //通过交互界面
        num_circle = PileUpFunction(vect_desiredPos);
    }
    else if (functionOption == 2)
    {
        vector<PosStruct>DesirePlacePos = userinterface(functionOption);
        num_circle = PileRemoveFunction(DesirePlacePos);
    }
    else
    {
        cout << "没有这个选项！" << endl;
        return 0;
    }
    for (int i = 0; i < num_circle; i++)
    {
        int num_traj_partial = 0 + 5 * i;                       //路径规划的第几部分

        send_len = send(s_server, "[1# IO.Set DOUT(20104),1]", 100, 0); 
        recv_len = recv(s_server, recv_buf, 100, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(1200);

        trackingTraj(s_server, num_traj_partial);
        catchObject(s_server);
        num_traj_partial++;
        trackingTraj(s_server, num_traj_partial);
        num_traj_partial++;
        trackingTraj(s_server, num_traj_partial);
        num_traj_partial++;
        trackingTraj(s_server, num_traj_partial);
        releaseObject(s_server);
        num_traj_partial++;
        trackingTraj(s_server, num_traj_partial);

        send_len = send(s_server, "[2# IO.Set DOUT(20104),0]", 100, 0);
        recv_len = recv(s_server, recv_buf, 200, 0);
        cout << recv_buf << endl;

        send_len = send(s_server, "[3# IO.Set DOUT(20103),0]", 100, 0); 
        recv_len = recv(s_server, recv_buf, 100, 0);
        cout << recv_buf << endl;
        memset(recv_buf, '\0', sizeof(recv_buf));
        Sleep(1200);
    }

    //关闭套接字
    closesocket(s_server);
    //释放DLL资源
    WSACleanup();
    return 0;
}
void initialization() {
    //初始化套接字库
    WORD w_req = MAKEWORD(2, 2);//版本号
    WSADATA wsadata;
    int err;
    err = WSAStartup(w_req, &wsadata);
    if (err != 0) {
        cout << "初始化套接字库失败！" << endl;
    }
    else {
        cout << "初始化套接字库成功！" << endl;
    }
    //检测版本号
    if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
        cout << "套接字库版本号不符！" << endl;
        WSACleanup();
    }
    else {
        cout << "套接字库版本正确！" << endl;
    }
    //填充服务端地址信息
}

void trackingTraj(SOCKET s_server, int i)
{
    //定义长度变量
    int send_len = 0;
    int recv_len = 0;
    //定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
    char recv_buf[200] = {};

    //将data文件上传至服务器
    string s = "data.txt";
    ostringstream os;
    os << i << s;
    cout << os.str() << endl;

    FtpControl::Upload("192.168.10.101", "data", os.str(), "data.txt");
    //执行关节点位文件
    send_len = send(s_server, "[1# PPB.Enable 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[2# Robot.Frame 1,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[3# PPB.ReadFile 1,/data/data.txt]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    //运行到起始点
    send_len = send(s_server, "[4# PPB.J2StartPoint 1,0,1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    //执行规划-
    send_len = send(s_server, "[5# PPB.Run 1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
}

void catchObject(SOCKET s_server)
{
    //定义长度变量
    int send_len = 0;
    int recv_len = 0;
    //定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
    char recv_buf[200] = {};

    //执行抓取
    send_len = send(s_server, "[8# IO.Set DOUT(20104),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[8# IO.Set DOUT(20103),1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
}

void releaseObject(SOCKET s_server)
{
    //定义长度变量
    int send_len = 0;
    int recv_len = 0;
    //定义发送缓冲区和接受缓冲区
    char send_buf[100] = {};
    char recv_buf[200] = {};

    //执行释放物体
    send_len = send(s_server, "[8# IO.Set DOUT(20103),0]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
    send_len = send(s_server, "[8# IO.Set DOUT(20104),1]", 100, 0);
    recv_len = recv(s_server, recv_buf, 200, 0);
    cout << recv_buf << endl;
    memset(recv_buf, '\0', sizeof(recv_buf));
    Sleep(1200);
}

vector<PosStruct> userinterface(int functionOption)
{
            if (functionOption == 1)
            {
                vector<PosStruct> vect_desiredPos;//存放轨迹期望点
                int num_inputPos = 0;
                bool isFinished_in_ChoosingPos = false;
                while (!isFinished_in_ChoosingPos)
                {
                    cout << "Sys:位置选择：【1~3】" << endl;
                    int posOption;
                    cin >> posOption;
                    if (posOption >= 1 && posOption <= 3)
                    {
                        num_inputPos++;
                        vect_desiredPos.push_back(getPos(posOption));
                        cout << "Sys:位置已选择！" << endl;

                        cout << "Sys：是否继续添加码垛位置？【0/1】" << endl;
                        bool isContinue = false;
                        cin >> isContinue;
                        if (!isContinue)
                        {
                            isFinished_in_ChoosingPos = true;
                            cout << "Sys：位置添加结束！" << endl;
                        }
                    }
                    else
                    {
                        cout << "Warning:位置未选择！" << endl;
                    }
                }
                return vect_desiredPos;
            }
            else
            {
                vector<PosStruct> PlacePos;
                int PileRemoveNum;
                cout << "Sys:选择拆垛个数：【1~3】" << endl;
                cin >> PileRemoveNum;
                for (int i = 1; i <= PileRemoveNum; i++)
                {
                    cout << "Sys:选择第" << i << "个放置垛位：【1~3】" << endl;
                    int Option;
                    cin >> Option;
                    PlacePos.push_back(getPos(Option));
                }
                return PlacePos;
            }
}

//选择码垛后执行这个函数
int PileUpFunction(vector<PosStruct> vect_desiredPos)
{
    //梯型速度规划
    CHLMotionPlan trajectoryPlanner;
    trajectoryPlanner.SetProfileCartesian(0.04, 0.04);
    trajectoryPlanner.SetSampleTime(0.001);      //s

    int height = 50;
    int num_circle = 0;
    while (vect_desiredPos.size() > 0)
    {
        int num_traj_partial = 0 + 5 * num_circle;
        PosStruct desired_Pos_Low = vect_desiredPos.back();

        PosStruct desired_Pos_High = desired_Pos_Low;
        desired_Pos_High.z += height;

        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle+1<<"-1"<<endl;
        trajectoryPlanner.SetPlanPoints(desired_Pos_High, desired_Pos_Low);
        trajectoryPlanner.GetPlanPointsCartesianLine(desired_Pos_High, desired_Pos_Low, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-2" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(desired_Pos_Low, desired_Pos_High);
        trajectoryPlanner.GetPlanPointsCartesianLine(desired_Pos_Low, desired_Pos_High, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

        PosStruct place_Pos_Low = getPos(0);
        place_Pos_Low.z += num_circle * 11;

        PosStruct place_Pos_High = place_Pos_Low;
        place_Pos_High.z += height;

        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-3" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(desired_Pos_High, place_Pos_High);
        trajectoryPlanner.GetPlanPointsCartesianLine(desired_Pos_High, place_Pos_High, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-4" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(place_Pos_High, place_Pos_Low);
        trajectoryPlanner.GetPlanPointsCartesianLine(place_Pos_High, place_Pos_Low, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-5" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(place_Pos_Low, place_Pos_High);
        trajectoryPlanner.GetPlanPointsCartesianLine(place_Pos_Low, place_Pos_High, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;

        num_circle++;
        vect_desiredPos.pop_back();
    }
    return num_circle;
}


//选择拆垛后执行这个函数
int PileRemoveFunction(vector<PosStruct> DesirePlacePos)
{
    int PileRemoveNum = DesirePlacePos.size();            //得到需要拆垛个数
    int SecureHight = 80;                 //路径规划到比目标位置高8cm的地方开始下降避免碰撞
    int Hight = 10;                                  //每拆一个垛多下降1cm
    
    CHLMotionPlan trajectoryPlanner;
    trajectoryPlanner.SetProfileCartesian(0.03, 0.03);
    trajectoryPlanner.SetSampleTime(0.001);

    int num_circle = 0;
    for (int i = 0; i < PileRemoveNum; i++)
    {
        int num_traj_partial = 0 + 5 * num_circle;     //第几段轨迹规划
        PosStruct RemovePos = getPos(0);        //拾取点位
        RemovePos.z = RemovePos.z + (PileRemoveNum - i - 1) * Hight;   
		PosStruct RemovePosHigh = RemovePos;    //拾取点位的安全点
		RemovePosHigh.z += SecureHight;
        PosStruct PlacePos = DesirePlacePos.back();   //放置点位
        PosStruct PlacePosHigh = PlacePos;   //放置点位的安全点
        PlacePosHigh.z += SecureHight;
        //第一段：从拆垛点上方8cm开始下移
        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-1" << endl;
        trajectoryPlanner.SetPlanPoints(RemovePosHigh, RemovePos);
        trajectoryPlanner.GetPlanPointsCartesianLine(RemovePosHigh, RemovePos, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;
        //第二段：从拆垛点上移8cm
        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-2" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(RemovePos, RemovePosHigh);
        trajectoryPlanner.GetPlanPointsCartesianLine(RemovePos, RemovePosHigh, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;
        //第三段：从拆垛点上方8cm移至放置点上方8cm
        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-3" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(RemovePosHigh, PlacePosHigh);
        trajectoryPlanner.GetPlanPointsCartesianLine(RemovePosHigh, PlacePosHigh, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;
        //第四段：从放置点上方8cm移至放置点
        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-4" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(PlacePosHigh, PlacePos);
        trajectoryPlanner.GetPlanPointsCartesianLine(PlacePosHigh, PlacePos, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;
        //第五段：从放置点移至放置点上方8cm
        cout << "Motion Planning in Cartesian Coordinate starts:" << num_circle + 1 << "-5" << endl;
        num_traj_partial++;
        trajectoryPlanner.SetPlanPoints(PlacePos, PlacePosHigh);
        trajectoryPlanner.GetPlanPointsCartesianLine(PlacePos, PlacePosHigh, num_traj_partial);
        cout << "Motion Planning in Cartesian Coordinate is finished!" << endl;
        //循环次数
        num_circle++;
        //清除第一个位置信息
        DesirePlacePos.pop_back();
    }
    return num_circle;
}

//获取码垛位置
PosStruct getPos(int posOption)
{
    PosStruct desired_Pos;
    int height = -7;
    switch (posOption)
    {
        //D
    case 0:
        desired_Pos.x = 363.829; desired_Pos.y = -136.856; desired_Pos.z = 459.244 + height;
        desired_Pos.yaw = -169.646; desired_Pos.pitch = 178.378; desired_Pos.roll = -29.744;
        break;
        //A
    case 1:
        desired_Pos.x = 485; desired_Pos.y = -136.856; desired_Pos.z = 459.244 + height;
        desired_Pos.yaw = -169.646; desired_Pos.pitch = 178.378; desired_Pos.roll = -29.744;
        break;

        //B
    case 2:
        desired_Pos.x = 485; desired_Pos.y = 140.747; desired_Pos.z = 459.244 + height;
        desired_Pos.yaw = -169.646; desired_Pos.pitch = 178.378; desired_Pos.roll = -29.744;
        break;

        //C
    case 3:
        desired_Pos.x = 363.829; desired_Pos.y = 140.747; desired_Pos.z = 459.244 + height;
        desired_Pos.yaw = -169.646; desired_Pos.pitch = 178.378; desired_Pos.roll = -29.744;
        break;

    case 4:
        desired_Pos.x = 441.78; desired_Pos.y = 86.9; desired_Pos.z = 877.035;
        desired_Pos.yaw = 103.388; desired_Pos.pitch = 170.195; desired_Pos.roll = -90.995;
        break;

    case 5:
        desired_Pos.x = 441.78; desired_Pos.y = 86.9; desired_Pos.z = 877.035;
        desired_Pos.yaw = 103.388; desired_Pos.pitch = 170.195; desired_Pos.roll = -90.995;
        break;

    case 6:
        desired_Pos.x = 441.78; desired_Pos.y = 86.9; desired_Pos.z = 877.035;
        desired_Pos.yaw = 103.388; desired_Pos.pitch = 170.195; desired_Pos.roll = -90.995;
        break;

    case 7:
        desired_Pos.x = 441.78; desired_Pos.y = 86.9; desired_Pos.z = 877.035;
        desired_Pos.yaw = 103.388; desired_Pos.pitch = 170.195; desired_Pos.roll = -90.995;
        break;

    case 8:
        desired_Pos.x = 441.78; desired_Pos.y = 86.9; desired_Pos.z = 877.035;
        desired_Pos.yaw = 103.388; desired_Pos.pitch = 170.195; desired_Pos.roll = -90.995;
        break;

        //SAFE POINT
    case 9:
        desired_Pos.x = 402.312; desired_Pos.y = 21.253; desired_Pos.z = 834.774;
        desired_Pos.yaw = -158.865; desired_Pos.pitch = 178.378; desired_Pos.roll = -29.744;
        break;

    }
    return desired_Pos;
}

