#ifndef interface_h
#define interface_h

#include "HRSDK/HR_JsonBase.h"

#include "HRSDK/HR_AutoLock.h"

#include <fstream>

// Sameple_XToStandby
bool XToStandby();

// Sample_SetSimulateRobot.cpp
int ReadSimRobot();
void SetSimRobot();
void SetRealRobot();

// Sample_BrakeControl.cpp
void CloseBrake();
void OpenBrake();

// Smaple_Grap.cpp
void GrapControl();
void GrpFreeDriver();

// Sample_Script.cpp
void StartScript();
void StopScript();
void RunScript();
void RunFunc();
void RunApp();

// Sample_BoxControl.cpp
void BoxInfo();
void SetDigitalOutput(const string &name, int nBit, int nVal);
int ReadDigitalIO(const string &name, int nBit);
void SetAO(int nBit, int nMode, double dVal = -1);
double ReadAIO(const string &name, int nBit, int &nMode);
void UseBTN();

// Sameple_BoxID.cpp
class SampleBoxID
{
private:
    string m_strIP0;
    string m_strIP1;
    int m_nBoxID0;
    int m_nBoxID1;
public:
    SampleBoxID(string IP1,string IP2);
    ~SampleBoxID();
    void ConfigIpBoxId();
    void SendCmdInThreadA();
    void SendCmdInThreadB();
};

// Sample_SysState.cpp
void ShowInitSysState();
void SetSystemParams();
void GetSystemParams();
void ShowErrorInfo();

// Sample_PointInfo.cpp
void ReadWaypointInfo();
void ReadPointInfo();

// Sample_RobotState.cpp
void ShowRealTimeMotionState();
void MotionDiffState();
//标定力
void setBaseInstallingAngle();
void loadIdentify();
void CalibrationForce();
// CoordivateTran.cpp
void TranQuaternionRPY();
void GetInverseForwardKin();
void BaseUcsTcpTran();
void PointTran();

// Sample_ConfigTcpUcs.cpp
void ConfigTcp4Point();
void ConfigTcp3Point();
void CalUcsLine();
void CalUcsPlane();
void CalUcs3PointFeatures();

// Sample_ForceControl.cpp
bool InitControlStrategy(const int nStrategy);
void InitForceControl();
bool MoveBegin();
void StarForce();
void ForceMoveL();
void StopForce();

// Sample_FreeDrive.cpp
void InitFreeDrive();
void ReadFTMotionFreedom();
void SetForceZero();
void StarFreeDrive();
void StopFreeDrive();

// Sample_Move.cpp
void JudgeBlendingState();
void MoveJ(double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6);
void MoveL(double dX, double dY, double dZ, double dRx, double dRy, double dRz);
void MoveRelJ();
void MoveRelL();
void WayPointRel();
void IsMotionDone();
void WayPointEx();
void WayPoint();
void WayPoint2();
void MoveC();
void MoveZ();
void MoveE();
void MoveS();
void MoveC();

// Sample_MovePath.cpp
void AddPathJ(const string &sPathName);
void AddPathL(const string &sPathName);
void MoveToPathBegin(const string &sPathName, const string &sMode);
void MovePathJ(const string &sPathName);
void MovePathL(const string &sPathName);
bool ReadPath(const string &sPathName);
void UpdatePathName(const string &sPathName, const string &sPathNewName);
void DeletePath(const string &sPathName);

// Sample_MovePathJ.cpp
void InitPathJ();
void PushMoveJ();
void EndPush();
void JudgeState();
void MovePathJ();

// Sample_MovePathL.cpp
void InitPathL();
void PushMoveL();
void EndPushL();
void JudgeStateL();
void MovePathL();

// Sample_Servo.cpp
void Init_Servo();
void ServoJ();
void ServoP();

// Sample_Trace.cpp
void MoveToTraceBegin();
void InitPosTraceParams();
void SetPoseTraceTargetPos();
void StarPosTrace();
void StopPosTrace();
void MoveTrace();
// Sample_Trace.cpp 暂未启用
void SetTraceParams();
void InitTraceParams();
void SetTraceUcs();
void StarTrace();
void StopTrace();




// Sample_ConnectToMoudleBus.cpp
void WriteToRegisters();
void ReadFromRegisters();


// json解析
int createsocket(const string &IP);
void Delete_characters(string &JsonString, string substr);
void GetOriginJson(string &JsonString);
void GudgeJsonGotTargetString(string &JsonData, string target);
string splitJsonString(string &JsonString, string target);
void receive(int sockfd, string &JsonString);
void GetJsonArrayVlaue(void *jsonVoid, const string &childName, const string &sonName, vector<string> &vecVal);
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, int &output);
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, string &output);
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, double &output);



//Json解析（面向对象版）
class ParsePortJsonData : public HR_JsonBase
{
private:
    // 连接使用变量
    int m_nSockfd;
    string m_strIp;

    // 两个线程共用变量，需要使用锁
    SaftLock m_WriteLock; // 安全锁，对数据m_strJsonData保护
    string m_strJsonData; // 10006端口读取到的数据
    int m_nStopParse;

    // 处理数据线程使用变量
    unsigned int m_unDataLength; // 数据长度
    size_t m_nPos;               // 字符串位置
    std::ofstream outputFile;

private:
    void JudgeJsonGotTargetString(const string target);
    void GetDataLength();
    void SplitJsonString();
    void ShowKey();
    void ShowValue();

public:
    ParsePortJsonData(const string &strIp);
    ~ParsePortJsonData();
    // 主线程连接
    void CreateSocket();

    // 接受数据线程
    void ReceiveData();

    // 处理数据线程
    void ParseJsonData();
};

//
void setBaseInstallingAngle();
void loadIdentify();
void CalibrationForce();
#endif