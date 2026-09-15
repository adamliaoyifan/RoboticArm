#include "HR_Pro.h"

#include "interface.h"

#include <HRSDK/HR_JsonBase.h>
#include <HRSDK/cJSON.h>

#include <iostream>
#include <unistd.h>
#include <thread>
#include <sys/socket.h>

/**
 *	@brief: Sample_XToStandby.cpp 启动接口，将机械臂从其它状态返回到Standby状态
 *	@param ：无
 *	@return : 无
 */
void UserCase_XToStandby()
{
    if (XToStandby())
    {
        std::cout << "XToStandby Success" << endl;
    }
    else
    {
        std::cout << "XToStandby Fail" << endl;
    }
}

/** Sample_SetSimulateRobot.cpp接口，读取机器人是否为模拟机器人，如果是真实机器人则设置为模拟机器人
 *	@brief:
 *	@param ：无
 *	@return : 无
 */
void UserCase_SetSimRobot()
{
    int nCurFSM = 0;
    for (int ElapsedTime = 0; ElapsedTime < 15; ElapsedTime += 1)
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        if (nCurFSM == 7)
        {
            break;
        }
        std::cout << "机器人当前不在断电状态，如需继续运行，请置到断电状态" << std::endl;
        sleep(1);
    }
    if (nCurFSM == 7)
    {
        if (ReadSimRobot() == 0) // 机器人为真实机器人
        {
            SetSimRobot();
        }
        else
        {
            SetRealRobot();
        }
        ReadSimRobot();
    }
}

/** Sample_BrakeControl.cpp接口,此示例对机器人进行抱闸和松闸操作
 *	@brief:
 *	@param ：无
 *	@return : 无
 */
void UserCase_BrakeControl()
{
    int nCurFSM = 0;
    for (int ElapsedTime = 0; ElapsedTime < 15; ElapsedTime += 1) // 每次循环大约1秒
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        if (nCurFSM == 24)
        {
            break;
        }
        std::cout << "机器人当前不在去使能状态，请置到去使能状态" << std::endl;
        sleep(1);
    }
    if (nCurFSM == 24)
    {
        CloseBrake(); // 对机器人部分的关节进行松闸并读取松抱闸的状态
        sleep(5);     // 这里延时为了展示机器人正处于松闸状态，其中的一些关节被松闸
        OpenBrake();  // 读取松闸的状态并对抱闸的关节进行抱闸
    }
    else
    {
        std::cout << "机器人不处于去使能状态，不能进行松抱闸操作" << std::endl;
    }
}

/** Sample_Grap.cpp接口
 *	@brief:
 *	@param ：无
 *	@return : 无
 */
void UserCase_Grap()
{
    GrapControl();
    GrpFreeDriver();
}

/**
 *	@brief: Sample_Script.cpp接口，这是一个控制脚本的示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_Script()
{
    StartScript();
    std::cout << "脚本已经开始运行" << std::endl;
    sleep(3); // 运行3秒后暂停脚本,保证机器人运行起来
    RunScript();
    sleep(3); // 运行3秒后暂停脚本,保证机器人运行起来
    StopScript();
    std::cout << "脚本已经停止" << std::endl;
    RunFunc();
    std::cout << "运行插件" << std::endl;
    RunApp();
}

/**
 *	@brief: Sample_BoxControl.cpp接口，这是一个电箱输入输出值控制和监控的示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_ControlBoxIO()
{
    int nRet = -1;

    BoxInfo(); // 显示当前电箱的信息。

    SetDigitalOutput("DO", 2, 1); // 设置数字量输出
    std::cout << "设置DO2的值为1"<<std::endl;
    SetDigitalOutput("CO", 7, 1);
     std::cout << "设置CO7的值为1"<<std::endl;
    string name = "DO";
    int nBit = 2;
    std::cout << "读取到"<<name << nBit << "的值为" << ReadDigitalIO(name, nBit) << std::endl; // 显示数字量输出
    std::cout << "读取到EDI0的值为" << ReadDigitalIO("EDI", 0)<<std::endl;

    SetAO(0, 2, 5.0); // 将通道0设置为电流模式，电流为5.0A
    std::cout << "设置AO0为电流模式，电流值为5.0A"<<std::endl;
    SetAO(1, 1);      // 将通道0设置为电压模式
    std::cout << "设置AO1为电压模式"<<std::endl;

    name = "AO";
    int nMode = -1;
    std::cout << ReadAIO(name, 0, nMode) << std::endl; // 读取AO通道0的值，读取时nMode有效,读取后更新模式
    std::cout << "模拟量模式为" << nMode << std::endl;
    std::cout << ReadAIO("AI", 1, nMode) << std::endl; // 读取AI时nMode无效

    UseBTN(); // 运行BTN
}

/**
 *	@brief: Sameple_BoxID.cpp 启动接口，多线程同时连接两个电箱并设置其各自的IO
 *	@param strIP1 ：电箱1的IP地址
 *	@param strIP2 ：电箱2的IP地址
 *	@return : 无
 */
void UserCase_BoxIDSample(string strIP1, string strIP2)
{
    // 这里特别说明，如果还是使用默认的ID，则后面控制会出现错误，所以需要改为和BoxID1不一样的ID，范围为[0~5]&&!BoxID1
    SampleBoxID sampleBoxID(strIP1, strIP2);
    sampleBoxID.ConfigIpBoxId();
    std::thread tSendCmdToBoxID0([&sampleBoxID]()
                                 { sampleBoxID.SendCmdInThreadA(); });
    std::thread tSendCmdToBoxID1([&sampleBoxID]()
                                 { sampleBoxID.SendCmdInThreadB(); });
    tSendCmdToBoxID0.join();
    tSendCmdToBoxID1.join();
    std::cout << "电箱ID示例运行结束" << std::endl;
}

/**
 *	@brief: Sample_SysState.cpp接口，这是一个读取系统参数和设置的用法示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_SysStateShow()
{
    std::cout << "---显示初始化系统参数---" << std::endl;
    ShowInitSysState(); // 初始状态显示
    std::cout << "---设置系统参数---" << std::endl;
    SetSystemParams(); // 系统参数配置
    std::cout << "---显示配置参数后系统状态---" << std::endl;
    GetSystemParams(); // 系统参数读取
    std::cout << "---读取错误信息并显示---" << std::endl;
    ShowErrorInfo(); // 显示错误信息
}

/**
 *	@brief: Sample_PointInfo.cpp接口
 *	@param ：无
 *	@return : 无
 */
void UserCase_PonitInfo()
{
    ReadWaypointInfo(); // 读取路点信息
    ReadPointInfo();    // 读取点位信息
}

/**
 *	@brief: Sample_RobotState.cpp接口，这是一个读取机器人运动状态接口的用法示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_RbtStateShow()
{
    int nRunMode = -1;
    std::cout << "请输入你想要读取的信息（1为读取实时状态，2为读取差分状态，其余为退出）：";
    std::cin >> nRunMode; // 等待用户的输入值
    if (nRunMode == 1)
    {
        ShowRealTimeMotionState();
    }
    else if (nRunMode == 2)
    {
        MotionDiffState();
    }
}

/**
 *	@brief: CoordivateTran.cpp接口，这是一个坐标变换的工具用法示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_CoordTran()
{
    std::cout << "---四元素和欧拉角互相转换---" << std::endl;
    TranQuaternionRPY();
    std::cout << "---机器人正逆解算---" << std::endl;
    GetInverseForwardKin();
    std::cout << "---用户坐标系和关节坐标系与基座坐标系之间转换---" << std::endl;
    BaseUcsTcpTran();
    std::cout << "---点位运算求解---" << std::endl;
    PointTran();
}

/**
 *	@brief: ConfigTcpUcs.cpp接口，这是一个配置TCP和UCS的一个示例，包含了正确配置和错误配置的说明
 *	@param ：无
 *	@return : 无
 */
void UserCase_ConfigTcpUcs()
{
    ConfigTcp4Point();      // 四点配置TCP
    ConfigTcp3Point();      // 三点配置TCP
    CalUcsLine();           // 线配置UCS
    CalUcsPlane();          // 面配置UCS
    CalUcs3PointFeatures(); // 轨迹特征配置UCS
}

/**
 *	@brief: Sample_ForceControl.cpp接口，这是一个恒力控/越障+MoveL运动的示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_ForceControl()
{
    int ControlStrategy = 0; // 设置力控策略为恒力模式 0:恒力模式 2：越障模式
    if (InitControlStrategy(ControlStrategy))
    {
        InitForceControl();
        if (MoveBegin())
        {
            StarForce();
            ForceMoveL();
            StopForce();
        }
        else
        {
            std::cout << "运行到初始位置超时" << std::endl;
        }
    }
    else
    {
        std::cout << "模式选择错误" << std::endl;
    }
}

/**
 *	@brief: Sample_FreeDrive.cpp接口，此示例展示自由驱动的一般使用方法
 *	@param ：无
 *	@return : 无
 */
void UserCase_FreeDrive()
{
    int UserInput = 0;
    InitFreeDrive();       // 初始化自由驱动参数
    ReadFTMotionFreedom(); // 读取力控自由驱动的末端自由度
    SetForceZero();        // 重新标定力传感器参数
    StarFreeDrive();       // 开启自由驱动
    std::cout << "需要关闭自由驱动？(输入任意值退出)";
    std::cin >> UserInput; // 等待输入结束
    StopFreeDrive();       // 停止自由驱动参数
}

/**
 *	@brief: Move.cpp 启动接口，需处于就绪状态才可下发的运动指令示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_MoveInStandby()
{
    MoveJ(0, 0, 90, 0, 90, 0); //运行到初始位置，保证每次运行效果一致
    // 需处于就绪状态才可下发的运动指令
    JudgeBlendingState();
    MoveRelJ();
    JudgeBlendingState();
    MoveRelL();
    JudgeBlendingState();
    MoveZ();
    JudgeBlendingState();
    MoveS();
    JudgeBlendingState();
}

/**
 *	@brief: Move.cpp 启动接口，路点运动指令使用示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_WayPoint()
{
    std::cout<<"运动到初始位置点"<<std::endl;
    MoveJ(0, 0, 90, 0, 90, 0);
    std::cout<<std::endl;
    WayPointRel();
    WayPointEx();
    WayPoint();
    JudgeBlendingState();//等待运动完成后接着运行其他示例
}

/**
 *	@brief: Move.cpp 启动接口，需要运动到初始点位才可运行指令示例，主要有MoveC和MoveE两个运动指令
 *	@param ：无
 *	@return : 无
 */
void UserCase_MoveInStartPoint()
{
    MoveJ(0, 0, 90, 0, 90, 0); // 运动到一定的关节位置
    JudgeBlendingState();
    std::cout << "运动到初始位置点" << std::endl;
    MoveL(420, 50, 443.5, 180, 0, 180); // 直线运动到起始点
    JudgeBlendingState();
    std::cout << "运行到初始位置点" << std::endl
              << "下面开始MoveC圆周运动" << std::endl;
    MoveC(); // 进行MoveC圆周运动
    JudgeBlendingState();
    std::cout << "MoveC运动完成" << std::endl;

    JudgeBlendingState();
    std::cout << "运动到初始位置点" << std::endl;
    MoveL(420, 0, 445, 180, 0, 180); // 运动到起始点
    JudgeBlendingState();
    std::cout << "运行到初始位置点" << std::endl
              << "下面开始MoveE椭圆运动" << std::endl;
    // MoveE运动
    MoveE();
    JudgeBlendingState();
    std::cout << "MoveE运动完成" << std::endl;
}

/**
 *	@brief: Move.cpp 启动接口，直线到圆弧运动示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_MoveCircle()
{
    MoveJ(0, 0, 90, 0, 90, 0); // 运动到一定的关节位置
    JudgeBlendingState();
    std::cout << "运动到初始位置点" << std::endl;
    MoveL(420, 50, 443.5, 180, 0, 180); // 直线运动到起始点
    JudgeBlendingState();
    std::cout << "运行到初始位置点" << std::endl
              << "下面开始MoveC圆周运动" << std::endl;
    MoveC(); // 进行MoveC圆周运动
    JudgeBlendingState();
    std::cout << "MoveC运动完成" << std::endl;

    MoveJ(0, 0, 90, 0, 90, 0); // 运动到一定的关节位置
    JudgeBlendingState();
    std::cout << "下面开始WayPoint2运动" << std::endl;
    WayPoint2(); // 进行圆周运动
    JudgeBlendingState();
    std::cout << "WayPoint2运动完成" << std::endl;
}

/** Sample_MovePath.cpp接口,此示例运行轨迹，对关节点位和空间点位推送轨迹并使用MovePathJ和MovePathL进行运行
 *	@brief:
 *	@param ：无
 *	@return : 无
 */
void UserCase_MovePath()
{
    string sPathJName = "Path_TestJ";
    string sPathLName = "Path_TestL";

    std::cout << "-----关节点位轨迹-----" << std::endl;
    std::cout << "添加Path_TestJ轨迹" << std::endl;
    AddPathJ(sPathJName);

    std::cout << "以关节运行Path_TestJ轨迹" << std::endl;
    MoveToPathBegin(sPathJName, "J");
    MovePathJ(sPathJName);

    std::cout << "以空间运行Path_TestJ轨迹" << std::endl;
    MoveToPathBegin(sPathJName, "J");
    MovePathL(sPathJName);

    std::cout << std::endl;

    std::cout << "-----空间点位轨迹-----" << std::endl;
    std::cout << "添加Path_TestL轨迹" << std::endl;
    AddPathL(sPathLName);

    std::cout << "以关节运行Path_TestL轨迹" << std::endl;
    MoveToPathBegin(sPathLName, "L");
    MovePathJ(sPathLName);

    std::cout << "以空间运行Path_TestL轨迹" << std::endl;
    MoveToPathBegin(sPathLName, "L");
    MovePathL(sPathLName);
    std::cout << std::endl;

    std::cout << "-----读取、重命名、删除轨迹-----" << std::endl;
    ReadPath(sPathLName);                   // 读取轨迹Path_TestL是否存在
    UpdatePathName(sPathJName, "Path_J"); // 将Path_TestJL更改名称为Path_JL
    DeletePath(sPathLName);                 // 删除轨迹Path_TestJ
}

/**
 *	@brief: MovepathJ.cpp 启动接口，关节坐标系轨迹运动示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_MovepathJ()
{
    MoveJ(0, 0, 90, 0, 90, 0);   // 运动到初始位置
    JudgeBlendingState();
    InitPathJ();
    PushMoveJ();
    EndPush();
    JudgeState();
    MovePathJ();
}

/**
 *	@brief: MovepathL.cpp 启动接口，空间坐标系轨迹运动示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_MovePathL()
{
    JudgeBlendingState();
    HRIF_MoveJ(0, 0, 420, 5, 445, 180, 1, 180, 0, 0, 90, 0, 90, 0, "TCP", "Base",
               50, 100, 50, 0, 0, 0, 0, "0"); // 运动到初始位置
    JudgeBlendingState();
    InitPathL();
    PushMoveL();
    EndPushL();
    JudgeStateL();
    MovePathL();
}

/**
 *	@brief: Sample_Servo.cpp 启动接口，ServoJ示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_ServoJ()
{
    int nRet = -1;
    MoveJ(0.000038, 0.000038, 89.999962, 0.000038, 89.999962, 0.000038); // 运动到初始位置
    JudgeBlendingState();
    Init_Servo();
    sleep(0.5);
    ServoJ();
    JudgeBlendingState();
    cout << "success" << endl; // 可通过此输出验证
}

/**
 *	@brief: Sample_Servo.cpp 启动接口，ServoP示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_ServoP()
{
    int nRet = -1;
    MoveJ(90, 0, 131, 0, 50, 0); // 运动到初始位置
    JudgeBlendingState();
    Init_Servo();
    sleep(0.5);
    ServoP();
    JudgeBlendingState();
    cout << "success" << endl; // 可通过此输出验证
}

/**
 *	@brief: Sample_Trace.cpp接口，这是一个位置跟随传送带运动的示例
 *	@param ：无
 *	@return : 无
 */
void UserCase_Trace()
{
    MoveToTraceBegin(); // 运动到起始点
    JudgeBlendingState();
    InitPosTraceParams();    // 初始化最大速度和PID参数
    SetPoseTraceTargetPos(); // 设置位置跟随的目标位置
    StarPosTrace();          // 开启位置跟随
    MoveTrace();             // 下发目标位置
    StopPosTrace();          // 关闭位置跟随
    /*相对跟踪*/

}

/** Sample_ConnectToModBus.cpp接口,此示例使用
 *	@brief:
 *	@param ：无
 *	@return : 无
 */
void UserCase_ConnectToModBus()
{
    WriteToRegisters();
    ReadFromRegisters();
}

/**
 *	@brief: portdata.cpp 启动接口，10006端口解析Json数据示例
 *	@param strIP ：需要连接的设备的IP，默认为本机
 *	@return : 无
 */
void ParsePortJsondataDemo(const string &strIP)
{
    int sockfd = createsocket(strIP);                        // 创建socket
    string Jsondata;                                         // 存放接收的Json数据
    thread tReceiveJsonData(receive, sockfd, ref(Jsondata)); // 线程实时获取Json数据
    tReceiveJsonData.detach();
    int count = 0;     // 解析Json包的次数，以此模拟时长
    string JsonString; // 存放单次Json包
    // 循环解析获取的每一次包
    while (count <= 20)
    {
        JsonString = splitJsonString(Jsondata, "LTBR"); // 切割出一个完整的socket包
        GetOriginJson(JsonString);                      // 清洗格式
        cJSON *root = cJSON_Parse(JsonString.c_str());  // 将Json数据转换为cJSON格式
        if (root == NULL)
        {
            cout << "格式转换失败，请检查元数据是否存在格式问题" << endl;
            cout << JsonString << endl;
        }
        string output;
        GetJsonVlaue(root, "RobotAuthorization", "AuthorizedTimeLeftMinutes", output); // 获取string类型的值
        cout << "RobotAuthorization: "
             << "AuthorizedTimeLeftMinutes: " << output << endl; // 输出
        vector<string> vecVal;
        GetJsonArrayVlaue(root, "HardLoad", "Slave_temperature", vecVal); // 获取数组类型的值
        cout << "HardLoad: "
             << "Slave_temperature: ";
        for (const auto &str : vecVal)
        {
            std::cout << str << ","; // 输出
        }
        std::cout << std::endl;
        usleep(50000);
        count++;
    }
    close(sockfd);
    cout << Jsondata << endl; // 可通过此输出验证延时性，当前写法不存在延时
}

/**
 *	@brief: portdata.cpp 启动接口，10006端口解析Json数据示例
 *	@param strIP ：需要连接的设备的IP，默认为本机
 *	@return : 无
 */
void ParsePortJson(const string &strIP)
{
    ParsePortJsonData ParsePortData(strIP);

    ParsePortData.CreateSocket(); // 连接到10006端口
    std::thread ShowThread([&ParsePortData]()
                           { ParsePortData.ParseJsonData(); }); // 使用lambda表达式来包装对成员函数的调用

    std::thread dataThread([&ParsePortData]()
                           { ParsePortData.ReceiveData(); }); // 使用lambda表达式来包装对成员函数的调用
    dataThread.join();
    ShowThread.join();
}

void UserCase_CalibrationForce(){
        //标定力
    setBaseInstallingAngle();
    loadIdentify();
    CalibrationForce();
}