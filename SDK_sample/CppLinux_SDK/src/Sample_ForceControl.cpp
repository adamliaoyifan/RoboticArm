#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 初始化力控模式
 *	@param nStrategy：设置力控策略为恒力模式 0:恒力模式 2：越障模式
 *	@return : 返回值为true则设置模式成功，返回值为false则设置模式失败
 */
bool InitControlStrategy(const int nStrategy)
{
    int nRet = -1;
    bool bRet = true; // 设置模式状态，如果参数错误则返回
    double dMax = 25; // 定义X/Y方向切向力最大值
    double dMin = 10; // 定义X/Y方向切向力最小值
    double dVel = 50; // 定义越障上抬最大速度

    // 这样写的目的是减少参数检查的步骤
    switch (nStrategy)
    {
    case 0:
        nRet = HRIF_SetForceControlStrategy(0, 0, nStrategy);
        IsSuccess(nRet, "HRIF_SetForceControlStrategy");
        break;
    case 2:
        nRet = HRIF_SetForceControlStrategy(0, 0, nStrategy);
        IsSuccess(nRet, "HRIF_SetForceControlStrategy");
        // 设置X/Y方向切向力最大值、最小值和上抬最大速度
        nRet = HRIF_SetTangentForceBounds(0, 0, dMax, dMin, dVel);
        IsSuccess(nRet, "HRIF_SetTangentForceBounds");
        break;
    default:
        bRet = false;
        break;
    }
    return bRet;
}

/**
 *	@brief: 初始化力控的参数
 *	@param：无
 *	@return : 无
 */
void InitForceControl()
{
    int nRet = -1;

    int nMode = 0; // 设置力控坐标系控制方向为Tool坐标系方向 1：Tool 0：当前ucs
    nRet = HRIF_SetForceToolCoordinateMotion(0, 0, nMode);
    IsSuccess(nRet, "HRIF_SetForceToolCoordinateMotion");

    // 设置质量/惯量参数
    double dXMass = 40;
    double dYMass = 40;
    double dZMass = 40;
    double dRxMass = 10;
    double dRyMass = 10;
    double dRzMass = 10;
    nRet = HRIF_SetMassParams(0, 0, dXMass, dYMass, dZMass, dRxMass, dRyMass, dRzMass);
    IsSuccess(nRet, "HRIF_SetMassParams");

    // 设置阻尼参数
    double dXDamp = 800;
    double dYDamp = 800;
    double dZDamp = 800;
    double dRxDamp = 40;
    double dRyDamp = 40;
    double dRzDamp = 40;
    nRet = HRIF_SetDampParams(0, 0, dXDamp, dYDamp, dZDamp, dRxDamp, dRyDamp, dRzDamp);
    IsSuccess(nRet, "HRIF_SetDampParams");

    // 设置刚度参数
    double dXStiff = 1000;
    double dYStiff = 1000;
    double dZStiff = 1000;
    double dRxStiff = 100;
    double dRyStiff = 100;
    double dRzStiff = 100;
    nRet = HRIF_SetStiffParams(0, 0, dXStiff, dYStiff, dZStiff, dRxStiff, dRyStiff, dRzStiff);
    IsSuccess(nRet, "HRIF_SetStiffParams");

    // 设置力控探寻自由度状态 Z方向
    int nX = 0;
    int nY = 0;
    int nZ = 1;
    int nRx = 0;
    int nRy = 0;
    int nRz = 0;
    nRet = HRIF_SetControlFreedom(0, 0, nX, nY, nZ, nRx, nRy, nRz);
    IsSuccess(nRet, "HRIF_SetControlFreedom");

    // 设置力控目标力大小 Z方向 20N
    double dXForce = 0;
    double dYForce = 0;
    double dZForce = 10;
    double dRxForce = 0;
    double dRyForce = 0;
    double dRzForce = 0;
    nRet = HRIF_SetForceControlGoal(0, 0, dXForce, dYForce, dZForce, dRxForce, dRyForce, dRzForce);
    IsSuccess(nRet, "HRIF_SetForceControlGoal");

    double dMaxLinearVelocity = 15; // 力控探寻直线速度
    double dMaxAngularVelocity = 5; // 力控探寻姿态角速度
    // 设置力控探寻最大直线速度及角速度
    nRet = HRIF_SetMaxSearchVelocities(0, 0, dMaxLinearVelocity, dMaxAngularVelocity);
    IsSuccess(nRet, "HRIF_SetMaxSearchVelocities");

    // 设置各自由度（X/Y/Z/RX/RY/RZ）力控探寻最大距离
    double Dis_X = 300;
    double Dis_Y = 300;
    double Dis_Z = 300;
    double Dis_RX = 20;
    double Dis_RY = 20;
    double Dis_RZ = 20;
    nRet = HRIF_SetMaxSearchDistance(0, 0, Dis_X, Dis_Y, Dis_Z, Dis_RX, Dis_RY, Dis_RZ);
    IsSuccess(nRet, "HRIF_SetMaxSearchDistance");

    // 设置恒力控稳定阶段边界
    double Pos_X = 101;
    double Pos_Y = 100;
    double Pos_Z = 100;
    double Pos_RX = 20;
    double Pos_RY = 20;
    double Pos_RZ = 20;
    double Neg_X = -100;
    double Neg_Y = -100;
    double Neg_Z = -100;
    double Neg_RX = -20;
    double Neg_RY = -20;
    double Neg_RZ = -20;
    HRIF_SetSteadyContactDeviationRange(0, 0, Pos_X, Pos_Y, Pos_Z, Pos_RX, Pos_RY, Pos_RZ, Neg_X, Neg_Y, Neg_Z, Neg_RX, Neg_RY, Neg_RZ);
    IsSuccess(nRet, "HRIF_SetSteadyContactDeviationRange");
}

/**
 *	@brief: 移动到力控初始位置
 *	@param：无
 *	@return : 无
 */
bool MoveBegin()
{
    int nRet = -1;
    bool bRet = false;
    string sTcpName = "TCP";  // 定义工具坐标变量
    string sUcsName = "Base"; // 定义用户坐标变量
    double dVelocity = 50;    // 定义运动速度
    double dAcc = 100;        // 定义运动加速度
    double dRadius = 50;      // 定义过渡半径
    int nIsSeek = 0;          // 定义是否使用检测 DI 停止
    int nIOBit = 0;           // 定义检测的 DI 索引
    int nIOState = 0;         // 定义检测的 DI 状态
    string strCmdID = "0";    // 定义路点 ID
    int nIsUseJoint = 0;      // 使用关节角度

    // 运动到探寻起点
    nRet = HRIF_MoveJ(0, 0,
                      420, 0, 445, 180, 0, 180,
                      0, 0, 0, 0, 0, 0,
                      sTcpName, sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_MoveJ");
    for (size_t i = 0; i < 120; i++)
    {
        bool bDone = false;
        // 判断运动是否完成
        nRet = HRIF_IsBlendingDone(0, 0, bDone);
        if (bDone == true)
        {
            break;
            bRet = true;
        }
        sleep(0.5);
    }
}

/**
 *	@brief: 开始力控探寻，需要力传感器的配置否则启动错误
 *	@param：无
 *	@return : 无
 */
void StarForce()
{
    int nRet = -1;
    int nState = 1;// 开启力控探寻
    nRet = HRIF_SetForceControlState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetForceControlState");
    if (nRet == 0)
    {
        for (size_t i = 0; i < 120; i++)
        {
            std::cout << "Start force search." << std::endl; // 如果启动成功，则等待力控探寻完成

            int nState1 = -1; // 0：关闭状态 1：开力控探寻状态 2：力控探寻完成状态 3：力控自由驱动状态
            // 读取当前力控状态
            nRet = HRIF_ReadForceControlState(0, 0, nState1);
            if (nState1 == 2) // 退出循环的条件是力控探寻完成
            {
                break;
            }
            sleep(0.2);
        }
    }
    else
    {
        string tmpstr;                                // 如果启动错误，则查询错误原因
        nRet = HRIF_GetErrorCodeStr(0, nRet, tmpstr); // 查询错误码并输出
        IsSuccess(nRet, "HRIF_GetErrorCodeStr");
        std::cout << "error:" << tmpstr << std::endl;
    }
}

/**
 *	@brief: 在力控模式下进行MoveL运动
 *	@param：无
 *	@return : 无
 */
void ForceMoveL()
{
    int nRet = -1;
    // MoveL路点运动
    nRet = HRIF_WayPoint(0, 0, 1, 420, 0, 445, 180, 0, 180, 0, 0, 0, 0, 0, 0, "TCP", "Base", 60, 500, 30, 0, 0, 0, 0, "ID0");
    IsSuccess(nRet, "HRIF_WayPoint ID0");
    for (size_t i = 0; i < 120; i++)
    {
        bool bDone = false;
        // 判断路点是否完成
        HRIF_IsBlendingDone(0, 0, bDone);
        if (bDone == true)
        {
            break;
        }
        sleep(0.5);
    }
    nRet = HRIF_WayPoint(0, 0, 1, 420, 200, 445, 180, 0, 180, 0, 0, 0, 0, 0, 0, "TCP", "Base", 60, 500, 30, 0, 0, 0, 0, "ID1");
    IsSuccess(nRet, "HRIF_WayPoint ID1");
    for (size_t i = 0; i < 120; i++)
    {
        bool bDone = false;
        // 判断路点是否完成
        HRIF_IsBlendingDone(0, 0, bDone);
        if (bDone == true)
        {
            break;
        }
        sleep(0.5);
    }
}

/**
 *	@brief: 运动结束后停止力控模式
 *	@param：无
 *	@return : 无
 */
void StopForce()
{
    int nState = 0;// 关闭力控
    int nRet = HRIF_SetForceControlState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetForceControlState");
    if (nRet == 0)
    {
        for (size_t i = 0; i < 60; i++)
        {
            int nState1 = -1; // 0：关闭状态 1：开力控探寻状态 2：力控探寻完成状态 3：力控自由驱动状态
            // 读取当前力控状态
            nRet = HRIF_ReadForceControlState(0, 0, nState1);
            std::cout << "当前状态为" << nState1 << std::endl;
            if (nState1 == 0) // 退出循环的条件是关闭状态
            {
                break;
            }
            sleep(0.2);
        }
    }
    else
    {
        string tmpstr;                                // 如果关闭错误，则查询错误原因
        nRet = HRIF_GetErrorCodeStr(0, nRet, tmpstr); // 查询错误码并输出
        IsSuccess(nRet, "HRIF_GetErrorCodeStr");
        std::cout << "error:" << tmpstr << std::endl;
    }
}