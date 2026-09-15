#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 运动到传送带起始点
 *	@param：无
 *	@return : 无
 */
void MoveToTraceBegin()
{
    int nRet = -1;
    string sTcpName = "TCP";  // 定义工具坐标变量
    string sUcsName = "Base"; // 定义用户坐标变量
    double dVelocity = 50;    // 定义运动速度
    double dAcc = 100;        // 定义运动加速度
    double dRadius = 50;      // 定义过渡半径
    int nIsSeek = 0;          // 定义是否使用检测 DI 停止
    int nIOBit = 0;           // 定义检测的 DI 索引
    int nIOState = 0;         // 定义检测的 DI 状态
    string strCmdID = "0";    // 定义路点 ID
    int nIsUseJoint = 1;      // 使用关节角度

    // 运动到跟随起点
    nRet = HRIF_MoveJ(0, 0,
                      0, 0, 0, 0, 0, 0,
                      0, 0, 90, 0, 90, 0,
                      sTcpName, sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_MoveJ");
}

/**
 *	@brief: 初始化位置跟随参数，设置跟随最大运动限制和跟随PID参数
 *	@param：无
 *	@return : 无
 */
void InitPosTraceParams()
{
    int nRet = -1;
    double dMaxLineVel = 30; // 设置最大跟随直线速度
    double dMaxOriVel = 20;  // 设置最大跟随关节速度
    nRet = HRIF_SetPoseTrackingMaxMotionLimit(0, 0, dMaxLineVel, dMaxOriVel);
    IsSuccess(nRet, "HRIF_SetPoseTrackingMaxMotionLimit");

    double dPosPID1 = 0;   // 设置直线P参数
    double dPosPID2 = 5;   // 设置直线I参数
    double dPosPID3 = 0.1; // 设置直线D参数
    double dOriPID1 = 0;   // 设置关节P参数
    double dOriPID2 = 5;   // 设置关节I参数
    double dOriPID3 = 0.1; // 设置关节D参数
    nRet = HRIF_SetPoseTrackingPIDParams(0, 0, dPosPID1, dPosPID2, dPosPID3, dOriPID1, dOriPID2, dOriPID3);
    IsSuccess(nRet, "HRIF_SetPoseTrackingPIDParams");
}

/**
 *	@brief: 设置位置跟随的目标位置,以 Z 方向保持 10mm 的距离为例
 *	@param：无
 *	@return : 无
 */
void SetPoseTraceTargetPos()
{
    // 位置跟随的目标位置
    double dX = 0;
    double dY = 0;
    double dZ = 10;
    double dRx = 0;
    double dRy = 0;
    double dRz = 0;
    int nRet = HRIF_SetPoseTrackingTargetPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
    IsSuccess(nRet, "HRIF_SetPoseTrackingTargetPos");
}

/**
 *	@brief: 启动位置跟随
 *	@param：无
 *	@return : 无
 */
void StarPosTrace()
{
    int nState = 1; // 设置状态参数
    int nRet = HRIF_SetPoseTrackingState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetPoseTrackingState To Star");
}

/**
 *	@brief: 停止位置跟随
 *	@param：无
 *	@return : 无
 */
void StopPosTrace()
{
    int nState = 0; // 设置状态参数
    int nRet = HRIF_SetPoseTrackingState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetPoseTrackingState To Stop");
}

/**
 *	@brief: 更新传送带运行的点位，下发目标位置
 *	@param：无
 *	@return : 无
 */
void MoveTrace()
{
    int nRet = -1;
    // 运动点位，这里主要是作MoveJ和MoveL的点位过渡，依旧是跟随起始点
    nRet = HRIF_WayPoint(0, 0, 1,
                         420, 200, 443.5, 180, 0, 180,
                         0, 0, 0, 90, 0, 90,
                         "TCP", "Base", 60, 500, 30, 1, 0, 0, 0, "ID0");
    IsSuccess(nRet, "HRIF_WayPoint ID0");
    // 相对位置目标初值
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;
    // 发送 10 次，示例为与目标 Z 方向距离由贴近到逐渐远离(参考坐标系为 TCP 坐标系)
    for (size_t i = 0; i < 10; i++)
    {
        nRet = HRIF_SetUpdateTrackingPose(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        dZ += 10.0;
        sleep(0.2); // 这里延时的意义是设置发送频率为5,按需求设置；要求比末端传感器刷新频率要小，否则会发送丢帧
    }
    // 发送 100 次，示例为与目标 Z 方向距离保持 10mm。此为在线闭环控制，维持的是运动趋势(趋势指的是速度)，当前的运动趋势，能够维持目标的误差不变，说明是最佳趋势。
    // 因此这段能看到末端维持一个能与目标 Z 方向距离保持不变的速度
    for (size_t i = 0; i < 100; i++)
    {
        nRet = HRIF_SetUpdateTrackingPose(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        sleep(0.2);
    }
    for (size_t i = 0; i < 10; i++)
    {
        nRet = HRIF_SetUpdateTrackingPose(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        dZ += 10.0;
        sleep(0.2);
    }
}

/*----------------------------相对轨迹运动----------------------------*/

/**
 *	@brief: 设置相对跟踪运动参数
 *	@return : 无
 */
void SetTraceParams()
{
    double nState = 1;
    double dDistance = 100;    // 设置跟踪状态和保持的相对距离
    double dAwayVelocity = 50; // 相对跟踪的运动的探寻速度
    double dBackVelocity = 50;
    // 设置相对跟踪运动控制参数并开启相对跟踪运动
    int nRet = HRIF_SetMoveTraceParams(0, 0, nState, dDistance, dAwayVelocity, dBackVelocity);
    IsSuccess(nRet, "HRIF_SetMoveTraceParams");
}

/**
 *	@brief: 设置传送带相对跟踪的初始化参数
 *	@param：无
 *	@return : 无
 */
void InitTraceParams()
{
    double dK = -14;        // 传感器计算参数
    double dB = 135;        // 传感器计算参数
    double dMaxLimit = 130; // 激光传感器检测距离最大值
    double dMinLimit = 65;  // 激光传感器检测距离最小值
    // 设置跟踪状态初始化参数
    int nRet = HRIF_SetMoveTraceInitParams(0, 0, dK, dB, dMaxLimit, dMinLimit);
    IsSuccess(nRet, "HRIF_SetMoveTraceInitParams");
}

/**
 *	@brief: 设置相对跟踪运动时末端的姿态
 *	@param：无
 *	@return : 无
 */
void SetTraceUcs()
{
    // 末端的姿态
    double dX = 0, dY = 0, dZ = 0; // 无意义参数，建议填为0
    double dRx = 180;
    double dRy = 0;
    double dRz = 180;
    // 设置相对跟踪运动时末端的姿态
    int nRet = HRIF_SetMoveTraceUcs(0, 0, dX, dY, dZ, dRx, dRy, dRz);
    IsSuccess(nRet, "HRIF_SetMoveTraceUcs");
}

/**
 *	@brief: 开启传送带相对跟踪
 *	@param：无
 *	@return : 无
 */
void StarTrace()
{
    int nState = 1; // 设置传送带跟踪开启
    // 开启传送带跟踪
    int nRet = HRIF_SetTrackingState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetTrackingState");
}

/**
 *	@brief: 关闭传送带相对跟踪
 *	@param：无
 *	@return : 无
 */
void StopTrace()
{
    int nState = 0; // 设置传送带跟踪关闭
    // 开启传送带跟踪
    int nRet = HRIF_SetTrackingState(0, 0, nState);
    IsSuccess(nRet, "HRIF_SetTrackingState");
}