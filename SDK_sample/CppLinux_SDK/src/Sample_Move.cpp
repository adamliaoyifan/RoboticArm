#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <string.h>

#include <cassert>

using namespace std;

/* CommonMoveInterface \ 运动相关公共接口
 * void JudgeBlendingState()
 * void IsMotionDone()
 * void MoveJ(double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
 * void MoveL(double dX, double dY, double dZ, double dRx, double dRy, double dRz)
 */

/**
 *	@brief: 判断机器人路点运动是否完成
 *	@param ：无
 *	@return : 无
 */
void JudgeBlendingState()
{
    bool bDone = false; // 判断路点是否运动完成
    int count = 0;

    // 判断30秒内机器人是否运动完成
    while (count <= 40)
    {
        HRIF_IsBlendingDone(0, 0, bDone); // 读取机器人路点运动是否到位
        count++;
        sleep(1);
        if (bDone)
        {
            break;
        }
    }
    if (!bDone)
    {
        cout << "状态错误，调用下方接口失败" << endl;
    }
}

/**
 *	@brief: 判断机器人运动是否完成，与路点运动完成判断在力控、跟随等场景中有区别
 *	@param ：无
 *	@return : 无
 */
void IsMotionDone()
{
    bool bDone = false; // 判断运动是否完成
    int count = 0;

    // 判断30秒内机器人运动是否完成
    while (count <= 30)
    {
        HRIF_IsMotionDone(0, 0, bDone); // 读取机器人运动是否到位
        count++;
        sleep(1);
        if (bDone)
        {
            break;
        }
    }
    if (!bDone)
    {
        cout << "状态错误，调用下方接口失败" << endl;
    }
}

/**
 *	@brief: 机器人关节运动
 *	@param ：无
 *	@return : 无
 */
void MoveJ(double dJ1, double dJ2, double dJ3, double dJ4, double dJ5, double dJ6)
{
    // 定义空间目标位置
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_MoveJ(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
                          dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_MoveJ");
}

/**
 *	@brief: 机器人空间直线运动
 *	@param ：无
 *	@return : 无
 */
void MoveL(double dX, double dY, double dZ, double dRx, double dRy, double dRz)
{
    // 定义关节目标位置
    double dJ1 = 0, dJ2 = 0, dJ3 = 90, dJ4 = 0, dJ5 = 90, dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_MoveL(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, sTcpName, sUcsName,
                          dVelocity, dAcc, dRadius, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_MoveL");
}

/* MoveInStandbyInterface \ 处于准备就绪状态下运行接口
 * void MoveRelJ()
 * void MoveRelL()
 * void MoveZ()
 * void MoveE() 需要运动到起始点
 * void MoveS()
 */

/**
 *	@brief: 机器人相对关节运动
 *	@param ：无
 *	@return : 无
 */
void MoveRelJ()
{
    // 定义轴 ID
    int nAxisID = 0;
    // 定义运动方向
    int nDirection = 0;
    // 定义运动距离
    double dDistance = 5;
    // 关节相对运动
    int nRet = HRIF_MoveRelJ(0, 0, nAxisID, nDirection, dDistance);
    IsSuccess(nRet, "HRIF_MoveRelJ");
}

/**
 *	@brief: 机器人相对空间直线运动
 *	@param ：无
 *	@return : 无
 */
void MoveRelL()
{
    // 定义轴 ID
    int nAxisID = 0;
    // 定义运动方向
    int nDirection = 0;
    // 定义运动距离
    double dDistance = 2;
    // 定义运动坐标类型
    int nToolMotion = 0;
    // 空间相对运动
    int nRet = HRIF_MoveRelL(0, 0, nAxisID, nDirection, dDistance, nToolMotion);
    IsSuccess(nRet, "HRIF_MoveRelL");
}

/**
 *	@brief: 机器人Z型轨迹运动
 *	@param ：无
 *	@return : 无
 */
void MoveZ()
{
    // 起始点位置
    double dStartPos_X = 420, dStartPos_Y = 0, dStartPos_Z = 445, dStartPos_Rx = 180, dStartPos_Ry = 0, dStartPos_Rz = 180;
    // 结束点位置
    double dEndPos_X = 420, dEndPos_Y = 100, dEndPos_Z = 445, dEndPos_Rx = 180, dEndPos_Ry = 0, dEndPos_Rz = 180;
    // 确定轨迹平面点位置
    double dPlanePos_X = 470, dPlanePos_Y = 50, dPlanePos_Z = 445, dPlanePos_Rx = 180, dPlanePos_Ry = 0, dPlanePos_Rz = 180;
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 50;
    // 宽度
    double dWidth = 50;
    // 密度
    double dDensity = 10;
    // 使用密度
    int nEnableDensity = 1;
    // 使用平面点
    int nEnablePlane = 1;
    // 是否在转折点等待-不等待
    int nEnableWaiTime = 0;
    // 正向转折点等待时间
    int nPosiTime = 0;
    // 负向转折点等待时间
    int nNegaTime = 0;
    // 定义过渡半径
    double dRadius = 5;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义路点 ID
    string strCmdID = "0";
    // 执行 Z 型轨迹运动
    int nRet = HRIF_MoveZ(0, 0, dStartPos_X, dStartPos_Y, dStartPos_Z, dStartPos_Rx, dStartPos_Ry, dStartPos_Rz,
                          dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry, dEndPos_Rz, dPlanePos_X, dPlanePos_Y,
                          dPlanePos_Z, dPlanePos_Rx, dPlanePos_Ry, dPlanePos_Rz, dVelocity, dAcc, dWidth, dDensity, nEnableDensity,
                          nEnablePlane, nEnableWaiTime, nPosiTime, nNegaTime, dRadius, sTcpName, sUcsName, strCmdID);
    IsSuccess(nRet, "HRIF_MoveZ");
}

/**
 *	@brief: 机器人椭圆运动，此接口除了需要满足处于准备就绪状态下才能运行，还需要运动到初始位置
 *	@param ：无
 *	@return : 无
 */
void MoveE()
{
    // 示教点 1
    double dP1[6] = {420, 0, 445, 180, 0, 180};
    // 示教点 2
    double dP2[6] = {460, 0, 445, 180, 0, 180};
    // 示教点 3
    double dP3[6] = {480, 10, 445, 180, 0, 180};
    // 示教点 4
    double dP4[6] = {460, 20, 445, 180, 0, 180};
    // 示教点 5
    double dP5[6] = {420, 20, 445, 180, 0, 180};
    // 运动模式
    double nOrientMode = 0;
    // 运动类型
    double nMoveType = 1;
    // 弧长
    int dArcLength = 360;
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 5;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义路点 ID
    string strCmdID = "0";
    // 执行椭圆运动
    int nRet = HRIF_MoveE(0, 0, dP1[0], dP1[1], dP1[2], dP1[3], dP1[4], dP1[5],
                          dP2[0], dP2[1], dP2[2], dP2[3], dP2[4], dP2[5],
                          dP3[0], dP3[1], dP3[2], dP3[3], dP3[4], dP3[5],
                          dP4[0], dP4[1], dP4[2], dP4[3], dP4[4], dP4[5],
                          dP5[0], dP5[1], dP5[2], dP5[3], dP5[4], dP5[5],
                          nOrientMode, nMoveType, dArcLength, dVelocity, dAcc, dRadius, sTcpName, sUcsName, strCmdID);
    IsSuccess(nRet, "HRIF_MoveE");
}

/**
 *	@brief: 机器人螺旋运动轨迹运动
 *	@param ：无
 *	@return : 无
 */
void MoveS()
{
    // 定义增量半径
    double dSpiralIncrement = 1;
    // 定义结束半径
    double dSpiralDiameter = 5;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 10;
    // 定义过渡半径
    double dRadius = 50;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行螺旋轨迹运动
    int nRet = HRIF_MoveS(0, 0, dSpiralIncrement, dSpiralDiameter, dVelocity, dAcc, dRadius, sTcpName, sUcsName, strCmdID);
    IsSuccess(nRet, "HRIF_MoveS");
}

/* WayPoint \ 路点运动运行接口
 * void WayPointRel()
 * void WayPoint()
 * void WayPointEx()
 */

/**
 *	@brief: 机器人路点相对运动
 *	@param ：无
 *	@return : 无
 */
void WayPointRel()
{
    // 定义运动类型
    int nType = 1;
    // 定义是否使用点位列表的点位
    int nPointList = 0;
    // 定义空间目标位置
    double dPos_X = 420, dPos_Y = 0, dPos_Z = 445, dPos_Rx = 180, dPos_Ry = 0, dPos_Rz = 180;
    // 定义关节目标位置
    double dPos_J1 = 0, dPos_J2 = 0, dPos_J3 = 90, dPos_J4 = 0, dPos_J5 = 90, dPos_J6 = 0;
    // 定义相对运动类型
    int nrelMoveType = 1;
    // 定义各轴\各方向是否运动
    int nAxisMask_1 = 1;
    int nAxisMask_2 = 1;
    int nAxisMask_3 = 1;
    int nAxisMask_4 = 1;
    int nAxisMask_5 = 1;
    int nAxisMask_6 = 1;
    // 定义运动距离
    double dTarget_1 = 50, dTarget_2 = 30, dTarget_3 = 0, dTarget_4 = -60, dTarget_5 = 0, dTarget_6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 路点相对运动
    int nRet = HRIF_WayPointRel(0, 0, nType, nPointList, dPos_X, dPos_Y, dPos_Z, dPos_Rx, dPos_Ry, dPos_Rz,
                                dPos_J1, dPos_J2, dPos_J3, dPos_J4, dPos_J5, dPos_J6, nrelMoveType, nAxisMask_1, nAxisMask_2, nAxisMask_3, nAxisMask_4, nAxisMask_5, nAxisMask_6, dTarget_1, dTarget_2, dTarget_3, dTarget_4, dTarget_5, dTarget_6, sTcpName, sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_WayPointRel");
    std::cout << "WayPointRel相对空间点位运动" << std::endl
              << "在此点基础上进行叠加值相对运动："
              << dPos_X << "," << dPos_Y << "," << dPos_Z << "," << dPos_Rx << "," << dPos_Ry << "," << dPos_Rz << std::endl
              << "在此基础上叠加量为："
              << dTarget_1 << "," << dTarget_2 << "," << dTarget_3 << "," << dTarget_4 << "," << dTarget_5 << "," << dTarget_6 << std::endl
              << std::endl;
}

/**
 *	@brief: 机器人路点运动
 *	@param ：无
 *	@return : 无
 */
void WayPoint()
{
    // 定义运动类型
    int nMoveType = 0;
    // 定义空间目标位置
    double dX = 10, dY = 0, dZ = 90, dRx = 0, dRy = 90, dRz = 0;
    // 定义关节目标位置
    double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_WayPoint(0, 0, nMoveType, dX, dY, dZ, dRx, dRy, dRz,
                             dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                             sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                             nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_WayPoint");
    std::cout << "WayPoint运动到关节点位"
              << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl
              << "其TCP为:" << sTcpName
              << "其UCS为:" << sUcsName << std::endl
              << std::endl;
}

/**
 *	@brief: 机器人路点运动
 *	@param ：无
 *	@return : 无
 */
void WayPointEx()
{
    // 定义运动类型
    int nMoveType = 1;
    // 定义空间目标位置
    double dX = 420, dY = 200, dZ = 445, dRx = 180, dRy = 0, dRz = 180;
    // 定义关节目标位置
    double dJ1 = 100, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 90, dJ6 = 0;
    // 定义工具坐标变量
    double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0;
    // 定义用户坐标变量
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0;
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义是否使用关节角度
    int nIsUseJoint = 1;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_WayPointEx(0, 0, nMoveType, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz,
                               dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, dVelocity, dAcc, dRadius, nIsUseJoint, nIsSeek, nIOBit,
                               nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_WayPointEx");
    std::cout << "WayPointEx运动到空间点位"
              << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl
              << "其TCP为:"
              << dTcp_X << "," << dTcp_Y << "," << dTcp_Z << "," << dTcp_Rx << "," << dTcp_Ry << "," << dTcp_Rz << std::endl
              << "其UCS为:"
              << dUcs_X << "," << dUcs_Y << "," << dUcs_Z << "," << dUcs_Rx << "," << dUcs_Ry << "," << dUcs_Rz << std::endl
              << std::endl;
}

/* MoveCircleInterface \ 圆周运动接口
 * void MoveC()
 * void WayPoint2()
 */

/**
 *	@brief: 机器人圆弧轨迹运动
 *	@param ：无
 *	@return : 无
 */
void MoveC()
{
    // 圆弧起始点位置
    double dStartPos_X = 420, dStartPos_Y = 50, dStartPos_Z = 443.5, dStartPos_Rx = 180, dStartPos_Ry = 0, dStartPos_Rz = 180;
    // 圆弧经过点位置
    double dAuxPos_X = 470, dAuxPos_Y = 100, dAuxPos_Z = 443.5, dAuxPos_Rx = 180, dAuxPos_Ry = 0, dAuxPos_Rz = 180;
    // 圆弧结束点位置
    double dEndPos_X = 520, dEndPos_Y = 50, dEndPos_Z = 443.5, dEndPos_Rx = 180, dEndPos_Ry = 0, dEndPos_Rz = 180;
    // 是否固定姿态
    int nFixedPosure = 0;
    // 圆弧类型
    int nMoveCType = 0;
    // 整圆圈数
    double dRadLen = 1;
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 50;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_MoveC(0, 0, dStartPos_X, dStartPos_Y, dStartPos_Z, dStartPos_Rx, dStartPos_Ry, dStartPos_Rz,
                          dAuxPos_X, dAuxPos_Y, dAuxPos_Z, dAuxPos_Rx, dAuxPos_Ry, dAuxPos_Rz, dEndPos_X, dEndPos_Y,
                          dEndPos_Z, dEndPos_Rx, dEndPos_Ry, dEndPos_Rz,
                          nFixedPosure, nMoveCType, dRadLen, dVelocity, dAcc, dRadius, sTcpName, sUcsName, strCmdID);
    IsSuccess(nRet, "HRIF_MoveC");
}

/**
 *	@brief: 机器人路点运动，支持直线，关节，圆弧运动
 *	@param ：无
 *	@return : 无
 */
void WayPoint2()
{
    // 定义运动类型，使用直线运动到空间目标点
    int nMoveType = 1;
    // 定义空间目标位置
    double dEndPos_X = 420, dEndPos_Y = 50, dEndPos_Z = 443.5, dEndPos_Rx = 180, dEndPos_Ry = 0, dEndPos_Rz = 180;
    // 定义空间途径位置
    double dAuxPos_X = 420, dAuxPos_Y = 0, dAuxPos_Z = 443.5, dAuxPos_Rx = 180, dAuxPos_Ry = 0, dAuxPos_Rz = 180;
    // 定义关节目标位置
    double dJ1 = 0, dJ2 = 0, dJ3 = 90, dJ4 = 0, dJ5 = 90, dJ6 = 0;
    // 定义工具坐标变量
    string sTcpName = "TCP";
    // 定义用户坐标变量
    string sUcsName = "Base";
    // 定义运动速度
    double dVelocity = 50;
    // 定义运动加速度
    double dAcc = 100;
    // 定义过渡半径
    double dRadius = 0;
    // 定义是否使用关节角度
    int nIsUseJoint = 0;
    // 定义是否使用检测 DI 停止
    int nIsSeek = 0;
    // 定义检测的 DI 索引
    int nIOBit = 0;
    // 定义检测的 DI 状态
    int nIOState = 0;
    // 定义路点 ID
    string strCmdID = "0";
    // 执行路点运动
    int nRet = HRIF_WayPoint2(0, 0, nMoveType, dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry,
                              dEndPos_Rz, dAuxPos_X, dAuxPos_Y, dAuxPos_Z, dAuxPos_Rx, dAuxPos_Ry, dAuxPos_Rz, dJ1, dJ2, dJ3, dJ4,
                              dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                              nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_WayPoint2,ID0");

    nMoveType = 2; // 修改为圆弧运动类型
    // 定义空间目标位置
    dEndPos_X = 520;
    dEndPos_Y = 50;
    dEndPos_Z = 443.5;
    dEndPos_Rx = 180;
    dEndPos_Ry = 0;
    dEndPos_Rz = 180;
    // 定义空间过程位置
    dAuxPos_X = 470;
    dAuxPos_Y = 100;
    dAuxPos_Z = 443.5;
    dAuxPos_Rx = 180;
    dAuxPos_Ry = 0;
    dAuxPos_Rz = 180;

    // 定义路点 ID
    strCmdID = "1";

    // 执行路点运动
    nRet = HRIF_WayPoint2(0, 0, nMoveType, dEndPos_X, dEndPos_Y, dEndPos_Z, dEndPos_Rx, dEndPos_Ry,
                          dEndPos_Rz, dAuxPos_X, dAuxPos_Y, dAuxPos_Z, dAuxPos_Rx, dAuxPos_Ry, dAuxPos_Rz, dJ1, dJ2, dJ3, dJ4,
                          dJ5, dJ6, sTcpName, sUcsName, dVelocity, dAcc, dRadius,
                          nIsUseJoint, nIsSeek, nIOBit, nIOState, strCmdID);
    IsSuccess(nRet, "HRIF_WayPoint2,ID1");
}