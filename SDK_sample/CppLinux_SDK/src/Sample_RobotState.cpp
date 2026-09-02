#include "CommonForInterface.h"
#include "HR_Pro.h"

#include <iostream>
#include <iomanip>
#include <unistd.h>

/**
 *	@brief: 显示机器人的运动的位置，速度，加速度状态，开始使用关节运动到起始位置，然后沿一段直线运动并显示机器人的运动信息。
 *	@param : 无
 *	@return : 无
 */
void ShowRealTimeMotionState()
{
    int nRet = -1;
    int nCurFSM = 0;       // 当前状态机状态，详细描述见接口说明文档
    string strCurFSM = ""; // 机器人运行状态简要描述
    bool bDone = false;    // 运动完成

    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;                         // 定义空间位置变量
    double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;                      // 定义关节位置变量
    double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0; // 定义工具坐标变量
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0; // 定义用户坐标变量
    nRet = HRIF_ReadActPos(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    IsSuccess(nRet, "HRIF_ReadActPos");
    std::cout << "机器人当前的位置为" << std::endl
              << "空间坐标:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl
              << "关节姿态:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl
              << " TCP坐标:" << dTcp_X << "," << dTcp_Y << "," << dTcp_Z << "," << dTcp_Rx << "," << dTcp_Ry << "," << dTcp_Rz << std::endl
              << "用户坐标:" << dUcs_X << "," << dUcs_Y << "," << dUcs_Z << "," << dUcs_Rx << "," << dUcs_Ry << "," << dUcs_Rz << std::endl;
    nRet = HRIF_MoveJ(0, 0,
                      0, 0, 0, 0, 0, 0,
                      0, 0, 0, 0, 0, 0, "TCP", "Base",
                      20, 100, 30, 1, 0, 0, 0, "0"); // 运动到该关节点的位置，速度20,加速度100。
    IsSuccess(nRet, "HRIF_MoveJ");
    for (size_t i = 0; i < 20; i++)
    {
        HRIF_IsBlendingDone(0, 0, bDone); // 判断路点是否完成
        if (bDone == true)
        {
            break;
        }
        sleep(1);
    }

    nRet = HRIF_MoveJ(0, 0,
                      0, 0, 0, 0, 0, 0,
                      0, 0, 90, 0, 90, 0, "TCP", "Base",
                      20, 100, 30, 1, 0, 0, 0, "0"); // 运动到该关节点的位置，速度20,加速度100。
    IsSuccess(nRet, "HRIF_MoveJ");

    std::cout << std::fixed << std::setprecision(4); // 设置输出格式为保留6为小数
    do
    {
        HRIF_ReadCurFSM(0, 0, nCurFSM, strCurFSM); // 读取机器人状态
        nRet = HRIF_ReadCmdJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "命令关节坐标:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl;

        nRet = HRIF_ReadActJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "现在关节坐标:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl;

        nRet = HRIF_ReadCmdTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        std::cout << "命令 TCP坐标:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl;

        nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        std::cout << "现在 TCP坐标:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl;

        nRet = HRIF_ReadCmdJointVel(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "命令关节速度:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl;

        nRet = HRIF_ReadActJointVel(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "现在关节速度:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl;

        nRet = HRIF_ReadCmdTcpVel(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        std::cout << "命令 TCP速度:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl;

        nRet = HRIF_ReadActTcpVel(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        std::cout << "现在 TCP速度:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl;

        nRet = HRIF_ReadCmdJointCur(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "命令关节电流:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl;

        nRet = HRIF_ReadActJointCur(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "现在关节电流:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl
                  << std::endl;
        usleep(500000);
    } while (nCurFSM == 25);
}

/**
 *	@brief:  显示机器人的运动的当前状态与控制命令的差距，开始使用关节运动到起始位置，然后沿一段直线运动并显示机器人的运动差距信息。
 *                  运动过程中循环显示机器人距离命令的差值
 *	@param : 无
 *	@return : 无
 */
void MotionDiffState()
{
    int nRet = -1;          // 返回值变量
    bool bMoveFlag = false; // 发送运动指令的状态变量，让其只执行一次
    int nCurFSM = 0;        // 状态机的值
    string strCurFSM = "";  // 状态机的描述

    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;                                              // 定义空间位置变量
    double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;                                           // 定义关节位置变量
    double dTarget_X = 0, dTarget_Y = 0, dTarget_Z = 0, dTarget_Rx = 0, dTarget_Ry = 0, dTarget_Rz = 0;    // 定义命令空间位置变量
    double dTarget_J1 = 0, dTarget_J2 = 0, dTarget_J3 = 0, dTarget_J4 = 0, dTarget_J5 = 0, dTarget_J6 = 0; // 定义命令关节位置变量

    double dCmdVel, dActVel; // 末端TCP的命令速度和实时速度

    // 这里准备使用MoveJ，先运动到起始位置
    nRet = HRIF_MoveJ(0, 0,
                      420, 0, 443.5, 180, 0, 180,
                      0, 0, 90, 0, 90, 0, "TCP", "Base",
                      20, 100, 30, 1, 0, 0, 0, "0"); // 运动到该关节点的位置，速度20,加速度100。
    IsSuccess(nRet, "HRIF_MoveJ");
    for (size_t i = 0; i < 20; i++)
    {
        bool bDone = false;
        HRIF_IsBlendingDone(0, 0, bDone); // 判断路点是否完成
        if (bDone == true)
        {
            break;
        }
        sleep(1);
    }
    // 进行MoveL直线运动
    nRet = HRIF_MoveL(0, 0,
                      420, 400, 443.5, 180, 0, 180,
                      0, 0, 0, 0, 0, 0,
                      "TCP", "Base", 50, 100, 30, 0, 0, 0, "1");

    std::cout << std::fixed << std::setprecision(4); // 设置输出格式为保留6为小数
    do
    {
        HRIF_ReadCurFSM(0, 0, nCurFSM, strCurFSM); // 读取机器人状态,如果运动中则不断的显示数据
        nRet = HRIF_ReadCmdJointPos(0, 0, dTarget_J1, dTarget_J2, dTarget_J3, dTarget_J4, dTarget_J5, dTarget_J6);
        nRet = HRIF_ReadActJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "关节坐标差:" << (dTarget_J1 - dJ1) << "," << (dTarget_J2 - dJ2) << "," << (dTarget_J3 - dJ3) << "," << (dTarget_J4 - dJ4) << "," << (dTarget_J5 - dJ5) << "," << (dTarget_J6 - dJ6) << std::endl;

        nRet = HRIF_ReadCmdTcpPos(0, 0, dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);
        nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
        std::cout << "位置坐标差:" << (dTarget_X - dX) << "," << (dTarget_Y - dY) << "," << (dTarget_Z - dZ) << "," << (dTarget_Rx - dRx) << "," << (dTarget_Ry - dRy) << "," << (dTarget_Rz - dRz) << std::endl;

        nRet = HRIF_ReadCmdJointVel(0, 0, dTarget_J1, dTarget_J2, dTarget_J3, dTarget_J4, dTarget_J5, dTarget_J6);
        nRet = HRIF_ReadActJointVel(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
        std::cout << "关节速度差:" << (dTarget_J1 - dJ1) << "," << (dTarget_J2 - dJ2) << "," << (dTarget_J3 - dJ3) << "," << (dTarget_J4 - dJ4) << "," << (dTarget_J5 - dJ5) << "," << (dTarget_J6 - dJ6) << std::endl;

        nRet = HRIF_ReadTcpVelocity(0, 0, dCmdVel, dActVel);
        std::cout << "TCP速度差:" << (dCmdVel - dActVel) << std::endl
                  << std::endl;
        usleep(500000);
    } while (nCurFSM == 25); // 机器人正在运动中
}
