#include "HR_Pro.h"
#include "CommonForInterface.h"

#include <iostream>
#include <array>
#include <unistd.h>

/**
 *	@brief: 这个函数展示抱闸的使用，这里对关节2和关节6进行抱闸，需要注意轴ID的填写
 *	@param ：无
 *	@return : 无
 */
void CloseBrake()
{
    int nRet = -1;
    int nCurFSM = 0;
    // 定义需要读取的机器人状态变量
    int nMovingState = 0, nEnableState = 0, nErrorState = 0, nErrorCode = 0, nErrorAxis = 0, nBreaking = 0;
    int nPause = 0, nEmergencyStop = 0, nSafeGuard = 0, nElectrify = 0, nIsConnectToBox = 0, nBlendingDone = 0, nInPos = 0;

    nRet = HRIF_OpenBrake(0, 0, 0); // 松闸（关节1）
    IsSuccess(nRet, "HRIF_OpenBrake,Joint2");//返回值判断的是指令发送状态

    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM); // 读取状态42为开关抱闸中
    } while (nCurFSM == 42);                   //退出循环则说明机器人松闸完成，状态机等待判断的是指令执行完成

    nRet = HRIF_OpenBrake(0, 0, 5); // 松闸（关节6）
    IsSuccess(nRet, "HRIF_OpenBrake,Joint6");  

    // 读取状态,提供了多个读取机器人状态的接口，只有应用场景有别，按需选用,这里用于读取nBreaking的值
    nRet = HRIF_ReadRobotFlags(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode,
                               nErrorAxis, nBreaking, nPause, nBlendingDone);
    if (nBreaking == 1)
    {
        std::cout << "当前处于开关抱闸状态" << std::endl;
    }
    else if (nBreaking == 0)
    {
        std::cout << "当前未开关抱闸状态" << std::endl;
    }
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM); // 读取状态42为开关抱闸中
    } while (nCurFSM == 42);                   //退出循环则说明机器人松闸完成

    std::array<int, 6> stateJs = {0}; // 使用std::array来存储关节状态
    // 读取各关节松/抱闸状态
    nRet = HRIF_ReadBrakeStatus(0, 0, stateJs[0], stateJs[1], stateJs[2], stateJs[3], stateJs[4], stateJs[5]);
    IsSuccess(nRet, "HRIF_ReadBrakeStatus");
    std::cout << "当前关节抱闸状态(0:抱闸,1:松闸)" << std::endl;
    for (int i = 0; i < stateJs.size(); ++i)
    {
        std::cout << "J" << (i + 1) << ":" << stateJs[i];
        if (i < stateJs.size() - 1) // 在非最后一个元素后添加逗号
        {
            std::cout << ",";
        }
    }
    std::cout << std::endl;
}

/**
 *	@brief: 这个函数展示如何将所有的抱闸的关节松闸，当然也可以对指定某个关节进行松闸
 *	@param ：无
 *	@return : 无
 */
void OpenBrake()
{
    int nRet = -1;
    int nCurFSM = 0;
    // 定义需要读取的机器人状态变量
    int nMovingState = 0, nEnableState = 0, nErrorState = 0, nErrorCode = 0, nErrorAxis = 0, nBreaking = 0;
    int nPause = 0, nEmergencyStop = 0, nSafeGuard = 0, nElectrify = 0, nIsConnectToBox = 0, nBlendingDone = 0, nInPos = 0;

    std::array<int, 6> stateJs = {0}; // 使用std::array来存储关节状态
    // 读取各关节松/抱闸状态
    nRet = HRIF_ReadBrakeStatus(0, 0, stateJs[0], stateJs[1], stateJs[2], stateJs[3], stateJs[4], stateJs[5]);
    IsSuccess(nRet, "HRIF_ReadBrakeStatus");
    std::cout << "当前关节抱闸状态(0:抱闸,1:松闸)" << std::endl;
    for (int i = 0; i < stateJs.size(); ++i)
    {
        std::cout << "J" << (i + 1) << ":" << stateJs[i];
        if (i < stateJs.size() - 1) // 在非最后一个元素后添加逗号
        {
            std::cout << ",";
        }
    }
    std::cout << std::endl;
    for (size_t i = 0; i < stateJs.size(); i++)
    {
        if (stateJs[i] == 1)
        {
            nRet = HRIF_CloseBrake(0, 0, i); // 如果抱闸闭合，则打开
            std::cout << "stateJs" << i << std::endl;
            do
            {
                HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM); // 读取状态42为开关抱闸中
            } while (nCurFSM == 42);

            IsSuccess(nRet, "HRIF_OpenBrake");
        }
    }

    std::cout << std::endl;
    // 读取状态,提供了多个读取机器人状态的接口，只有应用场景有别，按需选用,这里用于读取nBreaking的值
    nRet = HRIF_ReadRobotFlags(0, 0, nMovingState, nEnableState, nErrorState, nErrorCode,
                               nErrorAxis, nBreaking, nPause, nBlendingDone);
    if (nBreaking == 1)
    {
        std::cout << "当前处于开关抱闸状态" << std::endl;
    }
    else if (nBreaking == 0)
    {
        std::cout << "当前未开关抱闸状态" << std::endl;
    }
}