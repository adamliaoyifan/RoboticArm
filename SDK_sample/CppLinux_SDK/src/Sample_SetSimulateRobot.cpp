#include "HR_Pro.h"

#include "CommonForInterface.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 这个函数展示设置机器人为模拟机器人状态，作此封装仅为了输出机器人状态，减少主程序的代码量
 *	@param ：无
 *	@return : 放回机器人的模拟状态，0为机器人为真实机器人，1为机器人为模拟机器人
 */
int ReadSimRobot()
{
    int nSimulateRobot = -1;
    // 判断是否为模拟机器人
    int nRet = HRIF_IsSimulateRobot(0, nSimulateRobot);
    IsSuccess(nRet, "HRIF_IsSimulateRobot");
    switch (nSimulateRobot) // 判断机器人的状态并打印
    {
    case 0:
        std::cout << "机器人现在的状态为真实机器人。" << std::endl;
        break;
    case 1:
        std::cout << "机器人现在的状态为模拟机器人。" << std::endl;
        break;
    default:
        std::cout << "没有读取到机器人状态。" << std::endl;
        break;
    }
    return nSimulateRobot;
}


/**
 *	@brief: 这个函数将机器人设置为模拟机器人
 *	@param ：无
 *	@return : 无
 */
void SetSimRobot()
{
    int nRet = -1;
    nRet = HRIF_SetSimulation(0, 0, 1); // 设置机器人为模拟机器人
    IsSuccess(nRet, "HRIF_SetSimulation");
}

/**
 *	@brief: 这个函数将机器人设置为真实机器人
 *	@param ：无
 *	@return : 无
 */
void SetRealRobot()
{
    int nRet = -1;
    nRet = HRIF_SetSimulation(0, 0, 0); // 设置机器人为模拟机器人
    IsSuccess(nRet, "HRIF_SetSimulation");
}