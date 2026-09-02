#include "HR_Pro.h"

#include "CommonForInterface.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 这个函数展示控制机器人简单运动控制(暂停、继续、停止)的使用
 *	@param ：无
 *	@return : 无
 */
void GrapControl()
{
    int nRet = -1;
    // 当前取两个路点进行运动
    nRet = HRIF_WayPoint(0, 0, 0,
                         500, 0, 800, 180, 0, 180,
                         0, 0, 90, 0, 90, 0,
                         "TCP", "Base", 60, 100, 0,
                         1, 0, 0, 0, "0");
    IsSuccess(nRet, "HRIF_WayPointID0");
    nRet = HRIF_WayPoint(0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         0, 0, 0, 0, 0, 0,
                         "TCP", "Base", 60, 100, 0,
                         1, 0, 0, 0, "1");
    IsSuccess(nRet, "HRIF_WayPointID1");

    sleep(2); // 进行延时，此延时的目的是使机器人运动起来

    nRet = HRIF_GrpInterrupt(0, 0); // 机器人暂停运动，只有运动过程中发送指令才有效
    IsSuccess(nRet, "HRIF_GrpInterrupt");
    std::cout<<"机器人处于暂停状态"<<std::endl;
    sleep(3); // 此延时为了查看机器人暂停的状态，可以适当延长或者缩短

    nRet = HRIF_GrpContinue(0, 0); // 机器人继续运动，只有暂停状态下发送指令有效
    IsSuccess(nRet, "HRIF_GrpContinue");
    bool bDone = false;
    while (!bDone) // 等待这次运行结束
    {

        nRet = HRIF_IsBlendingDone(0, 0, bDone);
    }
    // 再次进行路点运动
    nRet = HRIF_WayPoint(0, 0, 0,
                         500, 0, 800, 180, 0, 180,
                         0, 0, 90, 0, 90, 0,
                         "TCP", "Base", 60, 100, 0,
                         1, 0, 0, 0, "2");
    IsSuccess(nRet, "HRIF_WayPointID2");

    sleep(3);                  // 延时3秒后停止运动，此3秒可替换为停止运动的条件
                               // 记录机器人当前的状态
    nRet = HRIF_GrpStop(0, 0); // 机器人停止运动
    IsSuccess(nRet, "HRIF_GrpStop");
    int nCurFSM = 0;
    nRet = HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM); // 获取机器人当前的状态
    std::cout << "当前状态机状态为：" << nCurFSM << std::endl;
    // 等待机器人停止运动完成
    do
    {
        nRet = HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM); // 获取机器人当前的状态
    } while (nCurFSM != 33);
    std::cout << "当前状态机状态为：" << nCurFSM << std::endl;
}

/**
 *	@brief: 这个函数展示零力示教的使用
 *	@param ：无
 *	@return : 无
 */
void GrpFreeDriver()
{
    int nCurFSM = 0; // 状态机
    int nRet = -1;
    for (size_t Count = 0; Count < 15; Count++)
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        Count++;
        sleep(1);          // 每秒查看一次机器人的状态
        if (nCurFSM == 33) //机器人处于准备就绪状态则退出循环
        {
            break;
        }
        std::cout << "机器人处于准备就绪状态" << std::endl;
    }
    if (nCurFSM == 33)
    {
        nRet = HRIF_GrpOpenFreeDriver(0, 0);
        IsSuccess(nRet, "HRIF_GrpOpenFreeDriver");
    }
    for (size_t Count = 0; Count < 15; Count++)
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        sleep(1); // 每秒查看一次机器人的状态
        if (nCurFSM == 31)
        {
            break;
        }
        std::cout << "机器人正在开启零力示教" << std::endl;
    }
    if (nCurFSM != 31)
    {
        std::cout << "机器人开启零力示教超时" << std::endl;
    }
    else
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        if (nCurFSM == 31) // 在零力示教模式下
        {
            std::cout << "机器人处于零力示教模式" << std::endl;
            sleep(10); // 延时,这里仅用于说明机器人处于零力示教状态
            HRIF_GrpCloseFreeDriver(0, 0); // 关闭零力示教
        }
        do // 机器人正在关闭零力示教后达到33
        {
            HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
            std::cout << "机器人正在关闭零力示教" << std::endl;
        } while (nCurFSM == 30); // 状态机变化为31-30-33
    }

    HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    if (nCurFSM != 31)
    {
        std::cout << "机器人此时不在零力示教模式" << std::endl;
    }
}
