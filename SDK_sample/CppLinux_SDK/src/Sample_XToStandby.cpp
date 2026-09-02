#include "interface.h"
#include "HR_Pro.h"

#include <iostream>
#include <unistd.h> 
#include <chrono>


enum stateName
{
    ElectricBoxDisconnect = 2,   //与电箱控制板断开
    EmergencyStopHandling = 4,   //急停处理中
    EmergencyStop = 5,           //5 急停
    Blackout_48V = 7,            //7 本体供电已切断
    SafeGuardError = 10,        //10 安全光幕Error
    SafeGuard = 12,             //12 安全光幕
    ControllerDisconnect = 14,   //14 控制器已处理于未初始化状态
    ControllerVersionError = 16, //16 控制器版本过低错误
    EtherCATError = 17,          //17 EtherCAT错误
    RobotOutofSafeSpace = 20,    //20 机器人超出安全空间
    RobotCollisionStop = 21,     //21 机器人安全碰撞停车
    Error = 22,                  //22 机器人错误
    Disable = 24,                //24 机器人去使能
    Moving = 25,                 //25 机器人运动中
    LongJogMoving = 26,          //26 机器人长点动运动中
    FreeDriver = 31,             //31 机器人处于零力示教
    RobotHolding = 32,           //32 机器人暂停
    StandBy = 33,                //33 机器人就绪
    ScriptRunning = 34,          //34 脚本运行中
    ScriptHolding = 36,          //36 脚本暂停
};


/**
 *	@brief: 这个函数读取机器人的状态，根据不同的状态执行不同的操作，使机器人到达准备就绪状态
 *	@param ：无
 *	@return : 无
 */
bool XToStandby()
{
    int nCurFSM = 0;
    string strCurFSM = "";
    bool bLoopFlag = true;                                  //循环标志，假则退出循环
    bool bEmergencyFlag = false;                            //首次进入急停状态标志
    bool bReFlag = false;                                   //任务是否成功标志,真则成功
    auto start = std::chrono::high_resolution_clock::now(); //获取任务的开始时间
    do
    {
        HRIF_ReadCurFSM(0, 0, nCurFSM, strCurFSM); //读取状态
        switch (nCurFSM)
        {
        //软件无法更改的状态或其高于就绪状态,退出循环任务
        case ElectricBoxDisconnect:
        case ControllerVersionError:
        case Moving:
        case FreeDriver:
        case RobotHolding:
        case ScriptRunning:
        case ScriptHolding:
            bLoopFlag = false;
            break;
        case RobotOutofSafeSpace:
            //以下两句调用顺序不可更改
            HRIF_GrpReset(0, 0);//首轮循环不生效，二次生效
            HRIF_MoveToSS(0, 0);//首轮循环生效，二次不生效
            break;
        case EmergencyStop:   //进入急停状态后，尝试一次GrpReset，若仍处于急停状态，说明急停按钮未上旋，出于保护继电器的目的，即退出循环
        case SafeGuardError: //进入安全光幕状态或安全光幕错误状态后，尝试一次GrpReset，若仍处于当前状态，说明CI信号仍未置高，不再作无谓的尝试，退出循环
        case SafeGuard:
            if (bEmergencyFlag == false)
            {
                HRIF_GrpReset(0, 0);
                bEmergencyFlag = true;
            }
            else
            {
                bLoopFlag = false;
            }
            break;
        case EtherCATError:
        case RobotCollisionStop:
        case Error:
            HRIF_GrpReset(0, 0);
            break;
        case Blackout_48V:
            HRIF_Electrify(0);
            break;
        case ControllerDisconnect:
            HRIF_Connect2Controller(0);
            break;
        case Disable:
            HRIF_GrpEnable(0, 0);
            break;
        case StandBy:
            bLoopFlag = false;
            bReFlag = true; //已进入Standby状态，任务成功
            break;
        default:
            break;
        }
        usleep(500000);//500ms
        auto end = std::chrono::high_resolution_clock::now();                               //获取最新时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); //计算任务运行时长
        //任务时长超过30秒则退出循环
        if (duration.count() >= 30000)
        {
            break;
        }
    } while (bLoopFlag);
    return bReFlag;
}
