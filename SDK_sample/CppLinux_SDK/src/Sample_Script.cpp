#include "HR_Pro.h"
#include "CommonForInterface.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 启动Script运行
 *	@param ：无
 *	@return : 无
 */
void StartScript()
{
    int nRet = -1;
    int nCurFSM = 0;
    // 运行脚本Main函数
    nRet = HRIF_StartScript(0);
    IsSuccess(nRet, "HRIF_StartScript");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        std::cout << "当前状态机状态为"<<nCurFSM << std::endl;
    } while (nCurFSM != 34); // 34为脚本处于运行状态
}

/**
 *	@brief: 停止Script运行，可以按照下面的方法进行
 *	@param ：无
 *	@return : 无
 */
void StopScript()
{
    int nCurFSM = 0;
    // 停止当前正在运行的脚本
    int nRet = HRIF_StopScript(0);
    IsSuccess(nRet, "HRIF_StopScript");
    std::cout <<"停止过程状态机状态变化如下"<<std::endl;
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
        std::cout << nCurFSM<<",";
    } while (nCurFSM != 33); // 33为脚本停止后最终状态，状态机变化为34(脚本运行中)->37(脚本停止中)->38(脚本完全停止)->33(准备就绪)
    std::cout<<std::endl;
}

/**
 *	@brief: 运行Script过程中暂停脚本运行，3秒后继续脚本
 *	@param ：无
 *	@return : 无
 */
void RunScript()
{
    int nRet = -1;
    int nCurFSM = 0;
    // 暂停脚本
    nRet = HRIF_PauseScript(0);
    IsSuccess(nRet, "HRIF_PauseScript");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    } while (nCurFSM != 36); // 36为脚本处于暂停状态
    std::cout << "脚本已经暂停" << std::endl;

    sleep(3); // 停止3秒后继续脚本
    // 继续脚本
    nRet = HRIF_ContinueScript(0);
    IsSuccess(nRet, "HRIF_ContinueScript");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    } while (nCurFSM != 34); // 34为脚本处于运行状态
    std::cout << "脚本已经重新运行" << std::endl;
}

/**
 *	@brief: 运行指定脚本函数的用法
 *	@param ：无
 *	@return : 无
 */
void RunFunc()
{
    std::cout << "运行函数" << std::endl;
    int nCurFSM = 0;
    string strFuncName = "Func_test"; // 指定运行函数 Func_test

    vector<string> param; // 定义函数输入参数
    int nRet = HRIF_RunFunc(0, strFuncName, param);
    IsSuccess(nRet, "HRIF_RunFunc");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    } while (nCurFSM == 34); // 34为脚本处于运行状态
}

/**
 *	@brief: 插件带参数执行，如果没有参数就默认为空
 *	@param ：无
 *	@return : 无
 */
void RunApp()
{
    // 命令名称
    string AppName = "HelloHR";
    // 命令名称
    string nCmdName = "ReadVersion";
    // 参数列表
    string sParams="0,0";
    // 执行插件命令
    int nRet = HRIF_HRAppCmd(0, AppName,nCmdName, sParams);
    IsSuccess(nRet, "HRIF_HRAppCmd");
    
}