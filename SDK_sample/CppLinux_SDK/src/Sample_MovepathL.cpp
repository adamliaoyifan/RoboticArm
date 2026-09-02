#include "CommonForInterface.h"
#include "HR_Pro.h"

#include <iostream>
#include <unistd.h>

using namespace std;

/**
 *	@brief: 初始化直线路径
 *	@param ：无
 *	@return : 无
 */
void InitPathL()
{
    string sPathName = "PathLTest"; // 轨迹名称
    double dVelocity = 100;       // 定义运动速度
    double dAcc = 500;            // 定义运动加速度
    double dJerk = 10000;         // 定义运动加加速度
    string sUcsName = "Base";     // 定义用户坐标
    string sTcpName = "TCP";      // 定义工具
    int nRet = HRIF_InitMovePathL(0, 0, sPathName, dVelocity, dAcc, dJerk, sUcsName, sTcpName); // 初始化空间连续轨迹运动
    IsSuccess(nRet, "HRIF_InitMovePathL");
}

/**
 *	@brief: 推送点位到轨迹
 *	@param ：无
 *	@return : 无
 */
void PushMoveL()
{

    string sPathName = "PathLTest"; // 轨迹名称
    int nMoveType = 1;            // 运动类型-MovePathL
    int nPointsSize = 6;          // 点位数量
    string sPoints = "420,5,445,180,1,180,422,11,445,180,0,180,423,22,445,180,0,180,424,28,445,180,0,180,427,30,445,180,0,180,420,55,445,180,0,180";
    int nRet = HRIF_PushMovePaths(0, 0, sPathName, nMoveType, nPointsSize, sPoints); // 下发空间目标点位
    IsSuccess(nRet, "HRIF_PushMovePaths");
    HRIF_PushMovePathL(0,0,sPathName,426,67,445,180,0,180);
    IsSuccess(nRet, "HRIF_PushMovePathL");
}

/**
 *	@brief: 停止推送
 *	@param ：无
 *	@return : 无
 */
void EndPushL()
{
    string sPathName = "PathLTest";                       // 轨迹名称
    int nRet = HRIF_EndPushPathPoints(0, 0, sPathName); // 下发完成，开始计算轨迹
    IsSuccess(nRet, "HRIF_EndPushPathPoints");
    cout << "正在计算轨迹" << endl;
}

/**
 *	@brief: 判断直线轨迹的状态
 *	@param ：无
 *	@return : 无
 */
void JudgeStateL()
{
    // 轨迹名称
    string sPathName = "PathLTest";

    int nStateJ = 0;     // MovePathJ 的状态
    int nErrorCodeJ = 0; // MovePathJ 的错误码
    int nStateL = 0;     // MovePathL 的状态
    int nErrorCodeL = 0; // MovePathL 的错误码
    int count = 0;
    while (count <= 30)
    {
        HRIF_ReadPathState(0, 0, sPathName, nStateJ, nErrorCodeJ, nStateL, nErrorCodeL); // 读取当前状态机状态及描述
        count++;
        sleep(1);
        //判断是否为所需状态3
        if (nStateL == 3)
        {
            break;
        }
    }
    if (nStateL != 3)
    {
        cout << "状态错误，调用下方接口失败" << endl;
    }
    cout << "该轨迹nStateJ："<<nStateJ ;
    cout <<"，nErrorCodeJ："<< nErrorCodeJ ;
    cout <<"，nStateL："<< nStateL ;
    cout << "，nErrorCodeL："<<nErrorCodeL << endl;
}

/**
 *	@brief: 运动直线轨迹
 *	@param ：无
 *	@return : 无
 */
void MovePathL()
{
    string sPathName = "PathLTest";               // 轨迹名称
    int nRet = HRIF_MovePathL(0, 0, sPathName); // 开始空间连续轨迹运动
    IsSuccess(nRet, "HRIF_InitMovePathL");
}
