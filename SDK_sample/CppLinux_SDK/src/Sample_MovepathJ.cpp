#include "CommonForInterface.h"
#include "HR_Pro.h"

#include <iostream>
#include <unistd.h>

using namespace std;

/**
 *	@brief: 初始化关节轨迹
 *	@param ：无
 *	@return : 无
 */
void InitPathJ()
{
    string sPathName = "PathTest";                                              // 轨迹名称
    double dSpeedRatio = 0.5;                                                  // 速度比
    double dRadius = 2;                                                        // 过渡半径
    int nRet = HRIF_StartPushMovePathJ(0, 0, sPathName, dSpeedRatio, dRadius); // 初始化关节连续轨迹运动
    IsSuccess(nRet, "HRIF_StartPushMovePathJ");
}

/**
 *	@brief: 推送关节轨迹点位
 *	@param ：无
 *	@return : 无
 */
void PushMoveJ()
{
    string sPathName = "PathTest"; // 轨迹名称
    int nMoveType = 0;            // 运动类型-MovePathJ
    int nPointsSize = 6;          // 点位数量
    string sPoints = "0,0,90,0,90,0, 10,20,90,0,90,0, 20,0,90,0,90,0,40,0,90,0,90,0, 60,0,90,0,90,0, 60,20,90,0,90,0";
    int nRet = HRIF_PushMovePaths(0, 0, sPathName, nMoveType, nPointsSize, sPoints); // 下发关节点位
    IsSuccess(nRet, "HRIF_PushMovePaths");
}

/**
 *	@brief: 停止推送关节轨迹点位
 *	@param ：无
 *	@return : 无
 */
void EndPush()
{
    string sPathName = "PathTest";                       // 轨迹名称
    int nRet = HRIF_EndPushPathPoints(0, 0, sPathName); // 下发完成，开始计算轨迹
    IsSuccess(nRet, "HRIF_EndPushPathPoints");
    cout << "正在计算轨迹" << endl;
}

/**
 *	@brief: 判断关节轨迹状态
 *	@param ：无
 *	@return : 无
 */
void JudgeState()
{
    string sPathName = "PathTest";
    int nState = 0;
    int count = 0;

    //判断10秒内轨迹目标点位是否计算完成
    while (count <= 30)
    {
        HRIF_ReadMovePathJState(0, 0, sPathName, nState); // 读取当前状态机状态及描述
        count++;
        sleep(1);
        if (nState == 3) //判断是否为所需状态
        {
            break;
        }
    }
    if (nState != 3)
    {
        cout << "状态错误，调用下方接口失败" << endl;
    }
}

/**
 *	@brief: 运动关节轨迹
 *	@param ：无
 *	@return : 无
 */
void MovePathJ()
{
    string sPathName = "PathTest";               // 轨迹名称
    int nRet = HRIF_MovePathJ(0, 0, sPathName); // 运动轨迹
    IsSuccess(nRet, "HRIF_MovePathJ");
}

/**
 *	@brief: 关节轨迹进行更换名字并删除，展示该功能用法
 *	@param ：无
 *	@return : 无
 */
void PathJ()
{
    string sPathName = "PathTest";               // 轨迹名称
    string sNewPathName = "PathTestNew";               // 轨迹名称
    int nRet = HRIF_UpdateMovePathJName(0,0,sPathName,sNewPathName);
    std::cout<<"更新轨迹成功"<<std::endl;
    nRet = HRIF_DelMovePathJ(0,0,sNewPathName);
    std::cout<<"删除轨迹成功"<<std::endl;
}