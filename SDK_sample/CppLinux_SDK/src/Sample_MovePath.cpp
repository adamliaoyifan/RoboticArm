#include "HR_Pro.h"
#include "CommonForInterface.h"

#include <iostream>
#include <sstream>
#include <unistd.h>


/**
 *	@brief: 添加关节点位类型的轨迹
 *	@param sPathName：添加关节轨迹的名称
 *	@return : 无
 */
void AddPathJ(const string &sPathName)
{
    int nRet = -1;
    int nRawDataType = 0;     // 原始点位类型  0：关节点位 1：空间点位
    double dSpeedRatio = 0.9; // 轨迹运动速度比
    double dRadius = 1;       // 过渡半径
    double dVelocity = 20;    // 轨迹运动速度
    double dAcc = 100;        // 轨迹运动加速度
    double dJerk = 10000;     // 轨迹运动加加速度
    nRet = HRIF_InitPath(0, 0, nRawDataType, sPathName, dSpeedRatio,
                         dRadius, dVelocity, dAcc, dJerk, "Base", "TCP"); // 初始化运动轨迹
    IsSuccess(nRet, "HRIF_InitPath");

    string sPoints = "-0.000002,0.000002,90.000002,0.000000,89.999998,-0.000002,\
                    0.000000,-0.109595,89.890271,0.000000,90.000134,0.000000,\
                    0.164350,-0.157160,89.842564,-0.000000,90.000276,0.164350,\
                    0.199823,-0.272106,89.727067,0.000000,90.000828,0.199823,\
                    0.309504,-0.314608,89.684285,-0.000000,90.001106,0.309504,\
                    0.419384,-0.315158,89.683732,0.000000,90.001110,0.419384,\
                    0.577144,-0.316237,89.682646,0.000000,90.001118,0.577144,\
                    0.596926,-0.464554,89.533034,-0.000000,90.002412,0.596926,\
                    0.728683,-0.474586,89.522897,0.000000,90.002517,0.728683,\
                    0.793393,-0.608074,89.387793,-0.000000,90.004132,0.793393,\
                    0.960643,-0.634289,89.361214,0.000000,90.004496,0.960643,\
                    0.988789,-0.744288,89.249521,-0.000000,90.006191,0.988789,\
                    1.138505,-0.793888,89.199068,0.000000,90.007044,1.138505,\
                    1.182030,-0.919081,89.071479,-0.000000,90.009440,1.182030,\
                    1.304458,-0.953735,89.036100,0.000000,90.010166,1.304458,\
                    1.374852,-1.059161,88.928301,-0.000000,90.012537,1.374852,\
                    1.510173,-1.114821,88.871289,0.000000,90.013890,1.510173,\
                    1.564960,-1.244853,88.737828,-0.000000,90.017319,1.564960,\
                    1.559940,-1.392845,88.585474,0.000000,90.021681,1.559940,\
                    1.718431,-1.433691,88.543338,-0.000000,90.022971,1.718431";
    nRet = HRIF_PushPathPoints(0, 0, sPathName, sPoints); // 向轨迹中批量推送原始点位
    IsSuccess(nRet, "HRIF_PushPathPoints");

    nRet = HRIF_EndPushPathPoints(0, 0, sPathName); // 结束推送
    IsSuccess(nRet, "HRIF_EndPushPathPoints");
}

/**
 *	@brief: 添加空间类型的轨迹
 *	@param sPathName：添加空间轨迹的名称
 *	@return : 无
 */
void AddPathL(const string &sPathName)
{
    int nRet = -1;
    int nRawDataType = 1;     // 原始点位类型  0：关节点位 1：空间点位，此函数中特别需要注意这里
    double dSpeedRatio = 0.8; // 轨迹运动速度比
    double dRadius = 5;       // 过渡半径
    double dVelocity = 20;    // 轨迹运动速度
    double dAcc = 100;        // 轨迹运动加速度
    double dJerk = 10000;     // 轨迹运动加加速度

    nRet = HRIF_InitPath(0, 0, nRawDataType, sPathName, dSpeedRatio,
                         dRadius, dVelocity, dAcc, dJerk, "Base", "TCP"); // 初始化运动轨迹
    IsSuccess(nRet, "HRIF_InitPath");

    string sPoints = "400,300,294.736,-180,-0,180,\
                398,300,299.287,180,0,180,\
                396,298.114,300,180,0,180,\
                394,293.077,300,-180,-0,180,\
                392,287.477,300,-180,-0,180,\
                390,282.677,300,-180,-0,180,\
                388,277.477,300,180,0,180,\
                386,272.277,300,180,0,180,\
                384,267.477,300,180,0,180,\
                382,262.277,300,180,0,180,\
                380,257.077,300,-180,-0,180,\
                378,252.677,300,-180,-0,180,\
                376,247.477,300,180,0,180,\
                374,242.277,300,-180,-0,180,\
                372,237.477,300,180,0,180,\
                370,232.277,300,-180,-0,180,\
                368,227.077,300,180,0,180,\
                366,221.877,300,-180,-0,180,\
                364,217.077,300,180,0,180,\
                362,211.877,300,180,0,180,\
                360,206.677,300,180,0,180,\
                358,201.971,300,180,0,180,\
                356,200.001,300,180,0,180,\
                354,200,294.988,-180,-0,180,\
                352,200,289.788,180,0,180,\
                350,200,284.988,180,0,180,\
                348,200,279.788,-180,0,180,\
                346,200,274.588,-180,-0,180,\
                344,200,269.388,180,0,180,\
                342,200,264.588,-180,-0,180,\
                340,200,259.388,-180,-0,180,\
                338,200,254.188,-180,-0,180,\
                336,200,249.388,-180,-0,180,\
                334,200,244.188,-180,-0,180,\
                332,200,238.988,-180,-0,180,\
                330,200,233.788,-180,-0,180,\
                328,200,228.588,-180,-0,180,\
                326,200,223.788,-180,0,180,\
                324,200,218.588,180,0,180,\
                322,200,213.388,-180,-0,180,\
                320,200,208.588,180,0,180,\
                318,200,203.392,-180,-0,180,\
                316,200,200.122,-180,0,180,\
                314,200,203.416,-180,-0,180,\
                312,200,208.593,-180,-0,180,\
                310,200,213.393,180,0,180,\
                308,200,218.593,180,0,180,\
                306,200,223.793,-180,-0,180,\
                304,200,228.593,180,0,180,\
                302,200,233.793,180,0,180,\
                300,200,238.993,180,0,180,\
                298,200,243.793,180,0,180,\
                296,200,248.993,180,0,180,\
                294,200,254.193,180,0,180,\
                292,200,258.993,-180,-0,180,\
                290,200,264.193,180,0,180,\
                288,200,269.393,180,0,180,\
                286,200,274.193,180,0,180,\
                284,200,279.393,180,0,180,\
                282,200,284.593,-180,-0,180,\
                280,200,289.393,180,0,180,\
                278,200,294.593,180,0,180";

    // 向轨迹中批量推送原始点位
    nRet = HRIF_PushPathPoints(0, 0, sPathName, sPoints);

    nRet = HRIF_EndPushPathPoints(0, 0, sPathName); // 结束推送
    IsSuccess(nRet, "HRIF_EndPushPathPoints");
}

/**
 *	@brief: 通过HRIF_ReadPathInfo读取轨迹信息，提取轨迹首个点位坐标，并运行到该点
 *	@param sPathName：需要运动到起始点的轨迹的名称
 *	@param sMode：轨迹原始点位的模式，L为空间点，J为关节点位
 *	@return : 无
 */
void MoveToPathBegin(const string &sPathName, const string &sMode)
{
    string result = " ";

    // 读取指定名称轨迹的信息
    int nRet = HRIF_ReadPathInfo(0, 0, sPathName, result);
    IsSuccess(nRet, "HRIF_ReadPathInfo");

    std::istringstream resultStream(result); // 将轨迹信息转化为字符串流
    std::string cell;                        // 用于存储结果分割的单元
    std::vector<string> cellresult;          // 将所有结果信息放在一个vector里面
    std::vector<double> datas;               // 用于记录轨迹起始点位关节信息

    while (std::getline(resultStream, cell, ','))
    {
        cellresult.push_back(cell);
    }

    std::vector<std::string> lastSixItems(cellresult.end() - 6, cellresult.end()); // 取最后六项数据
    for (int i = 0; i < 6; i++)
    {
        datas.push_back(std::stod(lastSixItems[i])); // 这里保证输出的是数字，直接转化为double类型
    }
    // 根据轨迹的类型来进行运动到起始坐标点
    if (sMode == "L")
    {
        // 运动到关节起始点
        nRet = HRIF_MoveJ(0, 0,
                          datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
                          0, 0, 0, 0, 0, 0,
                          "TCP", "Base", 50, 100, 50, 0, 0, 0, 0, "0");
        IsSuccess(nRet, "HRIF_MoveJ");
    }
    else if (sMode == "J")
    {
        // 运动到空间起始点
        nRet = HRIF_MoveJ(0, 0, 0, 0, 0, 0, 0, 0,
                          datas[0], datas[1], datas[2], datas[3], datas[4], datas[5],
                          "TCP", "Base", 50, 100, 50, 1, 0, 0, 0, "0");
        IsSuccess(nRet, "HRIF_MoveJ");
    }
    if (nRet == 0)
    {
        bool bDone = false; // 判断路点是否运动完成
        do
        {
            HRIF_IsBlendingDone(0, 0, bDone); // 读取机器人路点运动是否到位
        } while (!bDone);
    }
}

/**
 *	@brief: 如果轨迹存在并计算完成则运行关节轨迹
 *	@param sPathName：需要使用关节运行的轨迹的名称
 *	@return : 无
 */
void MovePathJ(const string &sPathName)
{
    int nRet = -1;
    int nStateJ = 0;     // MovePathJ的状态
    int nErrorCodeJ = 0; // MovePathJ的错误码
    int nStateL = 0;     // MovePathL的状态
    int nErrorCodeL = 0; // MovePathL的错误码
    bool bDone = false;  // 判断运动是否完成
    int count = 0;
    do
    {
        nRet = HRIF_ReadPathState(0, 0, sPathName, nStateJ, nErrorCodeJ, nStateL, nErrorCodeL); // 读取轨迹状态
        count++;
        sleep(0.1);
    } while (nStateJ != 3 && count < 200);
    std::cout << "轨迹状态:J:" << nStateJ << "和L:" << nStateL << std::endl;
    if (nStateJ == 3)
    {
        nRet = HRIF_MovePathJ(0, 0, sPathName); // 运动轨迹
        IsSuccess(nRet, "HRIF_MovePathJ");
        do
        {
            HRIF_IsMotionDone(0, 0, bDone); // 读取机器人运动是否运动完成
        } while (!bDone);
    }
    else
    {
        std::cout << "轨迹错误码为：" << nErrorCodeJ << std::endl; // 显示错误码
    }
}

/**
 *	@brief: 如果轨迹存在并计算完成则运行空间轨迹
 *	@param sPathName：需要直线运行的轨迹的名称
 *	@return : 无
 */
void MovePathL(const string &sPathName)
{
    int nRet = -1;
    int nStateJ = 0;     // MovePathJ的状态
    int nErrorCodeJ = 0; // MovePathJ的错误码
    int nStateL = 0;     // MovePathL的状态
    int nErrorCodeL = 0; // MovePathL的错误码
    int count = 0;
    // nRet = HRIF_ReadPathState(0, 0, sPathName, nStateJ, nErrorCodeJ, nStateL, nErrorCodeL); // 读取轨迹状态
    do
    {
        nRet = HRIF_ReadPathState(0, 0, sPathName, nStateJ, nErrorCodeJ, nStateL, nErrorCodeL); // 读取轨迹状态
        count++;
        sleep(1);
    } while (nStateL != 3 && count < 20); // 等待20秒轨迹的计算
    std::cout << "轨迹状态:J:" << nStateJ << "和L:" << nStateL << std::endl;
    if (nStateL == 3)
    {
        nRet = HRIF_MovePathL(0, 0, sPathName); // 运动轨迹
        IsSuccess(nRet, "HRIF_MovePathL");
        if (nRet == 0)
        {
            double MovePathOverride = 0.2; // 这个速度比会是原来的速度的一个比值,这里的设置会影响到示教器上的速度比
            nRet = HRIF_SetMovePathOverride(0, 0, MovePathOverride);
            IsSuccess(nRet, "HRIF_SetMovePathOverride");
        }
        else
        {
            std::cout << "运行轨迹错误" << std::endl;
        }
    }
    else
    {
        std::cout << "轨迹错误码为：" << nErrorCodeL << std::endl; // 显示错误码
    }
    bool bDone = false; // 判断运动是否完成
    bool bSetOverrideFlag = false;
    double dProcess = 0;
    int nIndex = 0;
    count = 0;
    HRIF_IsMotionDone(0, 0, bDone); // 读取机器人运动是否开始运动
    while (!bDone)
    {
        count++;
        HRIF_IsMotionDone(0, 0, bDone); // 读取机器人运动是否到位
        HRIF_ReadTrackProcess(0, 0, dProcess, nIndex);
        std::cout << "当前运行进度" << dProcess << std::endl;
        sleep(0.5);
        if (count > 20 && !bSetOverrideFlag) // 10秒后设置速度为0.6
        {
            nRet = HRIF_SetMovePathOverride(0, 0, 1);
            IsSuccess(nRet, "HRIF_SetMovePathOverride");
            bSetOverrideFlag = true;
        }
    }
}

/**
 *	@brief: 如果轨迹存在则运行轨迹
 *	@param sPathName：要查看的轨迹的名称
 *	@return : true则说明轨迹列表里有此轨迹，false则说明没有该轨迹
 */
bool ReadPath(const string &sPathName)
{
    string result = " ";                        // 轨迹列表
    int nRet = HRIF_ReadPathList(0, 0, result); // 读取轨迹列表
    IsSuccess(nRet, "HRIF_ReadPathList");
    std::cout << "轨迹列表为：" << result << std::endl; // 显示轨迹列表
    if (result.find(sPathName) != -1)
    {
        std::cout << "轨迹" << sPathName << "存在" << std::endl;
        return true;
    }
    else
    {
        std::cout << "轨迹" << sPathName << "不存在" << std::endl;
        return false;
    }
}

/**
 *	@brief: 如果轨迹存在则更改轨迹名称，这个为可选功能
 *	@param sPathName：轨迹更换前的名称
 *	@param sPathNewName：轨迹更换后的名称
 *	@return : 无
 */
void UpdatePathName(const string &sPathName, const string &sPathNewName)
{
    if (ReadPath(sPathName))
    {
        if (!ReadPath(sPathNewName))
        {
            int nRet = HRIF_UpdatePathName(0, 0, sPathName, sPathNewName); // 更新轨迹名称
            IsSuccess(nRet, "HRIF_UpdatePathName");
            if (nRet == 0)
            {
                std::cout << "轨迹" << sPathName << "更换名称为" << sPathNewName << std::endl;
            }
        }
    }
    else
    {
        std::cout << "轨迹" << sPathName << "更换名称失败" << std::endl;
    }
}

/**
 *	@brief: 如果轨迹存在则删除轨迹，这个为可选功能
 *	@param sPathName：要删除的轨迹的名称
 *	@return : 无
 */
void DeletePath(const string &sPathName)
{
    if (ReadPath(sPathName))
    {
        int nRet = HRIF_DelPath(0, 0, sPathName); //  删除轨迹
        IsSuccess(nRet, "HRIF_DelPath");
        if (nRet == 0)
        {
            std::cout << "轨迹" << sPathName << "删除成功" << std::endl;
        }
    }
    else
    {
        std::cout << "轨迹" << sPathName << "删除失败" << std::endl;
    }
}
