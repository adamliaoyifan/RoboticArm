#include "HR_Pro.h"

#include "CommonForInterface.h"
#include <iostream>
#include <array>
#include <unistd.h>

/**
 *	@brief: 读取当前路点ID的使用方法，刚发送路点运动时，路点ID不能读取，路点运动结束后，路点ID才是对应的ID
 *	@param ：无
 *	@return : 无
 */
void ReadWaypointInfo()
{
    int nRet = -1;
    int nCurFSM = 0;   //状态机变量
    string strCurWaypointID;//当前路点ID变量
    nRet = HRIF_WayPoint(0, 0, 0,
                         420, 0, 445, 180, 0, 180,
                         0, 0, 90, 0, 90, 0,            // 路点运动到当前关节位置
                         "TCP", "Base", 60, 100, 0,
                         1, 0, 0, 0, "WayPoint_Test");
    IsSuccess(nRet, "HRIF_WayPoint");
    //读取当前路点ID
    nRet = HRIF_ReadCurWaypointID(0, 0, strCurWaypointID);
    std::cout<<"这个错误说明HRIF_ReadCurWaypointID只能在路点运动过程中执行有效；";
    IsSuccess(nRet, "WayPoint Running,HRIF_ReadCurWaypointID");
    std::cout << "当前路点ID为" << strCurWaypointID << std::endl; // 这里显示的是WayPoint_Test，说明路点ID运行
    bool bDone = false;
    while (!bDone)
    {
        HRIF_IsBlendingDone(0, 0, bDone); // 读取机器人路点运动是否到位
        sleep(1);                         // 这个延时为保证机器人读取是否运动完成的状态
    }
    // 再次读取当前路点ID
    nRet = HRIF_ReadCurWaypointID(0, 0, strCurWaypointID); // 这里返回39502，说明运行结束后读取点位ID无效
    if (nRet == 39502)
    {
        std::cout<<"WayPoint结束时,HRIF_ReadCurWaypointID运行不成功"<<std::endl;
    }
}

/**
 *	@brief: 读取点位信息，通过查找点位列表获取点位名称，显示第一个点的点位信息
 *	@param ：无
 *	@return : 无
 */
void ReadPointInfo()
{
    vector<string> PointList;                       // 定义保存点位名称列表的变量
    int nRet = HRIF_ReadPointList(0, 0, PointList); // 读取点位名称列表
    IsSuccess(nRet, "HRIF_ReadPointList");
    if (!PointList.empty())
    {
        for (size_t i = 0; i < PointList.size(); i++)
        {
            std::cout << PointList[i];
            if (i < PointList.size() - 1) // 在非最后一个元素后添加逗号
            {
                std::cout << ",";
            }
        }
        std::cout << std::endl;

        double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;                      // 定义关节位置变量
        double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;                         // 定义空间位置变量
        double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0; // 定义工具坐标变量
        double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0; // 定义用户坐标变量
        for (size_t i = 0; i < PointList.size(); i++)
        {
            nRet = HRIF_ReadPointByName(0, 0, PointList[i], dX, dY, dZ, dRx, dRy, dRz,
                                        dJ1, dJ2, dJ3, dJ4, dJ5, dJ6,
                                        dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz,
                                        dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
            IsSuccess(nRet, "HRIF_ReadPointByName");
            std::cout << "点位" << i + 1 << "的信息如下" << std::endl
                      << "空间坐标:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl
                      << "关节姿态:" << dJ1 << "," << dJ2 << "," << dJ3 << "," << dJ4 << "," << dJ5 << "," << dJ6 << std::endl
                      << " TCP坐标:" << dTcp_X << "," << dTcp_Y << "," << dTcp_Z << "," << dTcp_Rx << "," << dTcp_Ry << "," << dTcp_Rz << std::endl
                      << "用户坐标:" << dUcs_X << "," << dUcs_Y << "," << dUcs_Z << "," << dUcs_Rx << "," << dUcs_Ry << "," << dUcs_Rz << std::endl;
        }
    }
    else
    {
        std::cout << "当前点位列表没有点位" << std::endl;
    }
}
