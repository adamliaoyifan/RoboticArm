#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 初始化设置自由驱动
 *	@param : 无
 *	@return : 无
 */
void InitFreeDrive()
{
    int nRet = -1;
    int nX = 0, nY = 0, nZ = 1, nRx = 0, nRy = 0, nRz = 0; // 定义力控自由驱动自由度状态，这里把z设为1代表只有z方向可以进行自由驱动
    // 设置力控自由驱动自由度状态
    nRet = HRIF_SetFreeDriveMotionFreedom(0, 0, nX, nY, nZ, nRx, nRy, nRz);
    IsSuccess(nRet, "HRIF_SetFreeDriveMotionFreedom");

    double dMaxLinearVelocity = 100;  // 定义自由驱动最大直线速度
    double dMaxAngularVelocity = 30; // 定义自由驱动最大角速度
    // 设置力控自由驱动最大直线速度及姿态角速度
    nRet = HRIF_SetMaxFreeDriveVel(0, 0, dMaxLinearVelocity, dMaxAngularVelocity);
    IsSuccess(nRet, "HRIF_SetMaxFreeDriveVel");

    double dLinear = 50;  // 定义平移柔顺度
    double dAngular = 50; // 定义旋转柔顺度
    // 设置力控自由驱动平移柔顺度和旋转柔顺度
    nRet = HRIF_SetFTFreeFactor(0, 0, dLinear, dAngular);
    IsSuccess(nRet, "HRIF_SetFTFreeFactor");


    // 设置正方向力最大阈值
    double dMax_X = 500, dMax_Y = 500, dMax_Z = 500;
    double dMax_Rx = 50, dMax_Ry = 50, dMax_Rz = 50;
    // 设置负方向力最大阈值
    double dMin_X = 300, dMin_Y = 300, dMin_Z = 300;
    double dMin_Rx = 30, dMin_Ry = 30, dMin_Rz = 30;
    // 设置力保护限制范围
    nRet = HRIF_SetForceDataLimit(0, 0, dMax_X, dMax_Y, dMax_Z, dMax_Rx, dMax_Ry, dMax_Rz,
                                  dMin_X, dMin_Y, dMin_Z, dMin_Rx, dMin_Ry, dMin_Rz);
    IsSuccess(nRet, "HRIF_SetForceDataLimit");

    // double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0; // 定义力传感器安装位置和方向
    // // 设置力传感器的安装位置和方向
    // nRet = HRIF_SetFreeDrivePositionAndOrientation(0, 0, dX, dY, dZ, dRx, dRy, dRz);
    // IsSuccess(nRet, "HRIF_SetFreeDrivePositionAndOrientation");

    double dForceThreshold=10;  // 定义力阈值
    double dTorqueThreshold=0.4; // 定义力矩阈值
    // 设置力控自由驱动启动阈值（力与力矩）
    nRet = HRIF_SetFTWrenchThresholds(0, 0, dForceThreshold, dTorqueThreshold);
    IsSuccess(nRet, "HRIF_SetFTWrenchThresholds");

    double dForce = 0;            // 定义补偿力大小
    double dX = 0, dY = 0, dZ = 0; // 定义补偿力在基坐标系下的矢量方向
    // 设置FreeDrive模式下的定向补偿力大小和矢量方向[x,y,z]
    nRet = HRIF_SetFreeDriveCompensateForce(0, 0, dForce, dX, dY, dZ);
    IsSuccess(nRet, "HRIF_SetFreeDriveCompensateForce");
}

/**
 *	@brief: 读取力控自由驱动的末端自由度
 *	@param : 无
 *	@return : 无
 */
void ReadFTMotionFreedom()
{
    int nX, nY, nZ, nRx, nRy, nRz;
    // 读取力控自由驱动的末端自由度
    int nRet = HRIF_ReadFTMotionFreedom(0, 0, nX, nY, nZ, nRx, nRy, nRz);
    std::cout << "自由驱动的末端自由度:"
              << nX << "," << nY << "," << nZ << "," << nRx << "," << nRy << "," << nRz
              << std::endl;
}

/**
 *	@brief: 重新标定力传感器数据
 *	@param : 无
 *	@return : 无
 */
void SetForceZero()
{
    int nRet = -1;
    // 对力控清零，在原有数据的基础上重新标定力传感器
    nRet = HRIF_SetForceZero(0, 0);
    sleep(5);
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;
    // 读取标定后力传感器数据
    nRet = HRIF_ReadFTCabData(0, 0, dX, dY, dZ, dRx, dRy, dRz);
    std::cout << "标定后力传感器数据:"
              << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz
              << std::endl;
}

/**
 *	@brief: 开启力控自由驱动
 *	@param : 无
 *	@return : 无
 */
void StarFreeDrive()
{    
    int nCurFSM=0;
    //运动到探寻起点
    int nRet = HRIF_MoveJ(0, 0,
                      420, 0, 445, 180, 0, 180,
                      0, 0, 0, 0, 0, 0,
                      "TCP", "Base", 50, 100, 50, 0, 0, 0, 0, "0");
    IsSuccess(nRet, "HRIF_MoveJ");
    while (true)
    {
        bool bDone = false;
        // 判断运动是否完成
        nRet = HRIF_IsBlendingDone(0, 0, bDone);
        if (bDone == true)
            break;
    }
    nRet = HRIF_SetForceFreeDriveMode(0, 0, 1); // 开启力控自由驱动
    IsSuccess(nRet, "HRIF_SetForceFreeDriveMode");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    } while (nCurFSM != 25); // 25为机器人运动中，开启力控自由驱动后，状态机变为25
}

/**
 *	@brief: 开启力控自由驱动
 *	@param : 无
 *	@return : 无
 */
void StopFreeDrive()
{
    int nCurFSM=0;
    int nRet = HRIF_SetForceFreeDriveMode(0, 0, 0);
    IsSuccess(nRet, "HRIF_SetForceFreeDriveMode");
    do
    {
        HRIF_ReadCurFSMFromCPS(0, 0, nCurFSM);
    } while (nCurFSM != 33); // 关闭力控自由驱动后，状态机最终变为33
}