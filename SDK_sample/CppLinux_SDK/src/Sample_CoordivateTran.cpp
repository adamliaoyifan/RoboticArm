#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>

/**
 *	@brief: 欧拉角和四元素的相互转换使用，将转换结果显示出来
 *	@param ：无
 *	@return :   无
 */
void TranQuaternionRPY()
{
    int nRet = -1;
    // 需要转换的四元素变量
    double dQuaW = 0, dQuaX = 0, dQuaY = 0, dQuaZ = 0;
    // 转换后的欧拉角结果
    double dRx = 30, dRy = 60, dRz = 90;
    // 欧拉角转换为四元素
    nRet = HRIF_RPY2Quaternion(0, 0, dRx, dRy, dRz, dQuaW, dQuaX, dQuaY, dQuaZ);
    IsSuccess(nRet, "HRIF_RPY2Quaternion");
    std::cout << "欧拉角dRx:" << dRx << "，dRy:" << dRy << "，dRz:" << dRz
              << "，转化的四元素为dQuaX:" << dQuaX << "，dQuaY:" << dQuaY << "，dQuaZ:" << dQuaZ << "，dQuaW:" << dQuaW << std::endl;
    dQuaW = dQuaW;
    dQuaX = -dQuaX;
    dQuaY = dQuaY - 1;
    dQuaZ = dQuaZ + 1;
    //四元素转换为欧拉角
    nRet = HRIF_Quaternion2RPY(0, 0, dQuaW, dQuaX, dQuaY, dQuaZ, dRx, dRy, dRz);
    IsSuccess(nRet, "HRIF_Quaternion2RPY");
    std::cout << "四元素为dQuaX:" << dQuaX << "，dQuaY:" << dQuaY << "，dQuaZ:" << dQuaZ << "，dQuaW:" << dQuaW
              << "，转化的欧拉角dRx:" << dRx << "，dRy:" << dRy << "，dRz:" << dRz << std::endl;
}

/**
 *	@brief: 正逆解计算求解，通过关节位置得出机器人末端的空间位置，通过末端的空间位置得出机器人关节位置，与机器人系统相关
 *	@param ：无
 *	@return :   无
 */
void GetInverseForwardKin()
{
    int nRet = -1;
    // 定义工具坐标变量
    string sTcpName = "TCP", sUcsName = "Base";
    double dX = 0, dY = 0, dZ = 0, dRx = 0, dRy = 0, dRz = 0;                         // 定义空间位置变量
    double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;                      // 定义关节位置变量
    double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0; // 定义工具坐标变量
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0; // 定义用户坐标变量
    nRet = HRIF_ReadActPos(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    IsSuccess(nRet, "HRIF_ReadActPos");
    cout.precision(6);
    // 定义转换结果
    double dTargetJ1 = 0, dTargetJ2 = 0, dTargetJ3 = 90, dTargetJ4 = 0, dTargetJ5 = 90, dTargetJ6 = 0;

    // 求逆解
    nRet = HRIF_GetInverseKin(0, 0,
                              dX, dY, dZ, dRx, dRy, dRz,                         // 需要转换的空间位置变量
                              dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, // 工具坐标变量 ，全为零代表计算的空间位置不是基于自定义的TCP坐标系
                              dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, // 用户坐标变量，全为零代表计算的空间位置不是基于用户坐标系
                              0, 0, 0, 0, 0, 0,                                  //参考关节位置变量
                              dTargetJ1, dTargetJ2, dTargetJ3, dTargetJ4, dTargetJ5, dTargetJ6);
    IsSuccess(nRet, "HRIF_GetInverseKin");
    std::cout << "该空间位置逆解后关节位置为" << std::endl
              << "J1:" << dTargetJ1 << "，J2:" << dTargetJ2 << "，J3:" << dTargetJ3
              << "，J4:" << dTargetJ4 << "，J5:" << dTargetJ5 << "，J6:" << dTargetJ6 << std::endl;

    nRet = HRIF_ReadActJointPos(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
    std::cout << "现在关节坐标为" << std::endl
              << "J1:" << dJ1 << "，J2:" << dJ2 << "，J3:" << dJ3
              << "，J4:" << dJ4 << "，J5:" << dJ5 << "，J6:" << dJ6 << std::endl;

    nRet = HRIF_ReadActPos(0, 0, dX, dY, dZ, dRx, dRy, dRz, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    IsSuccess(nRet, "HRIF_ReadActPos");

    // 定义转换后的空间位置结果
    double dTarget_X = 0, dTarget_Y = 0, dTarget_Z = 0, dTarget_Rx = 0, dTarget_Ry = 0, dTarget_Rz = 0;
    // 正解：已知目标关节，求对应的TCP末端的坐标
    nRet = HRIF_GetForwardKin(0, 0,
                              dJ1, dJ2, dJ3, dJ4, dJ5, dJ6, //目标关节位置变量
                              dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz, // 工具坐标变量 ，全为零代表计算的空间位置不是基于自定义的TCP坐标系
                              dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz, // 用户坐标变量，全为零代表计算的空间位置不是基于用户坐标系
                              dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);
    IsSuccess(nRet, "HRIF_GetForwardKin");
    std::cout << "该关节位置正解后空间坐标位置为" << std::endl
              << "X:" << dTarget_X << "，Y:" << dTarget_Y << "，Z:" << dTarget_Z
              << "，Rx:" << dTarget_Rx << "，Ry:" << dTarget_Ry << "，Rz:" << dTarget_Rz << std::endl;
    nRet = HRIF_ReadActTcpPos(0, 0, dX, dY, dZ, dRx, dRy, dRz);
    std::cout << "现在TCP坐标:" << dX << "," << dY << "," << dZ << "," << dRx << "," << dRy << "," << dRz << std::endl;
}

/**
 *	@brief: 基坐标系下的点在用户坐标系和工具坐标系下的位置及其逆过程
 *	@param ：无
 *	@return :   无
 */
void BaseUcsTcpTran()
{
    int nRet = -1;
    // 指定用户坐标系和工具坐标系下的迪卡尔坐标
    double dTarget_X = 0, dTarget_Y = 0, dTarget_Z = 0, dTarget_Rx = 0, dTarget_Ry = 0, dTarget_Rz = 0;
    // 基座坐标转换为用户坐标
    nRet = HRIF_Base2UcsTcp(0, 0,
                            520, 0, 850, 180, 0, 180, // 基座坐标系下的迪卡尔坐标位置
                            0, 0, 0, 0, 0, 0,         // dTarget所对应的工具坐标，为默认TCP
                            200, 200, -20, 0, 0, 0,   //用户坐标系的位置
                            dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);
    IsSuccess(nRet, "HRIF_Base2UcsTcp");
    std::cout << "用户坐标系下的该基座坐标系下点位坐标为" << std::endl
              << "X:" << dTarget_X << "，Y:" << dTarget_Y << "，Z:" << dTarget_Z << "，Rx:" << dTarget_Rx << "，Ry:" << dTarget_Ry << "，Rz:" << dTarget_Rz << std::endl;

    // 用户坐标转换为基座坐标
    nRet = HRIF_UcsTcp2Base(0, 0,
                            520, 0, 850, 180, 0, 180, // 用户坐标系下的迪卡尔坐标位置
                            0, 0, 0, 0, 0, 0,         // dTarget所对应的工具坐标，为默认TCP
                            200, 200, -20, 0, 0, 0,   //用户坐标系的位置
                            dTarget_X, dTarget_Y, dTarget_Z, dTarget_Rx, dTarget_Ry, dTarget_Rz);
    std::cout << "基座坐标系下的该用户坐标系下点位坐标为" << std::endl
              << "X:" << dTarget_X << "，Y:" << dTarget_Y << "，Z:" << dTarget_Z << "，Rx:" << dTarget_Rx << "，Ry:" << dTarget_Ry << "，Rz:" << dTarget_Rz << std::endl;
}

/**
 *	@brief: 这个函数对点的空间坐标进行运算并将结果显示出来，与机器人系统无关
 *	@param ：无
 *	@return :   无
 */
void PointTran()
{
    int nRet = -1;
    // 计算结果
    double dPose3_X = 0, dPose3_Y = 0, dPose3_Z = 0, dPose3_Rx = 0, dPose3_Ry = 0, dPose3_Rz = 0;
    // 计算点位加法结果
    nRet = HRIF_PoseAdd(0, 0,
                        420, 0, 445, 180, 0, 180,  // 定义需要计算的空间坐标1
                        420, 50, 445, 180, 0, 180, // 定义需要计算的空间坐标2
                        dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
    std::cout << "点位加法计算的结果如下" << std::endl
              << "X:" << dPose3_X << "，Y:" << dPose3_Y << "，Z:" << dPose3_Z
              << "，Rx:" << dPose3_Rx << "，Ry:" << dPose3_Ry << "，Rz:" << dPose3_Rz << std::endl;

    // 计算点位减法结果
    nRet = HRIF_PoseSub(0, 0,
                        420, 0, 445, 180, 0, 180,  // 定义需要计算的空间坐标1
                        420, 50, 445, 180, 0, 180, // 定义需要计算的空间坐标2
                        dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
    std::cout << "点位减法计算的结果如下" << std::endl
              << "X:" << dPose3_X << "，Y:" << dPose3_Y << "，Z:" << dPose3_Z
              << "，Rx:" << dPose3_Rx << "，Ry:" << dPose3_Ry << "，Rz:" << dPose3_Rz << std::endl;

    // 计算点位转换结果
    nRet = HRIF_PoseTrans(0, 0,
                          420, 0, 445, 180, 0, 180,  // 定义需要计算的空间坐标1
                          420, 50, 445, 180, 0, 180, // 定义需要计算的空间坐标2
                          dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
    std::cout << "点位转换的结果如下" << std::endl
              << "X:" << dPose3_X << "，Y:" << dPose3_Y << "，Z:" << dPose3_Z
              << "，Rx:" << dPose3_Rx << "，Ry:" << dPose3_Ry << "，Rz:" << dPose3_Rz << std::endl;

    // 计算坐标逆解结果
    nRet = HRIF_PoseInverse(0, 0,
                            420, 0, 445, 180, 0, 180, // 定义需要计算的空间坐标1
                            dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
    std::cout << "坐标点逆解转换的结果如下" << std::endl
              << "X:" << dPose3_X << "，Y:" << dPose3_Y << "，Z:" << dPose3_Z
              << "，Rx:" << dPose3_Rx << "，Ry:" << dPose3_Ry << "，Rz:" << dPose3_Rz << std::endl;

    // 计算结果
    double dDistance = 0, dAngle = 0;
    // 计算两个点的空间和关节距离结果
    nRet = HRIF_PoseDist(0, 0,
                         420, 0, 445, 180, 0, 180,  // 定义需要计算的空间坐标1
                         420, 50, 445, 180, 0, 180, // 定义需要计算的空间坐标2
                         dDistance, dAngle);
    std::cout << "坐标之间的距离为" << dDistance << "，坐标间角度为" << dAngle << std::endl;

    // 空间位置直线插补计算,这个用于取两个点所连直线上的alpha比例的点
    nRet = HRIF_PoseInterpolate(0, 0,
                                420, 0, 445, 180, 0, 180,  // 定义需要计算的空间坐标1
                                420, 50, 445, 180, 0, 180, // 定义需要计算的空间坐标2
                                0.5,                       // dAlpha插补比例
                                dPose3_X, dPose3_Y, dPose3_Z, dPose3_Rx, dPose3_Ry, dPose3_Rz);
    std::cout << "空间位置直线插补计算结果如下" << std::endl
              << "X:" << dPose3_X << "，Y:" << dPose3_Y << "，Z:" << dPose3_Z
              << "，Rx:" << dPose3_Rx << "，Ry:" << dPose3_Ry << "，Rz:" << dPose3_Rz << std::endl;
}
