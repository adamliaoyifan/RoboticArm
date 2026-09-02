#include "HR_Pro.h"
#include "CommonForInterface.h"
#include <iostream>

/**
 *	@brief: 使用四个点位进行配置TCP,对点位的准确性要求比较严苛，建议使用零力示教的方式进行获取点位
 *  @param ：无
 *	@return : 无
 */
void ConfigTcp4Point()
{
    int nRet = -1;
    string sTcpName = "Tcp4Point";
    int quality = -1; // 计算结果质量变量定义，0为点位正常，1为点位异常，2为点位较差
    int errorIndex_P1 = 0, errorIndex_P2 = 0, errorIndex_P3 = 0, errorIndex_P4 = 0;
    double dRetPose_X = 0, dRetPose_Y = 0, dRetPose_Z = 0, dRetPose_Rx = 0, dRetPose_Ry = 0, dRetPose_Rz = 0; // 计算结果变量定义
    double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0;                         // 定义读取到的工具坐标结果变量

    // 获取计算结果
    nRet = HRIF_CalTcp4P(0, 0,
                         420, 0, 443.5, 180, 0, 180,                            // 点位1
                         405.721, 54.109, 405.986, -147.956, -50.146, -124.541, // 点位2
                         380.058, 38.962, 359.734, -51.114, -53.766, -168.598,  // 点位3
                         433.168, -28.523, 423.034, 118.039, 41.505, -133.350,  // 点位4
                         dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz,
                         quality, errorIndex_P1, errorIndex_P2, errorIndex_P3, errorIndex_P4);
    IsSuccess(nRet, "HRIF_CalTcp4P");
    if (quality == 0)
    {
        // 新建指定名称的TCP和值
        nRet = HRIF_ConfigTCP(0, 0, sTcpName, dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
        nRet = HRIF_SetTCPByName(0, 0, sTcpName);
        std::cout << "配置TCP成功，请查看名字为" << sTcpName << "的TCP" << std::endl;
        // 读取工具坐标
        nRet = HRIF_ReadTCPByName(0, 0, sTcpName, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
        std::cout << "名字为" << sTcpName << "的TCP坐标如下" << std::endl
                  << "X:" << dTcp_X << "," << "Y:" << dTcp_Y << "," << "Z:" << dTcp_Z << "," << "Rx:" << dTcp_Rx << "," << "Ry:" << dTcp_Ry << "," << "Rz:" << dTcp_Rz << std::endl;
    }
    else if (quality == 1)
    {
        std::cout << "点位极差，不建议使用，建议重新配置点位" << std::endl
                  << "点位1计算" << ((errorIndex_P1 == 1) ? "异常," : "正常,") << std::endl
                  << "点位2计算" << ((errorIndex_P2 == 1) ? "异常," : "正常,") << std::endl
                  << "点位3计算" << ((errorIndex_P3 == 1) ? "异常," : "正常,") << std::endl
                  << "点位4计算" << ((errorIndex_P4 == 1) ? "异常," : "正常,") << std::endl;
    }
    else if (quality == 2)
    {
        std::cout << "点位异常。" << std::endl
                  << "点位1计算" << ((errorIndex_P1 == 1) ? "异常," : "正常,")
                  << "点位2计算" << ((errorIndex_P2 == 1) ? "异常," : "正常,")
                  << "点位3计算" << ((errorIndex_P3 == 1) ? "异常," : "正常,")
                  << "点位4计算" << ((errorIndex_P4 == 1) ? "异常" : "正常") << std::endl;
    }
}

/**
 *	@brief: 使用三个点位进行配置TCP
 *  @param ：无
 *	@return : 无
 */
void ConfigTcp3Point()
{
    int nRet = -1;
    // 计算结果变量定义，这个结果记录的需要配置的TCP的base坐标系的坐标值
    double dRetPose_X = 0, dRetPose_Y = 0, dRetPose_Z = 0, dRetPose_Rx = 0, dRetPose_Ry = 0, dRetPose_Rz = 0;

    int quality = -1; // 计算结果质量变量定义
    // 配置三点的平面
    nRet = HRIF_CalTcp3P(0, 0,
                         10, 0, 0, 10, 0, 0, // 点位1
                         0, 10, 0, 0, 10, 0, // 点位2
                         0, 10, 0, 0, 10, 0, // 点位3
                         dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz, quality);
    IsSuccess(nRet, "HRIF_CalTcp3P");
    if (nRet == 20031)
    {
        std::cout << "点位计算失败" << std::endl;
    }
    else
    {
        if (quality == 0)
        {
            // 显示成功的坐标系
            std::cout << "使用三点配置的TCP为" << std::endl
                      << "X:" << dRetPose_X << "，Y:" << dRetPose_Y << "，Z:" << dRetPose_Z
                      << "，Rx:" << dRetPose_Rx << "，Ry:" << dRetPose_Ry << "，Rz:" << dRetPose_Rz << std::endl;
            // 设置TCP坐标系，不写入配置文件，其生命周期为设置后到下一次设置新的TCP前或者到系统关机
            HRIF_SetTCP(0, 0, dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
            std::cout << "配置TCP成功" << std::endl;
            // 显示是否设置成功
            //  定义用户坐标变量
            double dTcp_X = 0, dTcp_Y = 0, dTcp_Z = 0, dTcp_Rx = 0, dTcp_Ry = 0, dTcp_Rz = 0;
            // 读取当前的用户坐标
            nRet = HRIF_ReadCurTCP(0, 0, dTcp_X, dTcp_Y, dTcp_Z, dTcp_Rx, dTcp_Ry, dTcp_Rz);
            std::cout << "当前TCP位置为" << std::endl
                      << "X:" << dTcp_X << "，Y:" << dTcp_Y << "，Z:" << dTcp_Z
                      << "，Rx:" << dTcp_Rx << "，Ry:" << dTcp_Ry << "，Rz:" << dTcp_Rz << std::endl;
        }
        else if (quality == 1)
        {
            std::cout << "点位极差，不建议使用，建议重新配置点位" << std::endl;
        }
        else if (quality == 2)
        {
            std::cout << "点位异常。" << std::endl;
        }
    }
}

/**
 *	@brief: 线配置UCS
 *  @param ：无
 *	@return : 无
 */
void CalUcsLine()
{
    int nRet = -1;
    // 下面两个点位是需要定义的两个点，可以替换为任何方式获取点位
    // 第一个点为原点，第一个点指向第二个点的方向为Y方向，Z方向与TCP方向相关X方向根据右手定则确定
    double dPose1_X = 10, dPose1_Y = 0, dPose1_Z = 0, dPose1_Rx = 0, dPose1_Ry = 0, dPose1_Rz = 0;  // 定义需要计算的空间坐标1
    double dPose2_X = 10, dPose2_Y = 10, dPose2_Z = 0, dPose2_Rx = 0, dPose2_Ry = 0, dPose2_Rz = 0; // 定义需要计算的空间坐标2
    // 计算结果变量定义
    double dRetPose_X = 0, dRetPose_Y = 0, dRetPose_Z = 0, dRetPose_Rx = 0, dRetPose_Ry = 0, dRetPose_Rz = 0;
    // 获取计算结果，函数调用后，nRet值为0，dRetPose_X~dRetPose_Rz的值分别为：10,0,0,0,0,0
    nRet = HRIF_CalUcsLine(0, 0,
                           dPose1_X, dPose1_Y, dPose1_Z, dPose1_Rx, dPose1_Ry, dPose1_Rz,
                           dPose2_X, dPose2_Y, dPose2_Z, dPose2_Rx, dPose2_Ry, dPose2_Rz,
                           dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
    std::cout << "使用线配置的用户坐标系为" << std::endl
              << "X:" << dRetPose_X << "，Y:" << dRetPose_Y << "，Z:" << dRetPose_Z
              << "，Rx:" << dRetPose_Rx << "，Ry:" << dRetPose_Ry << "，Rz:" << dRetPose_Rz << std::endl;

    // 设置用户坐标系，不写入配置文件，其声明周期为设置后到下一次设置新的UCS前或者到系统关机
    nRet = HRIF_SetUCS(0, 0,
                       dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
    // 定义用户坐标变量
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0;
    // 读取当前的用户坐标
    nRet = HRIF_ReadCurUCS(0, 0, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    std::cout << "当前用户坐标系为" << std::endl
              << "X:" << dUcs_X << "，Y:" << dUcs_Y << "，Z:" << dUcs_Z
              << "，Rx:" << dUcs_Rx << "，Ry:" << dUcs_Ry << "，Rz:" << dUcs_Rz << std::endl;
}

/**
 *	@brief: 面配置UCS
 *  @param ：无
 *	@return : 无
 */
void CalUcsPlane()
{
    // 下面两个点位是需要定义的三个点的空间位置，使用此三个点确定一个平面，点位获取可以替换为任何方式
    // 第一个点为原点，第一个点指向第二个点的方向为Y方向，Z方向为平面的法线，X方向根据右手定则确定
    double dPose1_X = 5, dPose1_Y = 0, dPose1_Z = 0;  // 定义需要计算的空间坐标1
    double dPose2_X = 5, dPose2_Y = 5, dPose2_Z = 0;  // 定义需要计算的空间坐标2
    double dPose3_X = -5, dPose3_Y = 0, dPose3_Z = 0; // 定义需要计算的空间坐标3
    // 计算UCS结果变量定义
    double dRetPose_X = 0, dRetPose_Y = 0, dRetPose_Z = 0, dRetPose_Rx = 0, dRetPose_Ry = 0, dRetPose_Rz = 0;
    // 获取计算结果，函数调用后，nRet值为0，dRetPose_X~dRetPose_Rz的值分别为：5,0,0,180,-0,180
    int nRet = HRIF_CalUcsPlane(0, 0,
                                dPose1_X, dPose1_Y, dPose1_Z,
                                dPose2_X, dPose2_Y, dPose2_Z,
                                dPose3_X, dPose3_Y, dPose3_Z,
                                dRetPose_X, dRetPose_Y, dRetPose_Z, dRetPose_Rx, dRetPose_Ry, dRetPose_Rz);
    std::cout << "使用面配置的用户坐标系为" << std::endl
              << "X:" << dRetPose_X << "，Y:" << dRetPose_Y << "，Z:" << dRetPose_Z
              << "，Rx:" << dRetPose_Rx << "，Ry:" << dRetPose_Ry << "，Rz:" << dRetPose_Rz << std::endl;
}

/**
 *	@brief: 3点轨迹特征配置UCS，
 *  @param ：无
 *	@return : 无
 */
void CalUcs3PointFeatures()
{
    double dPose1_X = 0, dPose1_Y = 0, dPose1_Z = 0;                                  // 定义需要计算的空间坐标1
    double dPose2_X = 0, dPose2_Y = 0, dPose2_Z = 0;                                  // 定义需要计算的空间坐标2
    double dPose3_X = 0, dPose3_Y = 0, dPose3_Z = 0;                                  // 定义需要计算的空间坐标3
    double dPose4_X = 0, dPose4_Y = 0, dPose4_Z = 0;                                  // 定义需要计算的空间坐标4
    double dPose5_X = 0, dPose5_Y = 0, dPose5_Z = 0;                                  // 定义需要计算的空间坐标5
    double dPose6_X = 0, dPose6_Y = 0, dPose6_Z = 0;                                  // 定义需要计算的空间坐标6
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0; // 计算结果
    // 计算结果
    int nRet = HRIF_PoseDefdFrame(0, 0,
                                  dPose1_X, dPose1_Y, dPose1_Z, dPose2_X, dPose2_Y, dPose2_Z,
                                  dPose3_X, dPose3_Y, dPose3_Z, dPose4_X, dPose4_Y, dPose4_Z,
                                  dPose5_X, dPose5_Y, dPose5_Z, dPose6_X, dPose6_Y, dPose6_Z,
                                  dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    std::cout << "使用轨迹三点特征配置的用户坐标系为" << std::endl
              << "X:" << dUcs_X << "，Y:" << dUcs_Y << "，Z:" << dUcs_Z
              << "，Rx:" << dUcs_Rx << "，Ry:" << dUcs_Ry << "，Rz:" << dUcs_Rz << std::endl;
        // 定义用户坐标名称
    string sUcsName = "PointSDKTest";
    // 新建指定名称的UCS和值
    nRet = HRIF_ConfigUCS(0, 0, sUcsName, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    // 使用名称设置UCS，新建完UCS后不会立刻生效，需要使用以下指令设置，此时在示教器主界面显示该UCS为主要的UCS
    nRet = HRIF_SetUCSByName(0, 0, sUcsName);

    // 读取用户坐标
    nRet = HRIF_ReadUCSByName(0, 0, sUcsName, dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rz);
    std::cout << sUcsName << "用户坐标系为" << std::endl
              << "X:" << dUcs_X << "，Y:" << dUcs_Y << "，Z:" << dUcs_Z
              << "，Rx:" << dUcs_Rx << "，Ry:" << dUcs_Ry << "，Rz:" << dUcs_Rz << std::endl;
}
