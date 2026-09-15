#include "HR_Pro.h"
#include "CommonForInterface.h"

#include <iostream>

/**
 *	@brief: 显示机器人的系统的状态，包括机器人的模型和机器人的版本号以及控制器是否启动和连接
 *	@param : 无
 *	@return : 无
 */
void ShowInitSysState()
{
    int nRet = -1;

    string strModel = ""; // 获取机器人模型
    // 读取机器人机型
    nRet = HRIF_ReadRobotModel(0, strModel);
    std::cout << "机器人型号为：" << strModel << std::endl;

    string strVer = "";        // 获取整体版本号
    int nCPSVersion = 0;       // 获取CPS版本号
    int nCodesysVersion = 0;   // 获取控制器版本号
    int nBoxVerMajor = 0;      // 获取电箱版本
    int nBoxVerMID = 0;        // 获取电箱固件版本
    int nBoxVerMin = 0;        // 获取电箱固件版本
    int nAlgorithmVer = 0;     // 获取算法版本
    int nElfinFirmwareVer = 0; // 获取驱动固件版本
    // 获取版本号
    nRet = HRIF_ReadVersion(0, 0, strVer, nCPSVersion, nCodesysVersion, nBoxVerMajor, nBoxVerMID, nBoxVerMin, nAlgorithmVer, nElfinFirmwareVer);
    std::cout << "机器人整体版本号:" << strVer << std::endl
              << "机器人CPS版本号:" << nCPSVersion << std::endl
              << "机器人控制器版本号:" << nCodesysVersion << std::endl
              << "机器人电箱版本:" << nBoxVerMajor << std::endl
              << "机器人电箱固件版本:" << nBoxVerMID << std::endl // 机器人控制器固件版本
              << "机器人电箱固件版本:" << nBoxVerMin << std::endl
              << "机器人算法版本:" << nAlgorithmVer << std::endl
              << "机器人驱动固件版本:" << nElfinFirmwareVer << std::endl;

    int nStarted = 0;
    // 判断机器人控制器是否启动完成
    nRet = HRIF_IsControllerStarted(0, nStarted);
    if (nRet == 0 && nStarted == 1)
    {
        std::cout << "机器人启动完成" << std::endl;
    }
    // 控制器是否连接
    bool Ret = HRIF_IsConnected(0);
    if (Ret)
    {
        std::cout << "机器人已连接" << std::endl;
    }
}

/**
 *	@brief: 设置机器人的系统的参数;特别注意空间运动范围，另外注意设置的有效范围，如果超出范围则设置不成功
 *                  速度比，负载参数，最大关节速度，最大关节加速度，最大关节加加速度，最大直线速度，最大直线加速度
 *	@param : 无
 *	@return : 无
 */
void SetSystemParams()
{
    int nRet = -1;
    double dOverride = 0.6; // 需要设置的速度比
    // 设置当前速度比
    nRet = HRIF_SetOverride(0, 0, dOverride);
    IsSuccess(nRet, "HRIF_SetOverride");

    // int nState = 1; // 需要设置的Tool运动状态
    // // 设置Tool运动状态
    // nRet = HRIF_SetToolMotion(0, 0, nState);
    // IsSuccess(nRet, "HRIF_SetToolMotion");

    double dMass = 0,dX = 0,dY = 0,dZ = 0;
    // 读取当前负载参数,这里保证运行示例不改变系统设置的参数
    nRet = HRIF_ReadPayload(0, 0, dMass, dX, dY, dZ);

    // 设置当前负载参数
    nRet = HRIF_SetPayload(0, 0, dMass, dX, dY, dZ, 3); // 电箱ID号,机器人ID号,质量,质心X方向偏移,质心Y方向偏移,质心Z方向偏移,更新数据库
    IsSuccess(nRet, "HRIF_SetPayload");

    // 设置最大关节速度；参数为J1,J2,3,J4,J5,J6的最大关节速度
    nRet = HRIF_SetJointMaxVel(0, 0, 180, 180, 180, 180, 180, 180); // 电箱ID号,机器人ID号,J1,J2,3,J4,J5,J6
    IsSuccess(nRet, "HRIF_SetJointMaxVel");

    // 设置最大关节加速度；参数为J1,J2,3,J4,J5,J6的最大关节加速度
    nRet = HRIF_SetJointMaxAcc(0, 0, 360, 360, 360, 360, 360, 360); // 电箱ID号,机器人ID号,J1,J2,3,J4,J5,J6
    IsSuccess(nRet, "HRIF_SetJointMaxAcc");

    // 设置最大直线速度
    nRet = HRIF_SetLinearMaxVel(0, 0, 100); // 电箱ID号,机器人ID号,最大直线速度
    IsSuccess(nRet, "HRIF_SetLinearMaxVel");

    // 设置最大直线加速度
    nRet = HRIF_SetLinearMaxAcc(0, 0, 2500); // 电箱ID号,机器人ID号,最大直线加速度
    IsSuccess(nRet, "HRIF_SetLinearMaxAcc");

    // 下面的设置会限制机械臂运动，建议再次确认无误后设置
    double dMaxJ1 = 360, dMaxJ2 = 135, dMaxJ3 = 153, dMaxJ4 = 360, dMaxJ5 = 180, dMaxJ6 = 360;       // 定义需要设置的关节最大运动范围
    double dMinJ1 = -360, dMinJ2 = -135, dMinJ3 = -153, dMinJ4 = -360, dMinJ5 = -180, dMinJ6 = -360; // 定义需要设置的关节最小运动范围
    // 设置关节运动范围
    nRet = HRIF_SetMaxAcsRange(0, 0, dMaxJ1, dMaxJ2, dMaxJ3, dMaxJ4, dMaxJ5, dMaxJ6,
                               dMinJ1, dMinJ2, dMinJ3, dMinJ4, dMinJ5, dMinJ6);
    IsSuccess(nRet, "HRIF_SetMaxAcsRange");

    // 下面的参数设置会限制机械臂运动，建议再次确认无误后设置，如果后续机械臂运动一直被限制,请调整将下面的值调大
    double dMaxX = 5000, dMaxY = 5000, dMaxZ = 1800;                                  // 定义需要设置的空间最大运动范围
    double dMinX = -5000, dMinY = -5000, dMinZ = 0;                                   // 定义需要设置的空间最小运动范围
    double dUcs_X = 0, dUcs_Y = 0, dUcs_Z = 0, dUcs_Rx = 0, dUcs_Ry = 0, dUcs_Rz = 0; // 定义用户坐标变量
    // 设置末端运动范围
    nRet = HRIF_SetMaxPcsRange(0, 0, dMaxX, dMaxY, dMaxZ, dMinX, dMinY, dMinZ,
                               dUcs_X, dUcs_Y, dUcs_Z, dUcs_Rx, dUcs_Ry, dUcs_Rx);
    IsSuccess(nRet, "HRIF_SetMaxPcsRange");

    int nSafeLevel = 5; // 定义安全风险等级
    // 设置安全风险等级
    nRet = HRIF_SetCollideLevel(0, 0, nSafeLevel);
    IsSuccess(nRet, "HRIF_SetCollideLevel");
}

/**
 *	@brief: 显示机器人的系统的参数;
 *                  速度比，负载参数，最大关节速度，最大关节加速度，最大关节加加速度，最大直线速度，最大直线加速度
 *	@param : 无
 *	@return : 无
 */
void GetSystemParams()
{
    int nRet = -1;

    double dOverride = 0; // 定义速度比变量
    // 读取当前速度比
    nRet = HRIF_ReadOverride(0, 0, dOverride);
    std::cout << "当前速度比：" << dOverride << std::endl;

    double dbMaxPayload = 0;
    // 读取末端最大负载
    nRet = HRIF_ReadMaxPayload(0, 0, dbMaxPayload);
    std::cout << "末端最大负载：" << dbMaxPayload << std::endl;

    double dMass = 0;
    double dX = 0;
    double dY = 0;
    double dZ = 0;
    // 读取当前负载参数
    nRet = HRIF_ReadPayload(0, 0, dMass, dX, dY, dZ);
    std::cout << "末端当前负载：" << dMass << std::endl
              << "质心X偏移值：" << dX
              << " , 质心Y偏移值：" << dY
              << " , 质心Z偏移值：" << dZ << std::endl;

    // 定义关节运动变量，此处重复使用
    double dJ1 = 0, dJ2 = 0, dJ3 = 0, dJ4 = 0, dJ5 = 0, dJ6 = 0;

    // 读取最大关节速度
    nRet = HRIF_ReadJointMaxVel(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
    std::cout << "最大关节速度" << std::endl
              << "dJ1：" << dJ1 << "，dJ2：" << dJ2 << "，dJ3：" << dJ3
              << "，dJ4：" << dJ4 << "，dJ5：" << dJ5 << "，dJ6：" << dJ6 << std::endl;

    // 读取最大关节加速度
    nRet = HRIF_ReadJointMaxAcc(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
    std::cout << "最大关节加速度" << std::endl
              << "dJ1：" << dJ1 << "，dJ2：" << dJ2 << "，dJ3：" << dJ3
              << "，dJ4：" << dJ4 << "，dJ5：" << dJ5 << "，dJ6：" << dJ6 << std::endl;

    // 读取最大关节加加速度
    nRet = HRIF_ReadJointMaxJerk(0, 0, dJ1, dJ2, dJ3, dJ4, dJ5, dJ6);
    std::cout << "最大关节加加速度" << std::endl
              << "dJ1：" << dJ1 << "，dJ2：" << dJ2 << "，dJ3：" << dJ3
              << "，dJ4：" << dJ4 << "，dJ5：" << dJ5 << "，dJ6：" << dJ6 << std::endl;

    // 定义直线最大运动变量
    double dMaxVel = 0;  // 速度
    double dMaxAcc = 0;  // 加速度
    double dMaxJerk = 0; // 加加速度
    // 读取最大直线运动参数
    nRet = HRIF_ReadLinearMaxSpeed(0, 0, dMaxVel, dMaxAcc, dMaxJerk);
    std::cout << "直线最大速度：" << dMaxVel << std::endl
              << "直线最大加速度：" << dMaxAcc << std::endl
              << "直线最大加加速度：" << dMaxJerk << std::endl;
}

/**
 *	@brief: 读取机器人的急停信息，显示机器人的错误码和各关节的错误信息。
 *	@param ：无
 *	@return : 无
 */
void ShowErrorInfo()
{
    int nRet = -1;
    int nESTO_Error = 0;        // 急停回路错误
    int nESTO = 0;              // 急停信号
    int nSafeGuard_Error = 0; // 安全光幕错误
    int nSafeGuard = 0;       // 安全光幕信号
    // 读取急停状态信息
    nRet = HRIF_ReadEmergencyInfo(0, 0, nESTO_Error, nESTO, nSafeGuard_Error, nSafeGuard);
    IsSuccess(nRet, "HRIF_ReadEmergencyInfo");
    std::cout << "急停状态信息为"<< std::endl
              << "急停回路错误：" << nESTO_Error << ","
              << "急停错误：" << nESTO << ","
              << "安全光幕错误：" << nSafeGuard_Error << ","
              << "安全光幕错误：" << nSafeGuard << std::endl;

    int nErrorCode = 0;                                       // 定义错误码变量
    int nJ1 = 0, nJ2 = 0, nJ3 = 0, nJ4 = 0, nJ5 = 0, nJ6 = 0; // 定义各轴错误码变量
    // 读取错误码
    nRet = HRIF_ReadAxisErrorCode(0, 0, nErrorCode, nJ1, nJ2, nJ3, nJ4, nJ5, nJ6);
    IsSuccess(nRet, "HRIF_ReadAxisErrorCode");
    std::cout << "错误码为" << nErrorCode << ",各轴错误信息如下" << std::endl
              << "J1:" << nJ1 << "J2:" << nJ2 << "J3:" << nJ3 << "J4:" << nJ4 << "J5:" << nJ5 << "J6:" << nJ6 << std::endl;
}