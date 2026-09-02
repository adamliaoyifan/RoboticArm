#include "HR_Pro.h"
#include "CommonForInterface.h"

#include <iostream>
#include <sstream>
#include <unistd.h>

void setBaseInstallingAngle()
{
    int nRet = -1;
    int nRotation=0;//设置机座旋转角度
    int nTilt=0;//设置机座倾斜角度
    nRet = HRIF_SetBaseInstallingAngle(0,0,nRotation,nTilt); // 设置机座安装角度
    IsSuccess(nRet, "HRIF_SetBaseInstallingAngle");
    int getNRotation=0;
    int getNTilt=0;
    nRet = HRIF_GetBaseInstallingAngle(0,0,nRotation,nTilt); // 读取机座安装角度，查看是否满足要求
    IsSuccess(nRet, "HRIF_GetBaseInstallingAngle");
}

void loadIdentify()
{
    int nRet = -1;
    double dX;//x方向的原始力数据；
    double dY;//y方向的原始力数据；
    double dZ;//z方向的原始力数据；
    double dRX;//RX方向的原始力数据；
    double dRY;//RY方向的原始力数据；
    double dRz;//RZ方向的原始力数据；
    nRet = HRIF_ReadForceData(0,0,dX,dY,dZ,dRX,dRY,dRz); // 开始负载识别
    IsSuccess(nRet, "HRIF_ReadForceData");

    int status=7;//负载识别状态
    int progress=0;//当前负载识别进度
    double mass=0;//负载质量
    double cx=0;//负载质心 X 坐标
    double cy=0;//负载质心 Y 坐标
    double cz=0;//负载质心 Z 坐标
    nRet = HRIF_GetLoadIdentifyResult(0,0,status,progress,mass,cx,cy,cz); // 读取机座安装角度，查看是否满足要求
    IsSuccess(nRet, "HRIF_GetLoadIdentifyResult");
}

void CalibrationForce()
{
    int nRet = -1;
    double dX=0;//x方向的原始力数据；
    double dY=0;//y方向的原始力数据；
    double dZ=0;//z方向的原始力数据；
    double dRX=0;//RX方向的原始力数据；
    double dRY=0;//RY方向的原始力数据；
    double dRz=0;//RZ方向的原始力数据；
    nRet = HRIF_ReadForceData(0,0,dX,dY,dZ,dRX,dRY,dRz); // 开始负载识别
    IsSuccess(nRet, "HRIF_ReadForceData");
    


     // 原始数据（以逗号分隔的字符串）
    std::string data = "420.000,-0.001,445.004,-180.000,0.001,-180.000,0.1,0,0,0.001,-0.002,0.007,"
                       "419.999,-0.002,445.000,114.295,0.000,180.000,2.8,-1.3,-1.9,0.021,0.037,0.024,"
                       "420.000,0.001,445.005,-98.256,0.001,-179.999,-3,1.5,-2.8,-0.013,-0.042,0.003,"
                       "419.998,0.002,445.003,180.000,-59.664,-180.000,-1,-2.6,-1.6,0.042,-0.02,-0.001,"
                       "420.001,0.002,444.995,180.000,53.128,180.000,1.3,2.4,-1.4,-0.027,0.013,0.018,"
                       "419.302,0.000,435.845,127.693,-50.301,-179.484,0.7,-3,-2.1,0.046,0.003,0.019,"
                       "420.000,-0.002,445.012,-92.510,1.413,-150.687,-2.8,1.6,-3.3,-0.012,-0.045,0.012,"
                       "409.266,-34.590,421.446,-82.100,40.722,-127.015,-1.2,3.1,-3.8,-0.032,-0.019,0.026";

    // 将字符串数据解析为 vector<double>
    std::vector<double> rawData;
    std::stringstream ss(data);
    std::string token;
    while (std::getline(ss, token, ',')) {
        rawData.push_back(std::stod(token));
    }

    // 计算点的数量（每组 12 个数据）
    size_t pointNum = rawData.size() / 12;

    // 创建 vector<vector<double>> 并填充数据
    std::vector<std::vector<double>> param(pointNum, std::vector<double>(12, 0));
    int cnt=0;
    for (size_t i = 0; i < pointNum; ++i) {
        for (size_t j = 0; j < 12; ++j) {
            param[i][j] = rawData[i * 12 + j];
            cnt++;
        }
    }
    cout<<cnt<<endl;

    nRet = HRIF_SetFTCalibration(0,0,pointNum,param); //设置标定的点位
    IsSuccess(nRet, "HRIF_SetFTCalibration");

    double Fx=0;
    double Fy=0;
    double Fz=0;
    double Mx=0;
    double My=0;
    double Mz=0;
    double x=0;
    double y=0;
    double z=0;
    double G=0;
    double InstallRotationAngle=0;
    nRet = HRIF_GetLastCalibParams(0,0,Fx,Fy,Fz,Mx,My,Mz,x,y,z,G,InstallRotationAngle); // 读取机座安装角度，查看是否满足要求
    IsSuccess(nRet, "HRIF_GetLastCalibParams");
}