#include "HR_Pro.h"
#include "CommonForInterface.h"
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>

/**
 *	@brief: 初始化Servo，使用 HRIF_StartServo
 *	@param ：无
 *	@return : 无
 */
void Init_Servo()
{
    double dServoTime = 0.02;    // Servo周期
    double dLookaheadTime = 0.2; // 前瞻时间
    // 启动机器人在线控制
    int nRet = HRIF_StartServo(0, 0, dServoTime, dLookaheadTime);
    IsSuccess(nRet, "HRIF_StartServo");
}

/**
 *	@brief: 运行ServoJ，使用 HRIF_PushServoJ，读取csv文件的关节坐标信息并推送
 *	@param ：无
 *	@return : 无
 */
void ServoJ()
{
    int nRet = -1;
    int count = 0;            //存储循环运行次数
    double dServoTime = 0.02; // Servo周期

    std::string csv_name = "E05_ServoJ_data.csv"; //点位文件
    std::ifstream file(csv_name);                 //打开点位文件
    std::string line = " ";                       //用于存储文件里面的每一行

    time_t currentTime, startTime; //声明两个时间
    time(&startTime);              //记录开始时间

    while (std::getline(file, line))
    {
        count = count + 1; //记录循环次数

        std::vector<double> JointPosition; //用于存储六个关节位置的坐标值
        JointPosition.clear();             //清空点位信息

        while (true)
        {
            time(&currentTime); //获取当前时间
            //此循环的目的是保证时间戳大于执行时间，否则会导致运动滞后
            if ((currentTime - startTime) > (dServoTime * (count + 1)))
            {
                break;
            }
            sleep(0.0001);
        }

        std::stringstream lineStream(line); //将每一行字符串转为字符串流
        std::string cell;                   //用于存储每一行字符串的每个数字

        // CSV 文件使用逗号作为分隔符，按行读取坐标值并转化为double类型
        while (std::getline(lineStream, cell, ','))
        {
            JointPosition.push_back(std::stod(cell));
        }
        //在线关节位置命令控制
        nRet = HRIF_PushServoJ(0, 0, JointPosition[0], JointPosition[1], JointPosition[2], JointPosition[3], JointPosition[4], JointPosition[5]);
        IsSuccess(nRet, "HRIF_PushServoJ");
        sleep(0.01);
    }
}

/**
 *	@brief: 运行ServoP，使用 HRIF_PushServoP，读取txt文件的位置坐标信息并推送
 *	@param ：无
 *	@return : 无
 */
void ServoP()
{
    int nRet = -1;
    int count = 0;            //存储循环运行次数
    double dServoTime = 0.02; // Servo周期

    std::string txt_name = "servoPdata.txt"; //点位文件
    std::ifstream file(txt_name);            //打开点位文件
    std::string line;                        //用于存储文件里面的每一行

    time_t currentTime, startTime; //声明两个时间
    time(&startTime);              //记录开始时间

    std::vector<double> dTcp(6, 0.0); //目标位置对应的TCP
    std::vector<double> dUcs(6, 0.0); //目标位置对应的UCS

    while (std::getline(file, line))
    {
        count = count + 1; //记录运行次数
        std::vector<double> vecCoord;  //目标迪卡尔坐标位置

        while (true)
        {
            time(&currentTime); //获取当前时间
            //此循环的目的是保证时间戳大于执行时间，否则会导致运动滞后
            if ((currentTime - startTime) > (dServoTime * (count + 1)))
            {
                break;
            }
            sleep(0.0001);
        }

        std::stringstream lineStream(line); //将每一行字符串转为字符串流
        std::string cell;                   //用于存储每一行字符串的每个数字

        while (std::getline(lineStream, cell, ',')) // txt 文件使用逗号作为分隔符
        {
            vecCoord.push_back(std::stod(cell));
        }

        //在线末端 TCP 位置命令控制
        nRet = HRIF_PushServoP(0, 0, vecCoord, dUcs, dTcp);
        IsSuccess(nRet, "HRIF_PushServoP");
        sleep(0.01);
    }
}