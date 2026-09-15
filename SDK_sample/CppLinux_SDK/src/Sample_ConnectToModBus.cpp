#include "HR_Pro.h"
#include "CommonForInterface.h"
#include <iostream>
#include <unistd.h>


/**
 *	@brief: 将数据写入末端寄存器
 *	@param ：无
 *	@return : 无
 */
void WriteToRegisters()
{
    int nSlaveID = 1;                   // 设置从站 ID
    int nFunction = 16;                 // 设置功能码，16代表写入
    int nRegAddr = 100;                // 设置寄存器起始地址
    int nRegCount = 2;                  // 设置寄存器数量
    vector<int> vecData = {1, 1}; // 设置寄存器数据
    int nRet = HRIF_WriteEndHoldingRegisters(0, 0, nSlaveID, nFunction, nRegAddr, nRegCount, vecData);
    IsSuccess(nRet, "HRIF_WriteEndHoldingRegisters");
    std::cout<<"写入到"<<nRegAddr<<"开始的"<<nRegCount<<"个寄存器的值为"<<std::endl;
    for (size_t i = 0; i < vecData.size(); ++i) {  
        std::cout << nRegAddr + i << ":" << vecData[i] << ",";  
    }   
    if (!vecData.empty()) {  
        std::cout << "\b \b"; // 使用退格符移除最后一个逗号和换行前的空格  
    }  
    std::cout<<std::endl;
}

/**
 *	@brief: 将数据从末端寄存器里面读取出来
 *	@param ：无
 *	@return : 无
 */
void ReadFromRegisters()
{
    int nSlaveID = 1;         // 设置从站 ID
    int nFunction = 3;        // 设置功能码，3代表读取
    int nRegAddr = 100;       // 设置寄存器起始地址
    int nRegCount = 4;        // 设置寄存器数量
    vector<int> vecData = {}; // 读取寄存器数据
    int nRet = HRIF_ReadEndHoldingRegisters(0, 0, nSlaveID, nFunction, nRegAddr, nRegCount, vecData);
    IsSuccess(nRet, "HRIF_ReadEndHoldingRegisters");
    auto iter = vecData.begin();
    auto end = vecData.end();
    std::cout<<"读取到"<<nRegAddr<<"开始的"<<nRegCount<<"个寄存器的值为"<<std::endl;
    for (size_t i = 0; i < vecData.size(); ++i) {  
        std::cout << nRegAddr + i << ":" << vecData[i] << ",";  
    }   
    if (!vecData.empty()) {  
        std::cout << "\b \b"; // 使用退格符移除最后一个逗号和换行前的空格  
    }  
    std::cout<<std::endl;
}