#include "CommonForInterface.h"
#include "HR_Pro.h"
#include "interface.h"

#include <iostream>
#include <unistd.h>
#include <thread>

/**
 *	@brief: 构造函数，对两台设备的IP和电箱ID进行赋值
 *	@param ：无
 *	@return : 无
 */
SampleBoxID::SampleBoxID(string IP0, string IP1)
    : m_nBoxID0(0), m_nBoxID1(1)
{
    m_strIP0 = IP0;
    m_strIP1 = IP1;
}

/**
 *	@brief: 析构函数，断开设备1
 *	@param ：无
 *	@return : 无
 */
SampleBoxID::~SampleBoxID()
{
    // HRIF_DisConnect(0);
    HRIF_DisConnect(1);
    std::cout << "与IP1断开连接" << std::endl;
}

/**
 *	@brief: 初始化配置IP和BoxID连接
 *	@param ：无
 *	@return : 无
 */
void SampleBoxID::ConfigIpBoxId()
{
    int nRet = -1;
    std::cout << "开始连接" << std::endl;
    // nRet = HRIF_Connect(m_nBoxID0, m_strIP0.c_str(), 10003); // BoxID0已经在主函数开始时已经，不用重复连接连接，配置逻辑不变
    std::cout << "电箱ID:" << m_nBoxID0 << "," << "连接设备0,IP:" << m_strIP0 << std::endl;
    nRet = HRIF_Connect(m_nBoxID1, m_strIP1.c_str(), 10003); // 线程2连接
    if (nRet == 0)
    {
        std::cout << "电箱ID:" << m_nBoxID1 << "," << "连接设备1,IP:" << m_strIP1 << std::endl;
    }
    else
    {
        std::cout << "IP:" << m_strIP1 << "连接失败" << std::endl;
    }
}

/**
 *	@brief: 线程A运行
 *	@param ：无
 *	@return : 无
 */
void SampleBoxID::SendCmdInThreadA()
{
    int nRet = -1;
    int nVal = 0;
    int nFlag = 0;
    do
    {
        HRIF_ReadBoxCO(m_nBoxID0, 0, nVal);                         // 读取CO0的状态
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 每20ms查看一次
        std::cout << "线程A读取BoxID0的CO0为" << nVal << std::endl;
        nFlag++;
    } while (nVal == 0 && nFlag < 100);
    if (nVal == 1)
    {
        nRet = HRIF_SetBoxCO(m_nBoxID1, 0, 1); // 将BoxID1的CO1置1
        std::cout << "线程A设置BoxID1的CO1为1" << std::endl;
    }
}

/**
 *	@brief: 线程B运行
 *	@param ：无
 *	@return : 无
 */
void SampleBoxID::SendCmdInThreadB()
{
    int nVal = -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    HRIF_SetBoxCO(m_nBoxID0, 0, 1); // 将BoxID0的CO0置1
    std::cout << "线程B设置BoxID0的CO0为1" << std::endl;
    HRIF_SetBoxCO(m_nBoxID0, 1, 1); // 将BoxID0的CO1置1
    std::cout << "线程B设置BoxID0的CO1为1" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 延时1秒
    HRIF_ReadBoxCO(m_nBoxID1, 0, nVal);                           // 读取CO0的状态
    std::cout << "BoxID1读取CO0的值为" << nVal << std::endl;
    HRIF_ReadBoxCO(m_nBoxID1, 1, nVal); // 读取CO1的状态
    std::cout << "BoxID1读取CO1的值为" << nVal << std::endl;
    HRIF_SetBoxCO(m_nBoxID0, 0, 0); // 将CO0置0
    HRIF_SetBoxCO(m_nBoxID0, 1, 0); // 将CO1置0
}
