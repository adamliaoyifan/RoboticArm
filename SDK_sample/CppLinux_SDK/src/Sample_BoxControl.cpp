#include "CommonForInterface.h"
#include "HR_Pro.h"
#include <iostream>
#include <unistd.h>

/**
 *	@brief: 将电箱信息打印出来
 *	@param：无
 *	@return : 无
 */
void BoxInfo()
{
    int nConnected = 0;         // 定义电箱是否连接
    int n48V_ON = 0;            // 48V上电状态
    double db48OUT_Voltag = 0;  // 48V电压值
    double db48OUT_Current = 0; // 48V电流值
    int nRemoteBTN = 0;         // 远程关机按钮状态
    int nThreeStageBTN = 0;     // 三段按钮状态
    // 读取电箱信息
    int nRet = HRIF_ReadBoxInfo(0, 0, nConnected, n48V_ON, db48OUT_Voltag, db48OUT_Current,
                                nRemoteBTN, nThreeStageBTN);
    if (nConnected == 1)
    {
        std::cout << "电箱已连接，";
        if (n48V_ON == 1)
        {
            std::cout << "48V已上电，"
                      << "48V电压值为" << db48OUT_Voltag << "，48V电流值为" << db48OUT_Current << std::endl;
        }
        else if (n48V_ON == 0)
        {
            std::cout << "48V未上电" << std::endl;
        }
    }
    else if (nConnected == 0)
    {
        std::cout << "电箱未连接" << std::endl;
    }
    std::cout << "远程关机按钮状态为:" << nRemoteBTN << "，三段按钮状态为:" << nThreeStageBTN << std::endl;
}

/**
 *	@brief: 设置数字量输出
 *	@param name：需要设置的名字，有CO，DO，EDO
 *  @param nbit: 设置的电箱的位，范围为1-7
 *  @param nVal:  设置对应位的值
 *	@return : 无
 */
void SetDigitalOutput(const string &name, int nBit, int nVal)
{
    int nRet = -1;
    if (name == "CO")
    {
        // 设置电箱第 nBit 位 CO 状态为 nVal
        nRet = HRIF_SetBoxCO(0, nBit, nVal);
        IsSuccess(nRet, "HRIF_SetBoxCO");
    }
    else if (name == "DO")
    {
        // 设置电箱第 nBit 位 DO 状态为 nVal
        nRet = HRIF_SetBoxDO(0, nBit, nVal);
        IsSuccess(nRet, "HRIF_SetBoxDO");
    }
    else if (name == "EDO")
    {
        // 设置末端 nBit 位 DO 状态
        nRet = HRIF_SetEndDO(0, 0, nBit, nVal);
        IsSuccess(nRet, "HRIF_SetEndDO");
    }
}

/**
 *	@brief: 读取数字量输出
 *	@param name：需要设置的名字，有CI，CO，DI，DO，EDI，EDO
 *  @param nbit: 设置的电箱的位，C/D的范围为0-7，ED的范围为0-2
 *	@return : 当前对应位的值
 */
int ReadDigitalIO(const string &name, int nBit)
{
    int nRet = -1;
    int nVal = -1;
    if (name == "CI")
    {
        // 读取电箱第 nBit 位 CI 状态为 nVal
        nRet = HRIF_ReadBoxCI(0, nBit, nVal);
        //IsSuccess(nRet, "HRIF_ReadBoxCI");//一般来说，读取的指令都不会存在错误，如有需要则开启判断
    }
    else if (name == "CO")
    {
        // 读取电箱第 nBit 位 CO 状态为 nVal
        nRet = HRIF_ReadBoxCO(0, nBit, nVal);
    }
    else if (name == "DI")
    {
        // 读取电箱第 nBit 位 DI 状态为 nVal
        nRet = HRIF_ReadBoxDI(0, nBit, nVal);
    }
    else if (name == "DO")
    {
        // 读取电箱第 nBit 位 DO 状态为 nVal
        nRet = HRIF_ReadBoxDO(0, nBit, nVal);
    }
    else if (name == "EDI")
    {
        // 读取末端 nBit位 DI 状态
        nRet = HRIF_ReadEndDI(0, 0, nBit, nVal);
    }
    else if (name == "EDO")
    {
        // 读取末端 nBit 位 DO 状态
        nRet = HRIF_ReadEndDO(0, 0, nBit, nVal);
    }
    return nVal;
}

/**
 *	@brief: 设置模拟量的值
 *	@param nBit :定义需要设置的通道，范围为0-1
 *	@param nMode ：设置对应的模式。1为电压，2为电流
 *	@param dVal ：定义设置模拟值，电压范围为0-10,电流范围为4-20，如果不填则只设置模式。
 *	@return : 无
 */
void SetAO(int nBit, int nMode, double dVal = -1)
{
    int nRet = -1;
    if (dVal < 0)
    {
        // 设置电箱第 nBit 通道 AO 模式为 nVal
        nRet = HRIF_SetBoxAOMode(0, nBit, nMode);
        IsSuccess(nRet, "HRIF_SetBoxAOMode");
    }
    else
    {
        if (dVal >= 0 && dVal <= 20.0)   //在此范围内才进行设置
        {
            // 设置电箱第 nBit 通道 AO 值为 dVal
            nRet = HRIF_SetBoxAOVal(0, nBit, dVal, nMode);
            IsSuccess(nRet, "HRIF_SetBoxAOVal");
        }
        else
        {
            std::cout << "值dVel超出设置范围" << std::endl;
        }
    }
}

/**
 *	@brief: 设置模拟量的值
  *	@param name ：选择需要读取的功能
 *	@param nBit :定义需要设置的通道
 *	@param nMode ：设置对应的模式必须为0或者1
 *	@return : 返回读取到的值
 */
double ReadAIO(const string &name, int nBit, int &nMode)
{
    int nRet = -1;
    double dVal = 0; // 读取结果
    if (name == "AI")
    {
        // 读取电箱 nBit 通道 AI 值
        nRet = HRIF_ReadBoxAI(0, nBit, dVal);
    }
    else if (name == "EAI")
    {
        // 读取末端 nBit 通道 AI 值
        nRet = HRIF_ReadEndAI(0, 0, nBit, dVal);
    }
    else if (name == "AO")
    {
        // 读取电箱 nBit 通道 AO 值
        nRet = HRIF_ReadBoxAO(0, nBit, nMode, dVal);
    }
    return dVal;
}

/**
 *	@brief: 启动，关闭BTN并读取十秒内BTN的值，注意BTN启动和关闭功能最好配套使用
  *	@param ：无
 *	@return : 无
 */
void UseBTN()
{
    int nRet = -1;
    int nStatus = 1; // 定义需要设置的状态
    // 启用末端按钮
    nRet = HRIF_EnableEndBTN(0, 0, 1);
    IsSuccess(nRet, "HRIF_EnableEndBTN");
    // 定义需要读取的位
    int nBit1 = 0,nBit2 = 0,nBit3 = 0,nBit4 = 0;
    for (size_t i = 0; i < 10; i++) //这里每秒读一次，读取十次末端的BTN值
    {
        nRet = HRIF_ReadEndBTN(0, 0, nBit1, nBit2, nBit3, nBit4);// 读取末端BTN的值
        IsSuccess(nRet, "HRIF_ReadEndBTN");
        //将BTN的每一位打印出来，可以替换为相应的任务
        std::cout << "第一位：" << nBit1 << "，第二位：" << nBit2 << "，第三位：" << nBit3 << "，第四位：" << nBit4 << std::endl;
        sleep(1);
    }
    // 关闭末端按钮
    nRet = HRIF_EnableEndBTN(0, 0, 0);
    IsSuccess(nRet, "HRIF_EnableEndBTN");
}