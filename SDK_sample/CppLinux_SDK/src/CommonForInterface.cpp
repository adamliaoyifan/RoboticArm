#include "HR_Pro.h"
#include <iostream>

/**
 *	@brief: 公共接口，用于判断SDK指令是否运行成功
 *	@param nRet：指令的输出结果，指令运行成功为0，不成功为其他
 *	@param interfacename：输入运行指令的名字或者说明
 *	@return : 无
 */
void IsSuccess(int nRet, const char *interfacename)
{
    if (interfacename != NULL)
    {
        if (nRet)
        {
            std::cout << interfacename << " Fail:" << nRet ; // 值不为0，接口调用失败
            std::string tmpstr;                                          // 如果启动错误，则查询错误原因
            HRIF_GetErrorCodeStr(0, nRet, tmpstr);
            std::cout << ",It mean error:" << tmpstr << std::endl;
        }
        else
        {
            std::cout << interfacename << " Success" << std::endl; // 值为0，接口调用成功
        }
    }
    else
    {
        std::cout << "接口参数错误" << std::endl;
    }
}