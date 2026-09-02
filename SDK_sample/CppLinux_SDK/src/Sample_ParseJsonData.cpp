# include "interface.h"
#include "HR_Pro.h"
#include "HRSDK/HR_JsonBase.h"
#include "HRSDK/HR_BaseSocket.h"
#include <iostream>
#include <sys/socket.h>
#include <algorithm>
#include <chrono>

ParsePortJsonData::ParsePortJsonData(const string &strIp) : m_strIp(strIp), m_nSockfd(-1), m_nStopParse(0), outputFile("output.json") //, m_Count(0)
{
}

ParsePortJsonData::~ParsePortJsonData()
{
    outputFile.close();
}

/**
 *	@brief: 创建Socket套接字并连接,主线程调用运行
 *	@param ：无
 *	@return : 无
 */
void ParsePortJsonData::CreateSocket()
{
    // 创建socket
    m_nSockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (m_nSockfd == -1)
    {
        std::cerr << "Failed to create socket\n";
    }
    // 设置服务器地址
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(10006);                        // 服务器端口,端口号为10006
    inet_pton(AF_INET, m_strIp.c_str(), &serverAddr.sin_addr); // 服务器IP
    // 连接服务器
    if (connect(m_nSockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Failed to connect to server\n";
        close(m_nSockfd);
    }
    std::cout << "Connected to server\n";
}

/**
 *	@brief: 接收指定socket的数据，读取数据线程调用
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::ReceiveData()
{
    char buffer[1024];
    ssize_t bytesReceived;
    do
    {
        bytesReceived = recv(m_nSockfd, buffer, sizeof(buffer), 0); // 获取socket返回的数据
        if (bytesReceived == -1)
        {
            std::cerr << "Failed to receive data\n";
            close(m_nSockfd);
            break;
        }
        {
            AutoSaftMutex saftmutex(&m_WriteLock);
            m_strJsonData.append(buffer, bytesReceived); // recv的字符添加进JsonData的句尾
            if (m_nStopParse == 1)
            {
                break;
            }
        } // 作用域锁数据写入保护
        usleep(50000);
    } while (true);
}

/**
 *	@brief: 对读取到的数据进行处理，处理数据线程调用
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::ParseJsonData()
{
    for (size_t i = 0; i < 100; i++)
    {
        std::cout << "第" << i << "次解析结果:"<<std::endl;
        outputFile<< "第" << i << "次解析结果:"<<std::endl;
        JudgeJsonGotTargetString("LTBR"); // 等待第一个标识符出现

        GetDataLength(); // 获取数据长度

        JudgeJsonGotTargetString("LTBR"); // 等待下一个标识符的出现

        SplitJsonString(); // 提取每一帧数据到变量里面

        ShowValue();
    }
    {
        AutoSaftMutex saftmutex(&m_WriteLock);
        m_nStopParse = 1;
    }
    ShowKey();  //展示所有可以读取的Key，可以使用里面的Key替换ShowValue函数里面的逻辑

    return;
}


/**
 *	@brief: 判断JsonData内是否已存在指定的字符串数据
 *	@param target: 查找的字符串
 *	@return : 无
 */
void ParsePortJsonData::JudgeJsonGotTargetString(string target)
{
    auto start = std::chrono::high_resolution_clock::now(); // 获取任务的开始时间
    do
    {
        {
            AutoSaftMutex saftmutex(&m_WriteLock);
            m_nPos = m_strJsonData.find(target); // 寻找目标字符的位置
        }
        auto end = std::chrono::high_resolution_clock::now();                               // 获取当前时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); // 计算任务运行时长
        int PortFrequency = 230;                                                            // 时间限制在端口的接收频率200ms基础上加30ms，超出则认定端口连接或数据接受存在问题
        if (duration.count() >= PortFrequency)
        {
            cout << "频率内接收不到消息，请查看端口与频率之间是否存在问题" << endl;
            cout << duration.count() << endl;
            break;
        }
        usleep(50000);
    } while (m_nPos == std::string::npos);
}

/**
 *	@brief: 接收指定socket的数据
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::GetDataLength()
{
    AutoSaftMutex saftmutex(&m_WriteLock);
    m_nPos = m_strJsonData.find("LTBR"); // 查找子字符串的位置

    if (m_nPos != std::string::npos)
    {
        m_strJsonData.erase(0, m_nPos + 4); // 如果找到子字符串，则删除包含LTBR在内的LTBR前面的数据，这个4便是LTBR的长度
    }
    else
    {
        cout << "未在字符串中查询到： LTBR 的位置，删除失败！" << endl;
        cout << m_strJsonData << endl;
    }
    // LTBR后面接着两个长度（包长度，数据长度）类型为unsigned int，下面提取数据的长度
    memcpy(&m_unDataLength, &m_strJsonData[sizeof(unsigned int)], sizeof(unsigned int)); // 读取 Data（data len）
    m_strJsonData.erase(0, sizeof(unsigned int) + sizeof(unsigned int));                 // 将这两个长度数据都删除后就是信息数据了
}

/**
 *	@brief: 将socket中recevie的 Json字符串通过目标字符串进行分包处理
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::SplitJsonString()
{
    AutoSaftMutex saftmutex(&m_WriteLock);
    string JsonString;                   // 存放每一帧数据
    m_nPos = m_strJsonData.find("LTBR"); // 寻找目标字符的位置
    if (m_nPos == std::string::npos)
    {
        cout << "找不到目标字符" << endl;
    }
    else if (m_nPos == m_unDataLength)
    {
        JsonString = m_strJsonData.substr(0, m_nPos);
        m_strJsonData.erase(0, m_nPos); // 删除目标字符前所有字符
        LoadString(JsonString);         // 加载到HR_JsonBase变量里面
        outputFile << JsonString << std::endl;
    }
    else
    {
        cout << "字符长度不匹配" << endl;
    }
}

/**
 *	@brief: 展示Json数据所有的Key
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::ShowKey()
{
    std::cout <<  "Json提供的Key，可以根据下面的Key读取需要的值" << std::endl; 
    vector<string> Key;
    GetAllKeys(Key);
    for (const auto &str : Key)
    {
        std::cout << str << "," << std::endl; // 输出Key
    }
    std::cout <<  "Json提供的Key，可以根据上面的Key读取需要的值" << std::endl; 
}


/**
 *	@brief: 展示Json数据Key对应的值，下面是一个展示
 *	@param : 无
 *	@return : 无
 */
void ParsePortJsonData::ShowValue()
{
    int EnableEndBTNResult = 0;
    string EnableEndBTN = "EndIO/EnableEndBTN";
    ReadValue(EnableEndBTN, EnableEndBTNResult);
    std::cout << "EnableEndBTN值为" << EnableEndBTNResult << " , ";

    string strKey = "";
    string strValue;
    for (size_t i = 0; i < 12; i++)
    {
        ReadArraryKeyValue("PosAndVel/Actual_Position", i, strKey, strValue);
        std::cout << "Actual_PCS_TCP[" << i << "]值为" << strValue << " , ";
    }
    std::cout << std::endl;
}