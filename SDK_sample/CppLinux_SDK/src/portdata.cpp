#include <iostream>
#include "HR_Pro.h"
#include <cassert>
#include <unistd.h>
#include <thread>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "HRSDK/HR_JsonBase.h"
#include "HRSDK/cJSON.h"
#include <algorithm>
#include <mutex>
#include <memory>

#define Package_Identifier "LTBR"
extern const char *IP;
mutex mtx;

/**
 *	@brief: 创建Socket套接字并连接
 *	@param ：无
 *	@return : 返回值为-1则创建连接过程失败，返回值不为-1则创建连接成功
 */
int createsocket(const string &IP)
{
    // 创建socket
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
    {
        std::cerr << "Failed to create socket\n";
        return -1;
    }
    // 设置服务器地址
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(10006);             // 服务器端口,端口号为10006
    inet_pton(AF_INET, IP.c_str(), &serverAddr.sin_addr); // 服务器IP
    // 连接服务器
    if (connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Failed to connect to server\n";
        close(sockfd);
        return -1;
    }
    std::cout << "Connected to server\n";
    return sockfd;
}

/**
 *	@brief: 删除字符串中的指定子字符串
 *	@param JsonString : 被操作的父字符串
 *	@param substr : 指定子字符串
 *	@return : 无
 */
void Delete_characters(string &JsonString, string substr)
{
    size_t pos = JsonString.find(substr); // 查找子字符串的位置
    // 如果找到子字符串，则删除
    if (pos != std::string::npos)
    {
        JsonString.erase(0, pos);
    }
    else
    {
        cout << "未在字符串中查询到：" << substr << "的位置，删除失败！" << endl;
        cout<<JsonString<<endl;
    }
}

/**
 *	@brief: 清理Json字符串的数据，将字符串清理成Json格式
 *	@param JsonString : Json字符串
 *	@return : 无
 */
void GetOriginJson(string &JsonString)
{
    //必须使用‘\000’清洗Json里面的空格符，否则此字符串无法转换cJSON格式的对象
    JsonString.erase(std::remove(JsonString.begin(), JsonString.end(), '\000'), JsonString.end());
    JsonString.erase(std::remove(JsonString.begin(), JsonString.end(), '\n'), JsonString.end());
    JsonString.erase(std::remove(JsonString.begin(), JsonString.end(), '\t'), JsonString.end());
}

/**
 *	@brief: 判断JsonData内是否已存在数据
 *	@param JsonData : Json字符串
 *	@return : 无
 */
void GudgeJsonGotTargetString(string &JsonData,string target)
{
    int PortFrequency = 200;
    auto start = std::chrono::high_resolution_clock::now(); //获取任务的开始时间
    size_t Pos = JsonData.find(target);  
    while (Pos == std::string::npos)
    {
        Pos = JsonData.find(target);                            // 寻找目标字符的位置
        auto end = std::chrono::high_resolution_clock::now();                               //获取当前时间
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start); //计算任务运行时长
        int ReadFrequency = PortFrequency +30;                                          //时间限制在端口的接收频率加30ms，超出则认定端口连接或数据接受存在问题
        if (duration.count() >= ReadFrequency)
        {
            cout << "频率内接收不到消息，请查看端口与频率之间是否存在问题" << endl;
            cout << duration.count() << endl;
            break;
        }
    }
}

/**
 *	@brief: 将socket中recevie的 Json字符串通过目标字符串进行分包处理
 *	@param JsonString : Json字符串
 *	@param target : 目标字符串
 *	@return : 无
 */
string splitJsonString(string &JsonData, string target)
{
    string JsonString;
    GudgeJsonGotTargetString(JsonData, target); //等待第一个标识符出现
    GudgeJsonGotTargetString(JsonData, "{");    //等待第一个“{”的出现
    Delete_characters(JsonData, "{");           //删除“{”之前的数据
    GudgeJsonGotTargetString(JsonData, target); //等待下一个标识符的出现
    size_t Pos = JsonData.find(target);         // 寻找目标字符的位置
    if (Pos == std::string::npos)
    {
        cout << "找不到目标字符" << endl;
    }
    else
    {
        JsonString = JsonData.substr(0, Pos);
        lock_guard<std::mutex> lock(mtx);
        JsonData.erase(0, Pos); // 删除目标字符前所有字符
    }
    return JsonString;
}

/**
 *	@brief: 接收指定socket的数据
 *	@param sockfd : socket套接字
 *	@return :返回接收完整的一次JsonString 
 */
void receive(int sockfd, string &JsonData)
{
    char buffer[1024];
    ssize_t bytesReceived;
    do
    {
        bytesReceived = recv(sockfd, buffer, sizeof(buffer), 0);//获取socket返回的数据
        if (bytesReceived == -1)
        {
            std::cerr << "Failed to receive data\n";
            close(sockfd);
            break;
        }
        lock_guard<std::mutex> lock(mtx);//互斥锁
        JsonData.append(buffer, bytesReceived);//recv的字符添加进JsonData的句尾
        usleep(50000);
    } while (true);
}

/**
 *	@brief: 获取Json的指定父键子键的数组类型的值
 *	@param jsonVoid : cJSON数据
 *	@param childName : 父键
 *	@param sonName : 子键
 *  @param vecVal : 数组类型值
 *	@return :无
 */
void GetJsonArrayVlaue(void *jsonVoid, const string &childName, const string &sonName, vector<string> &vecVal)
{
    cJSON *root = (cJSON *)jsonVoid;
    cJSON *child_results = cJSON_GetObjectItem(root, childName.c_str());      //获取父键对应的JSON数据
    cJSON *Arryresults = cJSON_GetObjectItem(child_results, sonName.c_str()); //获取子键对应的JSON数据
    HR_JsonBase::ReadJsonVal(Arryresults, vecVal);                            //读取子键的值
}

/**
 *	@brief: 获取Json的指定父键子键的类型的值
 *	@param jsonVoid : cJSON数据
 *	@param childName : 父键
 *	@param sonName : 子键
 *	@param output : int类型的值
 *	@return :无
 */
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, int &output)
{
    cJSON *root = (cJSON *)jsonVoid;
    cJSON *results = cJSON_GetObjectItem(root, childName.c_str()); //获取父键对应的JSON数据
    HR_JsonBase::ReadJsonVal(results, sonName, output);            //读取子键对应JSON值
}

/**
 *	@brief: 获取Json的指定父键子键的数组类型的值
 *	@param jsonVoid : cJSON数据
 *	@param childName : 父键
 *	@param sonName : 子键
 *	@param output : double类型的值
 *	@return :无
 */
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, double &output)
{
    cJSON *root = (cJSON *)jsonVoid;
    cJSON *results = cJSON_GetObjectItem(root, childName.c_str()); //获取父键对应的JSON数据
    HR_JsonBase::ReadJsonVal(results, sonName, output);            //读取子键对应JSON值
}

/**
 *	@brief: 获取Json的指定父键子键的数组类型的值
 *	@param jsonVoid : cJSON数据
 *	@param childName : 父键
 *	@param sonName : 子键
 *	@param output : double类型的值
 *	@return :无
 */
void GetJsonVlaue(void *jsonVoid, const string &childName, const string &sonName, string &output)
{
    cJSON *root = (cJSON *)jsonVoid;
    cJSON *results = cJSON_GetObjectItem(root, childName.c_str()); //获取父键对应的JSON数据
    HR_JsonBase::ReadJsonVal(results, sonName, output);            //读取子键对应JSON值
}
