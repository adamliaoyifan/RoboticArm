#include "HR_Pro.h"

#include <iostream>
#include <string>

int main()
{
    const std::string robot_ip = "192.168.0.10";
    const int robot_port = 10003;

    std::cout << "Connecting to " << robot_ip << ":" << robot_port << "..." << std::endl;
    const int nRet = HRIF_Connect(0, robot_ip.c_str(), robot_port);
    if (nRet != 0)
    {
        std::cout << "Connect failed, error code: " << nRet << std::endl;
        return 1;
    }

    std::cout << "Connected successfully." << std::endl;

    HRIF_DisConnect(0);
    std::cout << "Disconnected." << std::endl;
    return 0;
}
