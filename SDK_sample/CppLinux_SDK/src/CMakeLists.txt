CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(CppSdkSample)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -std=c++17 -march=native -pthread")

# Local SDK layout (preferred):
#   ../sdk/include/HRSDK/   HR_Pro.h, HR_JsonBase.h, cJSON.h, ...
#   ../sdk/lib/             libHR_Pro.so -> libHR_Pro.so.1
#   ../sdk/HRCPS/           libHR_SocketClient.so, libHR_sysBase.so, libHR_Log.so
#
# Or system install: /usr/local/include/HRSDK, /usr/local/lib, /usr/local/bin/Robot/HRCPS
SET(SDK_ROOT ${PROJECT_SOURCE_DIR}/../sdk)
IF(EXISTS "${SDK_ROOT}/include/HRSDK/HR_Pro.h")
  SET(SDK_INCLUDE_PATHS ${SDK_ROOT}/include/HRSDK)
  SET(SDK_LINK_PATHS ${SDK_ROOT}/lib)
  SET(SYS_LINK_PATHS ${SDK_ROOT}/HRCPS)
ELSE()
  SET(SDK_INCLUDE_PATHS /usr/local/include/HRSDK)
  SET(SDK_LINK_PATHS /usr/local/lib)
  SET(SYS_LINK_PATHS /usr/local/bin/Robot/HRCPS)
ENDIF()

# INCLUDE Path
SET(SOURCE_INCLUDE_PATHS ${PROJECT_SOURCE_DIR}/../include)
INCLUDE_DIRECTORIES(${SOURCE_INCLUDE_PATHS})
INCLUDE_DIRECTORIES(${SDK_INCLUDE_PATHS})

LINK_DIRECTORIES(${SDK_LINK_PATHS} ${SYS_LINK_PATHS})
SET(CMAKE_BUILD_RPATH ${SDK_LINK_PATHS} ${SYS_LINK_PATHS})

# OUTPUT Path
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)

# BUILD
SET(CppSdkSample_SRC 
        main.cpp                #主函数
        CommonForInterface.cpp  #公共接口函数
        UserCaseFunction.cpp
        Sample_XToStandby.cpp 
        Sample_SetSimulateRobot.cpp
        Sample_BrakeControl.cpp
        Smaple_Grap.cpp
        Sample_Script.cpp
        Sample_BoxControl.cpp
        Sample_BoxID.cpp 
        Sample_SysState.cpp
        Sample_PointInfo.cpp
        Sample_RobotState.cpp
        Sample_CoordivateTran.cpp
        Sample_ConfigTcpUcs.cpp
        Sample_ForceControl.cpp
        Sample_FreeDrive.cpp
        Sample_Move.cpp 
        Sample_MovePath.cpp
        Sample_MovepathJ.cpp  
        Sample_MovepathL.cpp 
        Sample_Servo.cpp 
        Sample_Trace.cpp
        Sample_ConnectToModBus.cpp
        portdata.cpp
        Sample_ParseJsonData.cpp
        Sample_CalibrationForce.cpp
)

add_executable(CppSdkSample ${CppSdkSample_SRC})
target_link_libraries(CppSdkSample ${SDK_LINK_PATHS}/libHR_Pro.so)
target_link_libraries(CppSdkSample ${SYS_LINK_PATHS}/libHR_SocketClient.so)

target_link_libraries(CppSdkSample ${SYS_LINK_PATHS}/libHR_sysBase.so)
target_link_libraries(CppSdkSample ${SYS_LINK_PATHS}/libHR_Log.so)

