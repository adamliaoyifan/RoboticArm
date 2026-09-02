CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(CppSdkSample)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# 1. Output executable to project-root bin/
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 2. Header search path: project include/
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

# 3. Library search path: project lib/
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_BUILD_RPATH ${PROJECT_SOURCE_DIR}/lib)

# 4. Build src/main.cpp as executable
ADD_EXECUTABLE(CppSdkSample src/main.cpp)

# 5. Link libHR_Pro.so from lib/
TARGET_LINK_LIBRARIES(CppSdkSample HR_Pro)
