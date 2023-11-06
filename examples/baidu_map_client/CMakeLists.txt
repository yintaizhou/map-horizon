cmake_minimum_required(VERSION 3.2)
set (CMAKE_C_COMPILER "/opt/compiler/gcc-12/bin/gcc")
set (CMAKE_CXX_COMPILER "/opt/compiler/gcc-12/bin/g++")

set (TARGET_NAME "baidu_map_client")
set (WEBSOCKETPP_ROOT "../../websocketpp-0.8.2")
set (BOOST_INCLUDE "../..//boost_1_83_0")
set (JSON_INCLUDE "../../json-3.11.2/include")

# 源文件
file (GLOB SOURCE_FILES *.cpp)
file (GLOB HEADER_FILES *.hpp)

# include头文件
include_directories (${WEBSOCKETPP_ROOT} ${BOOST_INCLUDE} ${JSON_INCLUDE} ${HEADER_FILES}) 
# 编译可执行文件
add_executable (${TARGET_NAME} ${SOURCE_FILES})
# 连接
target_link_libraries(${TARGET_NAME} pthread)
