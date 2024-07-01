cmake_minimum_required(VERSION 3.2)
set (CMAKE_C_COMPILER "/opt/compiler/gcc-12/bin/gcc")
set (CMAKE_CXX_COMPILER "/opt/compiler/gcc-12/bin/g++")

set (TARGET_NAME "test_bin")
file (GLOB_RECURSE SOURCE_FILES ./src/adas/v2/*.cc)
file (GLOB_RECURSE CJSON_FILES ./cjson/*.c)
file (GLOB_RECURSE MAIN_FILE ./test/main.cpp)
file (GLOB HEADER_FILES ./src/adas/v2/*.h ./cjson/*.h)

include_directories(./src/adas/v2/)
include_directories(./cjson/)
add_executable(${TARGET_NAME} ${SOURCE_FILES} ${CJSON_FILES} ${MAIN_FILE})
target_link_libraries(${TARGET_NAME} pthread)
