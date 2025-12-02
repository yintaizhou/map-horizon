cmake_minimum_required(VERSION 3.2)
set (CMAKE_C_COMPILER "/opt/compiler/gcc-12/bin/gcc")
set (CMAKE_CXX_COMPILER "/opt/compiler/gcc-12/bin/g++")

set (TARGET_NAME "baidu_map_client")
set (RSA_DEMO_NAME "rsa_demo")
set (AES_DEMO_NAME "aes_demo")
set (WEBSOCKETPP_ROOT "../../websocketpp-0.8.2")
set (BOOST_INCLUDE "../..//boost_1_83_0")
set (JSON_INCLUDE "../../json-3.11.2/include")

# 明确指定WebSocket客户端的源文件，避免包含RSA相关文件
set (WEBSOCKET_SOURCE_FILES 
    main.cpp
    # 如果有其他WebSocket相关的源文件，在这里添加
)

# RSA demo的源文件
set (RSA_SOURCE_FILES 
    rsa_demo.cpp
    rsa_crypto.cpp
)

# AES demo的源文件
set (AES_SOURCE_FILES 
    aes_demo.cpp
    aes_crypt.cpp
)

# include头文件路径（只包含目录，不包含具体文件）
include_directories (${WEBSOCKETPP_ROOT} ${BOOST_INCLUDE} ${JSON_INCLUDE}) 

# 编译WebSocket客户端
add_executable (${TARGET_NAME} 
    ${WEBSOCKET_SOURCE_FILES}
    rsa_crypto.cpp
    aes_crypt.cpp
)
# 连接库
target_link_libraries(${TARGET_NAME} pthread ssl crypto dl)

# 编译RSA demo
add_executable (${RSA_DEMO_NAME}
    rsa_demo.cpp
    rsa_crypto.cpp
)

# 链接OpenSSL库和动态链接库
target_link_libraries(${RSA_DEMO_NAME} ssl crypto dl)

# 编译AES demo
add_executable (${AES_DEMO_NAME}
    aes_demo.cpp
    aes_crypt.cpp
)
# 链接OpenSSL库和动态链接库
target_link_libraries(${AES_DEMO_NAME} ssl crypto dl)