/***************************************************************************
 *
 * Copyright (c) 2021 Baidu.com, Inc. All Rights Reserved
 * $Id$
 *
 **************************************************************************/
/**
 * @file can_protocol.h
 * @author css01
 * @date 2021/03/05 15:04:01
 * @brief send ADAS protocol to can bus
 *
 **/
#ifndef CAN_INCLUDE_CAN_PROTOCOL_H
#define CAN_INCLUDE_CAN_PROTOCOL_H

#include <iostream>
#include <string>
#include <bitset>
#include "can_message.h"
#include "common.h"
// #include "can_channel.h"

namespace adas {
namespace can {

class CanProtocol {
public:
    CanProtocol() : _type(MOTOROLA) {};
    ~CanProtocol() {};

    // @brief init can bus connection
    // @param 
    // @return 0 for ok, -1 for error
    // int init();
public:
    // @brief send postion message to can bus
    // @param 
    // @return 0 for ok, -1 for error
    int reorganize_position(const PositionMessage& position, std::bitset<64>& message);
    // @brief send stub message to can bus
    // @param
    // @return 0 for ok, -1 for error
    int reorganize_stub(const StubMessage& stub, std::bitset<64>& message);
    // @brief send segment message to can bus
    // @param
    // @return 0 for ok, -1 for error
    int reorganize_segment(const SegmentMessage& segment, std::bitset<64>& message);
    // @brief send short profile message to can bus
    // @param
    // @return 0 for ok, -1 for error
    int reorganize_shortprofile(const ProfileShortMessage& profile, std::bitset<64>& message);
    // @brief send long profile message to can bus
    // @param
    // @return 0 for ok, -1 for error
    int reorganize_longprofile(const ProfileLongMessage& profile, std::bitset<64>& message);
    // @brief send meta-data message to can bus
    // @param
    // @return 0 for ok, -1 for error
    int reorganize_metadata(const MetaMessage& meta, std::bitset<64>& message);

    // @brief set encode type, 默认摩托罗拉
    // @param type
    // @return
    void set_encode_type(const EncodeType& type);

    // @brief 设置消息的循环计数值
    // @param [out] message, [in] cyccnt
    // @return 0 for ok, -1 for error
    int set_cyccnt(std::bitset<64>& message, int cyccnt);

    // @brief 获取这个数据起始字节位置
    // @param 摩托罗拉编码起始字节位置
    // @return 根据type返回对应的位置
    int get_start_index(int index);

    // @brief 设置retrans字段为目标值
    // @param 目标值
    // @return 0 for ok, -1 for error
    int set_retrans(std::bitset<64>& message, int retrans);
private:
    // @brief set the specific value in message
    // @param message, struct to be set
    // @param data, data source
    // @param msg_from, The starting position of the set structure
    // @param data_from data_end, Starting and ending position of source data
    template<typename T> 
    void _set(std::bitset<64>& message,
                    const int& msg_from,
                    const T& data,
                    const int& data_from,
                    const int& data_end) {
        for (int i = data_from, j = msg_from; 
                        i <= data_end && j < message.size(); 
                        i++, j++) {
            message.set(j, data[i]);
        }
    }

    EncodeType _type;
};

} // namespace can
} // namespace adas

#endif
