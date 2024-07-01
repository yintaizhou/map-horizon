#pragma once
// Copyright 2024 Baidu Inc. All Rights Reserved.
// Author: CHEN ShuaiShuai (chenshuaishuai01@baidu.com)

#include <list>
#include <mutex>

namespace adas {
namespace protocol_v2 {

class Adasv2Channel {
public:
    Adasv2Channel() {}
    ~Adasv2Channel() {}

    void clear() {
        _buffer_mutex.lock();
        list0.clear();
        list1.clear();
        _buffer_mutex.unlock();
    }

    void push_list0(const std::string& ehp_json) {
        _buffer_mutex.lock();
        list0.push_back(ehp_json);
        _buffer_mutex.unlock();
    }

    void push_list1(const std::string& ehp_json) {
        _buffer_mutex.lock();
        list1.push_back(ehp_json);
        _buffer_mutex.unlock();
    }

    int pop(std::string& ehp_json) {
        std::lock_guard<std::mutex> lock(_buffer_mutex);
        if (0 != list0.size()) {
            ehp_json = list0.front();
            list0.pop_front();
            return 0;
        }

        if (0 != list1.size()) {
            ehp_json = list1.front();
            list1.pop_front();
            return 1;
        }

        return -1;
    }

private:
    std::mutex _buffer_mutex;
    // stub
    std::list<std::string> list0;

    // segment, profileshort, profilelong
    std::list<std::string> list1;
}; // class Adasv2Channel

} // namespace protocol_v2
} // namespace adas