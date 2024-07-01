#pragma once
// Copyright 2024 Baidu Inc. All Rights Reserved.
// Author: CHEN ShuaiShuai (chenshuaishuai01@baidu.com)

#include <string>
#include <math.h>
#include <map>

#include "cJSON.h"
#include "adas_v2_type.h"

namespace adas {
namespace protocol_v2 {

std::string log_time();

#define log_tag (log_time() + "[" + std::string(std::string(__FILE__) + ":" + std::to_string(__LINE__)) + "]")

const int EARTH_RADIUS = 6371393; // meter
const double PI = 3.141592653589793238462;

class CJsonSafeDelete {
public:
    CJsonSafeDelete(cJSON* monitor_ptr) : _monitor_ptr(monitor_ptr) {}
    ~CJsonSafeDelete() {
        if (nullptr != _monitor_ptr) {
            cJSON_Delete(_monitor_ptr);
        }
    }
private:
    cJSON* _monitor_ptr;
};

uint64_t get_cur_time_ms();
uint64_t get_cur_time_us();

int normalize_direction(double delta);
int normalize_speed(int speed);
int normalize_form_of_way(const LinkInfo& link_info);

double calculate_distance(double lon1, double lat1, double lon2, double lat2);

template<typename T>
struct sort_help_by_offset {
    bool operator() (const T& a, const T& b) {
        return a.offset < b.offset;
    }
};

template<typename T>
struct sort_help_by_pathid_offset {
    bool operator() (const T& a, const T& b) {
        if (a.path_index < b.path_index) {
            return true;
        } else if (a.path_index > b.path_index) {
            return false;
        }

        return a.offset < b.offset;
    }
};

static std::map<int, int> baidu_traffic_sign_2_std_protocol {
    {32, 0},
    {29, 1},
    {30, 2},
    {31, 3},
    {13, 4},
    {48, 5}, 
    {1, 9},
    {2, 10},
    {47, 13},
    {3, 14},
    {4, 17},
    {10, 26},
    {23, 28},
    {21, 29},
    {20, 30},
    {42, 32},
    {40, 33},
    {7, 40},
    {8, 41},
    {9, 42},
    {38, 43},
    {39, 44},
    {33, 46},
    {34, 47},
    {12, 52},
    {11, 55},
    {26, 56},
    {27, 57},
    {15, 61},
    {14, 62},
    {17, 66},
    {5, 67},
    {6, 68},
    {24, 69},
    {16, 74},
    {28, 76},
    {41, 82}
};

} // namespace protocol_v2
} // namespace adas