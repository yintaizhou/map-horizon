#pragma once
// Copyright 2024 Baidu Inc. All Rights Reserved.
// Author: CHEN ShuaiShuai (chenshuaishuai01@baidu.com)

#include <vector>

namespace adas {
namespace protocol_v2 {

struct Coord {
    double x = 0.0;
    double y = 0.0;
};

struct LocInfo {
    Coord coord;

    double speed = 0.0; // unit m/s
    double direction = 0.0; // gps方向
    int64_t timestamp = 0;
    double probability = 100.0;

    int64_t link_id = 0;
    int64_t link_index = 0; // 当前link位于path的索引
    double link_offset = 0.0; // 距离 link起点的距离，unit m
    double link_direction = 0.0; // link方向和正北方向的夹角

    std::string navi_route_id = ""; // 当前位置点所属的导航路线的唯一id
};

struct PathInfo {
    uint64_t path_id = 0;
    uint64_t sub_path_id = 0;
    double offset = 0.0;
    int type = 0;
    bool is_complex_intersection = 0;
    uint8_t relative_probability = 0.0;
    bool part_of_calculated_route = 0;
    bool is_last_stub_at_offset = 0;
    double turn_angle = 0.0;
    int pathclass = 0;
    int lanenums2e = 0;
    int lanenume2s = 0;
    int right_of_way = 0;
    int form_of_way = 0;
};

struct LinkInfo {
    uint64_t path_id = 0;
    uint64_t linkid = 0;
    int64_t link_index = 0; // 当前link位于path的索引
    double offset = 0.0; // 当前link位于path的offset
    double length = 0.0;
    double distance_to_pos = 0.0;
    uint8_t  road_grade = 0;
    uint8_t  pathclass = 1;
    std::vector<int> kinds;
    uint8_t  uflag = 1;
    uint8_t  lanenums2e = 0;
    uint8_t  lanenume2s = 0;
    uint8_t  speed_limit = 0;
    uint8_t  speed_limit_type = 0;
    int form_of_way = 0;

    std::vector<Coord> shapes;

    bool complex_intersection = false;
    uint8_t relative_probability = 0;
    bool part_of_calculated_route = false;
};

struct PositionMessage {
    int type = 1;
    int cyclic_counter = 0;
    int path_index = 0;
    int offset = 0;
    int position_index = 0; // 
    int position_age = 0;
    int speed = 0;
    int relative_heading = 0;
    int position_probability = 30; // 
    int position_confidence = 0; // 
    int current_lane = 0; // 
    int reserved = 0; // 
};

struct SegmentMessage {
    int type = 2;
    int cyclic_counter = 0;
    int retrans = 0; // 
    int path_index;
    int offset;
    int update = 0; //
    int functional_road_class;
    int form_of_way;
    int effective_speed_limit;
    int effective_speed_limit_type;
    int number_of_lanes_in_driving_direction = 7;
    int number_of_lanes_in_opposite_direction = 7; // 
    int tunnel = 0;
    int bridge = 0;
    int divided_road = 0;
    int built_up_area = 2;
    int complex_intersection = 0;
    int relative_probability = 0;
    int part_of_calculated_route = 0;
    int reserved = 0; // 
    int64_t link_id = 0;
};

struct StubMessage {
    int type = 3;
    int cyclic_counter = 0;
    int retrans = 0; // 
    int path_index;
    int offset;
    int update = 0; // 
    int sub_path_index;
    int turn_angle;
    int relative_probability;
    int functional_road_class;
    int form_of_way;
    int number_of_lanes_in_driving_direction;
    int number_of_lanes_in_opposite_direction; // 
    int complex_intersection; // 
    int right_of_way = 0; // 
    int part_of_calculated_route; // 
    int last_stub_at_offset; //

    std::vector<Coord> coords;
};

struct profile_long_traffic_sign {
    int sign_type = 0;
    int value = 0;
    int lane = 15;
    int vehicle_specific = 0;
    int time_specific = 0;
    int condition = 15;
    int sign_location = 0;
};

struct ProfileLongMessage {
    int type = 5;
    int cyclic_counter = 0;
    int retrans = 0;
    int path_index;
    int offset;
    int update = 0;
    int profile_type;
    int control_point = 0;
    uint32_t value = 0; // 1 || 2 == profile_type
    profile_long_traffic_sign traffic_sign; // only 8 == profile_type
};

struct ProfileShortMessage {
    int type = 4;
    int cyclic_counter = 0;
    int retrans = 0; // 
    int path_index;
    int offset;
    int update = 0; //
    int profile_type;
    int control_point = 0; // 
    int value0;
    int distance1 = 0;
    int value1 = 0;
    int accuracy = 0; // 
};

} // namespace protocol_v2
} // namespace adas