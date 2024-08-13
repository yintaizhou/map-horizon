// Adas map message
// Copyright (c) 2021 Baidu.com, Inc. All Rights Reserved
//
//   Autor    : yintaizou
//   Email    : yintaizhou@baidu.com
//   Wiki     : http://wiki.baidu.com/pages/viewpage.action?pageId=1380533306
//   Date     : 2021.02.25
#ifndef  ADAS_HORIZON_PROVIDER_MESSAGE_H  
#define  ADAS_HORIZON_PROVIDER_MESSAGE_H

#include <string>

namespace adas {
namespace can {

enum EncodeType {
    INTEL = 1,
    MOTOROLA = 2,
};

// POSITION message
struct PositionMessage {
    int type;
    int cyclic_counter;
    int path_index;
    int offset;
    int position_index;
    int position_age;
    int speed;
    int relative_heading;
    int position_probability;
    int position_confidence;
    int current_lane;
};

// STUB message
struct StubMessage {
    int type;
    int cyclic_counter;
    // 
    int retrans;
    int path_index;
    int offset;
    int update;
    int sub_path_index;
    int turn_angle;
    int relative_probability;
    int functional_road_class;
    int form_of_way;
    int number_of_lanes_in_driving_direction;
    int number_of_lanes_in_opposite_direction;
    int complex_intersection;
    int right_of_way;
    int part_of_calculated_route;
    int last_stub_at_offset;
};

// PROFILE SHORT message
struct ProfileShortMessage {
    int type;
    int cyclic_counter;
    // 
    int retrans;
    int path_index;
    int offset;
    int update;
    int profile_type;
    int control_point;
    int value0;
    int distance1;
    int value1;
    int accuracy;
};

// one of PROFILE LONG message                                                                         
struct TruckSpeed {                                                                                    
    int speed = 0;
    int weight = 0;
    int type = 0;
    int weather_restriction = 0;
    int hazardous_goods = 0;
    int reserved = 0;
    int time_dependent = 0;
    int time_valid = 0;
};

struct TrafficState {
    int speed = 0;
    int state = 0;
};

struct TrafficIncident {
    int incident_id = 0;
    int state = 0;
    int severity = 0;
    int influence = 0;
};

// PROFILE LONG message
struct ProfileLongMessage {
    int type;
    int cyclic_counter;
    int retrans;
    int path_index = 0;
    int offset = 0;
    int update = 0;
    int profile_type = 0;
    int control_point = 0;
    uint32_t value = 0;
    TruckSpeed truck_speed;
    TrafficState traffic_state;
    TrafficIncident traffic_incident;
};

// META message 
struct MetaMessage {
    int type;
    int cyclic_counter;
    int country_code;
    int region_code;
    int driving_side;
    int speed_units;
    int major_protocol_version;
    int minor_protocol_version;
    int minor_protocol_sub_version;
    int hardware_version;
    int map_provider;
    int map_version_y;
    int map_version_q;
};

// SEGEMTNET message
struct SegmentMessage {
    int type;
    int cyclic_counter;
    // 
    int retrans;
    int path_index;
    int offset;
    int update;
    int functional_road_class;
    int form_of_way;
    int effective_speed_limit;
    int effective_speed_limit_type;
    int number_of_lanes_in_driving_direction;
    int number_of_lanes_in_opposite_direction;
    int tunnel;
    int bridge;
    int divided_road;
    int built_up_area;
    int complex_intersection;
    int relative_probability;
    int part_of_calculated_route;
};

struct MessageSent {
    MessageSent() : timestamp(0), message("") {};
    int64_t timestamp;
    std::string message;
};

struct Response {
    int status;
    std::string message;
};

} // namespace can 
} // namespace adas
#endif // ADAS_HORIZON_PROVIDER_MESSAGE_H
/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
