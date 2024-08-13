/***************************************************************************
 *
 * Copyright (c) 2021 Baidu.com, Inc. All Rights Reserved
 * $Id$
 *
 **************************************************************************/
/**
 * @file can_protocol.cpp
 * @author css01
 * @date 2021/03/05 15:04:01
 * @brief can_protocol implementation
 *
 **/

#include "can_protocol.h"
#include "can_message.h"

namespace adas {
namespace can {

void CanProtocol::set_encode_type(const EncodeType& type) {
    _type = type;
}

int CanProtocol::set_cyccnt(std::bitset<64>& message, int cyccnt) {
    std::bitset<2> cyc(cyccnt);
    _set(message, get_start_index(22), cyc, 0, cyc.size() - 1);
    return 0;
}

int CanProtocol::set_retrans(std::bitset<64>& message, int retrans) {
    if (0 != retrans && 1 != retrans) {
        return -1;
    }
    std::bitset<1> retr(retrans);
    _set(message, get_start_index(25), retr, 0, retr.size() - 1);
    return 0;
}

// intel startbit = motorola startbit + 56 - 16 * floor(motorola startbit / 8)
int CanProtocol::get_start_index(int index) {
    if (_type == MOTOROLA) {
        return index;
    }
    int intel = index + 56 - (16 * (index / 8));
    return intel;
}

int CanProtocol::reorganize_position(const PositionMessage& position, std::bitset<64>& message) {
    // Response response;
    if (position.type != 1) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }
    // std::bitset<64> message;
    std::bitset<3>  type(position.type);
    std::bitset<13> offset(position.offset);
    std::bitset<2>  cyccnt(position.cyclic_counter);
    std::bitset<6>  pathidx(position.path_index);
    std::bitset<2>  idx(position.position_index);
    std::bitset<5>  posprobbb(position.position_probability);
    std::bitset<9>  age(position.position_age);
    std::bitset<3>  posconfdc(position.position_confidence);
    std::bitset<3>  curlane(position.current_lane);
    std::bitset<9>  spd(position.speed);
    std::bitset<8>  relhead(position.relative_heading);
    _set(message, get_start_index(0), offset, 8, offset.size() - 1);
    _set(message, get_start_index(5), type, 0, type.size() - 1);
    _set(message, get_start_index(8), offset, 0, 7);
    _set(message, get_start_index(16), pathidx, 0, pathidx.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), age, 8, 8);
    _set(message, get_start_index(25), posprobbb, 0, posprobbb.size() - 1);
    _set(message, get_start_index(30), idx, 0, idx.size() - 1);
    _set(message, get_start_index(32), age, 0, 7);
    _set(message, get_start_index(40), spd, 8, 8);
    _set(message, get_start_index(41), curlane, 0, curlane.size() - 1);
    _set(message, get_start_index(44), posconfdc, 0, posconfdc.size() - 1);
    _set(message, get_start_index(48), spd, 0, 7);
    _set(message, get_start_index(56), relhead, 0, relhead.size() - 1);
    // _write(message, response);
    return 0;
}
int CanProtocol::reorganize_stub(const StubMessage& stub, std::bitset<64>& message) {
    // Response response;
    if (stub.type != 3) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }
    // std::bitset<64> message;
    std::bitset<3>  msgtype(stub.type);
    std::bitset<13> offset(stub.offset);
    std::bitset<6>  pathidx(stub.path_index);
    std::bitset<2>  cyccnt(stub.cyclic_counter);
    std::bitset<1>  update(stub.update);
    std::bitset<1>  retr(stub.retrans);
    std::bitset<6>  subpathidx(stub.sub_path_index);
    std::bitset<3>  funcroadclass(stub.functional_road_class);
    std::bitset<5>  relprobb(stub.relative_probability);
    std::bitset<4>  formofway(stub.form_of_way);
    std::bitset<2>  cmplxinsct(stub.complex_intersection);
    std::bitset<2>  partofcalcroute(stub.part_of_calculated_route);
    std::bitset<8>  turnangle(stub.turn_angle);
    std::bitset<3>  numoflanedrvdir(stub.number_of_lanes_in_driving_direction);
    std::bitset<2>  numoflaneoppdir(stub.number_of_lanes_in_opposite_direction);
    std::bitset<2>  rtofway(stub.right_of_way);
    std::bitset<1>  laststub(stub.last_stub_at_offset);
    _set(message, get_start_index(0), offset, 8, offset.size() - 1);
    _set(message, get_start_index(5), msgtype, 0, msgtype.size() - 1);
    _set(message, get_start_index(8), offset, 0, 7);
    _set(message, get_start_index(16), pathidx, 0, pathidx.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), update, 0, update.size() - 1);
    _set(message, get_start_index(25), retr, 0, retr.size() - 1);
    _set(message, get_start_index(26), subpathidx, 0, subpathidx.size() - 1);
    _set(message, get_start_index(32), funcroadclass, 0, funcroadclass.size() - 1);
    _set(message, get_start_index(35), relprobb, 0, relprobb.size() - 1);
    _set(message, get_start_index(40), formofway, 0, formofway.size() - 1);
    _set(message, get_start_index(44), cmplxinsct, 0, cmplxinsct.size() - 1);
    _set(message, get_start_index(46), partofcalcroute, 0, partofcalcroute.size() - 1);
    _set(message, get_start_index(48), turnangle, 0, turnangle.size() - 1);
    _set(message, get_start_index(56), numoflanedrvdir, 0, numoflanedrvdir.size() - 1);
    _set(message, get_start_index(59), numoflaneoppdir, 0, numoflaneoppdir.size() - 1);
    _set(message, get_start_index(61), rtofway, 0, rtofway.size() - 1);
    _set(message, get_start_index(63), laststub, 0, laststub.size() - 1);
    // _write(message, response);
    return 0;
}
int CanProtocol::reorganize_segment(const SegmentMessage& segment, std::bitset<64>& message) {
    // Response response;
    if (segment.type != 2) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }
    // std::bitset<64> message;
    std::bitset<3>  msgtype(segment.type);
    std::bitset<13> offset(segment.offset);
    std::bitset<6>  pathidx(segment.path_index);
    std::bitset<2>  cyccnt(segment.cyclic_counter);
    std::bitset<1>  update(segment.update);
    std::bitset<1>  retr(segment.retrans);
    std::bitset<2>  builduparea(segment.built_up_area);
    std::bitset<2>  bridge(segment.bridge);
    std::bitset<2>  tunnel(segment.tunnel);
    std::bitset<3>  funcroadclass(segment.functional_road_class);
    std::bitset<5>  relprobb(segment.relative_probability);
    std::bitset<4>  formofway(segment.form_of_way);
    std::bitset<2>  cmplxinsct(segment.complex_intersection);
    std::bitset<2>  partofcalcroute(segment.part_of_calculated_route);
    std::bitset<3>  effspdlmttype(segment.effective_speed_limit_type);
    std::bitset<5>  effspdlmt(segment.effective_speed_limit);
    std::bitset<3>  numoflanedrvdir(segment.number_of_lanes_in_driving_direction);
    std::bitset<2>  numoflaneoppdir(segment.number_of_lanes_in_opposite_direction);
    std::bitset<2>  dividedroad(segment.divided_road);
    _set(message, get_start_index(0), offset, 8, offset.size() - 1);
    _set(message, get_start_index(5), msgtype, 0, msgtype.size() - 1);
    _set(message, get_start_index(8), offset, 0, 7);
    _set(message, get_start_index(16), pathidx, 0, pathidx.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), update, 0, update.size() - 1);
    _set(message, get_start_index(25), retr, 0, retr.size() - 1);
    _set(message, get_start_index(26), builduparea, 0, builduparea.size() - 1);
    _set(message, get_start_index(28), bridge, 0, bridge.size() - 1);
    _set(message, get_start_index(30), tunnel, 0, tunnel.size() - 1);
    _set(message, get_start_index(32), funcroadclass, 0, funcroadclass.size() - 1);
    _set(message, get_start_index(35), relprobb, 0, relprobb.size() - 1);
    _set(message, get_start_index(40), formofway, 0, formofway.size() - 1);
    _set(message, get_start_index(44), cmplxinsct, 0, cmplxinsct.size() - 1);
    _set(message, get_start_index(46), partofcalcroute, 0, partofcalcroute.size() - 1);
    _set(message, get_start_index(48), effspdlmttype, 0, effspdlmttype.size() - 1);
    _set(message, get_start_index(51), effspdlmt, 0, effspdlmt.size() - 1);
    _set(message, get_start_index(56), numoflanedrvdir, 0, numoflanedrvdir.size() - 1);
    _set(message, get_start_index(59), numoflaneoppdir, 0, numoflaneoppdir.size() - 1);
    _set(message, get_start_index(61), dividedroad, 0, dividedroad.size() - 1);
    // _write(message, response);
    return 0;
}
int CanProtocol::reorganize_shortprofile(const ProfileShortMessage& profile, std::bitset<64>& message) {
    // Response response;
    if (profile.type != 4) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }
    // std::bitset<64> message;
    std::bitset<13> offset(profile.offset);
    std::bitset<3>  msgtype(profile.type);
    std::bitset<6>  pathidx(profile.path_index);
    std::bitset<2>  cyccnt(profile.cyclic_counter);
    std::bitset<1>  update(profile.update);
    std::bitset<1>  retr(profile.retrans);
    std::bitset<1>  ctrlpoint(profile.control_point);
    std::bitset<5>  proftype(profile.profile_type);
    std::bitset<10> dist1(profile.distance1);
    std::bitset<2>  accurclass(profile.accuracy);
    std::bitset<10> value0(profile.value0);
    std::bitset<10> value1(profile.value1);
    _set(message, get_start_index(0), offset, 8, offset.size() - 1);
    _set(message, get_start_index(5), msgtype, 0, msgtype.size() - 1);
    _set(message, get_start_index(8), offset, 0, 7);
    _set(message, get_start_index(16), pathidx, 0, pathidx.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), update, 0, update.size() - 1);
    _set(message, get_start_index(25), retr, 0, retr.size() - 1);
    _set(message, get_start_index(26), ctrlpoint, 0, ctrlpoint.size() - 1);
    _set(message, get_start_index(27), proftype, 0, proftype.size() - 1);
    _set(message, get_start_index(32), dist1, 4, dist1.size() - 1);
    _set(message, get_start_index(38), accurclass, 0, accurclass.size() - 1);
    _set(message, get_start_index(40), value0, 6, value0.size() - 1);
    _set(message, get_start_index(44), dist1, 0, 3);
    _set(message, get_start_index(48), value1, 8, value1.size() - 1);
    _set(message, get_start_index(50), value0, 0, 5);
    _set(message, get_start_index(56), value1, 0, 7);
    // _write(message, response);
    return 0;
}
int CanProtocol::reorganize_longprofile(const ProfileLongMessage& profile, std::bitset<64>& message) {
    // Response response;
    if (profile.type != 5) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }

    // std::bitset<64> message;
    std::bitset<13> offset(profile.offset);
    std::bitset<3>  msgtype(profile.type);
    std::bitset<6>  pathidx(profile.path_index);
    std::bitset<2>  cyccnt(profile.cyclic_counter);
    std::bitset<1>  update(profile.update);
    std::bitset<1>  retr(profile.retrans);
    std::bitset<1>  ctrlpoint(profile.control_point);
    std::bitset<5>  proftype(profile.profile_type);
    _set(message, get_start_index(0), offset, 8, offset.size() - 1);
    _set(message, get_start_index(5), msgtype, 0, msgtype.size() - 1);
    _set(message, get_start_index(8), offset, 0, 7);
    _set(message, get_start_index(16), pathidx, 0, pathidx.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), update, 0, update.size() - 1);
    _set(message, get_start_index(25), retr, 0, retr.size() - 1);
    _set(message, get_start_index(26), ctrlpoint, 0, ctrlpoint.size() - 1);
    _set(message, get_start_index(27), proftype, 0, proftype.size() - 1);
    if (profile.profile_type == 9) { // 按照货车限速的格式排列数据
        std::bitset<8> speed(profile.truck_speed.speed);
        std::bitset<8> weight(profile.truck_speed.weight);
        std::bitset<3> hazardous_goods(profile.truck_speed.hazardous_goods);
        std::bitset<3> weather_restriction(profile.truck_speed.weather_restriction);
        std::bitset<2> type(profile.truck_speed.type);
        std::bitset<2> time_valid(profile.truck_speed.time_valid);
        std::bitset<1> time_dependent(profile.truck_speed.time_dependent);
        std::bitset<5> reserved(profile.truck_speed.reserved);
        _set(message, get_start_index(32), speed, 0, speed.size() - 1);
        _set(message, get_start_index(40), weight, 0, weight.size() - 1);
        _set(message, get_start_index(48), hazardous_goods, 0, hazardous_goods.size() - 1);
        _set(message, get_start_index(51), weather_restriction, 0, weather_restriction.size() - 1);
        _set(message, get_start_index(54), type, 0, type.size() - 1);
        _set(message, get_start_index(56), time_valid, 0, time_valid.size() - 1);
        _set(message, get_start_index(58), time_dependent, 0, time_dependent.size() - 1);
        _set(message, get_start_index(59), reserved, 0, reserved.size() - 1);
    } else if (profile.profile_type == 13) {
        std::bitset<8> speed(profile.traffic_state.speed);
        std::bitset<8> state(profile.traffic_state.state);
        _set(message, get_start_index(32), speed, 0, speed.size() - 1);
        _set(message, get_start_index(40), state, 0, state.size() - 1);
    } else if (profile.profile_type == 14) {
        std::bitset<16> incident_id(profile.traffic_incident.incident_id);
        std::bitset<2> state(profile.traffic_incident.state);
        std::bitset<2> severity(profile.traffic_incident.severity);
        std::bitset<3> influence(profile.traffic_incident.influence);
        _set(message, get_start_index(32), incident_id, 0, 7);
        _set(message, get_start_index(40), incident_id, 8, incident_id.size() - 1);
        _set(message, get_start_index(48), state, 0, state.size() - 1);
        _set(message, get_start_index(50), severity, 0, severity.size() - 1);
        _set(message, get_start_index(52), influence, 0, influence.size() - 1);
    } else {
        std::bitset<32> value(profile.value);
        _set(message, get_start_index(56), value, 0, 7);
        _set(message, get_start_index(48), value, 8, 15);
        _set(message, get_start_index(40), value, 16, 23);
        _set(message, get_start_index(32), value, 24, 31);
    }
    // _write(message, response);
    return 0;
}
int CanProtocol::reorganize_metadata(const MetaMessage& metadata, std::bitset<64>& message) {
    // Response response;
    if (metadata.type != 6) {
        // _fill_response(response, STATUS_DATA_ERROR);
        return -1;
    }
    // std::bitset<64> message;
    std::bitset<10> countrycode(metadata.country_code);
    std::bitset<3>  mapprovider(metadata.map_provider);
    std::bitset<3>  msgtype(metadata.type);
    std::bitset<9>  hwver(metadata.hardware_version);
    std::bitset<3>  minorsub(metadata.minor_protocol_sub_version);
    std::bitset<2>  majorr(metadata.major_protocol_version);
    std::bitset<2>  cyccnt(metadata.cyclic_counter);
    std::bitset<15> regioncode(metadata.region_code);
    std::bitset<1>  drvside(metadata.driving_side);
    std::bitset<6>  mapveryear(metadata.map_version_y);
    std::bitset<2>  mapverqtr(metadata.map_version_q);
    std::bitset<4>  minorr(metadata.minor_protocol_version);
    std::bitset<1>  spdunits(metadata.speed_units);
    _set(message, get_start_index(0), countrycode, 8, 9);
    _set(message, get_start_index(2), mapprovider, 0, mapprovider.size() - 1);
    _set(message, get_start_index(5), msgtype, 0, msgtype.size() - 1);
    _set(message, get_start_index(8), countrycode, 0, 7);
    _set(message, get_start_index(16), hwver, 8, 8);
    _set(message, get_start_index(17), minorsub, 0, minorsub.size() - 1);
    _set(message, get_start_index(20), majorr, 0, majorr.size() - 1);
    _set(message, get_start_index(22), cyccnt, 0, cyccnt.size() - 1);
    _set(message, get_start_index(24), hwver, 0, 7);
    _set(message, get_start_index(32), regioncode, 8, 14);
    _set(message, get_start_index(39), drvside, 0, drvside.size() - 1);
    _set(message, get_start_index(40), regioncode, 0, 7);
    _set(message, get_start_index(48), mapveryear, 0, mapveryear.size() - 1);
    _set(message, get_start_index(54), mapverqtr, 0, mapverqtr.size() - 1);
    _set(message, get_start_index(56), minorr, 0, minorr.size() - 1);
    _set(message, get_start_index(60), spdunits, 0, spdunits.size() - 1);
    // _write(message, response);
    return 0;
}

} // namespace can
} // namespace adas
