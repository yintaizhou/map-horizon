#include <iostream>
#include <algorithm>
#include <sys/time.h>
#include <chrono>
#include <thread>

#include "adas_v2_protocol.h"

namespace adas {
namespace protocol_v2 {

#define LOG(msg) _log(log_tag + msg)

const int PATH_MAX_DISTANCE = 8191;
const int POSITION_MAX_INTERVAL = 100;

std::atomic<int> position_cyclic(0);
std::atomic<int> profilelong_cyclic(0);
std::atomic<int> profileshort_cyclic(0);
std::atomic<int> stub_cyclic(0);
std::atomic<int> segment_cyclic(0);

void AdasV2Protocol::input_loc(const LocInfo& loc) {
    std::lock_guard<std::mutex> guard(public_func_mutex);

    LOG("input loc");

    if (loc.navi_route_id != _navi_route_id) {
        LOG("input loc navi_route_id:" + loc.navi_route_id + 
            " _navi_route_id:" + _navi_route_id);
        return;
    }

    const std::vector<LinkInfo> main_path_linkinfos = _link_infos[8];
    LOG("input loc main_path_linkinfos size:" + std::to_string(main_path_linkinfos.size()));
    for (size_t i = 0; i < main_path_linkinfos.size(); i++) {
        LOG("input loc linkid:" + std::to_string(loc.link_id)
            + " linkindex:" + std::to_string(loc.link_index)
            + " i:" + std::to_string(i) + " main_path_linkindex:" + std::to_string(main_path_linkinfos[i].link_index)
            + " main_path_linkid:" + std::to_string(main_path_linkinfos[i].linkid));

        if (loc.link_index == main_path_linkinfos[i].link_index &&
                loc.link_id == main_path_linkinfos[i].linkid) {
            PositionMessage position_message;
            position_message.path_index = 8;
            position_message.offset = main_path_linkinfos[i].offset + loc.link_offset;

            if (0 == loc.link_index) {
                double linklength_diff =  main_path_linkinfos[i].length - _navi_link_id_2_length[0][loc.link_id];
                if (0 < linklength_diff && (fabs(_navi_link_id_2_length[0][loc.link_id] - 0) > 1e-6)) {
                    position_message.offset += linklength_diff;
                }
            }

            uint64_t gps_loc_time = loc.timestamp;
            position_message.position_age = get_cur_time_ms() - gps_loc_time;
            if (position_message.position_age < 0) {
                position_message.position_age = 511;
            } else if (position_message.position_age > 2545) {
                position_message.position_age = 510;
            } else {
                position_message.position_age /= 5;
            }

            double speed = loc.speed;
            position_message.speed = 64 + (speed / 0.2);

            position_message.position_probability = 30;
            position_message.relative_heading = normalize_direction(loc.direction - loc.link_direction);

            int64_t start = get_cur_time_ms();
            _update_position_cache(position_message, "local_position");
            int64_t end = get_cur_time_ms();

            LOG("input loc update, linkid:" + std::to_string(loc.link_id) + 
                        " linkoffset:" + std::to_string(loc.link_offset) + 
                        " pathoffset:" + std::to_string(position_message.offset) +
                        " cost_ms:" + std::to_string(end - start));

            return;
        }
    }

    LOG("input loc update failed linkid:" + std::to_string(loc.link_id)
        + " linkindex:" + std::to_string(loc.link_index));
}

void AdasV2Protocol::input_ehp_info(const std::string& ehp_info) {
    std::lock_guard<std::mutex> guard(public_func_mutex);

    cJSON* monitor_json = cJSON_Parse(ehp_info.c_str());
    if (nullptr == monitor_json) {
        LOG("parse ehp_info json failed. " + ehp_info);
        return;
    }

    CJsonSafeDelete safe_delete(monitor_json);

    cJSON *cjson_navi_route_id_ptr = cJSON_GetObjectItem(monitor_json, "route_id");
    if (cJSON_IsString(cjson_navi_route_id_ptr)) {
        std::string navi_route_id = cjson_navi_route_id_ptr->valuestring;
        if (navi_route_id != _navi_route_id) {
            return;
        }
    }

    cJSON *cjson_version_ptr = cJSON_GetObjectItem(monitor_json, "version");
    if (cJSON_IsNumber(cjson_version_ptr)) {
        int64_t version = cjson_version_ptr->valuedouble;
        LOG("ehp version. cur_version:" + std::to_string(_ehp_version) + 
                            " update_version:" + std::to_string(version));
        if (_ehp_version != version) {
            _ehp_version = version;
            _buffer_channel.clear();
            _seted_position_message = false;

            pthread_mutex_lock(&_position_message_mutex);
            _position_message_cache = PositionMessage();
            pthread_mutex_unlock(&_position_message_mutex);

            _path_infos.clear();
            _link_infos.clear();

            _sended_max_segment_link_index.clear();
            _sended_max_lonlat_link_index.clear();
            _sended_max_slope_offset.clear();
            _sended_max_curvature_offset.clear();
            _sended_max_traffic_light_offset.clear();
            _sended_max_warning_info_offset.clear();

            _sended_max_mainpath_offset = -1;
            _sended_max_subpath_offset = -1;

            _send_invalid_position_message();
            _send_invalid_stub_message();
        }
    }

    cJSON *cjson_position_ptr = cJSON_GetObjectItem(monitor_json, "position");
    _process_position(cjson_position_ptr);

    cJSON *cjson_links_ptr = cJSON_GetObjectItem(monitor_json, "link");
    _parse_link_info(cjson_links_ptr);
    _send_segment();

    cJSON *cjson_paths_ptr = cJSON_GetObjectItem(monitor_json, "path");
    _parse_path_info(cjson_paths_ptr);
    _send_stub();

    // _send_lonlat();

    cJSON *cjson_slope_ptr = cJSON_GetObjectItem(monitor_json, "slope");
    _send_slope(cjson_slope_ptr);

    cJSON *cjson_curvature_ptr = cJSON_GetObjectItem(monitor_json, "curvature");
    _send_curvature(cjson_curvature_ptr);

    cJSON *cjson_traffic_light_ptr = cJSON_GetObjectItem(monitor_json, "traffic_light");
    _send_traffic_light(cjson_traffic_light_ptr);

    cJSON *cjson_warning_info_ptr = cJSON_GetObjectItem(monitor_json, "warning_info");
    _send_warning_info(cjson_warning_info_ptr);
}

void AdasV2Protocol::_send_warning_info(cJSON* cjson_warning_info_ptr) {
    if (!cJSON_IsArray(cjson_warning_info_ptr)) {
        LOG("send warning info failed. cjson warning info ptr isnot array");
        return;
    }

    std::vector<ProfileLongMessage> profilelongs;

    size_t warning_infos_size = cJSON_GetArraySize(cjson_warning_info_ptr);
    std::map<int64_t, int64_t> tmp_sended_max_warning_info_offset = _sended_max_warning_info_offset;
    for (size_t i = 0; i < warning_infos_size; i++) {
        cJSON *warning_info_item = cJSON_GetArrayItem(cjson_warning_info_ptr, i);
        cJSON *path_id_ptr = cJSON_GetObjectItemCaseSensitive(warning_info_item, "path_id");
        cJSON *type_code_ptr = cJSON_GetObjectItemCaseSensitive(warning_info_item, "type_code");
        cJSON *offset_ptr = cJSON_GetObjectItemCaseSensitive(warning_info_item, "offset");

        if (!cJSON_IsNumber(path_id_ptr) || !cJSON_IsNumber(type_code_ptr) || !cJSON_IsNumber(offset_ptr)) {
            continue;
        }

        int64_t path_id = path_id_ptr->valuedouble;
        int type_code = type_code_ptr->valueint;
        double offset = offset_ptr->valuedouble;

        if (baidu_traffic_sign_2_std_protocol.end() == baidu_traffic_sign_2_std_protocol.find(type_code)) {
            continue;
        }

        if (_sended_max_warning_info_offset[path_id] >= int64_t(offset * 100)) {
            continue;
        }
        if (tmp_sended_max_warning_info_offset[path_id] < int64_t(offset * 100)) {
            tmp_sended_max_warning_info_offset[path_id] = int64_t(offset * 100);
        }

        ProfileLongMessage profilelong_item;
        profilelong_item.retrans = 0;
        profilelong_item.path_index = path_id_ptr->valueint;
        profilelong_item.offset = offset_ptr->valueint;
        profilelong_item.update = 0;
        profilelong_item.profile_type = 8;
        profilelong_item.control_point = 0;
        profilelong_item.traffic_sign.sign_type = baidu_traffic_sign_2_std_protocol[type_code];
        profilelongs.push_back(profilelong_item);
    }

    _sended_max_warning_info_offset = tmp_sended_max_warning_info_offset;

    std::sort(profilelongs.begin(), profilelongs.end(), sort_help_by_pathid_offset<ProfileLongMessage>());

    for (size_t i = 0; i < profilelongs.size(); i++) {
        profilelongs[i].cyclic_counter = profilelong_cyclic;
        profilelong_cyclic++;
        profilelong_cyclic = profilelong_cyclic % 4;
    
        profilelongs[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_profilelong_to_json(profilelongs[i]);
        _buffer_channel.push_list1(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_send_traffic_light(cJSON* cjson_traffic_light_ptr) {
    if (!cJSON_IsArray(cjson_traffic_light_ptr)) {
        LOG("send traffic light failed. cjson traffic light ptr isnot array");
        return;
    }

    std::vector<ProfileLongMessage> profilelongs;

    size_t traffic_lights_size = cJSON_GetArraySize(cjson_traffic_light_ptr);
    std::map<int64_t, int64_t> tmp_sended_max_traffic_light_offset = _sended_max_traffic_light_offset;
    for (size_t i = 0; i < traffic_lights_size; i++) {
        cJSON *traffic_light_item = cJSON_GetArrayItem(cjson_traffic_light_ptr, i);
        cJSON *path_id_ptr = cJSON_GetObjectItemCaseSensitive(traffic_light_item, "path_id");
        cJSON *offset_ptr = cJSON_GetObjectItemCaseSensitive(traffic_light_item, "offset");
        if (!cJSON_IsNumber(path_id_ptr) || !cJSON_IsNumber(offset_ptr)) {
            continue;
        }

        int64_t path_id = path_id_ptr->valueint;
        double offset = offset_ptr->valueint;

        if (_sended_max_traffic_light_offset[path_id] >= int(offset * 100)) {
            continue;
        }
        if (tmp_sended_max_traffic_light_offset[path_id] < int64_t(offset * 100)) {
            tmp_sended_max_traffic_light_offset[path_id] = int64_t(offset * 100);
        }

        ProfileLongMessage profilelong_item;
        profilelong_item.retrans = 0;
        profilelong_item.path_index = path_id;
        profilelong_item.offset = offset;
        profilelong_item.update = 0;
        profilelong_item.profile_type = 8;
        profilelong_item.control_point = 0;
        profilelong_item.traffic_sign.sign_type = 254; // 红绿灯

        profilelongs.emplace_back(profilelong_item);
    }

    _sended_max_traffic_light_offset = tmp_sended_max_traffic_light_offset;

    std::sort(profilelongs.begin(), profilelongs.end(), sort_help_by_pathid_offset<ProfileLongMessage>());
    for (size_t i = 0; i < profilelongs.size(); i++) {
        profilelongs[i].cyclic_counter = profilelong_cyclic;
        profilelong_cyclic++;
        profilelong_cyclic = profilelong_cyclic % 4;

        profilelongs[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_profilelong_to_json(profilelongs[i]);
        _buffer_channel.push_list1(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_parse_path_info(cJSON* cjson_paths_ptr) {
    if (!cJSON_IsArray(cjson_paths_ptr)) {
        LOG("parse path topo failed. cjson_paths_ptr isnot a array");
        return;
    }

    _path_infos.clear();

    size_t path_array_size = cJSON_GetArraySize(cjson_paths_ptr);

    for (size_t i = 0; i < path_array_size; i++) {
        cJSON *path_item_ptr = cJSON_GetArrayItem(cjson_paths_ptr, i);
        PathInfo path_info;

        cJSON *id_item_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "id");
        if (!cJSON_IsNumber(id_item_ptr)) {
            continue;
        }
        path_info.sub_path_id = id_item_ptr->valueint;

        cJSON *pid_item_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "pid");
        if (!cJSON_IsNumber(pid_item_ptr)) {
            continue;
        }
        path_info.path_id = pid_item_ptr->valueint;
        if (8 != path_info.path_id) {
            continue;
        }

        cJSON *offset_item_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "offset");
        if (!cJSON_IsNumber(offset_item_ptr)) {
            continue;
        }
        path_info.offset = offset_item_ptr->valuedouble;

        cJSON *complex_intersection_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "is_complex_intersection");
        if (cJSON_IsTrue(complex_intersection_ptr)) {
            path_info.is_complex_intersection = true;
        } else {
            path_info.is_complex_intersection = false;
        }

        cJSON *relative_probability_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "relative_probability");
        if (!cJSON_IsNumber(relative_probability_ptr)) {
            continue;
        }
        path_info.relative_probability = relative_probability_ptr->valueint;

        cJSON *is_part_of_route_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "is_part_of_route");
        if (cJSON_IsTrue(is_part_of_route_ptr)) {
            path_info.part_of_calculated_route = true;
        } else {
            path_info.part_of_calculated_route = false;
        }

        cJSON *last_stub_at_offset_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "is_last_stub_at_offset");
        if (cJSON_IsTrue(last_stub_at_offset_ptr)) {
            path_info.is_last_stub_at_offset = true;
        } else {
            path_info.is_last_stub_at_offset = false;
        }

        cJSON *turn_angle_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "turn_angle");
        if (!cJSON_IsNumber(turn_angle_ptr)) {
            continue;
        }
        path_info.turn_angle = turn_angle_ptr->valuedouble;

        cJSON *functional_road_class_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "pathclass");
        if (!cJSON_IsNumber(functional_road_class_ptr)) {
            continue;
        }
        path_info.pathclass = functional_road_class_ptr->valueint;

        cJSON *lanenums2e_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "lanenums2e");
        if (!cJSON_IsNumber(lanenums2e_ptr)) {
            continue;
        }
        path_info.lanenums2e = lanenums2e_ptr->valueint;
        if (6 <= path_info.lanenums2e) {
            path_info.lanenums2e = 6;
        }

        cJSON *lanenume2s_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "lanenume2s");
        if (!cJSON_IsNumber(lanenume2s_ptr)) {
            continue;
        }
        path_info.lanenume2s = lanenume2s_ptr->valueint;
        if (2 <= path_info.lanenume2s) {
            path_info.lanenume2s = 2;
        }

        cJSON *right_of_way_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "right_of_way");
        if (!cJSON_IsNumber(right_of_way_ptr)) {
            continue;
        }
        path_info.right_of_way = right_of_way_ptr->valueint;

        cJSON *form_of_way_ptr = cJSON_GetObjectItemCaseSensitive(path_item_ptr, "form_of_way");
        if (!cJSON_IsNumber(form_of_way_ptr)) {
            continue;
        }
        path_info.form_of_way = form_of_way_ptr->valueint;
        _path_infos.push_back(path_info);
    }
    LOG("parse path info succ. path size:" + std::to_string(_path_infos.size()));
}

void AdasV2Protocol::_send_stub() {
    std::vector<StubMessage> continue_stub_messages;
    std::vector<StubMessage> sub_stub_messages;

    int64_t tmp_sended_max_mainpath_offset = _sended_max_mainpath_offset;
    int64_t tmp_sended_max_subpath_offset = _sended_max_subpath_offset;
    for (size_t i = 0; i < _path_infos.size(); i++) {
        const PathInfo& path_info = _path_infos[i];
        StubMessage stub_item;

        stub_item.sub_path_index = path_info.sub_path_id;
        stub_item.path_index = path_info.path_id;

        if (6 == stub_item.sub_path_index) {
            if (int64_t(path_info.offset * 100) <= _sended_max_mainpath_offset) {
                continue;
            }
            if (int64_t(path_info.offset * 100) > tmp_sended_max_mainpath_offset) {
                tmp_sended_max_mainpath_offset = path_info.offset * 100;
            }
            // continue stub 用 offset 找link的形点
            const std::vector<LinkInfo> main_path_linkinfos = _link_infos[8];
            double coord_length = 0.0;
            int matched_link = 0;
            for (size_t j = 0; j < main_path_linkinfos.size() && coord_length < 50.0; j++) {
                if (int64_t(main_path_linkinfos[j].offset * 100) >= int64_t(path_info.offset * 100)) {
                    matched_link++;
                    for (size_t k = 0; k < main_path_linkinfos[j].shapes.size() && coord_length < 50.0; k++) {
                        // 不是第一条匹配上的link，第一个点会和前面的点重合，不发
                        if (0 == k && 1 != matched_link) {
                            continue;
                        }
                        stub_item.coords.emplace_back(main_path_linkinfos[j].shapes[k]);
                        if (0 != k) {
                            coord_length += calculate_distance(main_path_linkinfos[j].shapes[k - 1].x,
                                                            main_path_linkinfos[j].shapes[k - 1].y,
                                                            main_path_linkinfos[j].shapes[k].x,
                                                            main_path_linkinfos[j].shapes[k].y);
                        }
                    }
                }
            }
        }
        if (8 < stub_item.sub_path_index) {
            if (int64_t(path_info.offset * 100) <= _sended_max_subpath_offset) { // 发送过了
                continue;
            }
            // 24/03/11 华为要求subpath也一起下发，不用只下发500米
            // if (int64_t(path_info.offset) >= _position_message_cache.offset + 500) { // 超过500米了
            //     continue;
            // }
            if (int64_t(path_info.offset * 100) > tmp_sended_max_subpath_offset) {
                tmp_sended_max_subpath_offset = path_info.offset * 100;
            }
            // sub stub用pathid 直接匹配link的形点
            if (_link_infos.end() != _link_infos.find(path_info.sub_path_id) && 
                    0 != _link_infos[path_info.sub_path_id].size()) {
                double coord_length = 0.0;
                for (size_t k = 0; k < _link_infos[path_info.sub_path_id][0].shapes.size(); k++) {
                    stub_item.coords.emplace_back(_link_infos[path_info.sub_path_id][0].shapes[k]);
                    if (0 != k) {
                        coord_length += calculate_distance(_link_infos[path_info.sub_path_id][0].shapes[k - 1].x,
                                                        _link_infos[path_info.sub_path_id][0].shapes[k - 1].y,
                                                        _link_infos[path_info.sub_path_id][0].shapes[k].x,
                                                        _link_infos[path_info.sub_path_id][0].shapes[k].y);
                    }
                    if (coord_length > 50.0) {
                        break;
                    }
                }

            }
        }
        stub_item.offset = path_info.offset;
        stub_item.complex_intersection = path_info.is_complex_intersection;
        stub_item.relative_probability = path_info.relative_probability;
        stub_item.part_of_calculated_route = path_info.part_of_calculated_route;
        stub_item.last_stub_at_offset = path_info.is_last_stub_at_offset;
        stub_item.turn_angle = normalize_direction(path_info.turn_angle);
        stub_item.functional_road_class = path_info.pathclass;
        stub_item.number_of_lanes_in_driving_direction = path_info.lanenums2e;
        stub_item.number_of_lanes_in_opposite_direction = path_info.lanenume2s;
        stub_item.right_of_way = path_info.right_of_way;
        stub_item.form_of_way = path_info.form_of_way;

        if (6 == stub_item.sub_path_index) {
            continue_stub_messages.push_back(stub_item);
        } else {
            sub_stub_messages.push_back(stub_item);
        }
    }

    _sended_max_mainpath_offset = tmp_sended_max_mainpath_offset;
    _sended_max_subpath_offset = tmp_sended_max_subpath_offset;

    std::sort(continue_stub_messages.begin(), continue_stub_messages.end(), sort_help_by_offset<StubMessage>());
    std::sort(sub_stub_messages.begin(), sub_stub_messages.end(), sort_help_by_offset<StubMessage>());

    LOG("send continue stub size:" + std::to_string(continue_stub_messages.size()) + 
        " sub stub size:" + std::to_string(sub_stub_messages.size()));

    for (size_t i = 0; i < continue_stub_messages.size(); i++) {
        continue_stub_messages[i].cyclic_counter = stub_cyclic;
        stub_cyclic++;
        stub_cyclic = stub_cyclic % 4;

        continue_stub_messages[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_stub_to_json(continue_stub_messages[i]);
        _buffer_channel.push_list0(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }

    for (size_t i = 0; i < sub_stub_messages.size(); i++) {
        sub_stub_messages[i].cyclic_counter = stub_cyclic;
        stub_cyclic++;
        stub_cyclic = stub_cyclic % 4;

        sub_stub_messages[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_stub_to_json(sub_stub_messages[i]);
        _buffer_channel.push_list0(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_send_curvature(cJSON* cjson_curvature_ptr) {
    if (!cJSON_IsArray(cjson_curvature_ptr)) {
        LOG("send slope failed. cjson curvature ptr isnot array");
        return;
    }

    std::vector<ProfileShortMessage> profileshorts;

    size_t curvature_array_size = cJSON_GetArraySize(cjson_curvature_ptr);
    std::map<int64_t, int64_t> tmp_sended_max_curvature_offset = _sended_max_curvature_offset;
    for (size_t i = 0; i < curvature_array_size; i++) {
        cJSON *curvature_item_ptr = cJSON_GetArrayItem(cjson_curvature_ptr, i);

        cJSON *path_id_item_ptr = cJSON_GetObjectItemCaseSensitive(curvature_item_ptr, "path_id");
        cJSON *offset_array_ptr = cJSON_GetObjectItemCaseSensitive(curvature_item_ptr, "offset");
        cJSON *stop_array_ptr = cJSON_GetObjectItemCaseSensitive(curvature_item_ptr, "step");

        if (!cJSON_IsNumber(path_id_item_ptr) || !cJSON_IsArray(offset_array_ptr) || !cJSON_IsArray(stop_array_ptr)) {
            continue;
        }

        int path_id = path_id_item_ptr->valueint;
        size_t offset_array_size = cJSON_GetArraySize(offset_array_ptr);
        size_t stop_array_size = cJSON_GetArraySize(stop_array_ptr);
        if (stop_array_size != offset_array_size) {
            continue;
        }

        if (_sended_max_curvature_offset.end() == _sended_max_curvature_offset.find(path_id)) {
            _sended_max_curvature_offset[path_id] = -1;
            tmp_sended_max_curvature_offset[path_id] = -1;
        }

        for (size_t j = 0; j < offset_array_size; j++) {
            ProfileShortMessage profileshort_item;
            profileshort_item.path_index = path_id;
            double offset = cJSON_GetArrayItem(offset_array_ptr, j)->valueint;

            if (int64_t(offset * 100) <= _sended_max_curvature_offset[path_id]) {
                continue;
            }

            profileshort_item.value0 = cJSON_GetArrayItem(stop_array_ptr, j)->valueint;
            profileshort_item.offset = offset;
            profileshort_item.profile_type = 1;

            size_t k = j + 1;
            if (k < offset_array_size) {
                int value1 = cJSON_GetArrayItem(stop_array_ptr, k)->valueint;
                profileshort_item.value1 = value1;
                if (1023 == value1) {
                    profileshort_item.distance1 = 1023;
                    LOG("curvature profile short item. distance1 is 1023");
                } else {
                    profileshort_item.distance1 = cJSON_GetArrayItem(offset_array_ptr, k)->valueint - 
                                                                            profileshort_item.offset;
                }
            } else {
                continue;
            }

            if (tmp_sended_max_curvature_offset[path_id] < int64_t(offset * 100)) {
                tmp_sended_max_curvature_offset[path_id] = int64_t(offset * 100);
            }
            profileshorts.emplace_back(profileshort_item);
        }
    }

    _sended_max_curvature_offset = tmp_sended_max_curvature_offset;

    std::sort(profileshorts.begin(), profileshorts.end(), sort_help_by_pathid_offset<ProfileShortMessage>());
    for (size_t i = 0; i < profileshorts.size(); i++) {
        profileshorts[i].cyclic_counter = profileshort_cyclic;
        profileshort_cyclic++;
        profileshort_cyclic = profileshort_cyclic % 4;

        profileshorts[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_profileshort_to_json(profileshorts[i]);
        _buffer_channel.push_list1(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_send_slope(cJSON* cjson_slope_ptr) {
    if (!cJSON_IsArray(cjson_slope_ptr)) {
        LOG("send slope failed. cjson slope ptr isnot array");
        return;
    }

    std::vector<ProfileShortMessage> profileshorts;

    size_t slope_array_size = cJSON_GetArraySize(cjson_slope_ptr);
    std::map<int64_t, int64_t> tmp_sended_max_slope_offset = _sended_max_slope_offset;
    for (size_t i = 0; i < slope_array_size; i++) {
        cJSON *slope_item_ptr = cJSON_GetArrayItem(cjson_slope_ptr, i);

        cJSON *path_id_item_ptr = cJSON_GetObjectItemCaseSensitive(slope_item_ptr, "path_id");
        cJSON *offset_array_ptr = cJSON_GetObjectItemCaseSensitive(slope_item_ptr, "offset");
        cJSON *stop_array_ptr = cJSON_GetObjectItemCaseSensitive(slope_item_ptr, "step");

        if (!cJSON_IsNumber(path_id_item_ptr) || !cJSON_IsArray(offset_array_ptr) || !cJSON_IsArray(stop_array_ptr)) {
            continue;
        }

        int64_t path_id = path_id_item_ptr->valuedouble;
        size_t offset_array_size = cJSON_GetArraySize(offset_array_ptr);
        size_t stop_array_size = cJSON_GetArraySize(stop_array_ptr);
        if (stop_array_size != offset_array_size) {
            continue;
        }

        if (_sended_max_slope_offset.end() == _sended_max_slope_offset.find(path_id)) {
            _sended_max_slope_offset[path_id] = -1;
            tmp_sended_max_slope_offset[path_id] = -1;
        }

        for (size_t j = 0; j < offset_array_size; j++) {
            ProfileShortMessage profileshort_item;
            profileshort_item.path_index = path_id;
            double offset = cJSON_GetArrayItem(offset_array_ptr, j)->valueint;

            if (int64_t(offset * 100) <= _sended_max_slope_offset[path_id]) {
                continue;
            }

            profileshort_item.value0 = cJSON_GetArrayItem(stop_array_ptr, j)->valueint;
            profileshort_item.offset = offset;
            profileshort_item.profile_type = 4;

            size_t k = j + 1;
            if (k < offset_array_size) {
                int value1 = cJSON_GetArrayItem(stop_array_ptr, k)->valueint;
                profileshort_item.value1 = value1;
                if (1023 == value1) {
                    profileshort_item.distance1 = 1023;
                    LOG("slope profile short item. distance1 is 1023");
                } else {
                    profileshort_item.distance1 = cJSON_GetArrayItem(offset_array_ptr, k)->valueint - 
                                                                            profileshort_item.offset;
                }
            } else {
                continue;
            }

            if (tmp_sended_max_slope_offset[path_id] < int64_t(offset * 100)) {
                tmp_sended_max_slope_offset[path_id] = int64_t(offset * 100);
            }
            profileshorts.emplace_back(profileshort_item);
        }
    }

    _sended_max_slope_offset = tmp_sended_max_slope_offset;

    std::sort(profileshorts.begin(), profileshorts.end(), sort_help_by_pathid_offset<ProfileShortMessage>());
    for (size_t i = 0; i < profileshorts.size(); i++) {
        profileshorts[i].cyclic_counter = profileshort_cyclic;
        profileshort_cyclic++;
        profileshort_cyclic = profileshort_cyclic % 4;

        profileshorts[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_profileshort_to_json(profileshorts[i]);
        _buffer_channel.push_list1(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_parse_link_info(cJSON* cjson_links_ptr) {
    if (!cJSON_IsArray(cjson_links_ptr)) {
        LOG("parse links failed. cjson_links_ptr isnot a array");
        return;
    }

    _link_infos.clear();

    size_t link_array_size = cJSON_GetArraySize(cjson_links_ptr);

    for (size_t i = 0; i < link_array_size; i++) {
        cJSON *link_item_ptr = cJSON_GetArrayItem(cjson_links_ptr, i);

        LinkInfo link_info;
        cJSON *path_id_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "path_id");
        if (!cJSON_IsNumber(path_id_ptr)) {
            continue;
        }
        link_info.path_id = path_id_ptr->valueint;

        cJSON *link_id_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "link_id");
        if (!cJSON_IsNumber(link_id_ptr)) {
            continue;
        }
        link_info.linkid = link_id_ptr->valuedouble;

        cJSON *offset_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "offset");
        if (!cJSON_IsNumber(offset_ptr)) {
            continue;
        }
        link_info.offset = offset_ptr->valuedouble;

        cJSON *length_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "length");
        if (!cJSON_IsNumber(length_ptr)) {
            continue;
        }
        link_info.length = length_ptr->valuedouble;

        cJSON *pathclass_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "pathclass");
        if (!cJSON_IsNumber(pathclass_ptr)) {
            continue;
        }
        link_info.pathclass = pathclass_ptr->valueint;

        cJSON *speed_limit_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "speed_limit");
        if (!cJSON_IsNumber(speed_limit_ptr)) {
            continue;
        }
        link_info.speed_limit = speed_limit_ptr->valueint;

        cJSON *link_index_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "link_index");
        if (!cJSON_IsNumber(link_index_ptr)) {
            continue;
        }
        link_info.link_index = link_index_ptr->valuedouble;

        cJSON *distance_to_pos_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "distance_to_pos");
        if (!cJSON_IsNumber(distance_to_pos_ptr)) {
            continue;
        }
        link_info.distance_to_pos = distance_to_pos_ptr->valuedouble;

        cJSON *form_of_way_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "form_of_way");
        if (!cJSON_IsNumber(form_of_way_ptr)) {
            continue;
        }
        link_info.form_of_way = form_of_way_ptr->valueint;

        cJSON *speed_limit_type_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "speed_limit_type");
        if (!cJSON_IsNumber(speed_limit_type_ptr)) {
            continue;
        }
        link_info.speed_limit_type = speed_limit_type_ptr->valueint;

        cJSON *lanenums2e_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "lanenum");
        if (!cJSON_IsNumber(lanenums2e_ptr)) {
            continue;
        }
        link_info.lanenums2e = lanenums2e_ptr->valueint;
        if (6 <= link_info.lanenums2e) {
            link_info.lanenums2e = 6;
        }

        cJSON *lanenume2s_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "lanenume2s");
        if (!cJSON_IsNumber(lanenume2s_ptr)) {
            continue;
        }
        link_info.lanenume2s = lanenume2s_ptr->valueint;
        if (2 <= link_info.lanenume2s) {
            link_info.lanenume2s = 2;
        }

        cJSON *complex_intersection_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "is_complex_intersection");
        if (cJSON_IsTrue(complex_intersection_ptr)) {
            link_info.complex_intersection = true;
        } else {
            link_info.complex_intersection = false;
        }

        cJSON *relative_probability_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "relative_probability");
        if (!cJSON_IsNumber(relative_probability_ptr)) {
            continue;
        }
        link_info.relative_probability = relative_probability_ptr->valueint;

        cJSON *part_of_calculated_route_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "is_part_of_route");
        if (cJSON_IsTrue(part_of_calculated_route_ptr)) {
            link_info.part_of_calculated_route = true;
        } else {
            link_info.part_of_calculated_route = false;
        }

        cJSON *kinds_array_ptr = cJSON_GetObjectItem(link_item_ptr, "kind");
        if (cJSON_IsArray(kinds_array_ptr)) {
            size_t kind_array_size = cJSON_GetArraySize(kinds_array_ptr);
            for (size_t j = 0; j < kind_array_size; j++) {
                cJSON *kind_item_ptr = cJSON_GetArrayItem(kinds_array_ptr, j);
                if (cJSON_IsNumber(kind_item_ptr)) {
                    link_info.kinds.push_back(kind_item_ptr->valueint);
                }
            }
        }

        cJSON *shape_array_ptr = cJSON_GetObjectItem(link_item_ptr, "shape");
        if (cJSON_IsArray(shape_array_ptr)) {
            size_t shape_array_size = cJSON_GetArraySize(shape_array_ptr);
            for (size_t j = 0; j < shape_array_size; j++) {
                cJSON *shape_item_ptr = cJSON_GetArrayItem(shape_array_ptr, j);
                if (cJSON_IsArray(shape_item_ptr)) {
                    if (2 == cJSON_GetArraySize(shape_item_ptr)) {
                        Coord coord;
                        coord.x = cJSON_GetArrayItem(shape_item_ptr, 0)->valuedouble;
                        coord.y = cJSON_GetArrayItem(shape_item_ptr, 1)->valuedouble;
                        link_info.shapes.push_back(coord);
                    }
                }
            }
        }

        cJSON *uflag_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "uflag");
        if (!cJSON_IsNumber(uflag_ptr)) {
            continue;
        }
        link_info.uflag = uflag_ptr->valueint;

        cJSON *road_grade_ptr = cJSON_GetObjectItemCaseSensitive(link_item_ptr, "road_grade");
        if (!cJSON_IsNumber(road_grade_ptr)) {
            continue;
        }
        link_info.road_grade = road_grade_ptr->valueint;

        _link_infos[link_info.path_id].push_back(link_info);
    }

    for (auto it = _link_infos.begin(); it != _link_infos.end(); it++) {
        std::sort(it->second.begin(), it->second.end(), sort_help_by_offset<LinkInfo>());
    }

    LOG("parse link info succ. link size:" + std::to_string(_link_infos.size()));
}

void AdasV2Protocol::_send_segment() {
    std::vector<SegmentMessage> segment_messages;

    for (auto it = _link_infos.begin(); it != _link_infos.end(); it++) {
        if (it->first >= 64 || it->first < 8) {
            continue;
        }

        int64_t max_sended_link_index = -1;
        if (_sended_max_segment_link_index.end() != _sended_max_segment_link_index.find(it->first)) {
            max_sended_link_index = _sended_max_segment_link_index[it->first];
        }

        for (size_t i = 0; i < it->second.size(); i++) {
            const LinkInfo& link_info = (it->second)[i];
            SegmentMessage segment_item;

            // 用于判断是否发过了
            if (_sended_max_segment_link_index.end() != _sended_max_segment_link_index.find(it->first) &&
                    _sended_max_segment_link_index[it->first] >= link_info.link_index) {
                continue;
            }

            // 用于记录更新本次循环发送过的最大index
            if (link_info.link_index > max_sended_link_index) {
                max_sended_link_index = link_info.link_index;
            }

            if (i != 0 && (int((it->second)[i - 1].offset) == int(link_info.offset))) {
                continue;
            }

            segment_item.path_index = it->first;
            segment_item.offset = link_info.offset;
            segment_item.functional_road_class = link_info.pathclass;
            segment_item.form_of_way = link_info.form_of_way;
            segment_item.effective_speed_limit = normalize_speed(link_info.speed_limit);
            segment_item.effective_speed_limit_type = link_info.speed_limit_type;
            segment_item.number_of_lanes_in_driving_direction = link_info.lanenums2e;
            segment_item.number_of_lanes_in_opposite_direction = link_info.lanenume2s;
            segment_item.complex_intersection = link_info.complex_intersection;
            segment_item.relative_probability = link_info.relative_probability;
            segment_item.part_of_calculated_route = link_info.part_of_calculated_route;
            segment_item.link_id = link_info.linkid;
            for (size_t j = 0; j < link_info.kinds.size(); j++) {
                if ((link_info.kinds[j] & 0xFF) == 0x0f) {
                    segment_item.tunnel = 1;
                }
                if ((link_info.kinds[j] & 0xFF) == 0x08) {
                    segment_item.bridge = 1;
                }
                if ((link_info.kinds[j] & 0xFF) == 0x02) {
                    segment_item.divided_road = 1;
                }
            }
            segment_item.built_up_area = link_info.uflag;
            segment_messages.emplace_back(segment_item);
        }
        if (max_sended_link_index > _sended_max_segment_link_index[it->first]) {
            _sended_max_segment_link_index[it->first] = max_sended_link_index;
        }
    }

    LOG("send segment size:" + std::to_string(segment_messages.size()));

    std::sort(segment_messages.begin(), segment_messages.end(), sort_help_by_pathid_offset<SegmentMessage>());
    for (size_t i = 0; i < segment_messages.size(); i++) {
        segment_messages[i].cyclic_counter = segment_cyclic;
        segment_cyclic++;
        segment_cyclic = segment_cyclic % 4;

        segment_messages[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_segment_to_json(segment_messages[i]);
        _buffer_channel.push_list0(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::_send_lonlat() {
    std::vector<ProfileLongMessage> profilelongs;
    for (auto it = _link_infos.begin(); it != _link_infos.end(); it++) {
        if (it->first >= 64 || it->first < 8) {
            continue;
        }

        const std::vector<LinkInfo>& link_infos = it->second;

        int64_t max_sended_link_index = -1;
        if (_sended_max_lonlat_link_index.end() != _sended_max_lonlat_link_index.find(it->first)) {
            max_sended_link_index = _sended_max_lonlat_link_index[it->first];
        }

        for (size_t i = 0; i < link_infos.size(); i++) {
            const LinkInfo& link_info = link_infos[i];

            if (_sended_max_lonlat_link_index.end() != _sended_max_lonlat_link_index.find(it->first) &&
                    _sended_max_lonlat_link_index[link_info.path_id] >= link_info.link_index) {
                continue;
            }

            if (link_info.link_index > max_sended_link_index) {
                max_sended_link_index = link_info.link_index;
            }

            float point_to_link_start = 0;
            for (size_t j = 0; j < link_info.shapes.size(); j++) {
                const Coord& loc = link_info.shapes[j];

                ProfileLongMessage profilelong_item;
                profilelong_item.retrans = 0;
                profilelong_item.path_index = link_info.path_id;
                profilelong_item.update = 0;
                profilelong_item.cyclic_counter = 0;
                profilelong_item.control_point = 0;
                float offset = link_info.offset;

                if (0 == j && 0 != i) {
                    continue;
                }
                if (0 != j) {
                    point_to_link_start += calculate_distance(
                        link_info.shapes[j - 1].x, link_info.shapes[j - 1].y,
                        loc.x, loc.y);
                    offset += point_to_link_start;
                }

                profilelong_item.offset = offset;
                profilelong_item.profile_type = 1;
                profilelong_item.value = (loc.x + 180.0) * 10000000;
                profilelongs.emplace_back(profilelong_item);

                profilelong_item.profile_type = 2;
                profilelong_item.value = (loc.y + 90.0) * 10000000;
                profilelongs.emplace_back(profilelong_item);
            }
        }

        if (max_sended_link_index > _sended_max_lonlat_link_index[it->first]) {
            _sended_max_lonlat_link_index[it->first] = max_sended_link_index;
        }
    }

    LOG("send lonlat size:" + std::to_string(profilelongs.size()));

    std::sort(profilelongs.begin(), profilelongs.end(), sort_help_by_pathid_offset<ProfileLongMessage>());
    for (size_t i = 0; i < profilelongs.size(); i++) {
        profilelongs[i].cyclic_counter = profilelong_cyclic;
        profilelong_cyclic++;
        profilelong_cyclic = profilelong_cyclic % 4;

        profilelongs[i].offset %= PATH_MAX_DISTANCE;
        std::string ehp_json = _convert_profilelong_to_json(profilelongs[i]);
        _buffer_channel.push_list1(ehp_json);
        pthread_cond_signal(&_adas_message_cond);
    }
}

void AdasV2Protocol::set_navi_route(const std::string& route) {
    std::lock_guard<std::mutex> guard(public_func_mutex);

    LOG("set_navi_route:" + route);

    cJSON* monitor_json = cJSON_Parse(route.c_str());
    if (nullptr == monitor_json) {
        LOG("parse route json failed. " + route);
        return;
    }
    CJsonSafeDelete safe_delete(monitor_json);

    cJSON *cjson_route_ptr = cJSON_GetObjectItem(monitor_json, "route");
    if (!cJSON_IsObject(cjson_route_ptr)) {
        LOG("parse route json failed. " + route);
        return;
    }

    cJSON *cjson_type_ptr = cJSON_GetObjectItem(cjson_route_ptr, "type");
    if (cJSON_IsNumber(cjson_type_ptr)) {
        int type = cjson_type_ptr->valueint;
        if (3 == type) {
            return;
        }
    }

    cJSON *cjson_navi_route_id_ptr = cJSON_GetObjectItem(cjson_route_ptr, "route_id");
    std::string navi_route_id = "";
    if (cJSON_IsString(cjson_navi_route_id_ptr)) {
        navi_route_id = cjson_navi_route_id_ptr->valuestring;
    }

    if (navi_route_id != _navi_route_id) {
        LOG(navi_route_id + " is not equal to _navi_route_id:" + _navi_route_id);
        _navi_route_id = navi_route_id;
        _buffer_channel.clear();
        _seted_position_message = false;
    }

    _navi_link_id_2_length.clear();

    cJSON *cjson_linkids_ptr = cJSON_GetObjectItem(cjson_route_ptr, "linkids");
    cJSON *cjson_linklength_ptr = cJSON_GetObjectItem(cjson_route_ptr, "linklength");
    if (!cJSON_IsArray(cjson_linkids_ptr) || !cJSON_IsArray(cjson_linklength_ptr)) {
        return;
    }

    size_t linkids_size = cJSON_GetArraySize(cjson_linkids_ptr);
    size_t linklength_size = cJSON_GetArraySize(cjson_linklength_ptr);
    if (linkids_size != linklength_size) {
        return;
    }

    for (size_t i = 0; i < linkids_size; i++) {
        int64_t linkid = cJSON_GetArrayItem(cjson_linkids_ptr, i)->valuedouble;
        int64_t linklength = cJSON_GetArrayItem(cjson_linklength_ptr, i)->valuedouble;

        _navi_link_id_2_length[i][linkid] = linklength;
    }
}

void AdasV2Protocol::set_log_file(const std::string& file) {
    std::lock_guard<std::mutex> guard(public_func_mutex);
    if (_run_log_thread) {
        return;
    }

    _log_file = file;
    _run_log_thread = true;
    if (0 != pthread_create(&_async_log_tid, nullptr, (void* (*)(void*))_async_log_pthread, this)) {
        _run_log_thread = false;
    }
    pthread_setname_np(_async_log_tid, "_async_tid");
}

void AdasV2Protocol::close_log() {
    std::lock_guard<std::mutex> guard(public_func_mutex);
    if (_run_log_thread) {
        _run_log_thread = false;
        pthread_cond_signal(&_async_log_cond);
        pthread_join(_async_log_tid, nullptr);
    }
}

void AdasV2Protocol::_log(const std::string& msg) {
    if (_run_log_thread) {
        pthread_mutex_lock(&_async_log_mutex);
        _async_log_buffer.push_back(msg + "\n");
        pthread_cond_signal(&_async_log_cond);
        pthread_mutex_unlock(&_async_log_mutex);
    } else {
        std::cout << msg << std::endl;
    }
}

void AdasV2Protocol::_process_position(cJSON* cjson_position_ptr) {
    PositionMessage position_message;
    if (!cJSON_IsObject(cjson_position_ptr)) {
        LOG("update position cached failed. cjson_position_ptr isnot a object");
        return;
    }

    cJSON *path_id_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "path_id");
    if (!cJSON_IsNumber(path_id_item_ptr)) {
        LOG("update position cached failed. path_id isnot a number");
        return;
    }
    position_message.path_index = path_id_item_ptr->valueint;

    cJSON *offset_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "offset");
    if (!cJSON_IsNumber(offset_item_ptr)) {
        LOG("update position cached failed. offset isnot a number");
        return;
    }
    position_message.offset = offset_item_ptr->valueint;

    cJSON *gps_loc_time_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "gps_loc_time");
    if (!cJSON_IsNumber(gps_loc_time_item_ptr)) {
        LOG("update position cached failed. gps_loc_time isnot a number");
        return;
    }
    uint64_t gps_loc_time = gps_loc_time_item_ptr->valuedouble;
    position_message.position_age = get_cur_time_ms() - gps_loc_time;
    if (position_message.position_age < 0) {
        position_message.position_age = 511;
    } else if (position_message.position_age > 2545) {
        position_message.position_age = 510;
    } else {
        position_message.position_age /= 5;
    }

    cJSON *speed_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "speed");
    if (!cJSON_IsNumber(speed_item_ptr)) {
        LOG("update position cached failed. speed isnot a number");
        return;
    }
    double speed = speed_item_ptr->valuedouble;
    position_message.speed = 64 + (speed / 0.2);

    cJSON *probability_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "probability");
    if (!cJSON_IsNumber(probability_item_ptr)) {
        LOG("update position cached failed. probability isnot a number");
        return;
    }
    position_message.position_probability = probability_item_ptr->valueint;

    cJSON *dir_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "dir");
    cJSON *link_dir_item_ptr = cJSON_GetObjectItemCaseSensitive(cjson_position_ptr, "link_dir");
    if (!cJSON_IsNumber(dir_item_ptr) || !cJSON_IsNumber(link_dir_item_ptr)) {
        LOG("update position cached failed. dir、link_Dir isnot a number");
        return;
    }
    position_message.relative_heading = normalize_direction(dir_item_ptr->valueint - link_dir_item_ptr->valueint);

    int64_t start = get_cur_time_ms();
    _update_position_cache(position_message, "server_position");
    int64_t end = get_cur_time_ms();
    
    LOG("input loc from server update," +
                        " path_index:" + std::to_string(position_message.path_index) + 
                        " path_offset:" + std::to_string(position_message.offset) +
                        " cost_ms:" + std::to_string(end - start));
}

void AdasV2Protocol::_update_position_cache(const PositionMessage& position_message, const std::string& from) {
    int64_t t0 = get_cur_time_ms();
    pthread_mutex_lock(&_position_message_mutex);
    int64_t t1 = get_cur_time_ms();

    // 检查position offset是否发生回退
    if (_position_message_cache.offset > position_message.offset) {
        LOG("update position cache failed. offset back. cache from " + from +
            " last offset:" + std::to_string(_position_message_cache.offset) + 
            " current offset:" + std::to_string(position_message.offset) + 
            " get_lock_time:" + std::to_string(t1 - t0) +
            "func exec time:" + std::to_string(get_cur_time_ms() - t0));
        pthread_mutex_unlock(&_position_message_mutex);
        return;
    }
    int64_t t2 = get_cur_time_ms();
    _position_message_cache = position_message;

    int64_t t3 = get_cur_time_ms();
    _seted_position_message = true;

    int64_t t4 = get_cur_time_ms();
    LOG("update position cache succ. from" + from + 
            " last offset:" + std::to_string(_position_message_cache.offset) + 
            " current offset:" + std::to_string(position_message.offset) + 
            " get_lock_time:" + std::to_string(t1 - t0) +
            " if_condition:" + std::to_string(t2 - t1) + 
            " update_position_cached:" + std::to_string(t3 - t2) + 
            " update_seted_flag:" + std::to_string(t4 - t3) +
            " func exec time:" + std::to_string(t4 - t0));
    pthread_cond_signal(&_position_message_cond);
    pthread_mutex_unlock(&_position_message_mutex);
}

std::string AdasV2Protocol::_convert_segment_to_json(const SegmentMessage& segment_message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Type", cJSON_CreateString("Av2Segment"));

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "MessageType", cJSON_CreateNumber(segment_message.type));
    cJSON_AddItemToObject(data, "CyclicCounter", cJSON_CreateNumber(segment_message.cyclic_counter));
    cJSON_AddItemToObject(data, "Retransmission", cJSON_CreateBool(segment_message.retrans));
    cJSON_AddItemToObject(data, "PathIndex", cJSON_CreateNumber(segment_message.path_index));
    cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(segment_message.offset));
    cJSON_AddItemToObject(data, "Update", cJSON_CreateBool(segment_message.update));
    cJSON_AddItemToObject(data, "FunctionalRoadClass", cJSON_CreateNumber(segment_message.functional_road_class));
    cJSON_AddItemToObject(data, "FormOfWay", cJSON_CreateNumber(segment_message.form_of_way));
    cJSON_AddItemToObject(data, "EffectiveSpeedLimit", cJSON_CreateNumber(segment_message.effective_speed_limit));
    cJSON_AddItemToObject(data, "EffectiveSpeedLimitType", 
                        cJSON_CreateNumber(segment_message.effective_speed_limit_type));
    cJSON_AddItemToObject(data, "NumberOfLane", 
                        cJSON_CreateNumber(segment_message.number_of_lanes_in_driving_direction));
    cJSON_AddItemToObject(data, "NumberOfLaneOpposite", 
                        cJSON_CreateNumber(segment_message.number_of_lanes_in_opposite_direction));
    cJSON_AddItemToObject(data, "Tunnel", cJSON_CreateNumber(segment_message.tunnel));
    cJSON_AddItemToObject(data, "Bridge", cJSON_CreateNumber(segment_message.bridge));
    cJSON_AddItemToObject(data, "DividedRoad", cJSON_CreateNumber(segment_message.divided_road));
    cJSON_AddItemToObject(data, "BuiltupArea", cJSON_CreateNumber(segment_message.built_up_area));
    cJSON_AddItemToObject(data, "ComplexIntersection", cJSON_CreateNumber(segment_message.complex_intersection));
    cJSON_AddItemToObject(data, "RelativeProbability", cJSON_CreateNumber(segment_message.relative_probability));
    cJSON_AddItemToObject(data, "PartOfCalculatedRoute", cJSON_CreateNumber(segment_message.part_of_calculated_route));
    cJSON_AddItemToObject(data, "Reserved", cJSON_CreateNumber(segment_message.complex_intersection));
    cJSON_AddItemToObject(data, "LinkId", cJSON_CreateNumber(segment_message.link_id));
    cJSON_AddItemToObject(root, "Data", data);

    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    std::string res(jsonString);
    free(jsonString);
    return res;
}

std::string AdasV2Protocol::_convert_position_to_json(const PositionMessage& position_message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Type", cJSON_CreateString("Av2Position"));

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "MessageType", cJSON_CreateNumber(position_message.type));
    cJSON_AddItemToObject(data, "CyclicCounter", cJSON_CreateNumber(position_message.cyclic_counter));
    cJSON_AddItemToObject(data, "PathIndex", cJSON_CreateNumber(position_message.path_index));
    cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(position_message.offset));
    cJSON_AddItemToObject(data, "PositionIndex", cJSON_CreateNumber(position_message.position_index));
    cJSON_AddItemToObject(data, "PositionAge", cJSON_CreateNumber(position_message.position_age));
    cJSON_AddItemToObject(data, "Speed", cJSON_CreateNumber(position_message.speed));
    cJSON_AddItemToObject(data, "RelativeHeading", cJSON_CreateNumber(position_message.relative_heading));
    cJSON_AddItemToObject(data, "PositionProbability", cJSON_CreateNumber(position_message.position_probability));
    cJSON_AddItemToObject(data, "PositionConfidence", cJSON_CreateNumber(position_message.position_confidence));
    cJSON_AddItemToObject(data, "CurrentLane", cJSON_CreateNumber(position_message.current_lane));
    cJSON_AddItemToObject(data, "Reserved", cJSON_CreateNumber(position_message.reserved));
    cJSON_AddItemToObject(root, "Data", data);

    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    std::string res(jsonString);
    free(jsonString);
    return res;
}

std::string AdasV2Protocol::_convert_profilelong_to_json(const ProfileLongMessage& profilelong_message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Type", cJSON_CreateString("Av2ProfileLong"));

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "MessageType", cJSON_CreateNumber(profilelong_message.type));
    cJSON_AddItemToObject(data, "CyclicCounter", cJSON_CreateNumber(profilelong_message.cyclic_counter));
    cJSON_AddItemToObject(data, "Retransmission", cJSON_CreateBool(profilelong_message.retrans));
    cJSON_AddItemToObject(data, "PathIndex", cJSON_CreateNumber(profilelong_message.path_index));
    cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(profilelong_message.offset));
    cJSON_AddItemToObject(data, "Update", cJSON_CreateBool(profilelong_message.update));
    cJSON_AddItemToObject(data, "ProfileType", cJSON_CreateNumber(profilelong_message.profile_type));
    cJSON_AddItemToObject(data, "ControlPoint", cJSON_CreateBool(profilelong_message.control_point));
    if (8 == profilelong_message.profile_type) {
        uint32_t value = 0;
        struct TrafficSign {
            uint32_t reversed = 0; // 1
            uint32_t signLocation = 0; // 3
            uint32_t condition = 0; // 4
            uint32_t timeSpecific = 0; // 2
            uint32_t vehicleSpecific = 0; // 2
            uint32_t lane = 0; // 4
            uint32_t signValue = 0; // 8
            uint32_t signType = 0; // 8
        };

        TrafficSign tmp_sign;
        tmp_sign.signType = profilelong_message.traffic_sign.sign_type;
        value |= tmp_sign.reversed & 0x1;  
        value |= (tmp_sign.signLocation & 0x7) << 1;  
        value |= (tmp_sign.condition & 0xf) << 4;  
        value |= (tmp_sign.timeSpecific & 0x3) << 8;  
        value |= (tmp_sign.vehicleSpecific & 0x3) << 10;  
        value |= (tmp_sign.lane & 0xF) << 12;  
        value |= (tmp_sign.signValue & 0xFF) << 16;  
        value |= (tmp_sign.signType & 0xFF) << 24;  

        cJSON_AddItemToObject(data, "Value", cJSON_CreateNumber(value));
    } else {
        cJSON_AddItemToObject(data, "Value", cJSON_CreateNumber(profilelong_message.value));
    }

    cJSON_AddItemToObject(root, "Data", data);

    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    std::string res(jsonString);
    free(jsonString);
    return res;
}

std::string AdasV2Protocol::_convert_profileshort_to_json(const ProfileShortMessage& profileshort_message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Type", cJSON_CreateString("Av2ProfileShort"));

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "MessageType", cJSON_CreateNumber(profileshort_message.type));
    cJSON_AddItemToObject(data, "CyclicCounter", cJSON_CreateNumber(profileshort_message.cyclic_counter));
    cJSON_AddItemToObject(data, "Retransmission", cJSON_CreateBool(profileshort_message.retrans));
    cJSON_AddItemToObject(data, "PathIndex", cJSON_CreateNumber(profileshort_message.path_index));
    cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(profileshort_message.offset));
    cJSON_AddItemToObject(data, "Update", cJSON_CreateBool(profileshort_message.update));
    cJSON_AddItemToObject(data, "ProfileType", cJSON_CreateNumber(profileshort_message.profile_type));
    cJSON_AddItemToObject(data, "ControlPoint", cJSON_CreateBool(profileshort_message.control_point));
    cJSON_AddItemToObject(data, "Value0", cJSON_CreateNumber(profileshort_message.value0));
    cJSON_AddItemToObject(data, "Distance1", cJSON_CreateNumber(profileshort_message.distance1));
    cJSON_AddItemToObject(data, "Value1", cJSON_CreateNumber(profileshort_message.value1));
    cJSON_AddItemToObject(data, "Accuracy", cJSON_CreateNumber(profileshort_message.accuracy));

    cJSON_AddItemToObject(root, "Data", data);

    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    std::string res(jsonString);
    free(jsonString);
    return res;
}

std::string AdasV2Protocol::_convert_stub_to_json(const StubMessage& stub_message) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Type", cJSON_CreateString("Av2Stub"));

    cJSON *data = cJSON_CreateObject();
    cJSON_AddItemToObject(data, "MessageType", cJSON_CreateNumber(stub_message.type));
    cJSON_AddItemToObject(data, "CyclicCounter", cJSON_CreateNumber(stub_message.cyclic_counter));
    cJSON_AddItemToObject(data, "Retransmission", cJSON_CreateBool(stub_message.retrans));
    cJSON_AddItemToObject(data, "PathIndex", cJSON_CreateNumber(stub_message.path_index));
    cJSON_AddItemToObject(data, "Offset", cJSON_CreateNumber(stub_message.offset));
    cJSON_AddItemToObject(data, "Update", cJSON_CreateBool(stub_message.update));
    cJSON_AddItemToObject(data, "SubPathIndex", cJSON_CreateNumber(stub_message.sub_path_index));
    cJSON_AddItemToObject(data, "TurnAngle", cJSON_CreateNumber(stub_message.turn_angle));
    cJSON_AddItemToObject(data, "RelativeProbability", cJSON_CreateNumber(stub_message.relative_probability));
    cJSON_AddItemToObject(data, "FunctionalRoadClass", cJSON_CreateNumber(stub_message.functional_road_class));
    cJSON_AddItemToObject(data, "FormOfWay", cJSON_CreateNumber(stub_message.form_of_way));
    cJSON_AddItemToObject(data, "NumberOfLane", cJSON_CreateNumber(stub_message.number_of_lanes_in_driving_direction));
    cJSON_AddItemToObject(data, "NumberOfLaneOpposite", cJSON_CreateNumber(stub_message.number_of_lanes_in_opposite_direction));
    cJSON_AddItemToObject(data, "ComplexIntersection", cJSON_CreateNumber(stub_message.complex_intersection));
    cJSON_AddItemToObject(data, "RightOfWay", cJSON_CreateNumber(stub_message.right_of_way));
    cJSON_AddItemToObject(data, "PartOfCalculatedRoute", cJSON_CreateNumber(stub_message.part_of_calculated_route));
    cJSON_AddItemToObject(data, "LastStubAtOffset", cJSON_CreateBool(stub_message.last_stub_at_offset));

    std::string coord_str = "";
    for (size_t i = 0; i < stub_message.coords.size(); i++) {
        coord_str += std::to_string(stub_message.coords[i].x) + "," + std::to_string(stub_message.coords[i].y);
        if (i + 1 < stub_message.coords.size()) {
            coord_str += ";";
        }
    }
    cJSON_AddItemToObject(data, "Coords", cJSON_CreateString(coord_str.c_str()));

    cJSON_AddItemToObject(root, "Data", data);

    char *jsonString = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    std::string res(jsonString);
    free(jsonString);
    return res;
}

void AdasV2Protocol::_send_invalid_stub_message() {
    StubMessage invalid_stub;
    invalid_stub.path_index = 0;
    invalid_stub.sub_path_index = 8;
    invalid_stub.offset = 8191;
    invalid_stub.turn_angle = 255;
    invalid_stub.relative_probability = 0;
    invalid_stub.functional_road_class = 0;
    invalid_stub.form_of_way = 0;
    invalid_stub.number_of_lanes_in_driving_direction = 0;
    invalid_stub.number_of_lanes_in_opposite_direction = 0;
    invalid_stub.complex_intersection = 0;
    invalid_stub.right_of_way = 0;
    invalid_stub.part_of_calculated_route = 0;
    invalid_stub.last_stub_at_offset = 0;

    invalid_stub.cyclic_counter = stub_cyclic;
    stub_cyclic++;
    stub_cyclic = stub_cyclic % 4;

    std::string ehp_json = _convert_stub_to_json(invalid_stub);
    _buffer_channel.push_list0(ehp_json);
    pthread_cond_signal(&_adas_message_cond);
}

void AdasV2Protocol::_send_invalid_position_message() {
    PositionMessage invalid_position;
    invalid_position.path_index = 2;
    invalid_position.offset = 8191;
    invalid_position.position_age = 0;
    invalid_position.speed = 0;
    invalid_position.relative_heading = 0;
    invalid_position.position_probability = 0;

    invalid_position.cyclic_counter = position_cyclic;
    position_cyclic++;
    position_cyclic = position_cyclic % 4;

    std::string ehp_json = _convert_position_to_json(invalid_position);
    ehp_v2_callback(ehp_json);
}

void* AdasV2Protocol::_adas_pthread(AdasV2Protocol* protocol) {
    while (!protocol->_exit) {
        pthread_mutex_lock(&(protocol->_adas_message_mutex));

        std::string ehp_json = "";

        int pop_status = protocol->_buffer_channel.pop(ehp_json);
        if (-1 != pop_status) {
            pthread_mutex_unlock(&(protocol->_adas_message_mutex));
            protocol->ehp_v2_callback(ehp_json);
            if (0 == pop_status) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            continue;
        }

        pthread_cond_wait(&(protocol->_adas_message_cond), &(protocol->_adas_message_mutex));
        pthread_mutex_unlock(&(protocol->_adas_message_mutex));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return nullptr;
}

void* AdasV2Protocol::_position_pthread(AdasV2Protocol* protocol) {
    while (!protocol->_exit) {
        int64_t t1 = get_cur_time_ms();
        pthread_mutex_lock(&(protocol->_position_message_mutex));

        // 有新的position生成后立即发送
        pthread_cond_wait(&(protocol->_position_message_cond), &(protocol->_position_message_mutex));
        if (!protocol->_seted_position_message || protocol->_exit) {
            pthread_mutex_unlock(&(protocol->_position_message_mutex));
            continue;
        }
        int64_t t2 = get_cur_time_ms();

        PositionMessage position_message = protocol->_position_message_cache;
        pthread_mutex_unlock(&(protocol->_position_message_mutex));

        position_message.cyclic_counter = position_cyclic;
        position_cyclic++;
        position_cyclic = position_cyclic % 4;

        position_message.offset %= PATH_MAX_DISTANCE;

        std::string position_json = protocol->_convert_position_to_json(position_message);
        int64_t t3 = get_cur_time_ms();
        protocol->ehp_v2_callback(position_json);
        int64_t t4 = get_cur_time_ms();
        protocol->LOG("_position_pthread mutex_lock_time_ms:" + std::to_string(t2 - t1) 
            + " process_time_ms:" + std::to_string(t3 - t2) 
            + " callback_time_ms:" + std::to_string(t4 - t3));
    }
    return nullptr;
}

void* AdasV2Protocol::_async_log_pthread(AdasV2Protocol* protocol) {
    std::ofstream log_stream;
    log_stream.open(protocol->_log_file);

    while (protocol->_run_log_thread) {
        pthread_mutex_lock(&(protocol->_async_log_mutex));

        if (0 != protocol->_async_log_buffer.size()) {
            std::string log = protocol->_async_log_buffer.front();
            protocol->_async_log_buffer.pop_front();
            pthread_mutex_unlock(&(protocol->_async_log_mutex));
            log_stream << log;
            log_stream.flush();
            continue;
        }

        pthread_cond_wait(&(protocol->_async_log_cond), &(protocol->_async_log_mutex));
        pthread_mutex_unlock(&(protocol->_async_log_mutex));
    }

    if (log_stream.is_open()) {
        log_stream.close();
    }
    return nullptr;
}

} // namespace protocol_v2
} // namespace adas
