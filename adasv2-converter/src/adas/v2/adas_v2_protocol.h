#pragma once
// Copyright 2024 Baidu Inc. All Rights Reserved.
// Author: CHEN ShuaiShuai (chenshuaishuai01@baidu.com)

#include <fstream>
#include <atomic>
#include <map>
#include <assert.h>
#include <pthread.h>

#include "adas_v2_utility.h"
#include "adas_v2_type.h"
#include "adas_v2_channel.h"

namespace adas {
namespace protocol_v2 {

class AdasV2Protocol {
public:
    AdasV2Protocol() : _exit(false), _run_log_thread(false), _seted_position_message(false) {
        _adas_message_cond = PTHREAD_COND_INITIALIZER;
        _position_message_cond = PTHREAD_COND_INITIALIZER;
        _async_log_cond = PTHREAD_COND_INITIALIZER;
        _adas_message_mutex = PTHREAD_MUTEX_INITIALIZER;
        _position_message_mutex = PTHREAD_MUTEX_INITIALIZER;
        _async_log_mutex = PTHREAD_MUTEX_INITIALIZER;
        if (0 != pthread_create(&_adas_message_tid, nullptr, (void* (*)(void*))_adas_pthread, this)) {
            assert(false);
        }

        if (0 != pthread_setname_np(_adas_message_tid, "_adas_tid")) {
            assert(false);
        }

        if (0 != pthread_create(&_position_message_tid, nullptr, (void* (*)(void*))_position_pthread, this)) {
            assert(false);
        }
        
        if (0 != pthread_setname_np(_position_message_tid, "_pos_tid")) {
            assert(false);
        }
    }
    ~AdasV2Protocol() {
        close_log();
        if (!_exit) {
            _exit = true;
            pthread_cond_signal(&_adas_message_cond);
            pthread_cond_signal(&_position_message_cond);
            pthread_join(_adas_message_tid, nullptr);
            pthread_join(_position_message_tid, nullptr);
        }
    }

public:
    /**
     * @brief 用于接收ehpv2消息的回调,派生类必须重写该函数用于回调数据
    */ 
    virtual void ehp_v2_callback(const std::string& ehp_v2_json) {
        return;
    }

    /**
     * @brief 设置目录路径,需要有目录的写权限,并且文件所在的路径是存在的。
     *        比如ccc/aaa/xxx/ddd.log 那么需要ccc/aaa/xxx存在,ddd.log可以不存在
     *        如果不设置该目录,则通过std::cout、std::cerr等标准输出日志
    */
    void set_log_file(const std::string& file);

    /**
     * close log stream
     */
    void close_log();

    /**
     * @brief 设置导航route，每次route ID一旦发生变更，所有的缓存和队列的数据全部清空，等待相对应的数据从ehp服务端下发之后在处理
     *         
    */
    void set_navi_route(const std::string& route);

    /**
     * @brief 输入ehp服务端下发的信息
    */
    void input_ehp_info(const std::string& ehp_info);

    /**
     * @brief 断网情况下，输入坐标，用于离线计算position
    */
    void input_loc(const LocInfo& loc);
private:
// 这把锁确保共有函数不会被同时调用
// 主要是确保线程不安全的map、stream等私有变量安全
std::mutex public_func_mutex;

private:
    std::atomic<bool> _exit;

    pthread_t _adas_message_tid;
    pthread_cond_t _adas_message_cond;
    pthread_mutex_t _adas_message_mutex;
    Adasv2Channel _buffer_channel;
    static void* _adas_pthread(AdasV2Protocol* protocol);

    pthread_t _position_message_tid;
    pthread_cond_t _position_message_cond;
    pthread_mutex_t _position_message_mutex;
    PositionMessage _position_message_cache;
    std::atomic<bool> _seted_position_message;
    static void* _position_pthread(AdasV2Protocol* protocol);

private:
    void _process_position(cJSON* cjson_position_ptr);

    // @brief 检查positon是否回退，如果没有回退写入cache，等待发送线程发送
    void _update_position_cache(const PositionMessage& position, const std::string& from);

    void _parse_path_info(cJSON* cjson_paths_ptr);
    void _send_stub();
    void _parse_link_info(cJSON* cjson_links_ptr);
    void _send_segment();
    void _send_lonlat();
    void _send_slope(cJSON* cjson_slope_ptr);
    void _send_curvature(cJSON* cjson_curvature_ptr);
    void _send_traffic_light(cJSON* cjson_traffic_light_ptr);
    void _send_warning_info(cJSON* cjson_warning_info_ptr);

    void _send_invalid_stub_message();
    void _send_invalid_position_message();

    std::string _convert_position_to_json(const PositionMessage& position_message);
    std::string _convert_segment_to_json(const SegmentMessage& segment_message);
    std::string _convert_profilelong_to_json(const ProfileLongMessage& profilelong_message);
    std::string _convert_profileshort_to_json(const ProfileShortMessage& profileshort_message);
    std::string _convert_stub_to_json(const StubMessage& stub_message);

    std::vector<PathInfo> _path_infos; // 用于position更新的时候，动态更新stub使用
    std::map<int64_t, std::vector<LinkInfo> > _link_infos; // 用于离线绑路更新positon用

    std::map<int64_t, int64_t> _sended_max_segment_link_index; // pathid -> link_index_on_path
    std::map<int64_t, int64_t> _sended_max_lonlat_link_index; // pathid -> link_index_on_path
    std::map<int64_t, int64_t> _sended_max_slope_offset; // pathid -> offset * 100;
    std::map<int64_t, int64_t> _sended_max_curvature_offset; // pathid -> offset * 100;
    std::map<int64_t, int64_t> _sended_max_traffic_light_offset; // pathid -> offset * 100;
    std::map<int64_t, int64_t> _sended_max_warning_info_offset; // pathid -> offset * 100;

    int64_t _sended_max_mainpath_offset = -1;
    int64_t _sended_max_subpath_offset = -1;

    std::string _navi_route_id = "";
    std::map<int64_t, std::map<int64_t, int64_t> > _navi_link_id_2_length; // linkindex -> linkid -> linklength
    int64_t _ehp_version = 0;

public:
    pthread_t _async_log_tid;
    pthread_cond_t _async_log_cond;
    pthread_mutex_t _async_log_mutex;
    static void* _async_log_pthread(AdasV2Protocol* protocol);

    std::atomic<bool> _run_log_thread;
    std::string _log_file;

    void _log(const std::string& msg);
    std::list<std::string>  _async_log_buffer;
}; // class AdasV2Protocol

} // namesapce protocol_v2
} // namespace adas