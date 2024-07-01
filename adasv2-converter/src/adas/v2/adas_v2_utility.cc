#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <iomanip>
#include <sstream>

#include "adas_v2_utility.h"

namespace adas {
namespace protocol_v2 {

std::string log_time() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    struct tm buf;
    localtime_r(&now_time_t, &buf);
    
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(&buf, "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << milliseconds.count();
    
    return ss.str();
}

uint64_t get_cur_time_ms() {
    struct timeval tm;
    uint64_t cur_time = 0;
    gettimeofday(&tm, NULL);
    cur_time = tm.tv_sec * 1000 + tm.tv_usec / 1000;
    return cur_time;
}

uint64_t get_cur_time_us() {
    struct timeval tm;
    uint64_t cur_time = 0;
    gettimeofday(&tm, NULL);
    cur_time = tm.tv_sec * 1000000 + tm.tv_usec;
    return cur_time;
}

double rad(double d) {
    return d * PI / 180.0;
}

double calculate_distance(double lon1, double lat1, double lon2, double lat2) {
    double rad_lat1 = rad(lat1);
    double rad_lat2 = rad(lat2);
    double a = rad_lat1 - rad_lat2;
    double b = rad(lon1) - rad(lon2);
    double s = 2 * asin(sqrt(pow(sin(a / 2), 2) + 
                cos(rad_lat1) * cos(rad_lat2) * pow(sin(b / 2), 2)));
    s = s * EARTH_RADIUS;
    return s;
}

int normalize_direction(double delta) {
    if (delta > 180) {
        delta -= 360;
    }

    if (delta < -180) {
        delta += 360;
    }

    if (int(delta) == 0) {
        return 0; // In path direction
    } else if (int(delta) == -180 || int(delta) == 180) {
        return 127; // Opposite direction
    } else {
        if (delta > 0) { // right
            return (delta / (double)(180.0 / 127.0));
        } else { // left
            return  (delta / (double)(180.0 / 127.0) + 253.0);
        }
    }
}

int normalize_speed(int speed) {
    if (speed <= 5) {
        return 1;
    } else if (speed > 5 && speed <= 7) {
        return 2;
    } else if (speed > 7 && speed <= 10) {
        return 3;
    } else if (speed > 10 && speed <= 120) {
        return 3 + (speed - 11) / 5 + 1;
    } else if (speed > 120) {
        int ret = 25 + (speed - 121) / 10 + 1;
        return ret > 29 ? 29 : ret; 
    } else {
        return 30; // unlimited
    }
}

int normalize_form_of_way(const LinkInfo& link_info) {
    if (1 == link_info.road_grade || 0 == link_info.road_grade) {
        return 1;
    } else if (link_info.lanenums2e + link_info.lanenume2s > 1) { 
        return 2;
    } else if (link_info.lanenums2e + link_info.lanenume2s == 1) {
        return 3;
    } else {
        for (int j = 0; j < link_info.kinds.size(); j++) {
            int kind = link_info.kinds[j];
            if ((kind & 0xFF) == 0x00) {
                return 4;
            }
        }
    }
    return 0;
}

bool link_attr_sort_help(const LinkInfo& a, const LinkInfo& b) {
    return a.offset < b.offset;
}

} // namespace protocol_v2
} // namespace adas