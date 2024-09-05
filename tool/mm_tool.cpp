#include <stdlib.h>
#include <math.h>


#include "mm_tool.h"

const double EARTH_RADIUS = 6378.137;
static const double PI  = 3.141592653589793238462;
static double rad(double d) {
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
    return 1000.0 * s;
}

void calculate_projection(double p_x, double p_y,
        double l_x1, double l_y1,
        double l_x2, double l_y2,
        double& dist_to_line,
        double& dist_to_snode,
        int& project_type) {
    double v1_x = p_x - l_x1;
    double v1_y = p_y - l_y1;
    double v2_x = l_x2 - l_x1;
    double v2_y = l_y2 - l_y1;
    double v1_len = sqrt(v1_x * v1_x + v1_y * v1_y);
    double v2_len = sqrt(v2_x * v2_x + v2_y * v2_y);
    double cos_angle1 = 0;
    if (v1_len > 0) {
        cos_angle1 = (v1_x * v2_x + v1_y * v2_y) / (v1_len * v2_len);
    }
    if (cos_angle1 > 1) {
        cos_angle1 = 1;
    } else if (cos_angle1 < -1) {
        cos_angle1 = -1;
    }
    double cos_angle2 = 0;

    if (cos_angle1 < 0) {
        /// 在首点之前
        dist_to_line = calculate_distance(p_x, p_y, l_x1, l_y1);
        dist_to_snode = 0;
        project_type = 1;
    } else {
        double v3_x = p_x - l_x2;
        double v3_y = p_y - l_y2;
        double v3_len = sqrt(v3_x * v3_x + v3_y * v3_y);
        if (v3_len > 0) {
            cos_angle2 = (-v2_x * v3_x + -v2_y * v3_y) / (v2_len * v3_len);
        }

        if (cos_angle2 < 0) {
            /// 在末点之后
            dist_to_line = calculate_distance(p_x, p_y, l_x2, l_y2);
            dist_to_snode = calculate_distance(l_x1, l_y1, l_x2, l_y2);
            project_type = 2;
        } else {
            double dist = calculate_distance(p_x, p_y, l_x1, l_y1);
            dist_to_snode = dist * cos_angle1;
            dist_to_line = dist * sqrt(1 - cos_angle1 * cos_angle1);
            project_type = 0;
        }
    }
}
