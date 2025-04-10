#include "bezier_motion.hpp"

#define u64 unsigned long long
#define MAP(x, a, b, c, d)((c) + ((((x) - (a)) * ((d) - (c)))/((b) - (a))))
#define BETWEEN_INCLUSIVE(a, min, max)((a) >= min && (a) <= max)

namespace bezier_motion {
    // vec2 class definitions
    vec2::vec2(const double X, const double Y) { x = X; y = Y; }
    vec2::vec2() { x = 0; y = 0; }
    vec2::vec2(const double a) { x = a; y = a; }
    vec2::vec2(const vec2& other) { x = other.x; y = other.y; }
    vec2 vec2::abs() { return vec2(std::fabs(x), std::fabs(y)); }
    vec2 vec2::operator+(const vec2 other) { return vec2(x + other.x, y + other.y); }
    vec2 vec2::operator-(const vec2 other) { return vec2(x - other.x, y - other.y); }
    vec2 vec2::operator*(const vec2 other) { return vec2(x * other.x, y * other.y); }
    vec2 vec2::operator/(const vec2 other) { return vec2(x / other.x, y / other.y); }
    vec2 vec2::pow(const vec2 other) { return vec2(std::pow(x, other.x), std::pow(y, other.y)); }
    vec2 vec2::modulo(const vec2 other) { return vec2(std::fmod(x, other.x), std::fmod(y, other.y)); }
    double vec2::magnitude() { return std::sqrt(x * x + y * y); }
    double vec2::distance(const vec2 other) { return vec2(other.x - x, other.y - y).magnitude(); }
    vec2 vec2::normalized() { double m = magnitude(); return vec2(x / m, y / m); }
    vec2 vec2::dot_product(const vec2 other) { return x * other.x + y * other.y; }
    vec2 vec2::seperated_distance(const vec2 other) { return vec2(other.x - x, other.y - y).abs(); }

    // bezier class definitions
    bezier::bezier(vec2 end_point, double density, vec2 control_point_position_percent) {
        m_end_point = end_point;
        m_density = density;
        m_control2 = vec2(0, 0);
        m_control_point_position_percent = control_point_position_percent;
    }

    void bezier::set_density(double new_density) { m_density = new_density; }
    double bezier::get_density() { return m_density; }

    double bezier::lerp(double a, double b, double t) { return b * t - a * (t - 1); }
    vec2 bezier::vec2_lerp(vec2 a, vec2 b, vec2 t) { return vec2(lerp(a.x, b.x, t.x), lerp(a.y, b.y, t.y)); }
    double bezier::cubic_bez(double a, double b, double c, double d, double t) {
        double ab = lerp(a, b, t);
        double bc = lerp(b, c, t);
        double cd = lerp(c, d, t);
        return lerp(lerp(ab, bc, t), lerp(bc, cd, t), t);
    }
    vec2 bezier::vec2_cubic_bez(vec2 a, vec2 b, vec2 c, vec2 d, vec2 t) {
        return vec2(cubic_bez(a.x, b.x, c.x, d.x, t.y), cubic_bez(a.y, b.y, c.y, d.y, t.y));
    }
    void bezier::vec2_cubic_bez_cumulative_dist_LUT(vec2 a, vec2 b, vec2 c, vec2 d, double LUT_output[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES]) {
        double t_rate = 1.0 / (CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1);
        double cumulative_dist = 0;
        vec2 prev = vec2_cubic_bez(a, b, c, d, vec2(0, 0));
        for (u64 i = 1; i < CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES; i++) {
            double t = i * t_rate;
            vec2 cur = vec2_cubic_bez(a, b, c, d, vec2(t, t));
            cumulative_dist += prev.distance(cur);
            LUT_output[i] = cumulative_dist;
            prev = cur;
        }
    }
    double bezier::vec2_cubic_bez_t_from_linear_t(double t, double LUT[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES]) {
        double max_dist = LUT[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1];
        double dist = MAP(t, 0, 1, 0, max_dist);
        for (u64 i = 0; i < CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1; i++) {
            if (BETWEEN_INCLUSIVE(dist, LUT[i], LUT[i + 1])) {
                return MAP(dist, LUT[i], LUT[i + 1], (double)i / (CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1), (double)(i + 1) / (CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1));
            }
        }
        return 0;
    }
    void bezier::cubic_bez_auto_control_points(vec2 prev_control2, vec2 a, vec2 b, vec2 pos_perc, bool no_smooth_joint) {
        m_control1 = no_smooth_joint ? vec2_lerp(a, b, vec2(pos_perc.x, 1 - pos_perc.y)) : (a * vec2(2) - prev_control2);
        m_control2 = vec2_lerp(a, b, vec2(1 - pos_perc.x, pos_perc.y));
    }
    double bezier::get_linear_t_rate(double arc_len) { return 1 / arc_len / m_density; }
    void bezier::loop_2_points_auto_control_points(vec2 a, vec2 b, vec2 prev_control2, bool no_smooth_joint, vec2 control_point_position_percent, std::function<void(vec2)> callback) {
        cubic_bez_auto_control_points(prev_control2, a, b, control_point_position_percent, no_smooth_joint);
        double LUT[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES];
        vec2_cubic_bez_cumulative_dist_LUT(a, m_control1, m_control2, b, LUT);
        double t_rate = get_linear_t_rate(LUT[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES - 1]);
        for (double t = 0; t <= 1; t += t_rate) {
            callback(vec2_cubic_bez(a, m_control1, m_control2, b, vec2_cubic_bez_t_from_linear_t(t, LUT)));
        }
    }
    void bezier::execute_pid_motion(vec2 prev_control2, bool no_smooth_joint, std::function<void(double)> linear_pid_callback, std::function<void(double)> angular_pid_callback) {
        vec2 prev_point(0, 0);
        loop_2_points_auto_control_points(vec2(0, 0), m_end_point, prev_control2, no_smooth_joint, m_control_point_position_percent, [&prev_point, angular_pid_callback, linear_pid_callback](vec2 point) {
            vec2 diff = point - prev_point;
            angular_pid_callback(std::atan2(diff.y, diff.x) * 180 / M_PI);
            linear_pid_callback(diff.magnitude());
            prev_point = point;
        });
    }
    vec2 bezier::get_control2() { return m_control2; }
}
