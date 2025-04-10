#pragma once

#ifdef __cplusplus
#include <cmath>
#include <vector>
#include <functional>

// Define the number of samples for the cubic Bezier lookup table
#define CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES 2222

namespace bezier_motion {
    class vec2 {
        public:
            double x = 0; // X-coordinate
            double y = 0; // Y-coordinate

            // Constructors
            vec2(const double X, const double Y);
            vec2();
            vec2(const double a);
            vec2(const vec2& other);

            // Utility methods for vector operations
            vec2 abs();
            vec2 operator+(const vec2 other);
            vec2 operator-(const vec2 other);
            vec2 operator*(const vec2 other);
            vec2 operator/(const vec2 other);
            vec2 pow(const vec2 other);
            vec2 modulo(const vec2 other);
            double magnitude();
            double distance(const vec2 other);
            vec2 normalized();
            vec2 dot_product(const vec2 other);
            vec2 seperated_distance(const vec2 other);
    };

    class bezier {
        private:
            double m_density = 0.1; // Density of the interpoled points on the curve
            vec2 m_control1 = vec2(0, 0); // First control point
            vec2 m_control2 = vec2(0, 0); // Second control point

            // Private utility methods
            double lerp(double a, double b, double t);
            vec2 vec2_lerp(vec2 a, vec2 b, vec2 t);
            double cubic_bez(double a, double b, double c, double d, double t);
            vec2 vec2_cubic_bez(vec2 a, vec2 b, vec2 c, vec2 d, vec2 t);
            void vec2_cubic_bez_cumulative_dist_LUT(vec2 a, vec2 b, vec2 c, vec2 d, double LUT_output[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES]);
            double vec2_cubic_bez_t_from_linear_t(double t, double LUT[CUBIC_BEZIER_LOOK_UP_TABLE_SAMPLES]);
            void cubic_bez_auto_control_points(vec2 prev_control2, vec2 a, vec2 b, vec2 pos_perc, bool no_smooth_joint);
            double get_linear_t_rate(double arc_len);
            void loop_2_points_auto_control_points(vec2 a, vec2 b, vec2 prev_control2, bool no_smooth_joint, vec2 control_point_position_percent, std::function<void(vec2)> callback);

        public:
            vec2 m_end_point = vec2(0, 0); // End point of the Bezier curve
            #define CONTROL_POINT_PERCENT_DEFAULT (bezier_motion::vec2(0.76, 0.76))
            vec2 m_control_point_position_percent = CONTROL_POINT_PERCENT_DEFAULT; // Control point position percentage

            // Constructor
            bezier(vec2 end_point, double density, vec2 control_point_position_percent);

            // Public methods
            void set_density(double new_density);
            double get_density();
            #define NO_PREVIOUS_CONTROL_POINT bezier_motion::vec2(), true
            #define PREVIOUS_CONTROL_POINT(prev_control2)(prev_control2), false
            void execute_pid_motion(vec2 prev_control2, bool no_smooth_joint, std::function<void(double)> linear_pid_callback, std::function<void(double)> angular_pid_callback);
            vec2 get_control2();
    };

    #undef u64
    #undef MAP
    #undef BETWEEN_INCLUSIVE
}
#endif