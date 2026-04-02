#ifndef CAM_LIDAR_PLANNER_ROS_UTILS_H
#define CAM_LIDAR_PLANNER_ROS_UTILS_H

#include <vector>
#include <cmath>

namespace CLP
{

    inline float deg2rad(float deg)
    {
        return deg * (M_PI / 180.0f);
    }

    inline float rad2deg(float rad)
    {
        return rad * (180.0f / M_PI);
    }

    /**
     * @brief Normalize angle to [-pi, pi]
     * @param theta Angle in radians
     */
    inline void normalize_angle(float &theta)
    {
        if (theta > deg2rad(180.0f))
            theta -= deg2rad(360.0f);
        else if (theta < (-1.0f) * deg2rad(180.0f))
            theta += deg2rad(360.0f);
    }

    /**
     * @brief Calculate Euclidean distance
     * @param x Delta X
     * @param y Delta Y
     * @return Distance
     */
    inline float hypot(float x, float y)
    {
        return std::sqrt(x * x + y * y);
    }

    /**
     * @brief Generate range of values (like numpy.arange)
     */
    template <typename T>
    std::vector<T> arange(T start, T stop, T step)
    {
        std::vector<T> values;
        for (T i = start; i <= stop + static_cast<T>(0.0001); i += step)
        {
            values.push_back(i);
        }
        return values;
    }

    template <typename T>
    T minimum(T a, T b)
    {
        return (a < b) ? a : b;
    }

    template <typename T>
    T maximum(T a, T b)
    {
        return (a > b) ? a : b;
    }

}

#endif
