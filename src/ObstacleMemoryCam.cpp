#include "cam_lidar_planner_ros/ObstacleMemoryCam.h"
#include "ros/ros.h"
namespace CLP
{
    ObstacleMemoryCam::ObstacleMemoryCam(int time_max) : time_max(time_max) {}
    void ObstacleMemoryCam::resizeVector(int cols)
    {
        size = cols;
        X.resize(size, 0.0f);
        Y.resize(size, 0.0f);
        Time.resize(size, 0);
        IsValid.resize(size, false);
    }

    void ObstacleMemoryCam::UpdateMemory()
    {
        for (int col = 0; col < size; col++)
        {
            Time[col]++;
            if (Time[col] >= time_max)
                IsValid[col] = false;
        }
    }
}