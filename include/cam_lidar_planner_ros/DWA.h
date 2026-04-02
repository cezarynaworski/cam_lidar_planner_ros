#ifndef CAM_LIDAR_PLANNER_ROS_DWA_H
#define CAM_LIDAR_PLANNER_ROS_DWA_H

#include "cam_lidar_planner_ros/local_planner.h"

namespace CLP
{

    class DWA : public LocalPlanner
    {
    public:
        DWAParams params;
        LidarParams lidar_params;

        DWA(ros::NodeHandle &nh) : LocalPlanner(nh) {};
        ~DWA() override {};

        // Override virtual methods
        void compute() override;
        void controller() override;
        void ProcesLidar() override;
    };

}

#endif