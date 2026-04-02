#ifndef CAM_LIDAR_PLANNER_ROS_APF_H
#define CAM_LIDAR_PLANNER_ROS_APF_H

#include "cam_lidar_planner_ros/local_planner.h"

namespace CLP
{

    class APF : public LocalPlanner
    {
    private:
        float ref_angle = 0;
        // Lidar grouping
        int *lidar_group = nullptr;
        int C;
        // TWIST callback (APF-specific)
        void TWIST_CALLBACK(const geometry_msgs::Twist::ConstPtr &msg);

    public:
        APFParams params;

        
        APF(ros::NodeHandle &nh);
        ~APF() override;

        
        void compute() override;
        void controller() override;
        void ProcesLidar() override;
        
        // APF-specific: lidar clustering
        void grouping2D(int size);
    };

}

#endif
