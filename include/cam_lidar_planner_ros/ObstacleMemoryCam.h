#ifndef CAM_LIDAR_PLANNER_ROS_OBSTACLE_MEMORY_CAM_H
#define CAM_LIDAR_PLANNER_ROS_OBSTACLE_MEMORY_CAM_H

#include <vector>
using namespace std;
namespace CLP
{

    // This class remembers obstacles detected by camera
    class ObstacleMemoryCam
    {
    public:
        ObstacleMemoryCam() {}
        ObstacleMemoryCam(int time_max);

        vector<float> X, Y;   // coordinates
        vector<int> Time;     // time validity for coordinates
        vector<bool> IsValid; // True if coordinates are valid

        int time_max = 80;

        void resizeVector(int cols); // fit to image frame

        // Set Variable IsValid to 0 if time has passed and increase "time"
        void UpdateMemory();

    private:
        unsigned int size = 0;
    };
}

#endif