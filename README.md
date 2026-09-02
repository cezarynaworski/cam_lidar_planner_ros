# cam_lidar_planner_ros

A ROS local planner library that combines data from **LIDAR** and **depth camera** for obstacle avoidance. Two algorithms are included — **APF** (Artificial Potential Fields) and **DWA** (Dynamic Window Approach) — and you can switch between them at runtime.

## What it does

The robot subscribes to odometry, a laser scan, and a depth camera image. Every loop iteration:

1. LIDAR ranges are transformed to world-frame points.
2. Depth image matrix is converted to table (the smallest value of depth is taken from each column) in order to increase algorithms efficiency and then transformed to world-frame points. 
3. APF or DWA uses both obstacle sources to compute a velocity command.
4. The command is published on `/cmd_vel`.

**APF** pulls the robot toward the goal with an attractive force and pushes it away from obstacles with repulsive forces. LIDAR points are grouped into clusters — only the closest point per cluster creates a repulsive field.

**DWA** samples velocity pairs (v, ω), simulates short trajectories, throws out ones that collide, and picks the best by scoring heading, obstacle clearance, and speed.

## ROS topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/odom` | `nav_msgs/Odometry` | Subscribe |
| `/scan` | `sensor_msgs/LaserScan` | Subscribe |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Subscribe |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Subscribe |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish (APF also subscribes for velocity feedback) |

## Dependencies

- ROS (Noetic)
- `sensor_msgs`, `nav_msgs`, `geometry_msgs`
- `cv_bridge` 
- `image_transport`

## Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/cezarynaworski/cam-lidar-planner-ros.git

cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Linking in your own package

To use `cam_lidar_planner_ros` from your node, you need to add the right dependencies.

**package.xml** — add the dependency:

```xml
<depend>cam_lidar_planner_ros</depend>
```


**CMakeLists.txt** — find the package and link:

```cmake
find_package(catkin REQUIRED COMPONENTS
  cam_lidar_planner_ros
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

## Usage

```cpp
#include "cam_lidar_planner_ros/planner_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_navigation_node");
    ros::NodeHandle nh;

    // Pick "APF" or "DWA"
    CLP::PlannerController planner_controller(nh, "DWA");

    // Optional: timeout for each GoToGoal call (default: 10.0 s)
    planner.goal_timeout_sec = 10.0f;

    // Blocks until the goal is reached (returns 0), ROS shuts down (returns -1),
    // or throws std::runtime_error on timeout.
    int result = -1;
    try
    {
      result = planner.GoToGoal(2.0f, 1.5f);
    }
    catch (const std::runtime_error &e)
    {
      ROS_ERROR("Planning failed: %s", e.what());
    }

    // Switch algorithm on the fly
    planner.ChangePlanner(nh, "APF");
    planner.GoToGoal(0.0f, 0.0f);

    return 0;
}
```

## Changing parameters

All parameters have defaults in `include/cam_lidar_planner_ros/types.h`. 
There are two types of parameters. Generic and algotitm specific.


If you need to change algorithm-specific parameters on the fly you need to dynamic cast planner pointer to APF or DWA class. 

```cpp
CLP::PlannerController planner_controller(nh, "DWA");
// Change generic parameter
planner_controller.planner->params.position_accuracy=0.1; 
// Change DWA parameters
auto *dwa = dynamic_cast<CLP::DWA *>(planner_controller.planner.get());
if (dwa)
{
    dwa->params.max_velocity = 0.3f;
}
```

You can also change planner_controller parameters

```cpp
 
planner.params.ROS_Loop_Rate_Hz = 20;                            
planner.params.goal_timeout_sec = 20.0f;                           
```



## License

This project is licensed under the [MIT License](LICENSE).

## Author

Cezary Naworski — [cezary.naworski98@gmail.com](mailto:cezary.naworski98@gmail.com)