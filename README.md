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

This package is a **library** — you must link it in your own node:

```cpp
#include "cam_lidar_planner_ros/planner_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_navigation_node");
    ros::NodeHandle nh;

    // Pick "APF" or "DWA"
    CLP::PlannerController planner(nh, "DWA");

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

All parameters have defaults in `include/cam_lidar_planner_ros/types.h`. You can change them after creating the planner by casting to the concrete type:

```cpp
CLP::PlannerController planner(nh, "DWA");

// Change DWA parameters
auto *dwa = dynamic_cast<CLP::DWA *>(planner.planner.get());
if (dwa)
{
    dwa->params.max_velocity = 0.3f;
    dwa->params.obstacle_distance = 0.25f;

    // DWA LIDAR filter 
    dwa->lidar_params.max_range = 1.0f;
}
```

```cpp
CLP::PlannerController planner(nh, "APF");

// Change APF parameters
auto *apf = dynamic_cast<CLP::APF *>(planner.planner.get());
if (apf)
{
    apf->params.att_coefficient = 1.5f;
    apf->params.v_max = 0.15f;
}
```

You can also change the goal-reached threshold, the control loop rate, and timeout:

```cpp
planner.planner->generic_params.position_accuracy = 0.1f;  // default: 0.05 m
planner.ROS_Loop_Rate_Hz = 20;                              // default: 10 Hz
planner.goal_timeout_sec = 20.0f;                           // default: 20.0 s, <= 0 disables timeout
```

### APF parameters — `APFParams`  accessed via `apf->params`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `att_coefficient` | 1.1547 | How strongly the goal attracts the robot |
| `rep_coefficient` | 0.732 | How strongly obstacles repel the robot |
| `range` | 0.35 m | LIDAR obstacles closer than this generate repulsion |
| `range_cam` | 0.6 m | Camera obstacles closer than this generate repulsion |
| `goal_maximum_distance` | 0.3 m | Attractive force is clamped beyond this distance |
| `position_toleration` | 0.1 m | Stop computing forces when this close to goal |
| `position_accuracy` | 0.2 m | Internal goal-reached check inside `compute()` |
| `v_max` | 0.2 m/s | Max linear speed |
| `omega_max` | 0.2π rad/s | Max angular speed |
| `error_theta_max` | π/4 rad | Heading error above which forward motion is reduced to zero |

### DWA parameters — `DWAParams` accessed via `dwa->params`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_velocity` | 0.2 m/s | Max linear speed |
| `max_acceleration` | 0.08 m/s² | Linear acceleration limit |
| `max_angular_velocity` | 0.4375π rad/s | Max angular speed |
| `max_angular_acceleration` | 1.5π rad/s² | Angular acceleration limit |
| `prediction_time` | 1.0 s | How far ahead to simulate trajectories |
| `sampling_period` | 0.25 s | Time step in trajectory simulation |
| `velocity_sampling_step` | 0.02 m/s | Resolution when sampling linear speeds |
| `angular_velocity_sampling_step` | 0.0625π rad/s | Resolution when sampling angular speeds |
| `position_accuracy` | 0.15 m | Internal goal-reached check inside `compute()` |
| `heading_weight` | 0.9 | Score weight for pointing toward the goal |
| `distance_weight` | 20.0 | Score weight for obstacle clearance |
| `velocity_weight` | 150.0 | Score weight for going faster |
| `obstacle_distance` | 0.2 m | Minimum distance to obstacle before a trajectory is rejected |

### DWA LIDAR filter — `LidarParams` accessed via `dwa->lidar_params`

Used by DWA to filter valid scan readings before trajectory collision checking. Not used by APF.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_range` | 0.1 m | Scan readings below this are ignored |
| `max_range` | 0.7 m | Scan readings above this are ignored |

### Camera parameters — `CameraParams` accessed via `planner->cam_params`



| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_height` | 0.22 m | Camera height above ground |
| `min_floor_height` | 0.03 m | Points below this height are treated as floor and ignored |
| `max_height` | 1.0 m | Points above this height are ignored |
| `min_depth` | 0.1 m | Minimum valid depth reading |
| `max_depth` | 0.7 m | Maximum valid depth reading (APF constructor overrides this to 1.5 m) |
| `depth_image_encoding` | `"32FC1"` | Image encoding|

### General

| Parameter | Default | Description |
|-----------|---------|-------------|
| `position_accuracy` | 0.05 m | How close to the goal counts as "reached" |
| `ROS_Loop_Rate_Hz` | 10 Hz | Control loop frequency |
| `goal_timeout_sec` | 10.0 s | Max time for a single `GoToGoal` call (`<= 0` means no timeout) |


## Project structure

```
include/cam_lidar_planner_ros/
  types.h                # All parameter structs
  utils.h                # Math helpers
  local_planner.h        # Abstract base class
  APF.h / DWA.h          # Algorithm headers
  ObstacleMemoryCam.h    # Camera obstacle memory
  planner_controller.h   # High-level controller
src/
  local_planner.cpp      # ROS setup, depth processing, callbacks
  APF.cpp                # Potential fields + LIDAR clustering
  DWA.cpp                # Trajectory sampling + scoring
  ObstacleMemoryCam.cpp  # Memory update logic
  planner_controller.cpp # Factory + GoToGoal loop
```

## License

This project is licensed under the [MIT License](LICENSE).

## Author

Cezary Naworski — [cezary.naworski98@gmail.com](mailto:cezary.naworski98@gmail.com)
