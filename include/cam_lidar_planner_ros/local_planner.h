#ifndef CAM_LIDAR_PLANNER_ROS_LOCAL_PLANNER_H
#define CAM_LIDAR_PLANNER_ROS_LOCAL_PLANNER_H
// ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/CameraInfo.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
// OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Standard
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
// Project
#include "cam_lidar_planner_ros/types.h"
#include "cam_lidar_planner_ros/ObstacleMemoryCam.h"
#include "cam_lidar_planner_ros/utils.h"

namespace CLP
{

    class LocalPlanner
    {
    protected:
        

        // Callbacks
        void ODO_CALLBACK(const nav_msgs::Odometry::ConstPtr &msg);
        void CAMINFO_CALLBACK(const sensor_msgs::CameraInfo::ConstPtr &msg);
        void IMAGE_CALLBACK(const sensor_msgs::ImageConstPtr &msg);
        void LASER_CALLBACK(const sensor_msgs::LaserScan::ConstPtr &msg);

        

    public:
        GenericParams generic_params;
        
    // Subscribers and Publishers
        ros::Subscriber scan_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber caminfo_sub;
        image_transport::Subscriber im_sub;
        ros::Subscriber v_sub;
        ros::Publisher ctr_pub;

        ros::NodeHandle n;

        
        RobotState state;
        float goal_x = 0.0f, goal_y = 0.0f;

        
        ControlOutput output;

        
        CameraParams cam_params;

        // Lidar
        int lidar_N;
        float angle_min, angle_max, angle_increment;
        float *lidar_x = nullptr, *lidar_y = nullptr, *ranges = nullptr;
        volatile bool firstinitLIDAR = false;

        // Init flags
        volatile bool firstinitIm = false, firstinitInfo = false;
        volatile bool firstinitProcesCAM = false, firstinitProcesLidar = false;

        
        ObstacleMemoryCam MemCAM;

        // Stored messages
        sensor_msgs::ImageConstPtr depth_msg_;
        sensor_msgs::LaserScan::ConstPtr lidar_msg_;

        
        LocalPlanner(ros::NodeHandle &nh);
        virtual ~LocalPlanner();

        
        virtual void compute() = 0;
        virtual void controller() = 0;
        virtual void ProcesLidar()=0;
        virtual void ProcesImage();
    };

}

#endif