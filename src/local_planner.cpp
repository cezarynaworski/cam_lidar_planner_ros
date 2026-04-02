#include "cam_lidar_planner_ros/local_planner.h"

using namespace std;

namespace CLP
{
    LocalPlanner::LocalPlanner(ros::NodeHandle &nh)
        : n(nh)
    {
        state = {0, 0, 0, 0, 0};
        output = {0, 0};

        image_transport::ImageTransport it(n);

        scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &LocalPlanner::LASER_CALLBACK, this);
        odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 1, &LocalPlanner::ODO_CALLBACK, this);
        caminfo_sub = n.subscribe("/camera/depth/camera_info", 1, &LocalPlanner::CAMINFO_CALLBACK, this);
        im_sub = it.subscribe("/camera/depth/image_raw", 1, &LocalPlanner::IMAGE_CALLBACK, this);

        ctr_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    LocalPlanner::~LocalPlanner()
    {
        delete[] lidar_x;
        delete[] lidar_y;
        delete[] ranges;
    }
    void LocalPlanner::ProcesImage()
    {
        if (!depth_msg_)
        {
            ROS_WARN_THROTTLE(1.0, "ProcesImage called with no depth message");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        
        try
        {
            if (cam_params.depth_image_encoding =="32FC1")
            {
                cv_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
            }
            else if (cam_params.depth_image_encoding == "16UC1")
            {
                cv_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
            }
            else
            {
                ROS_ERROR("Unsupported depth image encoding: %s", cam_params.depth_image_encoding.c_str());
                return;
            }
            //cv_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
            //cv_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
            //CHECK THAT!
            cv::Mat depth_image = cv_ptr->image;

            if (!firstinitProcesCAM)
            {
                cam_params.image_cols = depth_image.cols;
                cam_params.image_rows = depth_image.rows;
                MemCAM.resizeVector(cam_params.image_cols);
                firstinitProcesCAM = true;
            }
              
            for (int col = 0; col < cam_params.image_cols; col++)
            {
                float x;
                float min_value = 100.0f;

                for (int row = 0; row < cam_params.image_rows; row++)
                {
                    if (cam_params.depth_image_encoding == "16UC1")
                    {
                        uint16_t x16 = depth_image.at<uint16_t>(row, col);
                        x = static_cast<float>(x16) * 0.001f;
                    }
                    else if (cam_params.depth_image_encoding == "32FC1")
                    {
                        x = depth_image.at<float>(row, col);
                    }
                    
                    //uint16_t x16 = depth_image.at<uint16_t>(row, col);
                    //x = static_cast<float>(x16) * 0.001f;
                    //x = depth_image.at<float>(row, col);
                    if (cam_params.focal_length_y == 0.0f || cam_params.focal_length_x == 0.0f)
                    {
                        ROS_ERROR_ONCE("Focal length is zero, cannot process image");
                        return;
                    }
                    float z = ((row - cam_params.principal_point_y) * x / cam_params.focal_length_y) * (-1.0f) + cam_params.camera_height;

                    if (isfinite(x) && z > cam_params.min_floor_height && z < cam_params.max_height && x < cam_params.max_depth && x > cam_params.min_depth)
                    {
                        if (x < min_value)
                            min_value = x;
                    }
                }

                if (min_value < cam_params.max_depth)
                {
                    float y = ((col - cam_params.principal_point_x) * min_value / cam_params.focal_length_x) * (-1.0f);

                    MemCAM.X[col] = state.x + min_value * cos(state.theta) - y * sin(state.theta);
                    MemCAM.Y[col] = state.y + min_value * sin(state.theta) + y * cos(state.theta);

                    MemCAM.IsValid[col] = true;
                    MemCAM.Time[col] = 0;
                }
            }
            MemCAM.UpdateMemory();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    // --- Callbacks ---
    void LocalPlanner::ODO_CALLBACK(const nav_msgs::Odometry::ConstPtr &msg)
    {
        state.x = msg->pose.pose.position.x;
        state.y = msg->pose.pose.position.y;
        state.theta = 2.0f * atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        CLP::normalize_angle(state.theta);
    }

    void LocalPlanner::CAMINFO_CALLBACK(const sensor_msgs::CameraInfo::ConstPtr &msg)
    {
        if (!firstinitInfo)
        {
            cam_params.focal_length_x = msg->K[0];
            cam_params.focal_length_y = msg->K[4];
            cam_params.principal_point_x = msg->K[2];
            cam_params.principal_point_y = msg->K[5];
            firstinitInfo = true;
        }
    }

    void LocalPlanner::IMAGE_CALLBACK(const sensor_msgs::ImageConstPtr &msg)
    {
        depth_msg_ = msg;
        firstinitIm = true;
    }

    void LocalPlanner::LASER_CALLBACK(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        lidar_msg_ = msg;
        firstinitLIDAR = true;
    }

}