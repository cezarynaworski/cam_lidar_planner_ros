#ifndef CAM_LIDAR_PLANNER_ROS_TYPES_H
#define CAM_LIDAR_PLANNER_ROS_TYPES_H

#include <vector>
#include <cstdint>
#include <string>
namespace CLP {

struct GenericParams {
    float position_accuracy = 0.05f; // [m]
    
};

struct ControllerParams {
    int ROS_Loop_Rate_Hz = 10;
    float goal_timeout_sec = 20.0f;

};

struct RobotState {
    float x;           
    float y;           
    float theta;       
    float v;          
    float omega;       
};



struct ControlOutput {
    float linear_velocity;  ///< [m/s]
    float angular_velocity; ///< [rad/s]
};


struct DWAParams : public GenericParams {
    // Robot constraints
    float max_velocity = 0.2f;
    float max_acceleration = 0.08f;
    float max_angular_velocity = 0.4375f * 3.14159f;
    float max_angular_acceleration = 1.5f * 3.14159f;
    
    // Prediction parameters
    float prediction_time = 1.0f;
    float sampling_period = 0.25f;
    float velocity_sampling_step = 0.02f;
    float angular_velocity_sampling_step = 0.0625f * 3.14159f;
    //float position_accuracy = 0.15f;
    
    // Cost weights
    float heading_weight = 0.9f;
    float distance_weight = 20.0f;
    float velocity_weight = 150.0f;
    
    // Safety distance
    float obstacle_distance = 0.2f;
};


struct APFParams : public GenericParams {
    float att_coefficient = 1.1547f;
    float rep_coefficient = 0.732f;
    float range = 0.35f;
    float range_cam = 0.6f;
    float goal_maximum_distance = 0.3f;
    float position_toleration = 0.1f;
    //float position_accuracy = 0.2f;

    // Controller limits
    float omega_max = 0.2f * 3.14159f;
    float v_max = 0.2f;
    float error_theta_max = 3.14159f / 4.0f;
};


struct CameraParams {
    float focal_length_x = 1.0f;  
    float focal_length_y = 1.0f; 
    float principal_point_x = 0.0f; 
    float principal_point_y = 0.0f; 
    float camera_height = 0.22f;   
    float min_floor_height = 0.03f;
    float min_depth = 0.1f;
    float max_depth = 0.7f;
    float max_height = 1.0f;
    int image_rows = 0;
    int image_cols = 0;
    std::string depth_image_encoding = "32FC1";
};


struct LidarParams {
    float min_range = 0.1f;
    float max_range = 0.7f;
};

}  

#endif  