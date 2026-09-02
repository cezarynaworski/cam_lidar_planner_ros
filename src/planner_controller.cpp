#include "cam_lidar_planner_ros/planner_controller.h"
#include <stdexcept>

namespace CLP
{
    PlannerController::PlannerController(ros::NodeHandle &nh, const std::string &planner_type)
    {
        if (planner_type == "APF")
        {
            planner = std::make_unique<APF>(nh);
            ROS_INFO("APF planner initialized.");
        }
        else if (planner_type == "DWA")
        {
            planner = std::make_unique<DWA>(nh);
            ROS_INFO("DWA planner initialized.");
        }
        else
        {
            throw std::invalid_argument("Unknown planner type: " + planner_type);
        }
    }

    int PlannerController::GoToGoal(float x, float y)
    {
        planner->goal_x = x;
        planner->goal_y = y;
        ROS_INFO("Goal set to: (%f, %f)", x, y);

        ros::Rate loop_rate(params.ROS_Loop_Rate_Hz);
        while (!(planner->firstinitIm && planner->firstinitLIDAR && planner->firstinitInfo))
        {
            ROS_WARN("Waiting for all sensors to initialize...");
            loop_rate.sleep();
            ros::spinOnce();
        }

        ROS_INFO("Starting control loop to reach the goal...");
        const ros::Time start_time = ros::Time::now();

            while (ros::ok())
            {
                if (CLP::hypot(planner->state.x - x, planner->state.y - y) <= planner->params.position_accuracy)
                {

                    ROS_INFO("Goal reached: (%f, %f)", planner->state.x, planner->state.y);
                    return 0; // Success
                }

                if (params.goal_timeout_sec > 0.0f && (ros::Time::now() - start_time).toSec() >= params.goal_timeout_sec)
                {
                    geometry_msgs::Twist stop_msg;
                    planner->ctr_pub.publish(stop_msg);

                    ROS_ERROR("Goal timeout after %.2f s for target (%f, %f)", params.goal_timeout_sec, x, y);
                    throw std::runtime_error("GoToGoal timeout: target was not reached in configured time limit.");
                }

                planner->compute();
                planner->controller();
                ros::spinOnce();
                loop_rate.sleep();
            }
        

        return -1; // Failure (ROS shutdown)
    }

    void PlannerController::ChangePlanner(ros::NodeHandle &nh, const std::string &planner_type)
    {
        planner.reset(); // Clean up existing planner

        if (planner_type == "APF")
        {
            planner = std::make_unique<APF>(nh);
            ROS_INFO("Switched to APF planner.");
        }
        else if (planner_type == "DWA")
        {
            planner = std::make_unique<DWA>(nh);
            ROS_INFO("Switched to DWA planner.");
        }
        else
        {
            throw std::invalid_argument("Unknown planner type: " + planner_type);
        }
    }

   

}