#include "cam_lidar_planner_ros/DWA.h"

namespace CLP
{
    
    void DWA::ProcesLidar()
    {
        angle_min = lidar_msg_->angle_min;
        angle_max = lidar_msg_->angle_max;
        angle_increment = lidar_msg_->angle_increment;
        lidar_N = (int)((angle_max - angle_min) / angle_increment) + 1;

        if (!firstinitProcesLidar)
        {
            ranges = new float[lidar_N];
            lidar_x = new float[lidar_N];
            lidar_y = new float[lidar_N];
            firstinitProcesLidar = true;
        }

        for (int i = 0; i < lidar_N; i++)
        {
            ranges[i] = lidar_msg_->ranges[i];
            float xl = -(cos(angle_min + i * angle_increment) * ranges[i]);
            float yl = -(sin(angle_min + i * angle_increment) * ranges[i]);

            lidar_x[i] = state.x + xl * cos(state.theta) - yl * sin(state.theta);
            lidar_y[i] = state.y + xl * sin(state.theta) + yl * cos(state.theta);
        }
    }

    

    void DWA::controller()
    {
        geometry_msgs::Twist msg;
        msg.angular.z = output.angular_velocity;
        msg.linear.x = output.linear_velocity;
        ctr_pub.publish(msg);
    }

    void DWA::compute()
    {
        ProcesImage();
        ProcesLidar();

        if (CLP::hypot(goal_x - state.x, goal_y - state.y) > params.position_accuracy)
        {
            // Dynamic Window
            float DynamicWindow[4] = {
                CLP::maximum(0.0f, state.v - params.max_acceleration * params.prediction_time),
                CLP::minimum(params.max_velocity, state.v + params.max_acceleration * params.prediction_time),
                CLP::maximum(-params.max_angular_velocity, state.omega - params.max_angular_acceleration * params.prediction_time),
                CLP::minimum(params.max_angular_velocity, state.omega + params.max_angular_acceleration * params.prediction_time)};

            auto v_range = CLP::arange<float>(DynamicWindow[0], DynamicWindow[1], params.velocity_sampling_step);
            auto omega_range = CLP::arange<float>(DynamicWindow[2], DynamicWindow[3], params.angular_velocity_sampling_step);
            auto t_prediction_range = CLP::arange<float>(0.0f, params.prediction_time, params.sampling_period);

            int v_range_size = v_range.size();
            int omega_range_size = omega_range.size();
            int t_prediction_size = t_prediction_range.size();

            vector<vector<vector<vector<float>>>> eval_trajectory(v_range_size,
                vector<vector<vector<float>>>(omega_range_size,
                    vector<vector<float>>(t_prediction_size, vector<float>(3, 0.0f))));

            vector<vector<float>> eval_function(v_range_size, vector<float>(omega_range_size, 0));

            float opt_max_eval_fun = -999.0f;
            int opt_idx[2] = {0, 0};

            for (int i = 0; i < v_range_size; i++)
            {
                for (int j = 0; j < omega_range_size; j++)
                {
                    float eval_v = v_range[i];
                    float eval_omega = omega_range[j];
                    eval_trajectory[i][j][0][0] = state.x;
                    eval_trajectory[i][j][0][1] = state.y;
                    eval_trajectory[i][j][0][2] = state.theta;
                    bool stoploop = false;
                    float min_dist = 999;

                    for (int k = 1; k < t_prediction_size; k++)
                    {
                        float x_prev = eval_trajectory[i][j][k - 1][0];
                        float y_prev = eval_trajectory[i][j][k - 1][1];
                        float theta_prev = eval_trajectory[i][j][k - 1][2];

                        eval_trajectory[i][j][k][2] = theta_prev + params.sampling_period * eval_omega;
                        eval_trajectory[i][j][k][0] = x_prev + params.sampling_period * cos(theta_prev) * eval_v;
                        eval_trajectory[i][j][k][1] = y_prev + params.sampling_period * sin(theta_prev) * eval_v;

                        // Check collisions LIDAR
                        for (int b = 0; b < lidar_N; b++)
                        {
                            if (!isnan(ranges[b]) && !isinf(ranges[b]) && ranges[b] < lidar_params.max_range && ranges[b] > lidar_params.min_range)
                            {
                                float dist = CLP::hypot(lidar_x[b] - eval_trajectory[i][j][k][0], lidar_y[b] - eval_trajectory[i][j][k][1]);

                                if (eval_v > sqrt(2 * dist * params.max_acceleration / 2) || eval_omega > sqrt(2 * dist * params.max_angular_acceleration / 2) || dist <= params.obstacle_distance)
                                {
                                    stoploop = true;
                                    break;
                                }
                                if (dist < min_dist)
                                    min_dist = dist;
                            }
                        }

                        if (stoploop)
                            break;

                        // Check collisions CAM
                        for (int b = 0; b < cam_params.image_cols; b++)
                        {
                            if (MemCAM.IsValid[b] == true)
                            {
                                float dist = CLP::hypot(MemCAM.X[b] - eval_trajectory[i][j][k][0], MemCAM.Y[b] - eval_trajectory[i][j][k][1]);

                                if (eval_v > sqrt(2 * dist * params.max_acceleration / 2) || eval_omega > sqrt(2 * dist * params.max_angular_acceleration / 2) || dist <= params.obstacle_distance)
                                {
                                    stoploop = true;
                                    break;
                                }
                                if (dist < min_dist)
                                    min_dist = dist;
                            }
                        }

                        if (stoploop)
                            break;
                    }

                    if (stoploop)
                        continue;

                    float robot_theta = eval_trajectory[i][j][t_prediction_size - 1][2];
                    float goaltheta = atan2(goal_y - eval_trajectory[i][j][t_prediction_size - 1][1],
                                            goal_x - eval_trajectory[i][j][t_prediction_size - 1][0]);

                    CLP::normalize_angle(robot_theta);
                    CLP::normalize_angle(goaltheta);
                    float robot_e = robot_theta - goaltheta;
                    CLP::normalize_angle(robot_e);

                    float velocityReward = params.velocity_weight * eval_v;
                    float headingReward = params.heading_weight * cos(robot_e);
                    float distanceReward = params.distance_weight * min_dist;
                    

                    eval_function[i][j] = headingReward + distanceReward + velocityReward;

                    if (eval_function[i][j] > opt_max_eval_fun)
                    {
                        opt_idx[0] = i;
                        opt_idx[1] = j;
                        opt_max_eval_fun = eval_function[i][j];
                    }
                }
            }

            try
            {
                output.linear_velocity = v_range[opt_idx[0]];
                output.angular_velocity = omega_range[opt_idx[1]];
            }
            catch (...)
            {
                output.linear_velocity = 0;
                output.angular_velocity = 0;
            }
            state.v = output.linear_velocity;
            state.omega = output.angular_velocity;
        }
        else
        {
            output.linear_velocity = 0;
            output.angular_velocity = 0;
            state.v = output.linear_velocity;
            state.omega = output.angular_velocity;
        }

        
    }

}