#include "cam_lidar_planner_ros/APF.h"

namespace CLP
{
    APF::APF(ros::NodeHandle &nh) : LocalPlanner(nh)
    {
        // APF uses larger max depth for camera
        cam_params.max_depth = 1.5f;

        // APF-specific: subscribe to cmd_vel for current velocity feedback
        v_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &APF::TWIST_CALLBACK, this);
    }

    APF::~APF()
    {
        delete[] lidar_group;
    

    }
    //
    void APF::TWIST_CALLBACK(const geometry_msgs::Twist::ConstPtr &msg)
    {
        state.v = msg->linear.x;
        state.omega = msg->angular.z;
    }

    void APF::ProcesLidar()
    {
        if (!lidar_msg_)
        {
            ROS_WARN_THROTTLE(1.0, "APF::ProcesLidar called with no lidar message");
            return;
        }
        angle_min = lidar_msg_->angle_min;
        angle_max = lidar_msg_->angle_max;
        angle_increment = lidar_msg_->angle_increment;
        lidar_N = (int)((angle_max - angle_min) / angle_increment) + 1;

        if (!firstinitProcesLidar)
        {
            ranges = new float[lidar_N];
            lidar_x = new float[lidar_N];
            lidar_y = new float[lidar_N];
            lidar_group = new int[lidar_N];
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
        grouping2D(lidar_N);
    }

    

    void APF::grouping2D(int size)
    {
        C = 1;
        for (int i = 0; i < size; i++)
        {
            if (isnan(ranges[i]) || isinf(ranges[i]))
            {
                lidar_group[i] = 0;
                continue;
            }
            if (i == 0)
            {
                lidar_group[i] = C;
                continue;
            }
            if (isnan(ranges[i - 1]) || isinf(ranges[i - 1]))
                C++;
            else if (!isnan(ranges[i - 1]) && !isinf(ranges[i - 1]))
                if (fabs(ranges[i] - ranges[i - 1]) > 0.3)
                    C++;

            lidar_group[i] = C;
        }

        if (isfinite(ranges[size - 1]) && isfinite(ranges[0]))
        {
            if (fabs(ranges[size - 1] - ranges[0]) <= 0.3)
            {
                for (int i = size - 1; i > 1; i--)
                    if (lidar_group[i - 1] == lidar_group[i])
                        lidar_group[i] = 1;
                    else
                        break;
                lidar_group[size - 1] = 1;
            }
        }
    }

    void APF::controller()
    {
        geometry_msgs::Twist msg;

        float e = ref_angle - state.theta;
        CLP::normalize_angle(e);
        float abs_e = CLP::minimum(params.error_theta_max, fabs(e));
        float reduction_coefficient = (params.error_theta_max - abs_e) / params.error_theta_max;

        msg.angular.z = CLP::minimum(CLP::maximum(1.5f * e, -params.omega_max), params.omega_max);
        msg.linear.x = CLP::minimum(CLP::maximum(reduction_coefficient * output.linear_velocity, -params.v_max), params.v_max);

        ctr_pub.publish(msg);
    }

    void APF::compute()
    {
        ProcesImage();
        ProcesLidar();
        
        float distanceGoal = CLP::hypot(goal_x - state.x, goal_y - state.y);

        if (distanceGoal > params.position_toleration)
        {
            // Attractive force
            float U_att_x = params.att_coefficient * (state.x - goal_x);
            float U_att_y = params.att_coefficient * (state.y - goal_y);

            if (distanceGoal > params.goal_maximum_distance)
            {
                U_att_x *= 0.3f / distanceGoal;
                U_att_y *= 0.3f / distanceGoal;
            }

            float U_rep_x = 0;
            float U_rep_y = 0;

            // LIDAR repulsive (min values of grouping 2D)
            float min_dist = 1e10;
            float min_x = 0;
            float min_y = 0;

            for (int idxC = 1; idxC <= C; idxC++)
            {
                for (int i = 0; i < lidar_N; i++)
                {
                    if (lidar_group[i] == idxC)
                    {
                        if (ranges[i] < min_dist)
                        {
                            min_dist = ranges[i];
                            min_x = lidar_x[i];
                            min_y = lidar_y[i];
                        }
                    }
                }
                if (min_dist > 1e-6f && min_dist <= params.range)
                {
                    float U = params.rep_coefficient * (1.0f / params.range - 1.0f / min_dist) / (min_dist * min_dist);
                    U_rep_x += U * (state.x - min_x);
                    U_rep_y += U * (state.y - min_y);
                }
            }

            // Camera repulsive
            float min_dist2 = 1e10;

            for (int b = 0; b < cam_params.image_cols; b++)
            {
                if (MemCAM.IsValid[b] == true)
                {
                    float dist = CLP::hypot(MemCAM.X[b] - state.x, MemCAM.Y[b] - state.y);
                    if (dist <= min_dist2)
                    {
                        min_dist2 = dist;
                        min_x = MemCAM.X[b];
                        min_y = MemCAM.Y[b];
                    }
                }
            }

            if (min_dist2 > 1e-6f && min_dist2 <= params.range_cam)
            {
                
                float U = params.rep_coefficient * (1.0f / params.range_cam - 1.0f / min_dist2) / (min_dist2 * min_dist2);
                U_rep_x += U * (state.x - min_x);
                U_rep_y += U * (state.y - min_y);
            }

           

            // Rotation matrix
            float U_rep_xR = U_rep_x * cos(CLP::deg2rad(90)) - U_rep_y * sin(CLP::deg2rad(90));
            float U_rep_yR = U_rep_x * sin(CLP::deg2rad(90)) + U_rep_y * cos(CLP::deg2rad(90));

            float U_x = U_rep_xR + U_att_x;
            float U_y = U_rep_yR + U_att_y;

            output.linear_velocity = CLP::hypot(-U_x, -U_y);
            ref_angle = atan2(-U_y, -U_x);
        }
        else
        {
            output.linear_velocity = 0;
            ref_angle = state.theta;
        }

        
    }

}