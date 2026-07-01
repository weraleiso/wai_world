/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Simple base_link TF publisher
 * Author: W. A. Isop
 * Date: August 2019
 */

#ifndef GAZEBO_ROS_QUADCON_HH
#define GAZEBO_ROS_QUADCON_HH

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
    class PIDController
    {
        public:
        PIDController()
        {
            d_gain_p=0.0;
            d_gain_i=0.0;
            d_gain_d=0.0;
            d_time_constant=0.0;
            d_limit=0.0;
            d_input=0.0;
            d_delta_input=0.0;
            d_output=0.0;
            d_error_p,d_error_i,d_error_d=0.0;
        }
        ~PIDController()
        {
        }

        double d_gain_p;
        double d_gain_i;
        double d_gain_d;
        double d_time_constant;
        double d_limit;
        double d_input;
        double d_delta_input;
        double d_output;
        double d_error_p,d_error_i,d_error_d;

        void Initialize(double p=0.0,double i=0.0,double d=0.0,double tconstant=0.0,double limit=1.0)
        {
            d_gain_p=p;
            d_gain_i=i;
            d_gain_d=d;
            d_time_constant=tconstant;
            d_limit=limit;
        }

        /*
        double Update(double d_new_input,double d_x,double d_delta_x,double d_delta_t)
        {
            if(d_delta_t+d_time_constant>0.0)
            {
                d_delta_input=(d_new_input-d_input)/(d_delta_t+d_time_constant);
                d_input=(d_delta_t*d_new_input+d_time_constant*d_input)/(d_delta_t+d_time_constant);
            }
            d_error_p=d_input-d_x;
            d_error_d=d_delta_input-d_delta_x;
            d_error_i=d_error_i+d_delta_t*d_error_p;

            d_output= d_gain_p*d_error_p + d_gain_d*d_error_d + d_gain_i*d_error_i;

            if(d_output>d_limit) d_output=d_limit;
            if(d_output<-1.0*d_limit) d_output=-1.0*d_limit;

            return d_output;
        }
        */

        double Update(double d_cmd,double d_act,double d_dummy,double d_delta_t)
        {
            d_error_p=d_cmd-d_act;
            d_error_d=(d_cmd-d_act)/d_delta_t;
            d_error_i=d_error_i+d_delta_t*d_error_p;

            d_output= d_gain_p*d_error_p + d_gain_i*d_error_i + d_gain_d*d_error_d;

            if(d_output>d_limit) d_output=d_limit;
            if(d_output<-1.0*d_limit) d_output=-1.0*d_limit;

            return d_output;
        }

        void reset()
        {
            d_input=d_delta_input=d_output=d_error_p=d_error_d=d_error_i=0.0;
        }
    };

    class GazeboRosQuadcon : public ModelPlugin
    {
        /// \brief Constructor
        public: GazeboRosQuadcon();

        /// \brief Destructor
        public: virtual ~GazeboRosQuadcon();

        /// \brief Load the controller
        public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

        private:
        ros::NodeHandle m_nh;

        ros::Subscriber sub_cmd_vel;

        event::ConnectionPtr update_connection_;
        ros::Time m_tim_iteration_last;

        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        std::string m_s_world_name;
        std::string m_s_model_name;
        std::string m_s_link_name;

        double m_d_update_rate;
        double m_d_update_interval;
        double m_d_delta_t;
        bool b_enable_pos_controller;

        double m_d_mass;
        ignition::math::Vector3d v3d_d_inertia;

        ignition::math::Pose3d p3d_act_pose;
        ignition::math::Quaterniond qua_act_rot;
        ignition::math::Vector3d v3d_act_rot_euler;
        ignition::math::Quaterniond qua_act_rot_global_local;
        // v and a given in world frame
        ignition::math::Vector3d v3d_act_vel_lin;
        ignition::math::Vector3d v3d_act_vel_rot;
        ignition::math::Vector3d v3d_act_acc_lin;
        ignition::math::Vector3d v3d_act_acc_rot;

        // v and a given in lcoal quadcopter frame
        ignition::math::Vector3d v3d_act_vel_lin_local;
        ignition::math::Vector3d v3d_act_vel_rot_local;
        ignition::math::Vector3d v3d_act_acc_lin_local;
        ignition::math::Vector3d v3d_act_acc_rot_local;

        ignition::math::Vector3d v3d_cmd_pos_lin; // World coordinates X, Y, and Z for POS controller
        ignition::math::Vector3d v3d_cmd_pos_rot; // Euler angle commands - YAW, PITCH, and ROLL
        ignition::math::Vector3d v3d_cmd_vel_lin;
        ignition::math::Vector3d v3d_cmd_vel_rot;
        ignition::math::Vector3d v3d_cmd_acc_lin;
        ignition::math::Vector3d v3d_cmd_acc_rot;

        ignition::math::Vector3d v3d_out_acc_lin; // Force output
        ignition::math::Vector3d v3d_out_acc_rot; // Torque output

        // PID Controllers
        PIDController pid_pos_lin_x;
        PIDController pid_pos_lin_y;
        PIDController pid_pos_lin_z;
        PIDController pid_pos_rot_roll;
        PIDController pid_pos_rot_pitch;
        PIDController pid_pos_rot_yaw;
        PIDController pid_vel_lin_x;
        PIDController pid_vel_lin_y;
        PIDController pid_vel_lin_z;
        PIDController pid_vel_rot_roll;
        PIDController pid_vel_rot_pitch;
        PIDController pid_vel_rot_yaw;
        PIDController pid_acc_rot_roll;
        PIDController pid_acc_rot_pitch;
        PIDController pid_acc_rot_yaw;

        void cb_cmd_vel(const geometry_msgs::TwistPtr& msg);

        /// \brief Update the controller
        protected: virtual void UpdateChild();

    };
}

#endif

