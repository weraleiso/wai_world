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
 * Desc: State Info Plugin
 * Author: W. A. Isop
 * Date: August 2019
 */

#ifndef GAZEBO_ROS_STATE_HH
#define GAZEBO_ROS_STATE_HH

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
    class GazeboRosState : public ModelPlugin
    {
        /// \brief Constructor
        public: GazeboRosState();

        /// \brief Destructor
        public: virtual ~GazeboRosState();

        /// \brief Load the controller
        public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );

        public: void cb_sub_hea_reset(const std_msgs::HeaderPtr& msg);

        // Pointer to the update event connection
        private:
        ros::NodeHandle* node_handle_;

        event::ConnectionPtr update_connection_;
        ros::Time tim_iteration_last_;
        physics::WorldPtr world_;
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        float f_tf_pub_interval_;
        float f_label_offset_;
        std::string s_world_name_;
        std::string s_model_name_;
        std::string s_link_name_;

        // Members to indicate state
        ros::Publisher pub_pst_pose_world_;
        ros::Publisher pub_pst_pose_world_deg_;
        ros::Publisher pub_pst_pose_relative_;
        ros::Publisher pub_pst_pose_relative_deg_;
        ros::Publisher pub_tst_twist_;
        ros::Publisher pub_tst_twist_rviz_;
        ros::Publisher pub_ast_acceleration_;
        ros::Publisher pub_ast_acceleration_rviz_;
        ros::Publisher pub_wst_wrench_;
        ros::Publisher pub_wst_wrench_rviz_;
        ros::Publisher pub_tst_energy_;

        ros::Publisher pub_str_state_;
        ros::Publisher pub_mrk_state_label_;
        ros::Subscriber sub_hea_reset;

        gazebo::common::Time gaz_tim_start;
        gazebo::common::Time gaz_tim_current;
        ros::Time tim_current;
        ros::Time tim_ros_start;

        geometry_msgs::PoseStamped msg_pst_pose_world_;
        geometry_msgs::TwistStamped msg_pst_pose_world_deg_;
        geometry_msgs::PoseStamped msg_pst_pose_relative_;
        geometry_msgs::TwistStamped msg_pst_pose_relative_deg_;
        geometry_msgs::TwistStamped msg_twi_twist_;
        geometry_msgs::AccelStamped msg_acc_acceleration_;
        geometry_msgs::WrenchStamped msg_wre_wrench_;
        geometry_msgs::TwistStamped msg_twi_energy_;
        std_msgs::String msg_str_state_;
        visualization_msgs::Marker msg_mrk_state_label_;
        ignition::math::Pose3d p3d_pose_world;
        ignition::math::Pose3d p3d_pose_relative;
        ignition::math::Vector3d v3d_vel_linear;
        ignition::math::Vector3d v3d_vel_angular;
        ignition::math::Vector3d v3d_acc_linear;
        ignition::math::Vector3d v3d_acc_angular;
        ignition::math::Vector3d v3d_force;
        ignition::math::Vector3d v3d_torque;
        tf::Quaternion qua_angle_to_deg;
        tf::Matrix3x3 mat_angle_to_deg;
        double d_yaw;
        double d_pitch;
        double d_roll;
        double d_energy_potential;
        double d_energy_kinetic;
        double d_energy_total;
        // std::stringstream sst_state_info;
        bool b_enable_pub_rqt_multiplot;
        bool b_enable_cont_pub;

        /// \brief Update the controller
        protected: virtual void UpdateChild();

    };
}

#endif

