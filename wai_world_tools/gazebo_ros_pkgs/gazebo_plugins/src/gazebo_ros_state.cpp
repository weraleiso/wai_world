/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author W. A. Isop
 *  \desc   State Info Plugin
 */

#include <gazebo_plugins/gazebo_ros_state.h>
#include <ros/ros.h>



namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosState);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosState::GazeboRosState()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosState::~GazeboRosState()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosState::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    // Get the world name.
    this->world_=_parent->GetWorld();
    this->model_=_parent;

    if(_sdf->HasElement("worldFrame"))
    {
        s_world_name_=_sdf->GetElement("worldFrame")->Get<std::string>();
    }
    else
    {
        s_world_name_=this->world_->Name();
    }
    if(_sdf->HasElement("linkName"))
    {
        s_link_name_=_sdf->GetElement("linkName")->Get<std::string>();
    }
    else
    {
        s_link_name_="link_base";
    }
    s_model_name_=_parent->GetName();

    // Get link from model
    link_=this->world_->ModelByName(s_model_name_)->GetLink(s_link_name_);

    tim_iteration_last_=ros::Time::now();
    if(_sdf->HasElement("pubContinously"))
    {
        b_enable_cont_pub=_sdf->GetElement("pubContinously")->Get<bool>();
    }
    else
    {
        b_enable_cont_pub=false;
    }
    if(_sdf->HasElement("pubInterval"))
    {
        f_tf_pub_interval_=_sdf->GetElement("pubInterval")->Get<double>();
    }
    else
    {
        f_tf_pub_interval_=0.1; // Default is 10Hz
    }

    if(_sdf->HasElement("labelOffset"))
    {
        f_label_offset_=_sdf->GetElement("labelOffset")->Get<double>();
    }
    else
    {
        f_label_offset_=1.0; // Default is 1m
    }

    // rqt_plot replaced by rqt_multiplot, then timestamps are considered correctly!
    tim_ros_start=ros::Time::now();
    gaz_tim_current=this->world_->SimTime();
    gaz_tim_start=gaz_tim_current;
    //tim_current.sec=tim_ros_start.sec+(gaz_tim_current-gaz_tim_start).sec;
    //tim_current.nsec=tim_ros_start.nsec+(gaz_tim_current-gaz_tim_start).nsec;
    tim_current.sec=(gaz_tim_current-gaz_tim_start).sec;
    tim_current.nsec=(gaz_tim_current-gaz_tim_start).nsec;

    msg_pst_pose_world_.header.stamp=tim_current;//ros::Time::now();
    msg_pst_pose_world_.header.frame_id=s_world_name_;
    msg_pst_pose_world_.pose.position.x=0.0;
    msg_pst_pose_world_.pose.position.y=0.0;
    msg_pst_pose_world_.pose.position.z=0.0;
    msg_pst_pose_world_.pose.orientation.w=1.0;
    msg_pst_pose_world_.pose.orientation.x=0.0;
    msg_pst_pose_world_.pose.orientation.y=0.0;
    msg_pst_pose_world_.pose.orientation.z=0.0;
    msg_pst_pose_world_deg_.header.stamp=tim_current;//ros::Time::now();
    msg_pst_pose_world_deg_.header.frame_id=s_world_name_;
    msg_pst_pose_world_deg_.twist.linear.x=0.0;
    msg_pst_pose_world_deg_.twist.linear.y=0.0;
    msg_pst_pose_world_deg_.twist.linear.z=0.0;
    msg_pst_pose_world_deg_.twist.angular.x=0.0;
    msg_pst_pose_world_deg_.twist.angular.y=0.0;
    msg_pst_pose_world_deg_.twist.angular.z=0.0;

    msg_pst_pose_relative_.header.stamp=tim_current;//ros::Time::now();
    msg_pst_pose_relative_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_pst_pose_relative_.pose.position.x=0.0;
    msg_pst_pose_relative_.pose.position.y=0.0;
    msg_pst_pose_relative_.pose.position.z=0.0;
    msg_pst_pose_relative_.pose.orientation.w=1.0;
    msg_pst_pose_relative_.pose.orientation.x=0.0;
    msg_pst_pose_relative_.pose.orientation.y=0.0;
    msg_pst_pose_relative_.pose.orientation.z=0.0;
    msg_pst_pose_relative_deg_.header.stamp=tim_current;//ros::Time::now();
    msg_pst_pose_relative_deg_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_pst_pose_relative_deg_.twist.linear.x=0.0;
    msg_pst_pose_relative_deg_.twist.linear.y=0.0;
    msg_pst_pose_relative_deg_.twist.linear.z=0.0;
    msg_pst_pose_relative_deg_.twist.angular.x=0.0;
    msg_pst_pose_relative_deg_.twist.angular.y=0.0;
    msg_pst_pose_relative_deg_.twist.angular.z=0.0;

    msg_twi_twist_.header.stamp=tim_current;//ros::Time::now();
    msg_twi_twist_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_twi_twist_.twist.linear.x=0.0;
    msg_twi_twist_.twist.linear.y=0.0;
    msg_twi_twist_.twist.linear.z=0.0;
    msg_twi_twist_.twist.angular.x=0.0;
    msg_twi_twist_.twist.angular.y=0.0;
    msg_twi_twist_.twist.angular.z=0.0;

    msg_acc_acceleration_.header.stamp=tim_current;//ros::Time::now();
    msg_acc_acceleration_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_acc_acceleration_.accel.linear.x=0.0;
    msg_acc_acceleration_.accel.linear.y=0.0;
    msg_acc_acceleration_.accel.linear.z=0.0;
    msg_acc_acceleration_.accel.angular.x=0.0;
    msg_acc_acceleration_.accel.angular.y=0.0;
    msg_acc_acceleration_.accel.angular.z=0.0;

    msg_wre_wrench_.header.stamp=tim_current;//ros::Time::now();
    msg_wre_wrench_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_wre_wrench_.wrench.force.x=0.0;
    msg_wre_wrench_.wrench.force.y=0.0;
    msg_wre_wrench_.wrench.force.z=0.0;
    msg_wre_wrench_.wrench.torque.x=0.0;
    msg_wre_wrench_.wrench.torque.y=0.0;
    msg_wre_wrench_.wrench.torque.z=0.0;

    msg_twi_energy_.header.stamp=tim_current;//ros::Time::now();
    msg_twi_energy_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_twi_energy_.twist.linear.x=0.0;
    msg_twi_energy_.twist.linear.y=0.0;
    msg_twi_energy_.twist.linear.z=0.0;
    msg_twi_energy_.twist.angular.x=0.0;
    msg_twi_energy_.twist.angular.y=0.0;
    msg_twi_energy_.twist.angular.z=0.0;

    msg_str_state_.data="State Info...";

    msg_mrk_state_label_.header.frame_id=s_model_name_+"/"+s_link_name_;
    msg_mrk_state_label_.header.stamp=ros::Time();
    msg_mrk_state_label_.ns=s_model_name_+"/state";
    msg_mrk_state_label_.id=0;
    msg_mrk_state_label_.text="State Info...";
    msg_mrk_state_label_.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg_mrk_state_label_.action=visualization_msgs::Marker::ADD;
    msg_mrk_state_label_.pose.position.x=0.0;
    msg_mrk_state_label_.pose.position.y=0.0;
    msg_mrk_state_label_.pose.position.z=f_label_offset_;
    msg_mrk_state_label_.pose.orientation.x=0.0;
    msg_mrk_state_label_.pose.orientation.y=0.0;
    msg_mrk_state_label_.pose.orientation.z=0.0;
    msg_mrk_state_label_.pose.orientation.w=1.0;
    msg_mrk_state_label_.scale.x=0.125;
    msg_mrk_state_label_.scale.y=0.125;
    msg_mrk_state_label_.scale.z=0.125;
    msg_mrk_state_label_.color.r=0.0;
    msg_mrk_state_label_.color.g=1.0;
    msg_mrk_state_label_.color.b=1.0;
    msg_mrk_state_label_.color.a=1.0;

    p3d_pose_world.Reset();
    p3d_pose_relative.Reset();
    v3d_vel_linear.Set();
    v3d_vel_angular.Set();
    v3d_acc_linear.Set();
    v3d_acc_angular.Set();
    v3d_force.Set();
    v3d_torque.Set();
    d_yaw=0.0;
    d_pitch=0.0;
    d_roll=0.0;
    qua_angle_to_deg.setEulerZYX(0.0,0.0,0.0);
    mat_angle_to_deg.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
    d_energy_potential=0.0;
    d_energy_kinetic=0.0;
    d_energy_total=0.0;
    b_enable_pub_rqt_multiplot=false;

    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("state_info", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    node_handle_=new ros::NodeHandle(s_model_name_);

    pub_pst_pose_world_=node_handle_->advertise<geometry_msgs::PoseStamped>("state_pose",1);
    pub_pst_pose_world_deg_=node_handle_->advertise<geometry_msgs::TwistStamped>("state_pose_deg",1);
    pub_pst_pose_relative_=node_handle_->advertise<geometry_msgs::PoseStamped>("state_pose_rel",1);
    pub_pst_pose_relative_deg_=node_handle_->advertise<geometry_msgs::TwistStamped>("state_pose_rel_deg",1);
    pub_tst_twist_=node_handle_->advertise<geometry_msgs::TwistStamped>("state_twist",1);
    pub_tst_twist_rviz_=node_handle_->advertise<geometry_msgs::TwistStamped>("state_twist_rviz",1); // Two timestamps necessary (1 for simulation in Gazebo, 1 in real-time for RViz)
    pub_ast_acceleration_=node_handle_->advertise<geometry_msgs::AccelStamped>("state_acceleration",1);
    pub_ast_acceleration_rviz_=node_handle_->advertise<geometry_msgs::AccelStamped>("state_acceleration_rviz",1); // Two timestamps necessary (1 for simulation in Gazebo, 1 in real-time for RViz)
    pub_wst_wrench_=node_handle_->advertise<geometry_msgs::WrenchStamped>("state_wrench",1);
    pub_wst_wrench_rviz_=node_handle_->advertise<geometry_msgs::WrenchStamped>("state_wrench_rviz",1); // Two timestamps necessary (1 for simulation in Gazebo, 1 in real-time for RViz)
    pub_tst_energy_=node_handle_->advertise<geometry_msgs::TwistStamped>("state_energy",1);
    pub_str_state_=node_handle_->advertise<std_msgs::String>("state_string",1);
    pub_mrk_state_label_=node_handle_->advertise<visualization_msgs::Marker>("state_info",1);
    sub_hea_reset=node_handle_->subscribe("state_reset",1,&GazeboRosState::cb_sub_hea_reset,this);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosState::UpdateChild,this));
}

void GazeboRosState::cb_sub_hea_reset(const std_msgs::HeaderPtr& msg)
{
    if(std::strcmp(msg->frame_id.c_str(),"enable")==0)
    {
        gaz_tim_start.sec=msg->stamp.sec;
        gaz_tim_start.nsec=msg->stamp.nsec;
        b_enable_pub_rqt_multiplot=true;
        ROS_WARN("STATE Plugin: Time was RESET!");
    }
    else
    {
        b_enable_pub_rqt_multiplot=false;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosState::UpdateChild()
{
    // Publish state info
    if((ros::Time::now()-this->tim_iteration_last_).toSec()>=f_tf_pub_interval_)
    {
        tim_iteration_last_=ros::Time::now();

        /* Get parameters in WORLD frame:
        p3d_pose_relative=link_->WorldPose();
        v3d_vel_linear=link_->WorldLinearVel();
        v3d_vel_angular=link_->WorldAngularVel();
        v3d_acc_linear=link_->WorldLinearAccel();
        v3d_acc_angular=link_->WorldAngularAccel();
        v3d_force=link_->WorldForce();
        v3d_torque=link_->WorldTorque();
        d_energy_potential=link_->GetWorldEnergyPotential();
        d_energy_kinetic=link_->GetWorldEnergyKinetic();
        */
        p3d_pose_world=link_->WorldPose();
        p3d_pose_relative=link_->RelativePose();
        v3d_vel_linear=link_->RelativeLinearVel();
        v3d_vel_angular=link_->RelativeAngularVel();
        v3d_acc_linear=link_->RelativeLinearAccel();
        v3d_acc_angular=link_->RelativeAngularAccel();
        v3d_force=link_->RelativeForce();
        v3d_torque=link_->RelativeTorque();
        d_energy_potential=link_->GetWorldEnergyPotential();
        d_energy_kinetic=link_->GetWorldEnergyKinetic();
        d_energy_total=link_->GetWorldEnergy();

        if(     pub_pst_pose_world_
                && pub_pst_pose_relative_
                && pub_pst_pose_world_deg_
                && pub_pst_pose_relative_deg_
                && pub_tst_twist_
                && pub_ast_acceleration_
                && pub_wst_wrench_
                && pub_tst_energy_
                && pub_tst_twist_rviz_
                && pub_ast_acceleration_rviz_
                && pub_wst_wrench_rviz_
                && pub_str_state_
                && pub_mrk_state_label_
                && p3d_pose_world.IsFinite()
                && p3d_pose_relative.IsFinite()
                && v3d_vel_linear.IsFinite()
                && v3d_vel_angular.IsFinite()
                && v3d_acc_linear.IsFinite()
                && v3d_acc_angular.IsFinite()
                && v3d_force.IsFinite()
                && v3d_torque.IsFinite()
                && !isnan(d_energy_potential)
                && !isnan(d_energy_kinetic)
                && !isnan(d_energy_total)
                )
        {
            // rqt_plot replaced by rqt_multiplot, then timestamps are considered correctly!
            gaz_tim_current=this->world_->SimTime();
            //tim_current.sec=tim_ros_start.sec+(gaz_tim_current-gaz_tim_start).sec;
            //tim_current.nsec=tim_ros_start.nsec+(gaz_tim_current-gaz_tim_start).nsec;
            tim_current.sec=(gaz_tim_current-gaz_tim_start).sec;
            tim_current.nsec=(gaz_tim_current-gaz_tim_start).nsec;

            msg_pst_pose_world_.header.seq=0;
            msg_pst_pose_world_.header.stamp=tim_current;//ros::Time::now();
            msg_pst_pose_world_.header.frame_id=s_world_name_;
            msg_pst_pose_world_.pose.position.x=p3d_pose_world.Pos().X();
            msg_pst_pose_world_.pose.position.y=p3d_pose_world.Pos().Y();
            msg_pst_pose_world_.pose.position.z=p3d_pose_world.Pos().Z();
            msg_pst_pose_world_.pose.orientation.w=p3d_pose_world.Rot().W();
            msg_pst_pose_world_.pose.orientation.x=p3d_pose_world.Rot().X();
            msg_pst_pose_world_.pose.orientation.y=p3d_pose_world.Rot().Y();
            msg_pst_pose_world_.pose.orientation.z=p3d_pose_world.Rot().Z();
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_pst_pose_world_.publish(msg_pst_pose_world_);
            msg_pst_pose_world_deg_.header.seq=0;
            msg_pst_pose_world_deg_.header.stamp=tim_current;//ros::Time::now();
            msg_pst_pose_world_deg_.header.frame_id=s_world_name_;
            msg_pst_pose_world_deg_.twist.linear.x=p3d_pose_world.Pos().X();
            msg_pst_pose_world_deg_.twist.linear.y=p3d_pose_world.Pos().Y();
            msg_pst_pose_world_deg_.twist.linear.z=p3d_pose_world.Pos().Z();
            qua_angle_to_deg.setW(p3d_pose_world.Rot().W());
            qua_angle_to_deg.setX(p3d_pose_world.Rot().X());
            qua_angle_to_deg.setY(p3d_pose_world.Rot().Y());
            qua_angle_to_deg.setZ(p3d_pose_world.Rot().Z());
            mat_angle_to_deg.setRotation(qua_angle_to_deg);
            mat_angle_to_deg.getEulerYPR(d_yaw,d_pitch,d_roll);
            msg_pst_pose_world_deg_.twist.angular.x=d_roll*180.0/M_PI;
            msg_pst_pose_world_deg_.twist.angular.y=d_pitch*180.0/M_PI;
            msg_pst_pose_world_deg_.twist.angular.z=d_yaw*180.0/M_PI;
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_pst_pose_world_deg_.publish(msg_pst_pose_world_deg_);

            msg_pst_pose_relative_.header.seq=0;
            msg_pst_pose_relative_.header.stamp=tim_current;//ros::Time::now();
            msg_pst_pose_relative_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_pst_pose_relative_.pose.position.x=p3d_pose_relative.Pos().X();
            msg_pst_pose_relative_.pose.position.y=p3d_pose_relative.Pos().Y();
            msg_pst_pose_relative_.pose.position.z=p3d_pose_relative.Pos().Z();
            msg_pst_pose_relative_.pose.orientation.w=p3d_pose_relative.Rot().W();
            msg_pst_pose_relative_.pose.orientation.x=p3d_pose_relative.Rot().X();
            msg_pst_pose_relative_.pose.orientation.y=p3d_pose_relative.Rot().Y();
            msg_pst_pose_relative_.pose.orientation.z=p3d_pose_relative.Rot().Z();
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_pst_pose_relative_.publish(msg_pst_pose_relative_);
            msg_pst_pose_relative_deg_.header.seq=0;
            msg_pst_pose_relative_deg_.header.stamp=tim_current;//ros::Time::now();
            msg_pst_pose_relative_deg_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_pst_pose_relative_deg_.twist.linear.x=p3d_pose_relative.Pos().X();
            msg_pst_pose_relative_deg_.twist.linear.y=p3d_pose_relative.Pos().Y();
            msg_pst_pose_relative_deg_.twist.linear.z=p3d_pose_relative.Pos().Z();
            qua_angle_to_deg.setW(p3d_pose_relative.Rot().W());
            qua_angle_to_deg.setX(p3d_pose_relative.Rot().X());
            qua_angle_to_deg.setY(p3d_pose_relative.Rot().Y());
            qua_angle_to_deg.setZ(p3d_pose_relative.Rot().Z());
            mat_angle_to_deg.setRotation(qua_angle_to_deg);
            mat_angle_to_deg.getEulerYPR(d_yaw,d_pitch,d_roll);
            msg_pst_pose_relative_deg_.twist.angular.x=d_roll*180.0/M_PI;
            msg_pst_pose_relative_deg_.twist.angular.y=d_pitch*180.0/M_PI;
            msg_pst_pose_relative_deg_.twist.angular.z=d_yaw*180.0/M_PI;
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_pst_pose_relative_deg_.publish(msg_pst_pose_relative_deg_);

            msg_twi_twist_.header.stamp=tim_current;//ros::Time::now();
            msg_twi_twist_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_twi_twist_.twist.linear.x=v3d_vel_linear.X();
            msg_twi_twist_.twist.linear.y=v3d_vel_linear.Y();
            msg_twi_twist_.twist.linear.z=v3d_vel_linear.Z();
            msg_twi_twist_.twist.angular.x=0.0;//v3d_vel_angular.X();
            msg_twi_twist_.twist.angular.y=0.0;//v3d_vel_angular.Y();
            msg_twi_twist_.twist.angular.z=v3d_vel_angular.Z();
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_tst_twist_.publish(msg_twi_twist_);
            msg_twi_twist_.header.stamp=ros::Time::now();
            pub_tst_twist_rviz_.publish(msg_twi_twist_);

            msg_acc_acceleration_.header.stamp=tim_current;//ros::Time::now();
            msg_acc_acceleration_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_acc_acceleration_.accel.linear.x=v3d_acc_linear.X();
            msg_acc_acceleration_.accel.linear.y=v3d_acc_linear.Y();
            msg_acc_acceleration_.accel.linear.z=v3d_acc_linear.Z();
            msg_acc_acceleration_.accel.angular.x=0.0;//v3d_acc_angular.X();
            msg_acc_acceleration_.accel.angular.y=0.0;//v3d_acc_angular.X();
            msg_acc_acceleration_.accel.angular.z=v3d_acc_angular.X();
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_ast_acceleration_.publish(msg_acc_acceleration_);
            msg_acc_acceleration_.header.stamp=ros::Time::now();
            pub_ast_acceleration_rviz_.publish(msg_acc_acceleration_);

            msg_wre_wrench_.header.stamp=tim_current;//ros::Time::now();
            msg_wre_wrench_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_wre_wrench_.wrench.force.x=v3d_force.X();
            msg_wre_wrench_.wrench.force.y=v3d_force.Y();
            msg_wre_wrench_.wrench.force.z=v3d_force.Z();
            msg_wre_wrench_.wrench.torque.x=0.0;//v3d_torque.X();
            msg_wre_wrench_.wrench.torque.y=0.0;//v3d_torque.Y();
            msg_wre_wrench_.wrench.torque.z=v3d_torque.Z();
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_wst_wrench_.publish(msg_wre_wrench_);
            msg_wre_wrench_.header.stamp=ros::Time::now();
            pub_wst_wrench_rviz_.publish(msg_wre_wrench_);

            msg_twi_energy_.header.stamp=tim_current;//ros::Time::now();
            msg_twi_energy_.header.frame_id=s_model_name_+"/"+s_link_name_;
            msg_twi_energy_.twist.linear.x=d_energy_potential;
            msg_twi_energy_.twist.linear.y=d_energy_kinetic;
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_tst_energy_.publish(msg_twi_energy_);

            // Format output string with printf
            char cstr_state_info[512]="State Info...";
            sprintf(cstr_state_info,"X(L,R): %+08.3lf %+08.3lf %+08.3lf, %+08.3lf %+08.3lf %+08.3lf\nV(L,R): %+08.3lf %+08.3lf %+08.3lf, %+08.3lf %+08.3lf %+08.3lf\nA(L,R): %+08.3lf %+08.3lf %+08.3lf, %+08.3lf %+08.3lf %+08.3lf\nF(L,R): %+08.3lf %+08.3lf %+08.3lf, %+08.3lf %+08.3lf %+08.3lf\nE(P,K,T): %+08.3lf, %+08.3lf, %+08.3lf",
                    p3d_pose_world.Pos().X(),
                    p3d_pose_world.Pos().Y(),
                    p3d_pose_world.Pos().Z(),
                    p3d_pose_world.Roll(),
                    p3d_pose_world.Pitch(),
                    p3d_pose_world.Yaw(),
                    v3d_vel_linear.X(),
                    v3d_vel_linear.Y(),
                    v3d_vel_linear.Z(),
                    v3d_vel_angular.X(),
                    v3d_vel_angular.Y(),
                    v3d_vel_angular.Z(),
                    v3d_acc_linear.X(),
                    v3d_acc_linear.Y(),
                    v3d_acc_linear.Z(),
                    v3d_acc_angular.X(),
                    v3d_acc_angular.Y(),
                    v3d_acc_angular.Z(),
                    v3d_force.X(),
                    v3d_force.Y(),
                    v3d_force.Z(),
                    v3d_torque.X(),
                    v3d_torque.Y(),
                    v3d_torque.Z(),
                    d_energy_potential,d_energy_kinetic,d_energy_total);
            std::string s_cstr_state_info(cstr_state_info);
            /* Exemplary std stringstream for state summary:
                sst_state_info.str("");
                sst_state_info << std::setfill('0') << std::setw(7) << std::fixed << std::setprecision(3)
                           << "x_pos,x_rpy: " << p3d_pose.Pos() << "(m), " << p3d_pose.Roll() << " " << p3d_pose.Pitch() << " " << p3d_pose.Yaw() << "(rad)" << std::endl
                           << "v_lin,v_ang: " << v3d_vel_linear << "(m/s), " << v3d_vel_angular << "(rad/s)" << std::endl
                           << "a_lin,a_ang: " << v3d_acc_linear << "(m/s2), " << v3d_acc_angular << "(rad/s2)" << std::endl
                           << "Force,Torqu: " << v3d_force << "(N), " << v3d_torque << "(Nm)" << std::endl
                           << "E_pot,E_kin: " << d_energy_potential << "(J), " << d_energy_kinetic << "(J)";
            } */
            msg_str_state_.data=s_cstr_state_info;
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_str_state_.publish(msg_str_state_);

            msg_mrk_state_label_.header.stamp=ros::Time();
            msg_mrk_state_label_.text=s_cstr_state_info;
            if(b_enable_pub_rqt_multiplot || b_enable_cont_pub) pub_mrk_state_label_.publish(msg_mrk_state_label_);
        }

    }
}

}
