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
 *  \desc   Simple PID Attitude Controller
 */

#include <gazebo_plugins/gazebo_ros_quadcon.h>
#include <ros/ros.h>



namespace gazebo
{
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosQuadcon);

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    GazeboRosQuadcon::GazeboRosQuadcon():m_nh()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    GazeboRosQuadcon::~GazeboRosQuadcon()
    {
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the controller
    void GazeboRosQuadcon::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
        this->world_=_parent->GetWorld();
        this->model_=_parent;

        if(_sdf->HasElement("worldFrame"))
        {
            m_s_world_name=_sdf->GetElement("worldFrame")->Get<std::string>();
        }
        else
        {
            m_s_world_name=this->world_->Name();
        }
        if(_sdf->HasElement("linkName"))
        {
            m_s_link_name=_sdf->GetElement("linkName")->Get<std::string>();
        }
        else
        {
            m_s_link_name="link_base";
        }
        m_s_model_name=_parent->GetName();

        // Get link from model
        link_=this->world_->ModelByName(m_s_model_name)->GetLink(m_s_link_name);

        if(_sdf->HasElement("updateRate"))
        {
            m_d_update_rate=_sdf->GetElement("updateRate")->Get<double>();
        }
        else
        {
            m_d_update_rate=20.0;
        }
        m_d_update_interval=1.0/m_d_update_rate;
        m_d_delta_t=m_d_update_interval; // Calculate true delta_t every simulation interation

        // Get physical properties
        m_d_mass=link_->GetInertial()->Mass();
        v3d_d_inertia=link_->GetInertial()->PrincipalMoments();

        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("ros_gazebo_quadcopter_controller", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        // Initialize subscribers
        //sub_cmd_vel=m_nh.subscribe("/wai_world/"+m_s_model_name+"/cmd_vel",1,&GazeboRosQuadcon::cb_cmd_vel,this);

        // Initialize members
        b_enable_pos_controller=false;

        p3d_act_pose.Reset();
        qua_act_rot.Set(1.0,0.0,0.0,0.0);
        v3d_act_rot_euler.Set();
        qua_act_rot_global_local.Set(1.0,0.0,0.0,0.0);
        // v and a given in world frame
        v3d_act_vel_lin.Set();
        v3d_act_vel_rot.Set();
        v3d_act_acc_lin.Set();
        v3d_act_acc_rot.Set();
        // v and a given in lcoal quadcopter frame
        v3d_act_vel_lin_local.Set();
        v3d_act_vel_rot_local.Set();
        v3d_act_acc_lin_local.Set();
        v3d_act_acc_rot_local.Set();
        v3d_cmd_pos_lin.Set(); // Commands (controller inputs/outputs)
        v3d_cmd_pos_rot.Set();
        v3d_cmd_vel_lin.Set();
        v3d_cmd_vel_rot.Set();
        v3d_cmd_acc_lin.Set();
        v3d_cmd_acc_rot.Set();
        v3d_out_acc_lin.Set(); // Final controller outputs for accaleration (FORCEs)
        v3d_out_acc_rot.Set(); // Final controller outputs for accaleration (TORQUEs)

        // Y-Axis (Steered via ROLL-command!)
        pid_pos_lin_y.Initialize(0.1,0.0,0.1,0.0,100.0); //(!)
        pid_vel_lin_y.Initialize(0.1,0.0,0.1,0.0,10.0);
        pid_pos_rot_roll.Initialize(0.1,0.0,0.1,0.0,1.0);

        pid_pos_lin_x.Initialize(0.1,0.0,0.1,0.0,100.0); //(!)
        pid_vel_lin_x.Initialize(0.1,0.0,0.1,0.0,10.0);
        pid_pos_rot_pitch.Initialize(0.1,0.0,0.1,0.0,1.0);

        pid_pos_lin_z.Initialize(1.0,0.1); // POS-Lin Z
        pid_vel_lin_z.Initialize(1.0); // VEL-Lin Z

        pid_pos_rot_yaw.Initialize(0.1,0.0,0.0,0.0,10.0); // POS-Rot Z
        pid_vel_rot_yaw.Initialize(0.1,0.0,0.1,0.0,10.0); // VEL-Rot Z (High D Gain!)

        pid_vel_rot_roll.Initialize(1.0);
        pid_vel_rot_pitch.Initialize(1.0);
        pid_acc_rot_roll.Initialize(1.0);
        pid_acc_rot_pitch.Initialize(1.0);
        //pid_acc_rot_yaw.Initialize(1.0); Deprecated...

        // Test setpoints
        v3d_cmd_pos_lin.X()=1.0;
        v3d_cmd_pos_lin.Y()=1.0;
        v3d_cmd_pos_lin.Z()=1.0;
        v3d_cmd_pos_rot.Z()=1.0;

        m_tim_iteration_last=ros::Time::now();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_=event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosQuadcon::UpdateChild,this));
    }

    void GazeboRosQuadcon::cb_cmd_vel(const geometry_msgs::TwistPtr& msg)
    {
        v3d_cmd_vel_lin.Set(msg->linear.x,msg->linear.y,msg->linear.z);
        v3d_cmd_vel_rot.Z()=msg->angular.z;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Update the controller
    void GazeboRosQuadcon::UpdateChild()
    {
        m_d_delta_t=(ros::Time::now()-this->m_tim_iteration_last).toSec();
        if(m_d_delta_t>=m_d_update_interval)
        {
            this->m_tim_iteration_last=ros::Time::now();

            // SENSOR INPUT
            p3d_act_pose=link_->WorldPose();
            qua_act_rot=link_->WorldPose().Rot();
            v3d_act_rot_euler=link_->WorldPose().Rot().Euler();
            v3d_act_vel_lin=link_->WorldLinearVel();
            v3d_act_vel_rot=link_->WorldAngularVel();
            v3d_act_acc_lin=link_->WorldLinearAccel();
            v3d_act_acc_rot=link_->WorldAngularAccel();
            // Rotate v and a into local quadcopter frame
            qua_act_rot_global_local.Set(qua_act_rot.W(),0.0,0.0,qua_act_rot.Z());
            qua_act_rot_global_local.Normalize();
            //qua_act_rot_global_local=qua_act_rot;
            v3d_act_vel_lin_local=qua_act_rot_global_local.RotateVectorReverse(v3d_act_vel_lin);
            v3d_act_vel_rot_local=qua_act_rot.RotateVectorReverse(v3d_act_vel_rot);
            v3d_act_acc_lin_local=qua_act_rot_global_local.RotateVectorReverse(v3d_act_acc_lin);
            v3d_act_acc_rot_local=qua_act_rot.RotateVectorReverse(v3d_act_acc_rot);



            // CONTROLLER PROCESS
            // Y-coords via ROLL axis
            v3d_cmd_vel_lin.Y()=pid_pos_lin_y.Update(v3d_cmd_pos_lin.Y(),p3d_act_pose.Pos().Y(),v3d_act_vel_lin_local.Y(),m_d_delta_t);
            //v3d_cmd_pos_rot.X()=-1.0*pid_vel_lin_y.Update(v3d_cmd_vel_lin.Y(),v3d_act_vel_lin_local.Y(),v3d_act_acc_lin_local.Y(),m_d_delta_t);
            //v3d_cmd_acc_rot.X()=pid_pos_rot_roll.Update(v3d_cmd_pos_rot.X(),v3d_act_rot_euler.X(),v3d_act_vel_rot_local.X(),m_d_delta_t);
            v3d_cmd_acc_rot.X()=-1.0*pid_vel_lin_y.Update(v3d_cmd_vel_lin.Y(),v3d_act_vel_lin_local.Y(),v3d_act_acc_lin_local.Y(),m_d_delta_t);
            ROS_WARN("POS-Lin-Y C: %3.3f, A: %3.3f, O: %3.3f | VEL-Lin-Y C: %3.3f, A: %3.3f, O: %3.3f |  EUL-Rot-ROLL C: %3.3f, A: %3.3f, O: %3.3f",
                     v3d_cmd_pos_lin.Y(),p3d_act_pose.Pos().Y(),v3d_cmd_vel_lin.Y(),
                     v3d_cmd_vel_lin.Y(),v3d_act_vel_lin_local.Y(),v3d_cmd_pos_rot.X(),
                     v3d_cmd_pos_rot.X(),v3d_act_rot_euler.X(),v3d_cmd_acc_rot.X());

            // X-coords via PITCH axis
            v3d_cmd_vel_lin.X()=pid_pos_lin_x.Update(v3d_cmd_pos_lin.X(),p3d_act_pose.Pos().X(),v3d_act_vel_lin_local.X(),m_d_delta_t);
            //v3d_cmd_pos_rot.Y()=-1.0*pid_vel_lin_x.Update(v3d_cmd_vel_lin.X(),v3d_act_vel_lin_local.X(),v3d_act_acc_lin_local.X(),m_d_delta_t);
            //v3d_cmd_acc_rot.Y()=pid_pos_rot_pitch.Update(v3d_cmd_pos_rot.Y(),v3d_act_rot_euler.Y(),v3d_act_vel_rot_local.Y(),m_d_delta_t);
            v3d_cmd_acc_rot.Y()=+1.0*pid_vel_lin_x.Update(v3d_cmd_vel_lin.X(),v3d_act_vel_lin_local.X(),v3d_act_acc_lin_local.X(),m_d_delta_t);
            ROS_WARN("POS-Lin-X C: %3.3f, A: %3.3f, O: %3.3f | VEL-Lin-X C: %3.3f, A: %3.3f, O: %3.3f |  EUL-Rot-PITCH C: %3.3f, A: %3.3f, O: %3.3f",
                     v3d_cmd_pos_lin.X(),p3d_act_pose.Pos().X(),v3d_cmd_vel_lin.X(),
                     v3d_cmd_vel_lin.X(),v3d_act_vel_lin_local.X(),v3d_cmd_pos_rot.Y(),
                     v3d_cmd_pos_rot.Y(),v3d_act_rot_euler.Y(),v3d_cmd_acc_rot.Y());

            v3d_cmd_vel_lin.Z()=pid_pos_lin_z.Update(v3d_cmd_pos_lin.Z(),p3d_act_pose.Pos().Z(),v3d_act_vel_lin_local.Z(),m_d_delta_t);
            v3d_cmd_acc_lin.Z()=link_->GetInertial()->Mass()*pow(world_->Gravity().Length(),2.0)*(1.0+pid_vel_lin_z.Update(v3d_cmd_vel_lin.Z(),v3d_act_vel_lin_local.Z(),v3d_act_acc_lin_local.Z(),m_d_delta_t));
            //ROS_WARN("POS-Lin-Z CMD: %3.3f, ACT: %3.3f, OUT: %3.3f  |  VEL-Lin-Z CMD: %3.3f, ACT: %3.3f, OUT: %3.3f",v3d_cmd_pos_lin.Z(),p3d_act_pose.Pos().Z(),v3d_cmd_vel_lin.Z(),v3d_cmd_vel_lin.Z(),v3d_act_vel_lin_local.Z(),v3d_cmd_acc_lin.Z());

            v3d_cmd_vel_rot.Z()=pid_pos_rot_yaw.Update(v3d_cmd_pos_rot.Z(),v3d_act_rot_euler.Z(),v3d_act_vel_rot_local.Z(),m_d_delta_t);
            v3d_cmd_acc_rot.Z()=pid_vel_rot_yaw.Update(v3d_cmd_vel_rot.Z(),v3d_act_vel_rot_local.Z(),v3d_act_acc_rot_local.Z(),m_d_delta_t);
            //ROS_WARN("POS-Rot-Z CMD: %3.3f, ACT: %3.3f, OUT: %3.3f  |  VEL-Rot-Z CMD: %3.3f, ACT: %3.3f, OUT: %3.3f",v3d_cmd_pos_rot.Z(),v3d_act_rot_euler.Z(),v3d_cmd_vel_rot.Z(),v3d_cmd_vel_rot.Z(),v3d_act_vel_rot_local.Z(),v3d_cmd_acc_rot.Z());

            v3d_out_acc_lin.X()=0.0;
            v3d_out_acc_lin.Y()=0.0;
            v3d_out_acc_lin.Z()=v3d_cmd_acc_lin.Z();

            v3d_out_acc_rot.X()=v3d_cmd_acc_rot.X();
            v3d_out_acc_rot.Y()=v3d_cmd_acc_rot.Y();
            v3d_out_acc_rot.Z()=v3d_cmd_acc_rot.Z();



            // ACTUATOR OUTPUT
            //ROS_WARN("Fx-Fy-Fz: %3.3f - %3.3f - %3.3f ; Tx-Ty-Tz: %3.3f - %3.3f - %3.3f",v3d_out_acc_lin.X(),v3d_out_acc_lin.Y(),v3d_out_acc_lin.Z(),v3d_out_acc_rot.X(),v3d_out_acc_rot.Y(),v3d_out_acc_rot.Z());
            //link_->SetForce(v3d_out_acc_lin);
            //link_->SetTorque(v3d_out_acc_rot);
            link_->AddRelativeForce(v3d_out_acc_lin);
            link_->AddRelativeTorque(v3d_out_acc_rot-link_->GetInertial()->CoG().Cross(v3d_out_acc_lin));
        }
    }

}
