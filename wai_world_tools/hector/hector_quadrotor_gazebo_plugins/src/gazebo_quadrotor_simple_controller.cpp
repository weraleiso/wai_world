//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <hector_quadrotor_gazebo_plugins/gazebo_quadrotor_simple_controller.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>



namespace gazebo
{

    GazeboQuadrotorSimpleController::GazeboQuadrotorSimpleController()
    {
    }

    // Destructor
    GazeboQuadrotorSimpleController::~GazeboQuadrotorSimpleController()
    {
        updateConnection.reset();
        node_handle_->shutdown();
        delete node_handle_;
    }

    // Load the controller
    void GazeboQuadrotorSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        world=_model->GetWorld();
        model_name_=_model->GetName();
        link=_model->GetLink();
        link_name_=link->GetName();

        // Initialize members
        namespace_.clear();
        velocity_topic_="cmd_vel";
        //imu_topic_="imu";
        //odom_topic_="odom";
        max_force_=20.0;
        landing_takeoff_speed_=0.25;
        landing_takeoff_height_=1.0;
        contr_state_=0;
        b_enable_pos_controller=false;
        state_stamp=ros::Time::now();
        pose.Reset();
        euler.Set();
        velocity.Set();
        acceleration.Set();
        angular_velocity.Set();
        velocity_command_.angular.x=0.0;
        velocity_command_.angular.y=0.0;
        velocity_command_.angular.z=0.0;
        velocity_command_.linear.x=0.0;
        velocity_command_.linear.y=0.0;
        velocity_command_.linear.z=0.0;

        // Load parameters from sdf
        if (_sdf->HasElement("robotNamespace")) namespace_=_sdf->GetElement("robotNamespace")->Get<std::string>();
        if (_sdf->HasElement("cmdvelTopic")) velocity_topic_=_sdf->GetElement("cmdvelTopic")->Get<std::string>();
        if (_sdf->HasElement("imuTopic")) imu_topic_=_sdf->GetElement("imuTopic")->Get<std::string>();
        if (_sdf->HasElement("odomTopic")) odom_topic_=_sdf->GetElement("odomTopic")->Get<std::string>();
        if (_sdf->HasElement("maxForce")) max_force_=_sdf->GetElement("maxForce")->Get<double>();
        if (_sdf->HasElement("landingtakeoffSpeed")) landing_takeoff_speed_=_sdf->GetElement("landingtakeoffSpeed")->Get<double>();
        if (_sdf->HasElement("landingtakeoffHeight")) landing_takeoff_height_=_sdf->GetElement("landingtakeoffHeight")->Get<double>();
        if (_sdf->HasElement("bodyName") && _sdf->GetElement("bodyName")->GetValue())
        {
            link_name_=_sdf->GetElement("bodyName")->Get<std::string>();
            link=_model->GetLink(link_name_);
        }
        if (!link)
        {
            ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n",link_name_.c_str());
            return;
        }

        // Configure controllers
        controllers_.roll.Load(_sdf,"rollpitch");
        controllers_.pitch.Load(_sdf,"rollpitch");
        controllers_.yaw.Load(_sdf,"yaw");
        controllers_.velocity_x.Load(_sdf,"velocityXY");
        controllers_.velocity_y.Load(_sdf,"velocityXY");
        controllers_.velocity_z.Load(_sdf,"velocityZ");
        controllers_.position_x.Load(_sdf,"positionXY");
        controllers_.position_y.Load(_sdf,"positionXY");
        controllers_.position_z.Load(_sdf,"positionZ");

        // Get inertia and mass of quadrotor body
        inertia=link->GetInertial()->PrincipalMoments();
        mass=link->GetInertial()->Mass();

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        node_handle_=new ros::NodeHandle(namespace_);

        // Subscribers
        odom_subscriber_=node_handle_->subscribe("odom",1,&GazeboQuadrotorSimpleController::OdomCallback,this);
        imu_subscriber_=node_handle_->subscribe("imu",1,&GazeboQuadrotorSimpleController::ImuCallback,this);
        velocity_subscriber_=node_handle_->subscribe("cmd_vel",1,&GazeboQuadrotorSimpleController::CommandVelocityCallback,this);
        idle_subscriber_=node_handle_->subscribe("idle",1,&GazeboQuadrotorSimpleController::IdleCallback,this);
        startup_subscriber_=node_handle_->subscribe("startup",1,&GazeboQuadrotorSimpleController::StartupCallback,this);
        takeoff_subscriber_=node_handle_->subscribe("takeoff",1,&GazeboQuadrotorSimpleController::TakeoffCallback,this);
        land_subscriber_=node_handle_->subscribe("land",1,&GazeboQuadrotorSimpleController::LandCallback,this);
        shutdown_subscriber_=node_handle_->subscribe("shutdown",1,&GazeboQuadrotorSimpleController::ShutdownCallback,this);

        Reset();
        controlTimer.Load(world, _sdf);
        updateConnection=event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboQuadrotorSimpleController::Update,this));
    }

    // Callbacks
    void GazeboQuadrotorSimpleController::IdleCallback(const std_msgs::EmptyConstPtr&)
    {
        contr_state_=0; // IDLE
        ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State IDLE.");
    }
    void GazeboQuadrotorSimpleController::StartupCallback(const std_msgs::EmptyConstPtr&)
    {
        rotor_01_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_middle_velocity_controller/command",1,true);
        rotor_02_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_top_velocity_controller/command",1,true);
        std_msgs::Float64 msg_rotor_01_vel;
        std_msgs::Float64 msg_rotor_02_vel;
        msg_rotor_01_vel.data=15.0;
        msg_rotor_02_vel.data=-15.0;
        rotor_01_velocity_publisher_.publish(msg_rotor_01_vel);
        rotor_02_velocity_publisher_.publish(msg_rotor_02_vel);
        ros::spinOnce();

        ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State STARTING UP.");
    }
    void GazeboQuadrotorSimpleController::TakeoffCallback(const std_msgs::EmptyConstPtr&)
    {
        if(contr_state_==0)
        {
            rotor_01_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_middle_velocity_controller/command",1,true);
            rotor_02_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_top_velocity_controller/command",1,true);
            std_msgs::Float64 msg_rotor_01_vel;
            std_msgs::Float64 msg_rotor_02_vel;
            msg_rotor_01_vel.data=30.0;
            msg_rotor_02_vel.data=-30.0;
            rotor_01_velocity_publisher_.publish(msg_rotor_01_vel);
            rotor_02_velocity_publisher_.publish(msg_rotor_02_vel);
            ros::spinOnce();

            contr_state_=1;
            ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State TAKING OFF.");
        }
    }
    void GazeboQuadrotorSimpleController::LandCallback(const std_msgs::EmptyConstPtr&)
    {
        if(contr_state_==2)
        {
            rotor_01_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_middle_velocity_controller/command",1,true);
            rotor_02_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_top_velocity_controller/command",1,true);
            std_msgs::Float64 msg_rotor_01_vel;
            std_msgs::Float64 msg_rotor_02_vel;
            msg_rotor_01_vel.data=28.0;
            msg_rotor_02_vel.data=-28.0;
            rotor_01_velocity_publisher_.publish(msg_rotor_01_vel);
            rotor_02_velocity_publisher_.publish(msg_rotor_02_vel);
            ros::spinOnce();

            contr_state_=3;
            ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State LANDING.");
        }
    }
    void GazeboQuadrotorSimpleController::ShutdownCallback(const std_msgs::EmptyConstPtr&)
    {
        rotor_01_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_middle_velocity_controller/command",1,true);
        rotor_02_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_top_velocity_controller/command",1,true);
        std_msgs::Float64 msg_rotor_01_vel;
        std_msgs::Float64 msg_rotor_02_vel;
        msg_rotor_01_vel.data=0.0;
        msg_rotor_02_vel.data=0.0;
        rotor_01_velocity_publisher_.publish(msg_rotor_01_vel);
        rotor_02_velocity_publisher_.publish(msg_rotor_02_vel);
        ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State SHUTTING DOWN.");
    }

    void GazeboQuadrotorSimpleController::OdomCallback(const nav_msgs::OdometryConstPtr& state)
    {
        // Get flight altitude
        ignition::math::Vector3d velocity1(velocity);
        if (imu_topic_.empty())
        {
            pose.Pos().Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
            pose.Rot().Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
            euler=pose.Rot().Euler();
            angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
        }
        velocity.Set(state->twist.twist.linear.x,state->twist.twist.linear.y,state->twist.twist.linear.z);

        // Calculate acceleration
        double dt=!state_stamp.isZero() ? (state->header.stamp-state_stamp).toSec() : 0.0;
        state_stamp=state->header.stamp;
        if(dt>0.0)
        {
            acceleration=(velocity-velocity1)/dt;
        }
        else
        {
            acceleration.Set();
        }
    }
    void GazeboQuadrotorSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
    {
        pose.Rot().Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        euler=pose.Rot().Euler();
        angular_velocity=pose.Rot().RotateVector(ignition::math::Vector3d(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
    }
    void GazeboQuadrotorSimpleController::CommandVelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
    {
        velocity_command_= (*velocity);
    }

    // Update the controller
    void GazeboQuadrotorSimpleController::Update()
    {
        // Get new commands/state
        callback_queue_.callAvailable();

        double dt;
        if (controlTimer.update(dt) && dt > 0.0)
        {
            // Get Pose/Orientation from Gazebo (if no state subscriber is active)
            if(b_enable_pos_controller)
            {
                pose.Pos().Set(link->WorldPose().Pos().X(),link->WorldPose().Pos().Y(),link->WorldPose().Pos().Z());
                pose.Rot().Set(link->WorldPose().Rot().W(),link->WorldPose().Rot().X(),link->WorldPose().Rot().Y(),link->WorldPose().Rot().Z());
                euler=pose.Rot().Euler();
                velocity=link->WorldLinearVel();
                angular_velocity=link->WorldAngularVel();
                acceleration=link->WorldLinearAccel();
                /*
                ROS_WARN("POS: %3.3f,%3.3f,%3.3f, VEL: %3.3f,%3.3f,%3.3f, ACC: %3.3f,%3.3f,%3.3f",
                         pose.Pos().X(),
                         pose.Pos().Y(),
                         pose.Pos().Z(),
                         velocity.X(),
                         velocity.Y(),
                         velocity.Z(),
                         acceleration.X(),
                         acceleration.Y(),
                         acceleration.Z()
                         );
                */
            }

            // Get load factor as gain for Z-force
            ignition::math::Vector3d gravity_body=pose.Rot().RotateVector(world->Gravity());
            double gravity=gravity_body.Length();
            double load_factor=gravity*gravity/world->Gravity().Dot(gravity_body);

            // Rotate vectors to coordinate frames relevant for control
            ignition::math::Quaterniond heading_quaternion(cos(euler.Z()/2.0),0,0,sin(euler.Z()/2));
            ignition::math::Vector3d velocity_xy=heading_quaternion.RotateVectorReverse(velocity);
            ignition::math::Vector3d acceleration_xy=heading_quaternion.RotateVectorReverse(acceleration);
            ignition::math::Vector3d angular_velocity_body=pose.Rot().RotateVectorReverse(angular_velocity);

            // Always update following state variables
            if(b_enable_pos_controller)
            {
                //ROS_WARN("CONTROLLER - r: %3.3f, y: %3.3f, c: %3.3f",0.5,pose.Pos().Y(),velocity_command_.linear.y);
                velocity_command_.linear.x=controllers_.position_x.update(1.5,pose.Pos().X(),velocity_xy.X(),dt);
                velocity_command_.linear.y=controllers_.position_y.update(0.5,pose.Pos().Y(),velocity_xy.Y(),dt);
                velocity_command_.linear.z=controllers_.position_z.update(2.0,pose.Pos().Z(),velocity.Z(),dt);
                velocity_command_.angular.z=0.0;
            }
            double pitch_command=controllers_.velocity_x.update(velocity_command_.linear.x,velocity_xy.X(),acceleration_xy.X(),dt)/gravity;
            double roll_command=-controllers_.velocity_y.update(velocity_command_.linear.y,velocity_xy.Y(),acceleration_xy.Y(),dt)/gravity;
            torque.X()=inertia.X()*controllers_.roll.update(roll_command,euler.X(),angular_velocity_body.X(),dt);
            torque.Y()=inertia.Y()*controllers_.pitch.update(pitch_command,euler.Y(),angular_velocity_body.Y(),dt);
            torque.Z()=inertia.Z()*controllers_.yaw.update(velocity_command_.angular.z,angular_velocity.Z(),0,dt);
            //torque.Z()=inertia.Z()*controllers_.yaw.update(3.1415,euler.Z(),angular_velocity.Z(),dt);

            // Update depending on flight state
            if(contr_state_==2)
            {
                // State ACTIVE
                force.Z()=mass*(controllers_.velocity_z.update(velocity_command_.linear.z,velocity.Z(),acceleration.Z(),dt)+load_factor*gravity);
                if(max_force_>0.0 && force.Z()>max_force_) force.Z()=max_force_;
                if(force.Z()<0.0) force.Z()=0.0;
            }
            else if (contr_state_==1)
            {
                // State TAKING OFF
                force.Z()=1.0*mass*(controllers_.velocity_z.update(velocity_command_.linear.z,velocity.Z(),acceleration.Z(),dt)+load_factor*gravity);
                if(max_force_>0.0 && force.Z()>max_force_) force.Z()=max_force_;
                if(force.Z()<0.0) force.Z()=0.0;

                // Once TAKEN OFF --> Switch to ACTIVE
                if(link->WorldPose().Pos().Z()>landing_takeoff_height_)
                {
                    contr_state_=2;
                    ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State ACTIVE.");
                }
            }
            else if(contr_state_==3)
            {
                // State LANDING
                force.Z()=0.975*mass*gravity;
                if(max_force_>0.0 && force.Z()>max_force_) force.Z()=max_force_;
                if(force.Z()<0.0) force.Z()=0.0;

                // Once LANDED --> Switch to IDLE   velocity.Z()>-landing_takeoff_speed_/20.0 &&
                if(link->WorldPose().Pos().Z()<landing_takeoff_height_)
                {
                    rotor_01_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_middle_velocity_controller/command",1,true);
                    rotor_02_velocity_publisher_=node_handle_->advertise<std_msgs::Float64>("joint_"+model_name_+"_rotor_top_velocity_controller/command",1,true);
                    std_msgs::Float64 msg_rotor_01_vel;
                    std_msgs::Float64 msg_rotor_02_vel;
                    msg_rotor_01_vel.data=15.0;
                    msg_rotor_02_vel.data=-15.0;
                    rotor_01_velocity_publisher_.publish(msg_rotor_01_vel);
                    rotor_02_velocity_publisher_.publish(msg_rotor_02_vel);

                    contr_state_=0;
                    ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State IDLE.");
                }
            }
            else if(contr_state_==0)
            {
                // State IDLE
                controllers_.roll.reset();
                controllers_.pitch.reset();
                controllers_.yaw.reset();
                controllers_.velocity_x.reset();
                controllers_.velocity_y.reset();
                controllers_.velocity_z.reset();
                controllers_.position_x.reset();
                controllers_.position_y.reset();
                controllers_.position_z.reset();
                // Reset control values
                force.Set(0.0,0.0,0.0);
                torque.Set(0.0,0.0,0.0);
            }
            else
            {
                // State INVALID
                ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State INVALID.");
            }
        }

        // Set resuling forces and torques via Gazebo
        link->AddRelativeForce(force);
        link->AddRelativeTorque(torque-link->GetInertial()->CoG().Cross(force));
    }

    // Reset the controller
    void GazeboQuadrotorSimpleController::Reset()
    {
        controllers_.roll.reset();
        controllers_.pitch.reset();
        controllers_.yaw.reset();
        controllers_.velocity_x.reset();
        controllers_.velocity_y.reset();
        controllers_.velocity_z.reset();
        controllers_.position_x.reset();
        controllers_.position_y.reset();
        controllers_.position_z.reset();
        force.Set();
        torque.Set();
        pose.Reset();
        velocity.Set();
        angular_velocity.Set();
        acceleration.Set();
        euler.Set();
        state_stamp=ros::Time();

        contr_state_=0; // IDLE
        ROS_INFO_STREAM("[Flight Controller - "<<model_name_<<"]: State IDLE.");
    }

    //////////////////////////////////////////////////////////////////////////////
    // PID controller implementation
    GazeboQuadrotorSimpleController::PIDController::PIDController()
    {
    }

    GazeboQuadrotorSimpleController::PIDController::~PIDController()
    {
    }

    void GazeboQuadrotorSimpleController::PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
    {
        gain_p=0.0;
        gain_d=0.0;
        gain_i=0.0;
        time_constant=0.0;
        limit=-1.0;
        if (!_sdf) return;
        if (_sdf->HasElement(prefix+"ProportionalGain")) gain_p=_sdf->GetElement(prefix+"ProportionalGain")->Get<double>();
        if (_sdf->HasElement(prefix+"DifferentialGain")) gain_d=_sdf->GetElement(prefix+"DifferentialGain")->Get<double>();
        if (_sdf->HasElement(prefix+"IntegralGain"))     gain_i=_sdf->GetElement(prefix+"IntegralGain")->Get<double>();
        if (_sdf->HasElement(prefix+"TimeConstant"))     time_constant=_sdf->GetElement(prefix+"TimeConstant")->Get<double>();
        if (_sdf->HasElement(prefix+"Limit"))            limit=_sdf->GetElement(prefix+"Limit")->Get<double>();
    }

    double GazeboQuadrotorSimpleController::PIDController::update(double new_input, double x, double dx, double dt)
    {
        // limit command
        if (limit > 0.0 && fabs(new_input) > limit) new_input=(new_input < 0 ? -1.0 : 1.0)*limit;
        // filter command
        if (dt+time_constant > 0.0)
        {
            dinput=(new_input-input)/(dt+time_constant);
            input =(dt*new_input+time_constant*input)/(dt+time_constant);
        }
        // update proportional, differential and integral errors
        p=input-x;
        d=dinput-dx;
        i=i+dt*p;
        // update control output
        output=gain_p*p+gain_d*d+gain_i*i;
        return output;
    }

    void GazeboQuadrotorSimpleController::PIDController::reset()
    {
        input=dinput=0;
        p=i=d=output=0;
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorSimpleController)
} // namespace gazebo
