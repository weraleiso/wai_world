/*
 * Software License Agreement (BSD License)
 *
 *  Robot Operating System (ROS)
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Related utilized or refactored original work(s)/ROS-Package(s)
 *      - Name: Ingenuity Mars Helicopter
 *      - Resource(s) URL: https://science.nasa.gov/mission/mars-2020-perseverance/ingenuity-mars-helicopter/
 *  Copyright (c) 2012-2020
 *  Institution(s): NASA Jet Propulsion Laboratory
 *  Author(s): ...
 *  Maintainer(s): ...
 *  Additional Conditions/Notes: ...
 *  All rights reserved.
 *
 *  WAI World - WAI Ingenuity (wai_ingenuity)
 *  Copyright (c) 2019, Werner Alexander Isop
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1.Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3.Neither the name of this package/solution nor the names of its
 *     contributors may be used to endorse or promote products derived
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
 */

#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Joy.h>

// #include<visualization_msgs/Marker.h>
// #include<visualization_msgs/MarkerArray.h>

#include<tf/transform_datatypes.h>

#include<PID.h>



class WAIIngenuityControlCenter
{

    ros::NodeHandle nh;

    ros::Publisher pub_f64_rotor_middle_velocity;
    ros::Publisher pub_f64_rotor_top_velocity;
    ros::Publisher pub_twist_setpoint_velocity;
    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_land;
    ros::Subscriber sub_posestmpd_position_reference;
    ros::Subscriber sub_posestmpd_position_actual;
    ros::Subscriber sub_joy_controller;

    geometry_msgs::PoseStamped msg_posestmpd_position_reference;
    geometry_msgs::PoseStamped msg_posestmpd_position_actual;
    geometry_msgs::Twist msg_twist_setpoint_velocity;

    double F_NODE_SAMPLE_FREQUENCY;
    double F_DEFAULT_SETPOINT_TRANSLATION_X;
    double F_DEFAULT_SETPOINT_TRANSLATION_Y;
    double F_DEFAULT_SETPOINT_TRANSLATION_Z;
    double F_DEFAULT_SETPOINT_ROTATION_W;
    double F_DEFAULT_SETPOINT_ROTATION_X;
    double F_DEFAULT_SETPOINT_ROTATION_Y;
    double F_DEFAULT_SETPOINT_ROTATION_Z;

    double F_CONTROLLER_TRANSLATION_X_SAT_MIN;
    double F_CONTROLLER_TRANSLATION_X_SAT_MAX;
    double F_CONTROLLER_TRANSLATION_Y_SAT_MIN;
    double F_CONTROLLER_TRANSLATION_Y_SAT_MAX;
    double F_CONTROLLER_TRANSLATION_Z_SAT_MIN;
    double F_CONTROLLER_TRANSLATION_Z_SAT_MAX;
    double F_CONTROLLER_ROTATION_Z_SAT_MIN;
    double F_CONTROLLER_ROTATION_Z_SAT_MAX;
    double F_CONTROLLER_TRANSLATION_X_P;
    double F_CONTROLLER_TRANSLATION_X_I;
    double F_CONTROLLER_TRANSLATION_X_D;
    double F_CONTROLLER_TRANSLATION_Y_P;
    double F_CONTROLLER_TRANSLATION_Y_I;
    double F_CONTROLLER_TRANSLATION_Y_D;
    double F_CONTROLLER_TRANSLATION_Z_P;
    double F_CONTROLLER_TRANSLATION_Z_I;
    double F_CONTROLLER_TRANSLATION_Z_D;
    double F_CONTROLLER_ROTATION_Z_P;
    double F_CONTROLLER_ROTATION_Z_I;
    double F_CONTROLLER_ROTATION_Z_D;

    bool B_STATUS_MANUAL_TAKEOFF;


public:
    ~WAIIngenuityControlCenter()
    {
    }

    WAIIngenuityControlCenter()
    {
        // Load parameters
        nh.getParam("wai_ingenuity_control_center_node/F_NODE_SAMPLE_FREQUENCY", F_NODE_SAMPLE_FREQUENCY);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_TRANSLATION_X", F_DEFAULT_SETPOINT_TRANSLATION_X);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_TRANSLATION_Y", F_DEFAULT_SETPOINT_TRANSLATION_Y);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_TRANSLATION_Z", F_DEFAULT_SETPOINT_TRANSLATION_Z);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_ROTATION_W", F_DEFAULT_SETPOINT_ROTATION_W);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_ROTATION_X", F_DEFAULT_SETPOINT_ROTATION_X);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_ROTATION_Y", F_DEFAULT_SETPOINT_ROTATION_Y);
        nh.getParam("wai_ingenuity_control_center_node/F_DEFAULT_SETPOINT_ROTATION_Z", F_DEFAULT_SETPOINT_ROTATION_Z);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_X_SAT_MIN", F_CONTROLLER_TRANSLATION_X_SAT_MIN);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_X_SAT_MAX", F_CONTROLLER_TRANSLATION_X_SAT_MAX);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Y_SAT_MIN", F_CONTROLLER_TRANSLATION_Y_SAT_MIN);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Y_SAT_MAX", F_CONTROLLER_TRANSLATION_Y_SAT_MAX);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Z_SAT_MIN", F_CONTROLLER_TRANSLATION_Z_SAT_MIN);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Z_SAT_MAX", F_CONTROLLER_TRANSLATION_Z_SAT_MAX);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_ROTATION_Z_SAT_MIN", F_CONTROLLER_ROTATION_Z_SAT_MIN);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_ROTATION_Z_SAT_MAX", F_CONTROLLER_ROTATION_Z_SAT_MAX);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_X_P", F_CONTROLLER_TRANSLATION_X_P);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_X_I", F_CONTROLLER_TRANSLATION_X_I);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_X_D", F_CONTROLLER_TRANSLATION_X_D);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Y_P", F_CONTROLLER_TRANSLATION_Y_P);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Y_I", F_CONTROLLER_TRANSLATION_Y_I);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Y_D", F_CONTROLLER_TRANSLATION_Y_D);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Z_P", F_CONTROLLER_TRANSLATION_Z_P);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Z_I", F_CONTROLLER_TRANSLATION_Z_I);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_TRANSLATION_Z_D", F_CONTROLLER_TRANSLATION_Z_D);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_ROTATION_Z_P", F_CONTROLLER_ROTATION_Z_P);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_ROTATION_Z_I", F_CONTROLLER_ROTATION_Z_I);
        nh.getParam("wai_ingenuity_control_center_node/F_CONTROLLER_ROTATION_Z_D", F_CONTROLLER_ROTATION_Z_D);

        // Init publishers and subscribers
        //sub_joy_controller = nh.subscribe("/wai_world/world/joy", 1, &WAIIngenuityControlCenter::cb_sub_joy_controller, this);
        sub_posestmpd_position_reference = nh.subscribe("reference/position", 1, &WAIIngenuityControlCenter::callback_sub_posestmpd_position_reference, this);
        sub_posestmpd_position_actual=nh.subscribe("odom", 1, &WAIIngenuityControlCenter::callback_sub_posestmpd_position_actual, this);
        pub_f64_rotor_middle_velocity = nh.advertise<std_msgs::Float64>("joint_ingenuity_rotor_middle_velocity_controller/command",true);
        pub_f64_rotor_top_velocity = nh.advertise<std_msgs::Float64>("joint_ingenuity_rotor_top_velocity_controller/command",true);
        pub_twist_setpoint_velocity = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        pub_empty_takeoff = nh.advertise<std_msgs::Empty>("takeoff", 1, true);
        pub_empty_land = nh.advertise<std_msgs::Empty>("land", 1, true);

        // Init variables
        B_STATUS_MANUAL_TAKEOFF=false;
        msg_posestmpd_position_reference.header.stamp=ros::Time::now();
        msg_posestmpd_position_reference.header.frame_id="world";
        msg_posestmpd_position_reference.pose.position.x=F_DEFAULT_SETPOINT_TRANSLATION_X;
        msg_posestmpd_position_reference.pose.position.y=F_DEFAULT_SETPOINT_TRANSLATION_Y;
        msg_posestmpd_position_reference.pose.position.z=F_DEFAULT_SETPOINT_TRANSLATION_Z;
        msg_posestmpd_position_reference.pose.orientation.w=F_DEFAULT_SETPOINT_ROTATION_W;
        msg_posestmpd_position_reference.pose.orientation.x=F_DEFAULT_SETPOINT_ROTATION_X;
        msg_posestmpd_position_reference.pose.orientation.y=F_DEFAULT_SETPOINT_ROTATION_Y;
        msg_posestmpd_position_reference.pose.orientation.z=F_DEFAULT_SETPOINT_ROTATION_Z;
        msg_posestmpd_position_actual.header.stamp=ros::Time::now();
        msg_posestmpd_position_actual.header.frame_id="world";
        msg_posestmpd_position_actual.pose.position.x=0.0;
        msg_posestmpd_position_actual.pose.position.y=0.0;
        msg_posestmpd_position_actual.pose.position.z=0.0;
        msg_posestmpd_position_actual.pose.orientation.w=1.0;
        msg_posestmpd_position_actual.pose.orientation.x=0.0;
        msg_posestmpd_position_actual.pose.orientation.y=0.0;
        msg_posestmpd_position_actual.pose.orientation.z=0.0;
        msg_twist_setpoint_velocity.linear.x=0.0;
        msg_twist_setpoint_velocity.linear.y=0.0;
        msg_twist_setpoint_velocity.linear.z=0.0;
        msg_twist_setpoint_velocity.angular.x=0.0;
        msg_twist_setpoint_velocity.angular.y=0.0;
        msg_twist_setpoint_velocity.angular.z=0.0;
    }

    // Receive JOYPAD commands
    void cb_sub_joy_controller(const sensor_msgs::JoyPtr& msg)
    {
        // Steer INGENUITY via JOYPAD (only if safety switch is pressed!)
        if(msg->buttons[4]==1)
        {
            msg_twist_setpoint_velocity.linear.x=msg->axes[4];
            msg_twist_setpoint_velocity.linear.y=msg->axes[3];
            msg_twist_setpoint_velocity.linear.z=msg->axes[1];
            msg_twist_setpoint_velocity.angular.z=msg->axes[0];
        }
        else
        {
            msg_twist_setpoint_velocity.linear.x=0.0;
            msg_twist_setpoint_velocity.linear.y=0.0;
            msg_twist_setpoint_velocity.linear.z=0.0;
            msg_twist_setpoint_velocity.angular.z=0.0;
        }

        // Takeoff and land via JOYPAD
        if(msg->buttons[4]==1 && msg->buttons[0]==1)
        {
            //system("rostopic pub --once /wai_world/ingenuity/land std_msgs/Empty &");
			ros::Rate r_sleep(0.2);
            std_msgs::Empty msg_empty;
            pub_empty_land.publish(msg_empty);
            ros::spinOnce();
            r_sleep.sleep();

            std_msgs::Float64 msg_f64_rotor_velocity_middle;
            msg_f64_rotor_velocity_middle.data=0.0;
            std_msgs::Float64 msg_f64_rotor_velocity_top;
            msg_f64_rotor_velocity_top.data=0.0;
            pub_f64_rotor_middle_velocity.publish(msg_f64_rotor_velocity_middle);
            pub_f64_rotor_top_velocity.publish(msg_f64_rotor_velocity_top);
            B_STATUS_MANUAL_TAKEOFF=false;
        }
        if(msg->buttons[4]==1 && msg->buttons[2]==1 && B_STATUS_MANUAL_TAKEOFF==false)
        {
            //system("rostopic pub --once /wai_world/ingenuity/takeoff std_msgs/Empty &");
            ros::Rate r_sleep(0.2);
            std_msgs::Float64 msg_f64_rotor_velocity_middle;
            msg_f64_rotor_velocity_middle.data=-25.0;
            std_msgs::Float64 msg_f64_rotor_velocity_top;
            msg_f64_rotor_velocity_top.data=25.0;
            pub_f64_rotor_middle_velocity.publish(msg_f64_rotor_velocity_middle);
            pub_f64_rotor_top_velocity.publish(msg_f64_rotor_velocity_top);
            ros::spinOnce();
            r_sleep.sleep();
            std_msgs::Empty msg_empty;
            pub_empty_takeoff.publish(msg_empty);
            B_STATUS_MANUAL_TAKEOFF=true;
        }
    }

    // Receive reference via PoseStamped message
    void callback_sub_posestmpd_position_reference(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        msg_posestmpd_position_reference=(*msg);
    }

    // Receive actual pose from Gazebo Plugin
    void callback_sub_posestmpd_position_actual(const nav_msgs::Odometry::ConstPtr& msg)
    {
        nav_msgs::Odometry msg_odo_actual=(*msg);
        msg_posestmpd_position_actual.pose.position.x=msg_odo_actual.pose.pose.position.x;
        msg_posestmpd_position_actual.pose.position.y=msg_odo_actual.pose.pose.position.y;
        msg_posestmpd_position_actual.pose.position.z=msg_odo_actual.pose.pose.position.z;
        msg_posestmpd_position_actual.pose.orientation.w=msg_odo_actual.pose.pose.orientation.w;
        msg_posestmpd_position_actual.pose.orientation.x=msg_odo_actual.pose.pose.orientation.x;
        msg_posestmpd_position_actual.pose.orientation.y=msg_odo_actual.pose.pose.orientation.y;
        msg_posestmpd_position_actual.pose.orientation.z=msg_odo_actual.pose.pose.orientation.z;
    }

    // Initialize and run position controller
    void run()
    {
        ros::Rate loop_rate(F_NODE_SAMPLE_FREQUENCY);

        PID controller_pid_translation_x(1.0/F_NODE_SAMPLE_FREQUENCY,F_CONTROLLER_TRANSLATION_X_SAT_MIN,F_CONTROLLER_TRANSLATION_X_SAT_MAX,F_CONTROLLER_TRANSLATION_X_P,F_CONTROLLER_TRANSLATION_X_I,F_CONTROLLER_TRANSLATION_X_D);
        PID controller_pid_translation_y(1.0/F_NODE_SAMPLE_FREQUENCY,F_CONTROLLER_TRANSLATION_Y_SAT_MIN,F_CONTROLLER_TRANSLATION_Y_SAT_MAX,F_CONTROLLER_TRANSLATION_Y_P,F_CONTROLLER_TRANSLATION_Y_I,F_CONTROLLER_TRANSLATION_Y_D);
        PID controller_pid_translation_z(1.0/F_NODE_SAMPLE_FREQUENCY,F_CONTROLLER_TRANSLATION_Z_SAT_MIN,F_CONTROLLER_TRANSLATION_Z_SAT_MAX,F_CONTROLLER_TRANSLATION_Z_P,F_CONTROLLER_TRANSLATION_Z_I,F_CONTROLLER_TRANSLATION_Z_D);
        PID controller_pid_rotation_z(1.0/F_NODE_SAMPLE_FREQUENCY,F_CONTROLLER_ROTATION_Z_SAT_MIN,F_CONTROLLER_ROTATION_Z_SAT_MAX,F_CONTROLLER_ROTATION_Z_P,F_CONTROLLER_ROTATION_Z_I,F_CONTROLLER_ROTATION_Z_D);

        double controller_pid_translation_x_output=0.0;
        double controller_pid_translation_y_output=0.0;
        double controller_pid_translation_z_output=0.0;
        double controller_pid_rotation_z_output=0.0;

        while(ros::ok())
        {
            // User controller objects to calculate actuating signals
            controller_pid_translation_x_output = controller_pid_translation_x.Calculate(msg_posestmpd_position_reference.pose.position.x,msg_posestmpd_position_actual.pose.position.x);
            controller_pid_translation_y_output = controller_pid_translation_y.Calculate(msg_posestmpd_position_reference.pose.position.y,msg_posestmpd_position_actual.pose.position.y);
            controller_pid_translation_z_output = controller_pid_translation_z.Calculate(msg_posestmpd_position_reference.pose.position.z,msg_posestmpd_position_actual.pose.position.z);

            // Rotate translational speed vector from global to local coordinates
            tf::Vector3 vec_speed_command_global(controller_pid_translation_x_output,controller_pid_translation_y_output,controller_pid_translation_z_output);
            tf::Vector3 vec_speed_command_local;
            tf::Quaternion quat_rotation_actual_local;
            quat_rotation_actual_local.setW(msg_posestmpd_position_actual.pose.orientation.w);
            quat_rotation_actual_local.setX(msg_posestmpd_position_actual.pose.orientation.x);
            quat_rotation_actual_local.setY(msg_posestmpd_position_actual.pose.orientation.y);
            quat_rotation_actual_local.setZ(msg_posestmpd_position_actual.pose.orientation.z);
            /*
            ROS_INFO("xyz: %.2f;%.2f;%.2f; wxyz: %.2f;%.2f;%.2f;%.2f;",
                     msg_posestmpd_position_actual.pose.position.x,msg_posestmpd_position_actual.pose.position.y,msg_posestmpd_position_actual.pose.position.z,
                     msg_posestmpd_position_actual.pose.orientation.w,msg_posestmpd_position_actual.pose.orientation.x,msg_posestmpd_position_actual.pose.orientation.y,msg_posestmpd_position_actual.pose.orientation.z);
                     */
            tf::Matrix3x3 mat_rotation_actual_local(quat_rotation_actual_local);
            vec_speed_command_local=vec_speed_command_global*mat_rotation_actual_local;

            // Finally calculate output for rotational z-axis
            tf::Quaternion quat_rotation_reference;
            quat_rotation_reference.setW(msg_posestmpd_position_reference.pose.orientation.w);
            quat_rotation_reference.setX(msg_posestmpd_position_reference.pose.orientation.x);
            quat_rotation_reference.setY(msg_posestmpd_position_reference.pose.orientation.y);
            quat_rotation_reference.setZ(msg_posestmpd_position_reference.pose.orientation.z);
            tf::Matrix3x3 mat_rotation_reference(quat_rotation_reference);
            double yaw_position_reference, yaw_position_actual;
            double dummy_roll=0.0, dummy_pitch=0.0;
            mat_rotation_actual_local.getRPY(dummy_roll,dummy_pitch,yaw_position_actual);
            mat_rotation_reference.getRPY(dummy_roll,dummy_pitch,yaw_position_reference);

            // Calc with quaternions!
            tf::Quaternion quat_diff=quat_rotation_actual_local.inverse()*quat_rotation_reference;
            tf::Matrix3x3 mat_rotation_error(quat_diff);
            double yaw_position_error;
            mat_rotation_error.getRPY(dummy_roll,dummy_pitch,yaw_position_error);

            //controller_pid_rotation_z_output = controller_pid_rotation_z.calculate(yaw_position_reference,yaw_position_actual);
            controller_pid_rotation_z_output = controller_pid_rotation_z.Calculate(yaw_position_error,0.0);

            // Control output for Gazebo simulation
            msg_twist_setpoint_velocity.linear.x=vec_speed_command_local.getX();
            msg_twist_setpoint_velocity.linear.y=vec_speed_command_local.getY();
            msg_twist_setpoint_velocity.linear.z=vec_speed_command_local.getZ();
            msg_twist_setpoint_velocity.angular.x=0.0;
            msg_twist_setpoint_velocity.angular.y=0.0;
            msg_twist_setpoint_velocity.angular.z=controller_pid_rotation_z_output;
            /*
            ROS_INFO("x_out: %3.3f - y_out: %3.3f - z_out: %3.3f - yaw_out: %3.3f",
                     msg_twist_setpoint_velocity.linear.x,
                     msg_twist_setpoint_velocity.linear.y,
                     msg_twist_setpoint_velocity.linear.z,
                     msg_twist_setpoint_velocity.angular.z);
                     */

            // Publish twist message (only if not manually taken off with JOYPAD!)
            if(B_STATUS_MANUAL_TAKEOFF==false)
            {
                pub_twist_setpoint_velocity.publish(msg_twist_setpoint_velocity);
            }

            // Do the good old spinning stuff
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "wai_ingenuity_control_center_node");
    WAIIngenuityControlCenter wai_ingenuity_control_center;
    ROS_INFO("Initialized. About to run...");
    wai_ingenuity_control_center.run();
    ROS_INFO("Exiting...");
    return 0;
}
