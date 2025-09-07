/*
 * Software License Agreement (BSD License)
 *
 *  Robot Operating System (ROS)
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Related utilized or refactored original work(s)/ROS-Package(s)
 *      - Name: ...
 *      - Resource(s) URL: ...
 *  Copyright (c) ...-...
 *  Institution(s): ...
 *  Author(s): ...
 *  Maintainer(s): ...
 *  Additional Conditions/Notes: ...
 *  All rights reserved.
 *
 *  WAI World - WAI Marvin (wai_marvin)
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
 * --------------------- ETHICAL CONDITIONS OF USE --------------------
 *   1.ROLE: The roles of all actors (educators, learners, and AIS) must be clearly defined. Actors can either take leading, guiding, participating, or supporting roles. Respecting fundamental human rights, all use cases to develop skills and competencies, during the interactive process of teaching and learning, are essentially conducted by natural human actors. Educators are leading or guiding, learners are leading or participating. AIS are supporting, at different IOL - according to the educational stage, however, do not replace educators or learners.
 *   2.MULTIPLICITY: If possible, and not intended by the teaching/learning activity otherwise (e.g., homework), the interactive process of teaching and learning must involve N (1..*) natural human educators and n (1..*) natural human learners (at different IOL - according to the educational stage), and m (0..*) AIS.
 *   3.BEHAVIOR: If not intended by the teaching/learning activity otherwise (e.g, "learn from failure"), all actors must always promote (maintain, facilitate, or encourage) ethical behavior during the interactive process of teaching and learning.
 *   4.VISUAL REPRESENTATION: All natural human actors are visually represented as real or, at least, by their virtualization, at different IOL - according to the educational stage. AIS may be represented purely virtual at any time, however must not mimic natural human actors.
 *      4.1.SYNCHRONICITY: If possible (e.g., not restricted due to crisis), and not intended by the teaching/learning activity otherwise (e.g., "out-of-class" flipped classroom), the interactive process of teaching and learning must be conducted synchronously, at different IOL - according to the educational stage. Predominant asynchronous scenarios (e.g, pure offline prerecording of teaching/learning sessions and simple replaying) may be indicated (e.g., by distinct labels/colors, decreased quality of visual representation, etc.) or avoided in general.
 *      4.2.LOCATION: If possible (e.g., not restricted due to crisis), and not intended by the teaching/learning activity otherwise (e.g., "homework"), the interactive process of teaching and learning must be conducted colocated, at different IOL - according to the educational stage. Predominant dislocated scenarios may be conducted with respect to condition 4.1).
 *      4.3.EMBODIMENT: All actors must be, at any time, embodied. Natural human educators and learners must preserve visual coherence of their embodiment, at different IOL - according to the educational stage. AIS must be visually embodied and clearly labeled as such, including an indication of their system boundaries to avoid omnipresence and to increase transparency and trust.
 * --------------------------------------------------------------------
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



class WAIMarvinControlCenter
{
    ros::NodeHandle nh;

    std::string s_nodename;

    ros::Publisher pub_twi_setpoint_velocity;
    //ros::Publisher pub_mrk_marvin_energy;
    ros::Subscriber sub_joy_controller;
    ros::Subscriber sub_pst_position_reference;
    ros::Subscriber sub_odo_gazebo;
    //ros::Subscriber sub_energy_gazebo;

    // Visualization
    //visualization_msgs::Marker msg_mrk_marvin_energy;

    nav_msgs::Odometry msg_odo_gazebo;

    geometry_msgs::PoseStamped msg_pst_position_reference;
    geometry_msgs::PoseStamped msg_pst_position_actual;
    geometry_msgs::Twist msg_twist_setpoint_velocity;

    float f_inpdev_yaw_output;
    float f_inpdev_height_output;
    float f_inpdev_pitch_output;
    float f_inpdev_roll_output;

    float F_NODE_SAMPLE_FREQUENCY;
    float F_DEFAULT_SETPOINT_TRANSLATION_X;
    float F_DEFAULT_SETPOINT_TRANSLATION_Y;
    float F_DEFAULT_SETPOINT_TRANSLATION_Z;
    float F_DEFAULT_SETPOINT_ROTATION_W;
    float F_DEFAULT_SETPOINT_ROTATION_X;
    float F_DEFAULT_SETPOINT_ROTATION_Y;
    float F_DEFAULT_SETPOINT_ROTATION_Z;

    float F_CONTROLLER_TRANSLATION_X_SAT_MIN;
    float F_CONTROLLER_TRANSLATION_X_SAT_MAX;
    float F_CONTROLLER_TRANSLATION_Y_SAT_MIN;
    float F_CONTROLLER_TRANSLATION_Y_SAT_MAX;
    float F_CONTROLLER_TRANSLATION_Z_SAT_MIN;
    float F_CONTROLLER_TRANSLATION_Z_SAT_MAX;
    float F_CONTROLLER_ROTATION_Z_SAT_MIN;
    float F_CONTROLLER_ROTATION_Z_SAT_MAX;
    float F_CONTROLLER_TRANSLATION_X_P;
    float F_CONTROLLER_TRANSLATION_X_I;
    float F_CONTROLLER_TRANSLATION_X_D;
    float F_CONTROLLER_TRANSLATION_Y_P;
    float F_CONTROLLER_TRANSLATION_Y_I;
    float F_CONTROLLER_TRANSLATION_Y_D;
    float F_CONTROLLER_TRANSLATION_Z_P;
    float F_CONTROLLER_TRANSLATION_Z_I;
    float F_CONTROLLER_TRANSLATION_Z_D;
    float F_CONTROLLER_ROTATION_Z_P;
    float F_CONTROLLER_ROTATION_Z_I;
    float F_CONTROLLER_ROTATION_Z_D;


public:
    ~WAIMarvinControlCenter()
    {
    }

    WAIMarvinControlCenter()
    {
        s_nodename=ros::this_node::getName();
        // Load parameters
        nh.getParam(s_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY", F_NODE_SAMPLE_FREQUENCY);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_TRANSLATION_X", F_DEFAULT_SETPOINT_TRANSLATION_X);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_TRANSLATION_Y", F_DEFAULT_SETPOINT_TRANSLATION_Y);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_TRANSLATION_Z", F_DEFAULT_SETPOINT_TRANSLATION_Z);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_ROTATION_W", F_DEFAULT_SETPOINT_ROTATION_W);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_ROTATION_X", F_DEFAULT_SETPOINT_ROTATION_X);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_ROTATION_Y", F_DEFAULT_SETPOINT_ROTATION_Y);
        nh.getParam(s_nodename+"/"+"F_DEFAULT_SETPOINT_ROTATION_Z", F_DEFAULT_SETPOINT_ROTATION_Z);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_X_SAT_MIN", F_CONTROLLER_TRANSLATION_X_SAT_MIN);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_X_SAT_MAX", F_CONTROLLER_TRANSLATION_X_SAT_MAX);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Y_SAT_MIN", F_CONTROLLER_TRANSLATION_Y_SAT_MIN);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Y_SAT_MAX", F_CONTROLLER_TRANSLATION_Y_SAT_MAX);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Z_SAT_MIN", F_CONTROLLER_TRANSLATION_Z_SAT_MIN);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Z_SAT_MAX", F_CONTROLLER_TRANSLATION_Z_SAT_MAX);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_ROTATION_Z_SAT_MIN", F_CONTROLLER_ROTATION_Z_SAT_MIN);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_ROTATION_Z_SAT_MAX", F_CONTROLLER_ROTATION_Z_SAT_MAX);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_X_P", F_CONTROLLER_TRANSLATION_X_P);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_X_I", F_CONTROLLER_TRANSLATION_X_I);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_X_D", F_CONTROLLER_TRANSLATION_X_D);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Y_P", F_CONTROLLER_TRANSLATION_Y_P);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Y_I", F_CONTROLLER_TRANSLATION_Y_I);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Y_D", F_CONTROLLER_TRANSLATION_Y_D);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Z_P", F_CONTROLLER_TRANSLATION_Z_P);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Z_I", F_CONTROLLER_TRANSLATION_Z_I);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_TRANSLATION_Z_D", F_CONTROLLER_TRANSLATION_Z_D);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_ROTATION_Z_P", F_CONTROLLER_ROTATION_Z_P);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_ROTATION_Z_I", F_CONTROLLER_ROTATION_Z_I);
        nh.getParam(s_nodename+"/"+"F_CONTROLLER_ROTATION_Z_D", F_CONTROLLER_ROTATION_Z_D);


        // Init publishers and subscribers
        sub_pst_position_reference=nh.subscribe("reference/position", 1, &WAIMarvinControlCenter::cb_sub_pst_position_reference, this);
        sub_odo_gazebo=nh.subscribe("odom", 1, &WAIMarvinControlCenter::cb_sub_odo_gazebo, this);
        //sub_energy_gazebo=nh.subscribe("energy", 1, &WAIMarvinControlCenter::cb_sub_energy_gazebo, this);
        pub_twi_setpoint_velocity = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Init Marvin RViz
        //pub_mrk_marvin_energy=nh.advertise<visualization_msgs::Marker>("marvin_energy",1);

        msg_pst_position_reference.header.stamp=ros::Time::now();
        msg_pst_position_reference.header.frame_id="world";
        msg_pst_position_reference.pose.position.x=F_DEFAULT_SETPOINT_TRANSLATION_X;
        msg_pst_position_reference.pose.position.y=F_DEFAULT_SETPOINT_TRANSLATION_Y;
        msg_pst_position_reference.pose.position.z=F_DEFAULT_SETPOINT_TRANSLATION_Z;
        msg_pst_position_reference.pose.orientation.w=F_DEFAULT_SETPOINT_ROTATION_W;
        msg_pst_position_reference.pose.orientation.x=F_DEFAULT_SETPOINT_ROTATION_X;
        msg_pst_position_reference.pose.orientation.y=F_DEFAULT_SETPOINT_ROTATION_Y;
        msg_pst_position_reference.pose.orientation.z=F_DEFAULT_SETPOINT_ROTATION_Z;

        msg_pst_position_actual.header.stamp=ros::Time::now();
        msg_pst_position_actual.header.frame_id="world";
        msg_pst_position_actual.pose.position.x=0.0;
        msg_pst_position_actual.pose.position.y=0.0;
        msg_pst_position_actual.pose.position.z=0.0;
        msg_pst_position_actual.pose.orientation.w=1.0;
        msg_pst_position_actual.pose.orientation.x=0.0;
        msg_pst_position_actual.pose.orientation.y=0.0;
        msg_pst_position_actual.pose.orientation.z=0.0;

        msg_twist_setpoint_velocity.linear.x=0.0;
        msg_twist_setpoint_velocity.linear.y=0.0;
        msg_twist_setpoint_velocity.linear.z=0.0;
        msg_twist_setpoint_velocity.angular.x=0.0;
        msg_twist_setpoint_velocity.angular.y=0.0;
        msg_twist_setpoint_velocity.angular.z=0.0;

        // Init markers
        /*
        msg_mrk_marvin_energy.header.frame_id = "marvin/link_base";
        msg_mrk_marvin_energy.header.stamp=ros::Time();
        msg_mrk_marvin_energy.ns = s_nodename;
        msg_mrk_marvin_energy.id = 0;
        msg_mrk_marvin_energy.text="N/A";
        msg_mrk_marvin_energy.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        msg_mrk_marvin_energy.action = visualization_msgs::Marker::ADD;
        msg_mrk_marvin_energy.pose.position.x = 0.0;
        msg_mrk_marvin_energy.pose.position.y = 0.0;
        msg_mrk_marvin_energy.pose.position.z = 1.0;
        msg_mrk_marvin_energy.pose.orientation.x = 0.0;
        msg_mrk_marvin_energy.pose.orientation.y = 0.0;
        msg_mrk_marvin_energy.pose.orientation.z = 0.0;
        msg_mrk_marvin_energy.pose.orientation.w = 1.0;
        msg_mrk_marvin_energy.scale.x = 0.125;
        msg_mrk_marvin_energy.scale.y = 0.125;
        msg_mrk_marvin_energy.scale.z = 0.125;
        msg_mrk_marvin_energy.color.r = 0.0;
        msg_mrk_marvin_energy.color.g = 1.0;
        msg_mrk_marvin_energy.color.b = 1.0;
        msg_mrk_marvin_energy.color.a = 1.0;
        pub_mrk_marvin_energy.publish(msg_mrk_marvin_energy);
        */
    }

    // Receive reference via PoseStamped message
    void cb_sub_pst_position_reference(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        msg_pst_position_reference=(*msg);
    }

    void cb_sub_odo_gazebo(const nav_msgs::OdometryConstPtr& msg)
    {
        msg_odo_gazebo=*msg;
        msg_pst_position_actual.pose.position.x=msg_odo_gazebo.pose.pose.position.x;
        msg_pst_position_actual.pose.position.y=msg_odo_gazebo.pose.pose.position.y;
        msg_pst_position_actual.pose.position.z=msg_odo_gazebo.pose.pose.position.z;
        msg_pst_position_actual.pose.orientation.w=msg_odo_gazebo.pose.pose.orientation.w;
        msg_pst_position_actual.pose.orientation.x=msg_odo_gazebo.pose.pose.orientation.x;
        msg_pst_position_actual.pose.orientation.y=msg_odo_gazebo.pose.pose.orientation.y;
        msg_pst_position_actual.pose.orientation.z=msg_odo_gazebo.pose.pose.orientation.z;
    }

    // Receive Energy state from Gazebo
    /*
    void cb_sub_energy_gazebo(const std_msgs::StringConstPtr& msg)
    {
        msg_str_enegery=*msg;
        msg_mrk_marvin_energy.header.stamp=ros::Time();
        msg_mrk_marvin_energy.text=msg_str_enegery.data;
        pub_mrk_marvin_energy.publish(msg_mrk_marvin_energy);
    }
    */



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
            // Controller objects to calculate actuating signals
            controller_pid_translation_x_output=controller_pid_translation_x.Calculate(msg_pst_position_reference.pose.position.x,msg_pst_position_actual.pose.position.x);
            controller_pid_translation_y_output=controller_pid_translation_y.Calculate(msg_pst_position_reference.pose.position.y,msg_pst_position_actual.pose.position.y);
            controller_pid_translation_z_output=controller_pid_translation_z.Calculate(msg_pst_position_reference.pose.position.z,msg_pst_position_actual.pose.position.z);

            // Rotate translational speed vector from global to local coordinates
            tf::Vector3 vec_speed_command_global(controller_pid_translation_x_output,controller_pid_translation_y_output,controller_pid_translation_z_output);
            tf::Vector3 vec_speed_command_local;
            tf::Quaternion quat_rotation_actual_local;
            quat_rotation_actual_local.setW(msg_pst_position_actual.pose.orientation.w);
            quat_rotation_actual_local.setX(msg_pst_position_actual.pose.orientation.x);
            quat_rotation_actual_local.setY(msg_pst_position_actual.pose.orientation.y);
            quat_rotation_actual_local.setZ(msg_pst_position_actual.pose.orientation.z);
            //ROS_INFO("w_actual: %3.3f - x_actual: %3.3f - y_actual: %3.3f - z_actual: %3.3f",msg_lnkstat_position_actual.pose.orientation.w,msg_lnkstat_position_actual.pose.orientation.x,msg_lnkstat_position_actual.pose.orientation.y,msg_lnkstat_position_actual.pose.orientation.z);
            tf::Matrix3x3 mat_rotation_actual_local(quat_rotation_actual_local);
            vec_speed_command_local=vec_speed_command_global*mat_rotation_actual_local;

            // Finally calculate output for rotational z-axis
            tf::Quaternion quat_rotation_reference;
            quat_rotation_reference.setW(msg_pst_position_reference.pose.orientation.w);
            quat_rotation_reference.setX(msg_pst_position_reference.pose.orientation.x);
            quat_rotation_reference.setY(msg_pst_position_reference.pose.orientation.y);
            quat_rotation_reference.setZ(msg_pst_position_reference.pose.orientation.z);
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

            //controller_pid_rotation_z_output = controller_pid_rotation_z->calculate(yaw_position_reference, yaw_position_actual);
            controller_pid_rotation_z_output = controller_pid_rotation_z.Calculate(yaw_position_error, 0.0);

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

            // Publish twist message
            pub_twi_setpoint_velocity.publish(msg_twist_setpoint_velocity);

            // Do the good old spinning stuff
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    WAIMarvinControlCenter wai_marvin_control_center;
    ROS_INFO("Initialized. About to run...");
    wai_marvin_control_center.run();
    ROS_INFO("Exiting...");
    return 0;
}
