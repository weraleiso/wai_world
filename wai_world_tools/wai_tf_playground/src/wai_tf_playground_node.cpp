#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<tf/transform_listener.h>

#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>

#include<stdio.h>
#include<string.h>
#include<array>
#include<vector>
#include<math.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wai_tf_playground_node");
    ros::NodeHandle noh_nodehandle;

    tf::TransformBroadcaster tf_brc_broadcaster;
    tf::TransformListener tf_lst_listener;
    tf::StampedTransform tf_double_check;
    tf::Vector3 tf_vc3_translation;
    tf::Quaternion tf_qua_rotation;

    // 1st transform
    tf::Transform tf_transform_world_wrt_robot;
    tf_vc3_translation=tf::Vector3(1.0,2.0,3.0);
    tf_transform_world_wrt_robot.setOrigin(tf_vc3_translation);
    tf_qua_rotation.setRPY(0.0,30.0*M_PI/180.0,60.0*M_PI/180.0);
    tf_transform_world_wrt_robot.setRotation(tf_qua_rotation);

    // 2nd transform
    tf::Transform tf_transform_robot_wrt_sensor;
    tf_vc3_translation=tf::Vector3(5.0,0.0,0.0);
    tf_transform_robot_wrt_sensor.setOrigin(tf_vc3_translation);
    tf_qua_rotation.setRPY(0.0,0.0,0.0);
    tf_transform_robot_wrt_sensor.setRotation(tf_qua_rotation);

    // 3rd transform (point)
    tf::Transform tf_transform_sensor_wrt_point;
    tf_vc3_translation=tf::Vector3(0.0,0.0,-0.6);
    tf_transform_sensor_wrt_point.setOrigin(tf_vc3_translation);
    tf_qua_rotation.setRPY(0.0,0.0,0.0);
    tf_transform_sensor_wrt_point.setRotation(tf_qua_rotation);

    tf::Vector3 tf_vc3_point_s(0.0,0.0,-0.6);

    // Transform point in sensor frame to world frame
    tf::Vector3 tf_vc3_point_w=tf_transform_world_wrt_robot*tf_transform_robot_wrt_sensor*tf_vc3_point_s;
    ROS_WARN("tf_vc3_point_w (X,Y,Z): %3.3f, %3.3f, %3.3f",tf_vc3_point_w.getX(),tf_vc3_point_w.getY(),tf_vc3_point_w.getZ());
    // Results in:
    // tf_vc3_point_w (X,Y,Z): 3.015, 5.490, -0.020

    ros::Rate tf_rate(100);
    srand(time(NULL));

    while(ros::ok())
    {
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_world_wrt_robot,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "world","frame1"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_robot_wrt_sensor,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame1","frame2"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_sensor_wrt_point,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame2","frame3"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_world_wrt_robot,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame3","frame4"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_robot_wrt_sensor,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame4","frame5"));

        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_world_wrt_robot,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame5","frame6"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_robot_wrt_sensor,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame6","frame7"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_sensor_wrt_point,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame7","frame8"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_world_wrt_robot,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame8","frame9"));
        tf_brc_broadcaster.sendTransform(tf::StampedTransform(
                                             tf_transform_robot_wrt_sensor,
                                             ros::Time::now()-ros::Duration(double(rand())/double(RAND_MAX)*2.0),
                                             "frame9","frame10"));

        // Double check calcuations of point with simply "looking up" transform
        /*
        try
        {
            tf_lst_listener.lookupTransform("world","point",ros::Time(0),tf_double_check);
            ROS_WARN("Point from looked-up TF to double check (X,Y,Z): %3.3f, %3.3f, %3.3f",tf_double_check.getOrigin().getX(),
                                                            tf_double_check.getOrigin().getY(),
                                                            tf_double_check.getOrigin().getZ());
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        */

        tf_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
