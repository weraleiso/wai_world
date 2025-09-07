#ifndef WAI_OA_PROJECTOR_H
#define WAI_OA_PROJECTOR_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<iostream>
#include<math.h>
#include<vector>
#include<queue>
#include<string>
#include<string.h>
#include<sstream>
#include<fstream>



/////////////////////////////////////////////////
/// Auditorium Projector
/////////////////////////////////////////////////
class WAIOAProjector
{
    ros::NodeHandle* m_hdl_node;
    image_transport::ImageTransport* m_hdl_it;
    float m_f_node_sample_frequency;

    image_transport::TransportHints hints;

    image_transport::Subscriber m_sub_img_camera_robot_rgb;
    image_transport::Subscriber m_sub_img_camera_livecam;

    image_transport::Publisher m_pub_img_projector;

    tf::TransformBroadcaster* m_tfb_transforms;
    tf::Transform m_tf_world_wrt_projection;

    cv::VideoCapture m_vcp_play_video;
    cv::Mat m_mat_play_video_frame;

    sensor_msgs::ImagePtr m_msg_img_projector;
    cv::Mat m_mat_img_projector;
    cv::Mat m_mat_img_projector_transition;
    cv::Size m_siz_img_camera_on_projector;
    cv::Mat m_mat_img_camera_robotcam;
    cv::Mat m_mat_img_camera_livecam;

    std::string m_s_path_image;
    std::string m_s_path_image_default;
    std::string m_s_path_video;
    std::string m_s_transition_mode;
    float m_f_transition_time;
    ros::Time m_tim_transition_start;

    ros::Timer m_tmr_projector;
    ros::Timer m_tmr_projector_ooi;
    ros::Timer m_tmr_projector_play_video;

    void cb_m_sub_img_camera_robot_rgb(const sensor_msgs::ImageConstPtr&);
    void cb_m_sub_img_camera_livecam(const sensor_msgs::ImageConstPtr&);
    void cb_tmr_projector(const ros::TimerEvent& event);
    void cb_tmr_projector_ooi(const ros::TimerEvent& event);
    void cb_tmr_projector_play_video(const ros::TimerEvent& event);

public:
    WAIOAProjector();
    ~WAIOAProjector();

    void Initialize(ros::NodeHandle* hdl_node,
                    image_transport::ImageTransport* hdl_it,
                    tf::TransformBroadcaster* tfb_transforms,
                    float f_node_sample_frequency,
                    std::string s_path_image_init);
    void UpdateModel(cv::Mat mat_img_projector,std::string s_path_image);
    void UpdateView(std::string s_transition_mode="",float f_transition_time=3.0);
    void SetProjectionPose(float f_pose_x,float f_pose_y,float f_pose_z,float f_pose_yaw);
    tf::Transform GetProjectionPose();

    void DisablePlayVideo();
    void EnablePlayVideo(std::string s_path_video,std::string s_logo_image);
    void DisableRobotcam();
    void EnableRobotcam(std::string s_robot_name="ingenuity");
    void DisableLivecam();
    void EnableLivecam();
};


#endif //WAI_OA_PROJECTOR_H
