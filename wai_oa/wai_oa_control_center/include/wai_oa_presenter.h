#ifndef WAI_OA_PRESENTER_H
#define WAI_OA_PRESENTER_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<iostream>
#include<math.h>
#include<vector>
#include<queue>
#include<string>
#include<string.h>
#include<sstream>
#include<fstream>

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/CameraInfo.h>

#include<cv_bridge/cv_bridge.h>
//#include<opencv2/cvaux.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<wai_oa_image_processor.h>
#include<wai_lowpass_filter.h>
#include<wai_kalman_filter.h>
#include<wai_reps.h>

#include<QApplication>
#include<QDesktopWidget>
#include<QPoint>



/////////////////////////////////////////////////
/// Auditorium Presenter
/////////////////////////////////////////////////
class WAIOAPresenter
{
    ros::NodeHandle* m_hdl_node;
    image_transport::ImageTransport* m_hdl_it;
    float m_f_node_sample_frequency;

    std::string m_s_path_nodename;
    std::string m_s_path_resources;
    std::string m_s_hand_right_text;
    bool m_b_enable_body_interaction;
    bool m_b_enable_avatar;
    bool m_b_enable_kmf;
    int m_i_presence_mode;

    // Declare hints
    image_transport::TransportHints hints_camera_rgb_color;
    image_transport::TransportHints hints_camera_rgbd_color;
    image_transport::TransportHints hints_camera_rgbd_depth;

    float m_f_camera_rgbd_resolution_x;
    float m_f_camera_rgbd_resolution_y;
    float m_f_camera_rgbd_resolution_scale;
    float m_f_camera_rgbd_fx;
    float m_f_camera_rgbd_fy;
    float m_f_camera_rgbd_cx;
    float m_f_camera_rgbd_cy;
    float m_f_camera_rgbd_p_fx;
    float m_f_camera_rgbd_p_fy;
    float m_f_camera_rgbd_p_cx;
    float m_f_camera_rgbd_p_cy;
    float m_f_camera_rgbd_range_min;
    float m_f_camera_rgbd_range_max;
    float m_f_camera_rgbd_mask_width;
    float m_f_camera_rgbd_threshold_dist;
    float m_f_camera_rgbd_threshold_bounds;

    // Subscribers for 2D/3D-Visualization of Presenter
    image_transport::Subscriber sub_img_camera_rgbd_rgb;
    image_transport::Subscriber sub_img_camera_rgbd_depth;
    image_transport::Subscriber sub_img_camera_virtual_rgbd_rgb;
    image_transport::Subscriber sub_img_camera_virtual_rgbd_depth;
    ros::Subscriber sub_cai_img_camera_rgbd_rgb;
    ros::Subscriber sub_cai_img_camera_rgbd_depth;
    image_transport::Subscriber sub_img_camera_2d;
    ros::Subscriber sub_pcl_camera_virtual;

    // Publishers for 2D/3D-Visualization of Presenter
    image_transport::Publisher pub_img_oa_camera_rgbd_rgb;
    image_transport::Publisher pub_img_oa_camera_rgbd_depth;
    ros::Publisher pub_cai_oa_camera_rgbd_rgb;
    ros::Publisher pub_cai_oa_camera_rgbd_depth;
    image_transport::Publisher pub_img_camera_virtual_rgbd_rgb;
    image_transport::Publisher pub_img_camera_virtual_rgbd_depth;
    ros::Publisher pub_cai_img_camera_virtual_rgbd_rgb;
    ros::Publisher pub_cai_img_camera_virtual_rgbd_depth;
    image_transport::Publisher pub_img_camera_2d;
    ros::Publisher pub_pcl_camera_virtual;
    ros::Publisher pub_pos_world_wrt_head;

    // ROS TF-Broadcaster
    tf::TransformBroadcaster* m_tfb_transforms;

    // TRANSFORMS
    tf::Vector3 m_vc3_joypad_input_axis;
    tf::Vector3 m_vc3_camera_rgbd_wrt_hand_right;
    tf::Vector3 m_vc3_camera_rgbd_wrt_hand_left;
    tf::Vector3 m_vc3_camera_rgbd_wrt_head;
    tf::Matrix3x3 m_m33_vc3_hands_and_head;
    WAILowpassFilter m_lpf_presenter_pose;
    tf::Transform m_tf_presenter;
    tf::Transform m_tf_world_wrt_presenter_camera;
    tf::Transform m_tf_presenter_camera_wrt_camera_optical;
    tf::Transform m_tf_camera_optical_wrt_hand_left;
    tf::Transform m_tf_camera_optical_wrt_hand_right;
    tf::Transform m_tf_camera_optical_wrt_head;
    tf::Transform m_tf_world_wrt_hand_left;
    tf::Transform m_tf_world_wrt_hand_right;
    tf::Transform m_tf_world_wrt_head;
    //tf::Transform* m_tf_world_wrt_head_buffer;
    geometry_msgs::Pose m_msg_pos_world_wrt_head;

    // Filter
    WAIKalmanFilter m_kmf_hand_right;
    ros::Time tim_hand_last_iter;

    // CAMERA INFO messages
    sensor_msgs::CameraInfo msg_cai_img_camera_virtual_rgbd_rgb;
    sensor_msgs::CameraInfo msg_cai_img_camera_virtual_rgbd_depth;

    // IMAGE PROCESSING helper members
    WAIOAImageProcessor ImageProcessor;
    WAIOAImageProcessingStrategy* s;
    sensor_msgs::ImagePtr msg_img_camera_rgbd_rgb;
    sensor_msgs::ImagePtr msg_img_camera_rgbd_depth;
    sensor_msgs::ImagePtr msg_img_camera_virtual_rgbd_rgb;
    sensor_msgs::ImagePtr msg_img_camera_virtual_rgbd_depth;
    sensor_msgs::ImagePtr msg_img_camera_2d;
    cv::Mat mat_img_camera_rgbd_rgb;
    cv::Mat mat_img_camera_rgbd_depth;
    cv::Mat mat_img_camera_2d;
    cv::Mat mat_img_camera_rgbd_depth_32fc1;
    cv::Mat mat_img_camera_rgbd_depth_32fc1_detector;
    cv::Mat mat_depth_uint16;
    //cv::Ptr<cv::BackgroundSubtractor> p_bg_sub;
    //cv::Mat mat_bg_sub_frame, mat_bg_sub_mask_fg;
    //cv::Mat mat_gc_mask,mat_bgd_model,mat_fgd_model;

    std_msgs::ColorRGBA col_red_trans,col_red,col_red_opaque;
    std_msgs::ColorRGBA col_green_trans,col_green,col_green_opaque;
    std_msgs::ColorRGBA col_blue_trans,col_blue,col_blue_opaque;
    std_msgs::ColorRGBA col_oa,col_oa_opaque,col_oa_trans,col_oa_shiny;

    WAIReps* wai_oa_rep_ooi_hand_right;
    WAIReps* wai_oa_rep_ooi_hand_left;
    WAIReps* wai_oa_rep_ooi_head;

    ros::Timer m_tmr_presenter;
    ros::Timer m_tmr_avatar_2d;

    QPoint qt_qpo_mouse_cursor_pos;
    QPoint qt_qpo_mouse_cursor_pos_old;
    cv::Point cv_pnt_mouse_cursor_vel;
    cv::Mat mat_img_avatar_2d;

    void cb_tmr_presenter(const ros::TimerEvent& event);
    void cb_tmr_avatar_2d(const ros::TimerEvent& event);

    void cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_virtual_rgbd_rgb(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_virtual_rgbd_depth(const sensor_msgs::ImageConstPtr&);
    void cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr&);
    void cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr&);
    void cb_sub_img_camera_2d(const sensor_msgs::ImageConstPtr&);
    void cb_sub_pcl_camera_virtual(const sensor_msgs::PointCloud2ConstPtr&);

public:
    WAIOAPresenter();
    ~WAIOAPresenter();
    void Initialize(ros::NodeHandle* hdl_node,
                    image_transport::ImageTransport* hdl_it,
                    std::string s_path_nodename,
                    std::string s_path_resources,
                    tf::TransformBroadcaster* tfb_transforms,
                    float f_node_sample_frequency,
                    float f_camera_rgbd_resolution_x,
                    float f_camera_rgbd_resolution_y,
                    float f_camera_rgbd_resolution_scale,
                    float f_camera_rgbd_fx,
                    float f_camera_rgbd_fy,
                    float f_camera_rgbd_cx,
                    float f_camera_rgbd_cy,
                    float f_camera_rgbd_range_min,
                    float f_camera_rgbd_range_max,
                    float f_camera_rgbd_mask_width,
                    float f_camera_rgbd_threshold_dist,
                    float f_camera_rgbd_threshold_bounds
                    );
    void UpdateModel(int i_presence_mode=0,bool b_enable_kmf=false);
    void UpdateView();
    int GetPresenceMode();
    void ToggleBodyInteraction();
    bool GetBodyInteractionEnabled();
    void ToggleAvatar();
    bool GetAvatarEnabled();
    void SetJoypadInputAxis(tf::Vector3 vc3_joypad_input_axis);
    tf::Vector3 GetHandLeftPosition();
    tf::Vector3 GetHandRightPosition();
    void SetupHandText(std::string s_hand_right_text);
    void SetupRespawn2D3D(std::string s_rep_respawn_2d3d);
    void SetPresenterPose(float f_pose_x,float f_pose_y,float f_pose_z,float f_pose_yaw);
    tf::Transform GetPresenterLPFPose();
    void DisableCameraRGB();
    void DisableCameraRGBD();
    void ProcessBodyInteraction();
};


#endif //WAI_OA_PRESENTER_H
