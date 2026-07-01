#ifndef WAI_OA_AUDIENCE_LISTENER_H
#define WAI_OA_AUDIENCE_LISTENER_H



/////////////////////////////////////////////////
/// Standard C++ libraries
/////////////////////////////////////////////////
#include<iostream>
#include<math.h>
#include<vector>
#include<queue>
#include<string>
#include<string.h>
#include<sstream>
#include<fstream>
#include<stdio.h>
#include<sys/types.h>
#include<dirent.h>
#include<unistd.h>



/////////////////////////////////////////////////
/// Qt libraries
/////////////////////////////////////////////////
#include<QApplication>
#include<QMessageBox>
#include<QInputDialog>
#include<QLineEdit>
#include<QVBoxLayout>
#include<QComboBox>
#include<QDialogButtonBox>
#include<QSpinBox>
#include<QLabel>
#include<QFileDialog>
#include<QDesktopWidget>
#include<QPoint>



/////////////////////////////////////////////////
/// ROS libraries
/////////////////////////////////////////////////
#include<ros/ros.h>
#include<ros/package.h>

#include<std_msgs/Empty.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>
#include<std_msgs/Float32MultiArray.h>

#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Wrench.h>
#include<geometry_msgs/PoseStamped.h>

#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include<sensor_msgs/Joy.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/CameraInfo.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<image_transport/image_transport.h>

#include<gazebo_msgs/SpawnModel.h>
#include<gazebo_msgs/DeleteModel.h>
#include<gazebo_msgs/LinkStates.h>

//#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include<cv_bridge/cv_bridge.h>
//#include<opencv/cv.h>
//#include<opencv/highgui.h>
//#include<cvaux.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<audio_common_msgs/AudioData.h>

#include<X11/Xlib.h>
#include<X11/keysym.h>



/////////////////////////////////////////////////
/// 3rd party libraries
/////////////////////////////////////////////////
#include<view_controller_msgs/CameraPlacement.h>



/////////////////////////////////////////////////
/// Helper libraries
/////////////////////////////////////////////////
#include<wai_oa_image_processor.h>
#include<wai_reps.h>
#include<wai_rviz_markers.h>
#include<X11/Xlib.h>
#include<X11/keysym.h>



/////////////////////////////////////////////////
/// Helper definitions
/////////////////////////////////////////////////

#define GET_OBJECT_NAME(Variable) (#Variable)

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;



/////////////////////////////////////////////////
/// Class definition for state pattern
/////////////////////////////////////////////////
class WAIOAAudienceListener;

class WAIAuditState
{
protected:
    WAIOAAudienceListener* wai_audit_context;

public:
    virtual ~WAIAuditState()
    {
    }

    void SetContext(WAIOAAudienceListener *context)
    {
        this->wai_audit_context = context;
    }

    virtual void HandleSetupModel()=0;
    virtual void HandleSetupView()=0;
    virtual void HandleSetupLeaveState()=0;
};

class StateListening : public WAIAuditState
{
public:
    void HandleSetupModel();
    void HandleSetupView();
    void HandleSetupLeaveState();
};



/////////////////////////////////////////////////
/// Implementation class of ros node
/////////////////////////////////////////////////
class WAIOAAudienceListener
{
    // Define ROS parameters from launch file
    float F_NODE_SAMPLE_FREQUENCY;
    float F_SCENE_TRANSITION_TIMEOUT;
    float F_SCENE_TRIGGER_TIMEOUT;
    float F_SCENE_TRIGGER_COLL_THRES;
    float F_CAMERA_RGBD_RESOLUTION_X;
    float F_CAMERA_RGBD_RESOLUTION_Y;
    float F_CAMERA_RGBD_RESOLUTION_SCALE;
    float F_CAMERA_RGBD_FX;
    float F_CAMERA_RGBD_FY;
    float F_CAMERA_RGBD_CX;
    float F_CAMERA_RGBD_CY;
    float F_CAMERA_RGBD_P_FX;
    float F_CAMERA_RGBD_P_FY;
    float F_CAMERA_RGBD_P_CX;
    float F_CAMERA_RGBD_P_CY;
    float F_CAMERA_RGBD_RANGE_MIN;
    float F_CAMERA_RGBD_RANGE_MAX;
    float F_CAMERA_RGBD_THRESHOLD_DIST;
    float F_CAMERA_RGBD_THRESHOLD_BOUNDS;
    bool B_ENABLE_KALMAN;
    bool B_ENABLE_CAMERA_RVIZ_FLY_IN;
    bool B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW;
    bool B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW;
    bool B_ENABLE_BODY_INTERACTION;
    bool B_ENABLE_AVATAR;
    float F_LISTENER_MOOD; // Ethical properties
    float F_LISTENER_PRESENCE_VISUAL;

    bool B_EVENT_INPDEV_SESSION_LOAD;
    bool B_EVENT_INPDEV_SCENE_NEXT;
    bool B_EVENT_INPDEV_SCENE_PREV;
    bool B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT;
    bool B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE;
    int I_CAMERA_RVIZ_CYCLE_VIEW;
    bool B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW;
    bool B_EVENT_INPDEV_MODE_PRESENCE_VISUAL;
    bool B_EVENT_INPDEV_BODY_INTERACTION;
    bool B_EVENT_INPDEV_AVATAR;
    bool B_EVENT_INPDEV_SKETCH;
    bool B_EVENT_INPDEV_SHARE;
    bool B_EVENT_INPDEV_MOOD;
    bool B_EVENT_INPDEV_SELF_EVALUATE;
    int I_EVENT_INPDEV_REQUEST;
    int I_EVENT_INFO_FROM_PRESENTER;

    // Define main ROS handles
    ros::NodeHandle m_hdl_node;
    image_transport::ImageTransport m_hdl_it;

    // Define resource paths
    std::string m_s_path_nodename;
    std::string m_s_audience_id;
    std::string m_s_path_resources;
    std::string m_s_path_resources_logo;
    std::string m_s_path_resources_reps;
    std::string m_s_path_resources_sessions_listener;
    QString m_qst_path_session;

    // Define colors
    std_msgs::ColorRGBA m_col_invisible;
    std_msgs::ColorRGBA m_col_white;
    std_msgs::ColorRGBA m_col_black;
    std_msgs::ColorRGBA m_col_grey;
    std_msgs::ColorRGBA m_col_red_light_opaque;
    std_msgs::ColorRGBA m_col_red_trans,m_col_red,m_col_red_opaque;
    std_msgs::ColorRGBA m_col_green_trans,m_col_green,m_col_green_opaque;
    std_msgs::ColorRGBA m_col_blue_trans,m_col_blue,m_col_blue_opaque;
    std_msgs::ColorRGBA m_col_cyan,m_col_cyan_trans,m_col_cyan_opaque;
    std_msgs::ColorRGBA m_col_orange,m_col_orange_trans;
    std_msgs::ColorRGBA m_col_oa,m_col_oa_opaque,m_col_oa_trans,m_col_oa_shiny;

    // Define ROS publishers
    ros::Publisher m_pub_s_oa_status;
    ros::Publisher m_pub_cpl_rviz_camera;
    ros::Publisher m_pub_hea_ping_to_presenter;
    ros::Publisher m_pub_fma_ethical_properties_to_presenter;
    ros::Publisher m_pub_msg_hea_audience_request;
    ros::Publisher m_pub_cai_img_camera_audit_rgbd_rgb;
    ros::Publisher m_pub_cai_img_camera_audit_rgbd_depth;
    image_transport::Publisher m_pub_img_camera_audit_rgbd_rgb;
    image_transport::Publisher m_pub_img_camera_audit_rgbd_depth;
    //image_transport::Publisher m_pub_img_projector;
    //image_transport::Publisher m_pub_img_camera_2d;
    ros::Publisher m_pub_img_projector;
    ros::Publisher m_pub_img_camera_2d;
    std::vector<int> m_par_img_compression={cv::IMWRITE_JPEG_QUALITY,75};
    std::vector<uchar> m_uch_buf_compression;
    sensor_msgs::CompressedImage EncodeImage(cv::Mat mat_input);
    /*
    std::vector<int> m_par_img_projector={cv::IMWRITE_JPEG_QUALITY,75};
    std::vector<uchar> m_uch_buf_projector;
    std::vector<int> m_par_img_avatar_2d={cv::IMWRITE_JPEG_QUALITY,75};
    std::vector<uchar> m_uch_buf_avatar_2d;
    std::vector<int> m_par_img_camera_2d={cv::IMWRITE_JPEG_QUALITY,75};
    std::vector<uchar> m_uch_buf_camera_2d;
    */

    // Define ROS subscribers
    image_transport::Subscriber m_sub_img_camera_rgbd_rgb;
    image_transport::Subscriber m_sub_img_camera_rgbd_depth;
    ros::Subscriber m_sub_cai_img_camera_rgbd_rgb;
    ros::Subscriber m_sub_cai_img_camera_rgbd_depth;
    //image_transport::Subscriber m_sub_img_camera_robot_rgb; Not used currently!
    //image_transport::Subscriber m_sub_img_camera_livecam;
    ros::Subscriber m_sub_img_camera_livecam;
    //image_transport::Subscriber m_sub_img_camera_2d;
    ros::Subscriber m_sub_img_camera_2d;
    ros::Subscriber m_sub_joy_controller;
    ros::Subscriber m_sub_bol_mob_3d_mode;
    ros::Subscriber m_sub_bol_mob_camera_rviz_follow_presenter;
    ros::Subscriber m_sub_bol_mob_body_interaction;
    ros::Subscriber m_sub_bol_mob_avatar;
    ros::Subscriber m_sub_bol_mob_question;
    ros::Subscriber m_sub_bol_mob_break;
    ros::Subscriber m_sub_bol_mob_message;
    ros::Subscriber m_sub_cpl_rviz_camera;
    ros::Subscriber m_sub_lns_gazebo;
    ros::Subscriber m_sub_pcl_camera_virtual;
    ros::Subscriber m_sub_hea_ping_from_presenter;
    ros::Subscriber m_sub_hea_info_from_presenter;

    // Define timers
    ros::Timer m_tmr_avatar_2d;
    QPoint m_qt_qpo_mouse_cursor_pos;
    QPoint m_qt_qpo_mouse_cursor_pos_old;
    cv::Point m_cv_pnt_mouse_cursor_vel;

    // Define ROS messages
    sensor_msgs::Joy m_msg_joy_input_dev;
    std_msgs::Header m_msg_hea_audience_request;
    std_msgs::Header m_msg_hea_info_from_presenter;
    gazebo_msgs::LinkStates m_msg_lns_gazebo;
    view_controller_msgs::CameraPlacement m_msg_cpl_rviz_camera;

    // Define other helper objects
    ros::Time m_tim_trigger_action;
    ros::Time m_tim_event_received_img_camera_2d;
    ros::Time m_tim_event_received_img_camera_3d;
    int m_i_scene_count_current;
    int m_i_scene_count_max;
    int i_teleprompter_counter;
    std::string s_teleprompter_text;
    tf::Vector3 m_vc3_camera_rgbd_wrt_head;

    // Define objects for image processing
    sensor_msgs::ImagePtr m_msg_img_camera_rgbd_rgb;
    sensor_msgs::ImagePtr m_msg_img_camera_rgbd_depth;
    cv::Mat m_mat_img_camera_rgbd_rgb;
    cv::Mat m_mat_img_camera_rgbd_depth;
    cv::Mat m_mat_img_projector;
    cv::Mat m_mat_img_camera_2d;
    cv::Mat m_mat_img_avatar_2d;
    cv::Mat m_mat_img_teleprompter;
    cv::Mat m_mat_img_camera_robot_rgb;
    cv::Mat m_mat_img_camera_livecam;
    cv::Mat m_mat_img_camera_rgbd_depth_32fc1;
    cv::Mat m_mat_img_camera_rgbd_depth_32fc1_processed;

    // Helper objects for body interaction
    WAIOAImageProcessor m_wai_oa_image_processor;
    WAIOAImageProcessingStrategy* m_wai_oa_image_processing_strategy;

    // Define helper symbols
    WAISymbol* m_wai_oaa_trg_action;
    WAISymbol* m_wai_oaa_ooi_head;

    // Define display object for keyboard inputs
    Display* m_dsp_x11_display;

    // Apply singleton and state pattern to WAIOAAudienceListener node
    static WAIOAAudienceListener* m_ins_node;
    WAIAuditState *m_wai_oa_audience_listener_state;

    WAIOAAudienceListener();
    ~WAIOAAudienceListener();

public:
    // Run method (entry point)
    void run();

    // Constructor and Deconstructor
    static WAIOAAudienceListener* getInstance();

    // State pattern helper methods
    void StateTransitionTo(WAIAuditState* state);
    void RequestSetupModel();
    void RequestSetupView();
    void RequestSetupLeaveState();

    void SetupCameraRviz(int i_scene);
    void SetupVirtualPresenter(int i_scene);
    void SetupRobot(int i_scene);

    void SetOAStatusLabel(std::string s_oa_status);
    void SetActionTriggerLabel(std::string s_label);
    void TransitionCameraRviz(geometry_msgs::Point pnt_eye,geometry_msgs::Point pnt_focus,int i_duration=3,std::string s_frame="world",int i_mode=0);
    int GetSceneCountCurrent();
    void InteractionVoiceFromFile(std::string s_filename,bool b_blocking=false);
    bool KeyIsPressed(KeySym ks);
    void CheckTriggersFromKeyboard();
    bool CheckTriggersFromCollission(tf::Vector3,tf::Vector3,float);

    // Other method definitions
    std::vector<double> vec_camera_rviz_startup;
    std::vector<double> vec_camera_rviz_default;
    std::vector<double> vec_camera_rviz_default_fpv;
    std::vector<double> vec_camera_rviz_on_presenter;
    std::vector<double> vec_camera_rviz_on_projection;

    // Callback method definitions
    void cb_tmr_avatar_2d(const ros::TimerEvent& event);
    void cb_sub_joy_controller(const sensor_msgs::JoyPtr&);
    void cb_sub_hea_ping_from_presenter(const std_msgs::HeaderPtr&);
    void cb_sub_hea_info_from_presenter(const std_msgs::HeaderPtr&);
    void cb_sub_bol_mob_3d_mode(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_camera_rviz_follow_presenter(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_body_interaction(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_avatar(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_question(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_break(const std_msgs::BoolPtr&);
    void cb_sub_bol_mob_message(const std_msgs::BoolPtr&);
    void cb_sub_cpl_rviz_camera(const view_controller_msgs::CameraPlacementPtr&);
    void cb_sub_lns_gazebo(const gazebo_msgs::LinkStatesPtr&);
    void cb_sub_img_camera_livecam(const sensor_msgs::CompressedImageConstPtr&);
    void cb_sub_img_camera_2d(const sensor_msgs::CompressedImageConstPtr&);
    void cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr&);
    void cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr&);
    void cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr&);
};



#endif //WAI_OA_AUDIENCE_LISTENER_H
