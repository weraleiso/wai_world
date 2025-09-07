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

#include<sound_play/sound_play.h>
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
#include<wai_oa_sketch.h>
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
    bool B_ENABLE_3D_MODE;

    bool B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT;
    bool B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE;
    int I_CAMERA_RVIZ_CYCLE_VIEW;
    bool B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW;
    bool B_EVENT_INPDEV_PRESENCE_MODE;
    bool B_EVENT_INPDEV_BODY_INTERACTION;
    bool B_EVENT_INPDEV_AVATAR;
    bool B_EVENT_INPDEV_SKETCH;
    int I_EVENT_INPDEV_REQUEST;
    int I_EVENT_INFO_FROM_PRESENTER;

    // Define main ROS handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    sound_play::SoundClient sc;

    // Define resource paths
    std::string s_path_nodename;
    std::string s_audience_id;
    std::string s_path_resources;
    std::string s_path_resources_logo;
    std::string s_path_resources_representatives;
    std::string s_path_resources_slides;

    // Define ROS broadcaster
    tf::TransformBroadcaster brc_transforms;
    tf::Transform tf_camera_rgbd_wrt_hand;

    // Define colors
    std_msgs::ColorRGBA col_invisible;
    std_msgs::ColorRGBA col_white;
    std_msgs::ColorRGBA col_black;
    std_msgs::ColorRGBA col_grey;
    std_msgs::ColorRGBA col_red_light_opaque;
    std_msgs::ColorRGBA col_red_trans,col_red,col_red_opaque;
    std_msgs::ColorRGBA col_green_trans,col_green,col_green_opaque;
    std_msgs::ColorRGBA col_blue_trans,col_blue,col_blue_opaque;
    std_msgs::ColorRGBA col_cyan,col_cyan_trans,col_cyan_opaque;
    std_msgs::ColorRGBA col_orange,col_orange_trans;
    std_msgs::ColorRGBA col_oa,col_oa_opaque,col_oa_trans,col_oa_shiny;

    // Define ROS publishers
    ros::Publisher pub_cpl_rviz_camera;
    ros::Publisher pub_hea_ping_to_presenter;
    ros::Publisher pub_msg_hea_audience_request;
    ros::Publisher pub_s_oa_status;
    ros::Publisher pub_cai_img_camera_audit_rgbd_rgb;
    ros::Publisher pub_cai_img_camera_audit_rgbd_depth;
    image_transport::Publisher pub_img_camera_audit_rgbd_rgb;
    image_transport::Publisher pub_img_camera_audit_rgbd_depth;
    image_transport::Publisher pub_img_projector;
    image_transport::Publisher pub_img_camera_2d;

    // Define triggers, OOIs, etc.
    WAIReps* wai_oaa_trg_action;
    WAIReps* wai_oaa_ooi_head;

    // Define ROS subscribers
    image_transport::Subscriber sub_img_camera_rgbd_rgb;
    image_transport::Subscriber sub_img_camera_rgbd_depth;
    ros::Subscriber sub_cai_img_camera_rgbd_rgb;
    ros::Subscriber sub_cai_img_camera_rgbd_depth;
    //image_transport::Subscriber sub_img_camera_robot_rgb; Not used currently!
    image_transport::Subscriber sub_img_camera_livecam;
    image_transport::Subscriber sub_img_camera_2d;
    ros::Subscriber sub_joy_controller;
    ros::Subscriber sub_bol_mob_3d_mode;
    ros::Subscriber sub_bol_mob_camera_rviz_follow_presenter;
    ros::Subscriber sub_bol_mob_body_interaction;
    ros::Subscriber sub_bol_mob_avatar;
    ros::Subscriber sub_bol_mob_question;
    ros::Subscriber sub_bol_mob_break;
    ros::Subscriber sub_bol_mob_message;
    ros::Subscriber sub_cpl_rviz_camera;
    ros::Subscriber sub_lns_gazebo;
    ros::Subscriber sub_pcl_camera_virtual;
    ros::Subscriber sub_hea_ping_from_presenter;
    ros::Subscriber sub_hea_info_from_presenter;

    // Define ROS messages
    sensor_msgs::Joy msg_joy_input_dev;
    std_msgs::Header msg_hea_audience_request;
    std_msgs::Header msg_hea_info_from_presenter;
    gazebo_msgs::LinkStates msg_lns_gazebo;
    view_controller_msgs::CameraPlacement msg_cpl_rviz_camera;

    WAIRvizMarkers* mrk_audit_head;
    visualization_msgs::MarkerArray msg_mrk_audit_array;

    // Define Sketch
    WAIOASketch wai_oa_sketch;

    // Define other helper objects
    ros::Time tim_trigger_action;
    ros::Time tim_event_received_img_camera_2d;
    ros::Time tim_event_received_img_camera_3d;

    int i_scene_count;

    tf::Vector3 vc3_camera_rgbd_wrt_head;

    // Define objects for image processing
    sensor_msgs::ImagePtr msg_img_camera_rgbd_rgb;
    sensor_msgs::ImagePtr msg_img_camera_rgbd_depth;
    sensor_msgs::ImagePtr msg_img_projector;
    sensor_msgs::ImagePtr msg_img_camera_2d;
    cv::Mat mat_img_camera_rgbd_rgb;
    cv::Mat mat_img_camera_rgbd_depth;
    cv::Mat mat_img_projector;
    cv::Mat mat_img_camera_2d;
    cv::Mat mat_img_teleprompter;
    int i_teleprompter_counter;
    std::string s_teleprompter_text;
    cv::Mat mat_img_camera_robot_rgb;
    cv::Mat mat_img_camera_livecam;
    cv::Mat mat_img_camera_rgbd_depth_32fc1;
    cv::Mat mat_img_camera_rgbd_depth_32fc1_processed;
    cv::Mat mat_depth_uint16;
    WAIOAImageProcessor ImageProcessor;
    WAIOAImageProcessingStrategy* s;

    // Define display object for keyboard inputs
    Display* dsp_x11_display;

    // Define singleton pattern for node instance
    static WAIOAAudienceListener* instance;
    WAIOAAudienceListener();
    ~WAIOAAudienceListener();

    // Define state pattern for node
    WAIAuditState *wai_audit_state;

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
    void PlaySound(std::string s_name_sound);
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
    void cb_sub_img_camera_livecam(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_2d(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr&);
    void cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr&);
    void cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr&);
    void cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr&);
};



#endif //WAI_OA_AUDIENCE_LISTENER_H
