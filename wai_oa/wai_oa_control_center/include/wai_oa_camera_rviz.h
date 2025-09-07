#ifndef WAI_OA_CAMERA_RVIZ_H
#define WAI_OA_CAMERA_RVIZ_H



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
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<gazebo_msgs/GetModelState.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<view_controller_msgs/CameraPlacement.h>

#include<wai_lowpass_filter.h>
#include<wai_reps.h>



/////////////////////////////////////////////////
/// Class definition of WAIOACameraRviz
/////////////////////////////////////////////////

class WAIOACameraRviz
{
    ros::NodeHandle* m_hdl_node;
    float m_f_node_sample_frequency;

    std::string m_s_path_nodename;
    std::string m_s_path_resources;
    std::string m_s_frame;
    std::string m_s_frame_old;
    double m_d_duration;
    double m_d_duration_old;
    int m_i_mode;

    int m_i_camera_rviz_view_count;

    geometry_msgs::Point m_pnt_camera_rviz_eye_current;
    geometry_msgs::Point m_pnt_camera_rviz_focus_current;
    geometry_msgs::Point m_pnt_camera_rviz_eye_old;
    geometry_msgs::Point m_pnt_camera_rviz_focus_old;

    geometry_msgs::Point m_pnt_camera_rviz_eye_startup;
    geometry_msgs::Point m_pnt_camera_rviz_focus_startup;
    geometry_msgs::Point m_pnt_camera_rviz_eye_default;
    geometry_msgs::Point m_pnt_camera_rviz_focus_default;

    geometry_msgs::Point m_pnt_camera_rviz_idle_frame_origin;
    geometry_msgs::Point m_pnt_camera_rviz_idle_orbit_eye;
    geometry_msgs::Point m_pnt_camera_rviz_idle_cylindric_focus;

    ros::ServiceClient ser_cli_gazebo_get_model_state;

    tf::Transform m_tf_camera_rviz_follow_presenter_head;
    std::string m_s_rep_name_follow;
    tf::Vector3 m_vc3_rep_follow;

    std::vector< std::vector<double>> m_vec_camera_rviz_views_predefined;
    std::vector<double> m_vec_camera_rviz_target;

    std::vector<double> m_vec_camera_rviz_break; // not in use
    std::vector<double> m_vec_camera_rviz_default; // [0]
    std::vector<double> m_vec_camera_rviz_graph_3d;
    std::vector<double> m_vec_camera_rviz_graph_eval;
    std::vector<double> m_vec_camera_rviz_overview;
    std::vector<double> m_vec_camera_rviz_presenter;
    std::vector<double> m_vec_camera_rviz_presenter_fpv;
    std::vector<double> m_vec_camera_rviz_projection;
    std::vector<double> m_vec_camera_rviz_startup; // not in use
    std::vector<double> m_vec_camera_rviz_wim; // [7]

    ros::Publisher m_pub_cpl_camera_rviz;
    ros::Subscriber m_sub_pos_camera_rviz;
    ros::Subscriber m_sub_pos_camera_rviz_follow_presenter_head;
    view_controller_msgs::CameraPlacement msg_cpl_camera_rviz;
    geometry_msgs::PoseStamped m_msg_pos_camera_rviz;

    tf::TransformBroadcaster* m_tfb_transforms;
    tf::Transform m_tf_camera_wrt_detail;

    ros::Timer m_tmr_camera_rviz_idle;
    ros::Time m_tim_camera_rviz_idle_start;
    ros::Timer m_tmr_camera_rviz_follow_rep;
    ros::Timer m_tmr_camera_rviz_follow_presenter;
    ros::Timer m_tmr_camera_rviz_ooi;
    float m_f_camera_rviz_idle_amplitude;
    float m_f_camera_rviz_idle_omega;
    int m_i_camera_rviz_idle_mode;
    bool m_b_camera_follow_presenter_enabled;
    bool m_b_camera_follow_rep_enabled;

    std_msgs::ColorRGBA col_oa,col_oa_opaque,col_oa_trans,col_oa_shiny;

    WAIReps* m_wai_oa_rep_ooi_camera_rviz;

public:
    WAIOACameraRviz();
    ~WAIOACameraRviz();

    void cb_sub_pos_camera_rviz(const geometry_msgs::PoseStampedPtr& msg);
    void cb_sub_pos_camera_rviz_follow_presenter_head(const geometry_msgs::PosePtr& msg);
    void cb_tmr_camera_rviz_idle(const ros::TimerEvent& event);
    void cb_tmr_camera_rviz_follow_rep(const ros::TimerEvent& event);
    void cb_tmr_camera_rviz_follow_presenter(const ros::TimerEvent& event);
    void cb_tmr_camera_rviz_ooi(const ros::TimerEvent& event);

    void Initialize(ros::NodeHandle* hdl_node,
                    tf::TransformBroadcaster* tfb_transforms,
                    std::string s_path_nodename,
                    std::string s_path_resources,
                    float f_node_sample_frequency,
                    float f_duration=3.0,
                    std::string s_frame="world",
                    int i_mode=0);
    void UpdateModel(std::vector<double> vec_camera_rviz_target,bool b_cycle=false, bool b_back_forth=false, int i_view_index=-1,float f_duration=3.0,std::string s_frame="world",int i_mode=0,bool b_camera_rviz_store_prev=false);
    void UpdateView();

    std::string GetCurrentFrame();
    std::string GetOldFrame();
    void RestorePreviousView();
    void SaveCurrentViewAsPrevious();
    void SetTransformDetail(tf::Transform tf_camera_wrt_detail);
    bool GetCameraFollowPresenterEnabled();
    //bool GetCameraFollowRepEnabled();
    void ToggleCameraFollowPresenter();
    void TransitionCameraRviz(geometry_msgs::Point pnt_eye,geometry_msgs::Point pnt_focus,double d_duration=3.0,std::string s_frame="world",int i_mode=0);
    void UpdateCameraRvizViews(std::vector< std::vector<double>> vec_camera_rviz_views_predefined);
    void CameraIdleStart(float f_camera_rviz_idle_amplitude,
                         float f_camera_rviz_idle_fomega,
                         int i_camera_rviz_idle_mode,
                         tf::Vector3 vc3_camera_rviz_idle_frame_origin=tf::Vector3(0.0,0.0,0.0));
    void CameraIdleStop();
    void GetRepPositionViaService(std::string s_rep_name);
    void CameraFollowRepStart(std::string s_rep_name_follow);
    void CameraFollowRepStop();//(bool b_rest_prev_view=false);
    void CameraFollowPresenterStart();
    void CameraFollowPresenterStop(bool b_rest_prev_view=false);
};

#endif //WAI_OA_CAMERA_RVIZ_H
