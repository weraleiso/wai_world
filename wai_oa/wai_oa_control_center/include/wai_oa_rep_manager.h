#ifndef WAI_OA_REP_MANAGER_H
#define WAI_OA_REP_MANAGER_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<sensor_msgs/image_encodings.h>

#include<image_transport/image_transport.h>

#include<iostream>
#include<iomanip>
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

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Wrench.h>

#include<gazebo_msgs/SpawnModel.h>
#include<gazebo_msgs/DeleteModel.h>
#include<gazebo_msgs/GetModelState.h>
#include<gazebo_msgs/SetModelState.h>
#include<gazebo_msgs/ApplyBodyWrench.h>
#include<gazebo_msgs/ModelState.h>
#include<gazebo_msgs/ModelStates.h>
#include<gazebo_msgs/LinkStates.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<wai_reps.h>



/////////////////////////////////////////////////
/// Auditorium Projector
/////////////////////////////////////////////////
class WAIOARepManager
{
    ros::NodeHandle* m_hdl_node;
    std::string m_s_path_nodename;
    std::string m_s_path_reps;
    float m_f_node_sample_frequency;
    int m_i_counter_downsample;

    std::vector<std::string> m_vec_s_reps;
    std::vector<std::string> m_vec_s_reps_details;
    std::vector<std::string> m_vec_s_reps_details_selection;
    std::stringstream m_sst_oa_detail_label;
    std::stringstream m_sst_oa_detail_state_label;
    std::string m_s_rep_selected;
    std::string m_s_rep_detail_selected;
    bool m_b_rep_detail_rot;
    int m_i_rep_selected;
    int m_i_rep_detail_selected;
    float m_f_rep_detail_yaw;
    float m_f_rep_detail_pitch;
    float m_f_rep_detail_roll;

    tf::Quaternion m_qua_oa_detail;
    tf2_msgs::TFMessage msg_tf2_transforms,msg_tf2_transforms_sub;
    gazebo_msgs::LinkStates msg_lns_gazebo,msg_lns_gazebo_sub;
    gazebo_msgs::ModelStates msg_mds_gazebo,msg_mds_gazebo_sub;

    std_msgs::ColorRGBA col_invisible;
    std_msgs::ColorRGBA col_black,col_white;
    std_msgs::ColorRGBA col_oa,col_oa_opaque,col_oa_trans,col_oa_shiny,col_cyan_opaque;

    ros::ServiceClient ser_cli_gazebo_get_model_state;
    ros::ServiceClient ser_cli_gazebo_set_model_state;
    ros::ServiceClient ser_cli_gazebo_apply_body_wrench;
    ros::ServiceClient ser_cli_gazebo_delete_model;
    ros::ServiceClient ser_cli_gazebo_spawn_model;
    tf::Vector3 m_vc3_rep_state_pos;
    tf::Quaternion m_qua_rep_state_ori;
    tf::Vector3 m_vc3_rep_state_eul;
    tf::Vector3 m_vc3_rep_state_vel_lin;
    tf::Vector3 m_vc3_rep_state_vel_ang;

    ros::Subscriber sub_lns_gazebo;
    ros::Subscriber sub_mds_gazebo;
    ros::Subscriber sub_tf2_transforms;

    ros::Publisher pub_mrk_oa_detail_label;
    //std::vector<ros::Publisher> vec_pub_wre_force;
    //std::vector<ros::Publisher> vec_pub_pos_state;

    WAIRvizMarkers* mrk_oa_detail_label;
    WAIReps* wai_oa_rep_detail;
    ros::Publisher pub_mrk_eval_metaphor;
    visualization_msgs::Marker mrk_eval_metaphor;

    ros::Timer m_tmr_rep_manager;
    ros::Timer m_tmr_rep_manager_eval_metaphor;

    void cb_sub_lns_gazebo(const gazebo_msgs::LinkStatesConstPtr&);
    void cb_sub_tf2_transforms(const tf2_msgs::TFMessageConstPtr&);
    void cb_sub_mds_gazebo(gazebo_msgs::ModelStates);

    void cb_tmr_rep_manager(const ros::TimerEvent& event);
    void cb_tmr_rep_manager_eval_metaphor(const ros::TimerEvent& event);

public:
    WAIOARepManager();
    ~WAIOARepManager();

    void Initialize(ros::NodeHandle* hdl_node,std::string s_path_nodename,std::string s_path_reps,float f_node_sample_frequency);
    void UpdateModel(std::string s_rep,std::string s_detail);
    void UpdateView();
    void UpdateRepsDetailsLabel();
    void EnableRepAndDetail();
    void DisableRepAndDetail();
    void ToggleRepDetailRotation();
    void EnableEvalMetaphors();
    void DisableEvalMetaphors();
    void InitEvalMetaphor();
    std::vector<std::string> GetCurrentRepsIndex();
    std::vector<std::string> GetCurrentRepsDetailsIndex();
    bool GetRepExists(std::string s_rep_name);
    void GetRepStateViaService(std::string s_rep_name,std::string s_rep_detail_name,tf::Vector3* vc3_position,tf::Quaternion* qua_orientation,tf::Vector3* vc3_euler_angles,tf::Vector3* vc3_vel_linear,tf::Vector3* vc3_vel_angular);
    void SetRepForceViaService(std::string s_name,std::string s_link_name,tf::Vector3 vc3_force,tf::Vector3 vc3_rep_torque=tf::Vector3(0.0,0.0,0.0),double d_rep_force_duration=-1.0);
    void SetRepStateViaService(std::string s_rep_name,tf::Vector3 vc3_rep_position,tf::Vector3 vc3_rep_orientation=tf::Vector3(0.0,0.0,0.0),tf::Vector3 vc3_rep_vel_lin=tf::Vector3(0.0,0.0,0.0),tf::Vector3 vc3_rep_vel_ang=tf::Vector3(0.0,0.0,0.0));
    void SetRepStateViaServiceQuaternion(std::string s_rep_name,tf::Vector3 vc3_rep_position,tf::Quaternion qua_rep_orientation=tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3 vc3_rep_vel_lin=tf::Vector3(0.0,0.0,0.0),tf::Vector3 vc3_rep_vel_ang=tf::Vector3(0.0,0.0,0.0));

    gazebo_msgs::LinkStates* GetRepLinkStates();
    std::vector<ros::Publisher>* GetRepStatePublishers();
    std::string GetRepSelectedClean();
    std::string GetRepDetailSelectedClean();
    std::string GetRepSelected();
    std::string GetRepDetailSelected();
    std::string GetRepDetailsFormatted(std::string s_detail,int i_char_count=40);
    std::string CleanupStringDetail(std::string s_string);
    std::string CleanupStringRep(std::string s_string);
};


#endif //WAI_OA_REP_MANAGER_H
