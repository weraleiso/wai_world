#ifndef WAI_OA_REP_SEQUENCER_H
#define WAI_OA_REP_SEQUENCER_H



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
#include<std_msgs/String.h>
#include<geometry_msgs/Point.h>
#include<rosgraph_msgs/Clock.h>
#include<tf/transform_datatypes.h>

#include<wai_oa_rep_manager.h>



/////////////////////////////////////////////////
/// Class definition of WAIOARepSequencer
/////////////////////////////////////////////////


class WAIOARepSequencer
{
    ros::NodeHandle* m_hdl_node;
    WAIOARepManager* m_oa_rep_manager;

    float m_f_node_sample_frequency;

    // SEQUENCER helper members
    ros::Subscriber m_sub_clk_gazebo;
    ros::Publisher m_pub_str_rep_sequence;
    ros::Publisher m_pub_hea_reset; // Reset time of REP's state plugin

    std_msgs::Header m_msg_hea_reset;
    std_msgs::String m_msg_str_rep_sequence;
    rosgraph_msgs::Clock m_msg_clk_gazebo;

    int m_i_rep_seq_scene;
    std::string m_s_rep_seq_name;
    float m_f_rep_seq_duration;
    float m_f_rep_seq_elapsed;
    std::vector<double> m_vec_d_rep_seq_state;
    std::vector<double> m_vec_d_rep_seq_twist;
    std::vector<double> m_vec_d_rep_seq_force;
    std::vector<bool> m_vec_b_rep_seq_state;
    std::vector<bool> m_vec_b_rep_seq_twist;
    std::vector<bool> m_vec_b_rep_seq_force;
    tf::Vector3 m_vc3_rep_seq_force;
    tf::Vector3 m_vc3_rep_seq_torque;
    tf::Vector3 m_vc3_rep_seq_position;
    tf::Vector3 m_vc3_rep_seq_orientation;
    tf::Quaternion m_qua_rep_seq_orientation;
    tf::Vector3 m_vc3_rep_seq_vel_linear;
    tf::Vector3 m_vc3_rep_seq_vel_angular;
    std::string m_s_state_info;
    ros::Time m_tim_setup_rep_start;
    char m_c_state_info[2048];
    bool m_b_rep_seq_enabled;
    bool m_b_rep_seq_save_start_time;
    bool m_b_rep_seq_changed;

    void cb_sub_clk_gazebo(const rosgraph_msgs::ClockPtr&);

public:
    WAIOARepSequencer();
    ~WAIOARepSequencer();

    void Initialize(ros::NodeHandle* hdl_node,
                    WAIOARepManager* oa_rep_manager,
                    float f_node_sample_frequency);
    void UpdateModel(int i_rep_seq_scene,
                     std::string s_rep_seq_name,
                     float f_rep_seq_duration,
                     std::vector<double> vec_d_rep_seq_state,
                     std::vector<double> vec_d_rep_seq_twist,
                     std::vector<double> vec_d_rep_seq_force);
    void UpdateView();
    void ResetSequencerStates();
    void ResetSequencerPlots();
    void ResetSequencerTimestamp(bool b_enable=false);
    void UpdateSequencerStatesInfo();
    void SpinAndWaitForSeconds(float f_seconds);
};

#endif //WAI_OA_REP_SEQUENCER_H
