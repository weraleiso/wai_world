#ifndef WAI_OA_MARVIN_H
#define WAI_OA_MARVIN_H

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
#include<cstdlib>
#include<ctime>

#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<gazebo_msgs/LinkStates.h>
#include<gazebo_msgs/SetModelState.h>

#include<std_srvs/Empty.h>

#include<picovoice_msgs/GetIntentActionResult.h>
#include<picovoice_msgs/GetTranscriptAction.h>
#include<picovoice_msgs/GetLLMResponseAction.h>
#include<picovoice_msgs/GetSynthetizationAction.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<view_controller_msgs/CameraPlacement.h>

#include<sound_play/sound_play.h>

#include<wai_reps.h>



////////////////////////////////////////////////////
/// Class definition of WAIOAMarvin
////////////////////////////////////////////////////
static bool BothAreSpaces(char lhs, char rhs)
{
    return (lhs == rhs) && (lhs == ' ');
}

class WAIOAMarvin
{
    ros::NodeHandle* m_hdl_node;
    std::string m_s_path_nodename;
    sound_play::SoundClient* m_hdl_snd_client;

    float m_f_node_sample_frequency;

    std::string m_s_rep;
    std::string m_s_marvin_rep_pickup;

    ros::ServiceClient m_gazebo_engage_marvin;
    ros::ServiceClient m_gazebo_disengage_marvin;
    ros::ServiceClient m_gazebo_set_model_state;

    ros::Subscriber sub_pic_intent_action_result;
    ros::Subscriber sub_pic_transcript_action_result;
    ros::Subscriber sub_str_transcript_action_result;
    ros::Subscriber sub_pic_llm_response_action_result;
    ros::Subscriber sub_pic_synthetization_action_result;
    ros::Publisher pub_pic_llm_response_action_goal;
    ros::Publisher pub_pic_synthetization_action_goal;
    ros::Publisher pub_pst_marvin_reference;
    ros::Publisher pub_mrk_marvin_mood;

    WAIRvizMarkers* mrk_marvin_mood;

    picovoice_msgs::GetIntentActionResult msg_pic_intent_action_result;
    picovoice_msgs::GetTranscriptActionResult msg_pic_transcript_action_result;
    picovoice_msgs::GetLLMResponseActionResult msg_pic_llm_response_action_result;
    picovoice_msgs::GetSynthetizationActionResult msg_pic_synthetization_action_result;
    picovoice_msgs::GetLLMResponseActionGoal msg_pic_llm_response_action_goal; 
    picovoice_msgs::GetSynthetizationActionGoal msg_pic_synthetization_action_goal;

    gazebo_msgs::LinkStates* m_lns_gazebo_linkstates;
    std::vector<ros::Publisher>* m_vec_pub_reps_state;
    geometry_msgs::PoseStamped m_pst_marvin_actual;
    geometry_msgs::PoseStamped m_pst_marvin_hook_actual;
    geometry_msgs::PoseStamped m_pst_rep_actual;
    geometry_msgs::PoseStamped m_pst_marvin_reference;

    std::string m_s_path_synthetization;
    bool m_b_rep_pickup;
    float m_f_rep_pickup_flight_height;
    float m_f_marvin_rep_pickup_offset;
    float m_f_marvin_rep_flyto_offset;
    int m_i_mood;

    WAIReps* m_wai_oa_rep_marvin_hologram;
    std_msgs::ColorRGBA m_col_marvin_hologram;
    std::string m_s_setup_marvin_hologram;
    std::string m_s_hologram_resource_path;
    tf::Quaternion m_qua_oa_marvin_hologram;
    float m_f_marvin_hologram_yaw;
    float m_f_marvin_hologram_alpha;

    ros::Timer m_tmr_marvin;
    ros::Timer m_tmr_marvin_hologram;

public:
    WAIOAMarvin();
    ~WAIOAMarvin();

    void cb_sub_pic_intent_action_result(const picovoice_msgs::GetIntentActionResultConstPtr& msg);
    void cb_sub_pic_transcript_action_result(const picovoice_msgs::GetTranscriptActionResultConstPtr& msg);
    void cb_sub_str_transcript_action_result(const std_msgs::StringConstPtr& msg);
    void cb_sub_pic_llm_response_action_result(const picovoice_msgs::GetLLMResponseActionResultConstPtr& msg);
    void cb_sub_pic_synthetization_action_result(const picovoice_msgs::GetSynthetizationActionResultConstPtr& msg);
    void cb_tmr_marvin(const ros::TimerEvent& event);
    void cb_tmr_marvin_hologram(const ros::TimerEvent& event);
    void Initialize(ros::NodeHandle* hdl_node,
                    std::string s_path_nodename,
                    sound_play::SoundClient* hdl_snd_client,
                    float f_node_sample_frequency,
                    gazebo_msgs::LinkStates* lns_gazebo_linkstates);
                    //,std::vector<ros::Publisher>* vec_pub_reps_state);
    void UpdateModel(std::string s_text);
    void UpdateView();
    std::string CleanupStringDetail(std::string s_string);
    void Say(std::string s_say);
    void SayText(std::string s_text);
    void MarvinSynthesizeText(std::string s_text);
    void Acknowledge();
    void ReportOnTaskComplete();
    void TellMood();
    void TellAge();
    void TellWeight();
    void TellSatisfaction();
    void TellAboutYourself();
    void EnableHologram(std::string s_hologram_resource_path,std::string s_setup_marvin_hologram);
    void DisableHologram();
    void RepSetState(std::string s_rep_set_state,geometry_msgs::Pose pos_rep_set_state);
    void Engage(geometry_msgs::PoseStamped pst_reference_engage);
    void Shutdown();
    void FlyToWaypoint(geometry_msgs::PoseStamped pst_reference_waypoint,bool b_blocking=true);
    void RepPickup(std::string s_rep,geometry_msgs::PoseStamped pst_reference_rep_drop,float f_rep_pickup_flight_height=5.0,float f_marvin_rep_pickup_offset=1.0);
};

#endif //WAI_OA_MARVIN_H
