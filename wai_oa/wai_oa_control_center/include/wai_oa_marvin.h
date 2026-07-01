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
#include<std_msgs/Empty.h>
#include<std_msgs/String.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>

#include<gazebo_msgs/LinkStates.h>
#include<gazebo_msgs/SetModelState.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<std_srvs/Empty.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<wai_pid_controller.h>

#include<wai_reps.h>

// -=[INTEL]=-
// Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
// preserve natural human presence in the use of AIS in education"
// In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
// Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
// --> Following the Mediator pattern, all ACTORS own a reference to Intel.
#include<wai_oa_intel.h>
class WAIOAIntel;



////////////////////////////////////////////////////
/// Class definition of WAIOAMarvin
////////////////////////////////////////////////////
static bool BothAreSpaces(char lhs, char rhs)
{
    return (lhs == rhs) && (lhs == ' ');
}

class WAIOAMarvin:WAIArtificial
{
    ros::NodeHandle* m_hdl_node;
    std::string m_s_path_nodename;

    float m_f_node_sample_frequency;

    std::string m_s_rep;
    std::string m_s_marvin_rep_pickup;

    //ros::ServiceClient m_gazebo_engage_marvin;
    //ros::ServiceClient m_gazebo_disengage_marvin;
    ros::ServiceClient m_gazebo_set_model_state;

    ros::Subscriber sub_pic_intent_action_result;
    ros::Subscriber sub_pic_transcript_action_result;
    ros::Subscriber sub_str_transcript_action_result;
    ros::Subscriber sub_pic_llm_response_action_result;
    ros::Subscriber sub_pic_synthetization_action_result;
    ros::Subscriber sub_odo_gazebo;
    ros::Publisher pub_twi_setpoint_velocity;
    ros::Publisher pub_emp_marvin_takeoff;
    ros::Publisher pub_emp_marvin_land;
    ros::Publisher pub_pic_llm_response_action_goal;
    ros::Publisher pub_pic_synthetization_action_goal;
    ros::Publisher pub_mrk_marvin_mood;

    WAIRvizMarkers* mrk_marvin_mood;

    PID controller_pid_translation_x;
    PID controller_pid_translation_y;
    PID controller_pid_translation_z;
    PID controller_pid_rotation_z;
    double controller_pid_translation_x_output;
    double controller_pid_translation_y_output;
    double controller_pid_translation_z_output;
    double controller_pid_rotation_z_output;

    gazebo_msgs::LinkStates* m_lns_gazebo_linkstates;
    nav_msgs::Odometry msg_odo_gazebo;
    geometry_msgs::PoseStamped msg_pst_position_reference;
    geometry_msgs::PoseStamped msg_pst_position_actual;
    geometry_msgs::Twist msg_twist_setpoint_velocity;
    geometry_msgs::PoseStamped m_pst_marvin_actual;
    geometry_msgs::PoseStamped m_pst_marvin_hook_actual;
    geometry_msgs::PoseStamped m_pst_rep_actual;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_home;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_someroom;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_introduction;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_present;

    std::string m_s_path_synthetization;
    bool m_b_rep_pickup;
    float m_f_rep_pickup_flight_height;
    float m_f_marvin_rep_pickup_offset;
    float m_f_marvin_rep_flyto_offset;
    float m_f_mood;
    float m_f_mood_variation;

    WAISymbol* m_wai_oa_rep_marvin_hologram;
    std_msgs::ColorRGBA m_col_marvin_hologram;
    std::string m_s_setup_marvin_hologram;
    std::string m_s_hologram_resource_path;
    tf::Quaternion m_qua_oa_marvin_hologram;
    float m_f_marvin_hologram_yaw;
    float m_f_marvin_hologram_alpha;

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

    ros::Timer m_tmr_marvin;
    ros::Timer m_tmr_marvin_hologram;

    // -=[INTEL]=-
    // Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
    // preserve natural human presence in the use of AIS in education"
    // In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
    // Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
    // --> Following the Mediator pattern, all ACTORS own a reference to Intel.
    WAIOAIntel* m_wai_oa_intel;

public:
    WAIOAMarvin();
    ~WAIOAMarvin();

    void Initialize(ros::NodeHandle* hdl_node,
                    WAIOAIntel* wai_oa_intel,
                    std::string s_path_nodename,
                    float f_node_sample_frequency,
                    gazebo_msgs::LinkStates* lns_gazebo_linkstates);
    void UpdateModel(std::string s_text);
    void UpdateView();

    void cb_tmr_marvin(const ros::TimerEvent& event);
    void cb_tmr_marvin_hologram(const ros::TimerEvent& event);

    void cb_sub_odo_gazebo(const nav_msgs::OdometryConstPtr& msg);

    std::string CleanupStringDetail(std::string s_string);

    void RepSetState(std::string s_rep_set_state,geometry_msgs::Pose pos_rep_set_state);
    void RepPickup(std::string s_rep,geometry_msgs::PoseStamped pst_reference_rep_drop,float f_rep_pickup_flight_height=5.0,float f_marvin_rep_pickup_offset=1.0);



    // -=[INTEL]=-
    // Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
    // preserve natural human presence in the use of AIS in education"
    // In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
    // Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
    // --> Exemplary method to request an ethically critical use case:
    void RequestToIncreaseBrightnessWithoutCaringAboutTheColor(); // Use case is delegated to Intel
    void ActuallyIncreaseBrightnessWithoutCaringAboutTheColor(); // Intel responds and mediates (takes further necessary actions)

    // EMOTIONS
    float GetMoodCurrent();
    void SetMoodCurrent();
    void OverrideMoodCurrent();

    // BASIC interactions
    void Disengage();
    void Engage();

    // MOBILITY interactions
    geometry_msgs::PoseStamped GetPose();
    void FlyToWaypoint(geometry_msgs::PoseStamped pst_reference_waypoint,bool b_blocking=false);
    void FlyToHome();
    void RequestFlyToMakeRoom();
    void FlyToMakeRoom();
    void FlyToIntroduction();
    void FlyToPresent();

    // AUDITORY interactions
    void InteractionPresent(std::string s_speech_to_present);
    void InteractionVoice(std::string s_voice);
    void InteractionVoiceCommand();

    // VISUAL interactions
    void DisableHologram();
    void EnableHologram(std::string s_hologram_resource_path,std::string s_setup_marvin_hologram);

    // PROVIDE ETHICAL LOW-LEVEL PROPERTIES
    std::string GetActorTypeRole();
    int GetActorMultiplicty();
    std::string GetActorVisRep();
};

#endif //WAI_OA_MARVIN_H
