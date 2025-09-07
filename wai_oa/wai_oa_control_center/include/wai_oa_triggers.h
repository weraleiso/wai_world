#ifndef WAI_OA_TRIGGERS_H
#define WAI_OA_TRIGGERS_H



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

#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Joy.h>
#include<sensor_msgs/JoyFeedback.h>
#include<sensor_msgs/JoyFeedbackArray.h>
#include<rviz_vive_plugin_msgs/Controller.h>
#include<rviz_vive_plugin_msgs/ControllerVibration.h>

#include<picovoice_msgs/GetIntentActionResult.h>

#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<X11/Xlib.h>
#include<X11/keysym.h>



/////////////////////////////////////////////////
/// Trigger base class (abstract command)
/////////////////////////////////////////////////
class WAIOpenAuditorium;

class WAIAuditoriumTrigger
{
public:
    WAIAuditoriumTrigger();
    ~WAIAuditoriumTrigger();

    virtual void Activate()=0;
};


/////////////////////////////////////////////////
/// Trigger child classes (derived commands)
/////////////////////////////////////////////////
class WAIAuditoriumTriggerPresenceMode:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerPresenceMode(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerPresenceMode();

    void Activate();
};

class WAIAuditoriumTriggerSceneSelectPrev:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerSceneSelectPrev(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerSceneSelectPrev();

    void Activate();
};

class WAIAuditoriumTriggerSceneSelectNext:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerSceneSelectNext(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerSceneSelectNext();

    void Activate();
};

class WAIAuditoriumTriggerSceneSelectNumber:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerSceneSelectNumber(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerSceneSelectNumber();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizDefault:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizDefault(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizDefault();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizCycle:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizCycle(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizCycle();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizSelect:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizSelect(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizSelect();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizEnforce:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizEnforce(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizEnforce();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizIdle:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizIdle(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizIdle();

    void Activate();
};

class WAIAuditoriumTriggerCameraRvizFollow:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerCameraRvizFollow(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerCameraRvizFollow();

    void Activate();
};

class WAIAuditoriumTriggerBodyInteraction:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerBodyInteraction(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerBodyInteraction();

    void Activate();
};

class WAIAuditoriumTriggerAvatar:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAvatar(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAvatar();

    void Activate();
};

class WAIAuditoriumTriggerLectern:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerLectern(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerLectern();

    void Activate();
};

class WAIAuditoriumTriggerTable:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerTable(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerTable();

    void Activate();
};

class WAIAuditoriumTriggerAudio:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudio(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudio();

    void Activate();
};

class WAIAuditoriumTriggerSketch:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerSketch(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerSketch();

    void Activate();
};

class WAIAuditoriumTriggerLearningModePlen:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerLearningModePlen(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerLearningModePlen();

    void Activate();
};

class WAIAuditoriumTriggerLearningModeCoop:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerLearningModeCoop(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerLearningModeCoop();

    void Activate();
};

class WAIAuditoriumTriggerRespawn2D3D:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerRespawn2D3D(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerRespawn2D3D();

    void Activate();
};

class WAIAuditoriumTriggerForce:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerForce(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerForce();

    void Activate();
};

class WAIAuditoriumTriggerVoiceCommand:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerVoiceCommand(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerVoiceCommand();

    void Activate();
};

class WAIAuditoriumTriggerVoicePrompt:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerVoicePrompt(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerVoicePrompt();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalGraph:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalGraph(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalGraph();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalGraphInspect:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalGraphInspect(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalGraphInspect();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalMetaphoreBowl:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalMetaphoreBowl(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalMetaphoreBowl();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalMetaphoreBalance:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalMetaphoreBalance(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalMetaphoreBalance();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalMetaphoreMarvin();

    void Activate();
};

class WAIAuditoriumTriggerAudienceRequestIncoming:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceRequestIncoming(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceRequestIncoming();

    void Activate();
};

class WAIAuditoriumTriggerAudienceRequestAccept:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceRequestAccept(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceRequestAccept();

    void Activate();
};

class WAIAuditoriumTriggerAudienceRequestReject:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceRequestReject(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceRequestReject();

    void Activate();
};

class WAIAuditoriumTriggerAudienceFocusNumber:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceFocusNumber(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceFocusNumber();

    void Activate();
};
class WAIAuditoriumTriggerAudienceFocusDown:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceFocusDown(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceFocusDown();

    void Activate();
};
class WAIAuditoriumTriggerAudienceFocusUp:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceFocusUp(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceFocusUp();

    void Activate();
};
class WAIAuditoriumTriggerAudienceFocusLeft:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceFocusLeft(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceFocusLeft();

    void Activate();
};
class WAIAuditoriumTriggerAudienceFocusRight:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceFocusRight(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceFocusRight();

    void Activate();
};
class WAIAuditoriumTriggerAudienceSelect:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceSelect(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceSelect();

    void Activate();
};
class WAIAuditoriumTriggerAudienceMessage:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceMessage(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceMessage();

    void Activate();
};
class WAIAuditoriumTriggerAudienceListenerMessage:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceListenerMessage(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceListenerMessage();

    void Activate();
};
class WAIAuditoriumTriggerAudienceKick:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceKick(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceKick();

    void Activate();
};

class WAIAuditoriumTriggerAudienceEvalPartMinus:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalPartMinus(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalPartMinus();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalPartTilde:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalPartTilde(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalPartTilde();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalPartPlus:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalPartPlus(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalPartPlus();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalPartScore:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalPartScore(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalPartScore();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamInsufficient:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamInsufficient(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamInsufficient();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamSufficient:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamSufficient(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamSufficient();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamSatisfactory:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamSatisfactory(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamSatisfactory();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamGood:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamGood(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamGood();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamVeryGood:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamVeryGood(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamVeryGood();

    void Activate();
};
class WAIAuditoriumTriggerAudienceEvalExamScore:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerAudienceEvalExamScore(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerAudienceEvalExamScore();

    void Activate();
};

class WAIAuditoriumTriggerRepDetailFocus:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerRepDetailFocus(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerRepDetailFocus();

    void Activate();
};
class WAIAuditoriumTriggerRepSelect:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerRepSelect(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerRepSelect();

    void Activate();
};
class WAIAuditoriumTriggerRepDetailRot:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerRepDetailRot(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerRepDetailRot();

    void Activate();
};

class WAIAuditoriumTriggerIntroduction:public WAIAuditoriumTrigger
{
    WAIOpenAuditorium* m_wai_auditorium;

public:
    WAIAuditoriumTriggerIntroduction(WAIOpenAuditorium* wai_auditorium);
    ~WAIAuditoriumTriggerIntroduction();

    void Activate();
};




class WAIOATriggers
{
    ros::NodeHandle* m_hdl_node;
    float m_f_node_sample_frequency;

    WAIOpenAuditorium* m_wai_open_auditorium;

    Display* dsp_x11_display;

    // TRIGGER STATUS - Helper members
    ros::Publisher m_pub_trigger_status;
    std_msgs::String m_msg_str_trigger_status;

    // JOYPAD - Helper members
    ros::Subscriber m_sub_joy_controller;
    ros::Publisher m_pub_jfa_joy_input_dev_feedback;
    sensor_msgs::Joy m_msg_joy_input_dev;
    sensor_msgs::JoyFeedbackArray m_msg_jfa_joy_input_dev_feedback;

    // HTC VIVE - Helper members
    ros::Subscriber m_sub_htc_vive_controller_left;
    ros::Subscriber m_sub_htc_vive_controller_right;
    ros::Publisher m_pub_vib_htc_vive_input_dev_feedback_left;
    ros::Publisher m_pub_vib_htc_vive_input_dev_feedback_right;
    rviz_vive_plugin_msgs::Controller m_msg_htc_vive_input_dev_left;
    rviz_vive_plugin_msgs::Controller m_msg_htc_vive_input_dev_right;

    // MOBILE DEVICE - Helper members
    ros::Subscriber sub_bol_trg_mob_scene_select_prev;
    ros::Subscriber sub_bol_trg_mob_scene_select_next;
    ros::Subscriber sub_bol_trg_mob_scene_select_number;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_default;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_cycle;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_select;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_enforce;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_idle;
    ros::Subscriber sub_bol_trg_mob_camera_rviz_follow;
    ros::Subscriber sub_bol_trg_mob_introduction;
    ros::Subscriber sub_bol_trg_mob_presence_mode;
    ros::Subscriber sub_bol_trg_mob_body_interaction;
    ros::Subscriber sub_bol_trg_mob_avatar;
    ros::Subscriber sub_bol_trg_mob_lectern;
    ros::Subscriber sub_bol_trg_mob_table;
    ros::Subscriber sub_bol_trg_mob_audio;
    ros::Subscriber sub_bol_trg_mob_voice_command;
    ros::Subscriber sub_bol_trg_mob_voice_prompt;
    ros::Subscriber sub_bol_trg_mob_audience_eval_graph;
    ros::Subscriber sub_bol_trg_mob_audience_eval_metaphore_bowl;
    ros::Subscriber sub_bol_trg_mob_audience_eval_metaphore_balance;
    ros::Subscriber sub_bol_trg_mob_audience_eval_metaphore_marvin;
    ros::Subscriber sub_bol_trg_mob_audience_request_reject;
    ros::Subscriber sub_bol_trg_mob_audience_request_accept;
    ros::Subscriber sub_bol_trg_mob_audience_focus_down;
    ros::Subscriber sub_bol_trg_mob_audience_focus_up;
    ros::Subscriber sub_bol_trg_mob_audience_focus_left;
    ros::Subscriber sub_bol_trg_mob_audience_focus_right;
    ros::Subscriber sub_bol_trg_mob_audience_select;
    ros::Subscriber sub_bol_trg_mob_audience_eval_part_minus;
    ros::Subscriber sub_bol_trg_mob_audience_eval_part_tilde;
    ros::Subscriber sub_bol_trg_mob_audience_eval_part_plus;
    ros::Subscriber sub_bol_trg_mob_audience_eval_exam_insufficient;
    ros::Subscriber sub_bol_trg_mob_audience_eval_exam_sufficient;
    ros::Subscriber sub_bol_trg_mob_audience_eval_exam_satisfactory;
    ros::Subscriber sub_bol_trg_mob_audience_eval_exam_good;
    ros::Subscriber sub_bol_trg_mob_audience_eval_exam_very_good;
    ros::Subscriber sub_bol_trg_mob_rep_detail_focus;
    ros::Subscriber sub_bol_trg_mob_rep_detail_rot;
    ros::Subscriber sub_bol_trg_mob_rep_detail_select;
    ros::Subscriber sub_bol_trg_mob_learning_mode_plen;
    ros::Subscriber sub_bol_trg_mob_learning_mode_coop;

    bool b_trg_mob_scene_select_prev; // Due to X11 lib def - #define Bool int - conflicting with bool definition
    bool b_trg_mob_scene_select_next;
    bool b_trg_mob_scene_select_number;
    bool b_trg_mob_camera_rviz_default;
    bool b_trg_mob_camera_rviz_cycle;
    bool b_trg_mob_camera_rviz_select;
    bool b_trg_mob_camera_rviz_enforce;
    bool b_trg_mob_camera_rviz_idle;
    bool b_trg_mob_camera_rviz_follow;
    bool b_trg_mob_introduction;
    bool b_trg_mob_presence_mode;
    bool b_trg_mob_body_interaction;
    bool b_trg_mob_avatar;
    bool b_trg_mob_lectern;
    bool b_trg_mob_table;
    bool b_trg_mob_audio;
    bool b_trg_mob_voice_command;
    bool b_trg_mob_voice_prompt;
    bool b_trg_mob_audience_eval_graph;
    bool b_trg_mob_audience_eval_metaphore_bowl;
    bool b_trg_mob_audience_eval_metaphore_balance;
    bool b_trg_mob_audience_eval_metaphore_marvin;
    bool b_trg_mob_audience_request_reject;
    bool b_trg_mob_audience_request_accept;
    bool b_trg_mob_audience_focus_down;
    bool b_trg_mob_audience_focus_up;
    bool b_trg_mob_audience_focus_left;
    bool b_trg_mob_audience_focus_right;
    bool b_trg_mob_audience_select;
    bool b_trg_mob_audience_eval_part_minus;
    bool b_trg_mob_audience_eval_part_tilde;
    bool b_trg_mob_audience_eval_part_plus;
    bool b_trg_mob_audience_eval_exam_insufficient;
    bool b_trg_mob_audience_eval_exam_sufficient;
    bool b_trg_mob_audience_eval_exam_satisfactory;
    bool b_trg_mob_audience_eval_exam_good;
    bool b_trg_mob_audience_eval_exam_very_good;
    bool b_trg_mob_rep_detail_focus;
    bool b_trg_mob_rep_select;
    bool b_trg_mob_rep_detail_rot;
    bool b_trg_mob_learning_mode_plen;
    bool b_trg_mob_learning_mode_coop;

    // INCOMING EVENTS - Helper members
    ros::Subscriber sub_hea_audience_request;
    std_msgs::Header m_msg_hea_audience_request;
    bool m_b_hea_audience_request_received;

    // POINT AND CLICK - Helper members
    ros::Subscriber sub_pts_rviz_clicked;
    geometry_msgs::PointStamped m_msg_pts_rviz_clicked;
    bool m_b_pts_rviz_clicked_received;

    // VOICE - Helper members
    ros::Subscriber sub_pic_intent_action_result;
    picovoice_msgs::GetIntentActionResult msg_pic_intent_action_result;
    bool b_voi_scene_prev;
    bool b_voi_scene_next;
    bool b_voi_camera_rviz_default;
    bool b_voi_camera_rviz_cycle;
    bool b_voi_camera_rviz_enforce;
    bool b_voi_camera_rviz_idle;
    bool b_voi_camera_rviz_follow;
    bool b_voi_introduction;
    bool b_voi_presence_mode;
    bool b_voi_body_interaction;
    bool b_voi_avatar;
    bool b_voi_lectern;
    bool b_voi_table;
    bool b_voi_audio;
    bool b_voi_audience_eval_graph;
    bool b_voi_audience_eval_metaphorebowl;
    bool b_voi_audience_eval_metaphorebalance;
    bool b_voi_audience_eval_metaphoremarvin;
    bool b_voi_audience_request_reject;
    bool b_voi_audience_request_accept;
    bool b_voi_audience_focus_down;
    bool b_voi_audience_focus_up;
    bool b_voi_audience_focus_left;
    bool b_voi_audience_focus_right;
    bool b_voi_wim_audience_select;
    bool b_voi_audience_eval_part_minus;
    bool b_voi_audience_eval_part_tilde;
    bool b_voi_audience_eval_part_plus;
    bool b_voi_audience_eval_exam_insufficient;
    bool b_voi_audience_eval_exam_sufficient;
    bool b_voi_audience_eval_exam_satisfactory;
    bool b_voi_audience_eval_exam_good;
    bool b_voi_audience_eval_exam_very_good;
    bool b_voi_rep_detail_focus;
    bool b_voi_rep_detail_select;
    bool b_voi_rep_detail_rot;
    bool b_voi_learning_mode_plen;
    bool b_voi_learning_mode_coop;

    // TRIGGERs
    WAIAuditoriumTrigger* m_trg_scene_select_prev;
    WAIAuditoriumTrigger* m_trg_scene_select_next;
    WAIAuditoriumTrigger* m_trg_scene_select_number;
    WAIAuditoriumTrigger* m_trg_camera_rviz_default;
    WAIAuditoriumTrigger* m_trg_camera_rviz_cycle;
    WAIAuditoriumTrigger* m_trg_camera_rviz_select;
    WAIAuditoriumTrigger* m_trg_camera_rviz_enforce;
    WAIAuditoriumTrigger* m_trg_camera_rviz_idle;
    WAIAuditoriumTrigger* m_trg_camera_rviz_follow;
    WAIAuditoriumTrigger* m_trg_introduction;
    WAIAuditoriumTrigger* m_trg_presence_mode;
    WAIAuditoriumTrigger* m_trg_body_interaction;
    WAIAuditoriumTrigger* m_trg_avatar;
    WAIAuditoriumTrigger* m_trg_lectern;
    WAIAuditoriumTrigger* m_trg_table;
    WAIAuditoriumTrigger* m_trg_audio;
    WAIAuditoriumTrigger* m_trg_sketch;
    WAIAuditoriumTrigger* m_trg_respawn_2d3d;
    WAIAuditoriumTrigger* m_trg_force;
    WAIAuditoriumTrigger* m_trg_voice_command;
    WAIAuditoriumTrigger* m_trg_voice_prompt;
    WAIAuditoriumTrigger* m_trg_audience_eval_graph;
    WAIAuditoriumTrigger* m_trg_audience_eval_graph_inspect;
    WAIAuditoriumTrigger* m_trg_audience_eval_metaphore_bowl;
    WAIAuditoriumTrigger* m_trg_audience_eval_metaphore_balance;
    WAIAuditoriumTrigger* m_trg_audience_eval_metaphore_marvin;
    WAIAuditoriumTrigger* m_trg_audience_request_incoming;
    WAIAuditoriumTrigger* m_trg_audience_request_reject;
    WAIAuditoriumTrigger* m_trg_audience_request_accept;
    WAIAuditoriumTrigger* m_trg_audience_focus_number;
    WAIAuditoriumTrigger* m_trg_audience_focus_down;
    WAIAuditoriumTrigger* m_trg_audience_focus_up;
    WAIAuditoriumTrigger* m_trg_audience_focus_left;
    WAIAuditoriumTrigger* m_trg_audience_focus_right;
    WAIAuditoriumTrigger* m_trg_audience_select;
    WAIAuditoriumTrigger* m_trg_audience_message;
    WAIAuditoriumTrigger* m_trg_audience_listener_message;
    WAIAuditoriumTrigger* m_trg_audience_kick;
    WAIAuditoriumTrigger* m_trg_audience_eval_part_minus;
    WAIAuditoriumTrigger* m_trg_audience_eval_part_tilde;
    WAIAuditoriumTrigger* m_trg_audience_eval_part_plus;
    WAIAuditoriumTrigger* m_trg_audience_eval_part_score;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_insufficient;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_sufficient;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_satisfactory;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_good;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_very_good;
    WAIAuditoriumTrigger* m_trg_audience_eval_exam_score;
    WAIAuditoriumTrigger* m_trg_rep_detail_focus;
    WAIAuditoriumTrigger* m_trg_rep_detail_rot;
    WAIAuditoriumTrigger* m_trg_rep_detail_select;
    WAIAuditoriumTrigger* m_trg_learning_mode_plen;
    WAIAuditoriumTrigger* m_trg_learning_mode_coop;

    ros::Time m_tim_trigger_activated;
    ros::Time m_tim_last_audience_request_received;
    float m_f_trigger_timeout;
    bool m_b_triggers_available;
    std::string m_s_oa_event_inpdev_last;

    ros::Timer m_tmr_trigger_joypad_feedback;

    void cb_m_sub_joy_controller(const sensor_msgs::JoyPtr&);
    void cb_tmr_trigger_joypad_feedback(const ros::TimerEvent& event);

    void cb_m_sub_htc_vive_controller_left(const rviz_vive_plugin_msgs::ControllerPtr& msg);
    void cb_m_sub_htc_vive_controller_right(const rviz_vive_plugin_msgs::ControllerPtr& msg);

    void cb_sub_bol_trg_mob_scene_select_prev(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_scene_select_next(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_scene_select_number(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_default(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_cycle(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_select(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_enforce(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_idle(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_camera_rviz_follow(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_introduction(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_presence_mode(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_body_interaction(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_avatar(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_lectern(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_table(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audio(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_voice_command(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_voice_prompt(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_graph(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_metaphore_bowl(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_metaphore_balance(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_metaphore_marvin(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_request_reject(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_request_accept(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_focus_down(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_focus_up(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_focus_left(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_focus_right(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_select(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_part_minus(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_part_tilde(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_part_plus(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_exam_insufficient(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_exam_sufficient(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_exam_satisfactory(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_exam_good(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_audience_eval_exam_very_good(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_rep_detail_focus(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_rep_detail_rot(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_rep_detail_select(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_learning_mode_plen(const std_msgs::BoolPtr&);
    void cb_sub_bol_trg_mob_learning_mode_coop(const std_msgs::BoolPtr&);

    void cb_sub_hea_audience_request(const std_msgs::HeaderPtr&);
    void cb_sub_pts_rviz_clicked(const geometry_msgs::PointStampedPtr& msg);

    void cb_sub_pic_intent_action_result(const picovoice_msgs::GetIntentActionResultConstPtr& msg);

public:
    WAIOATriggers();
    ~WAIOATriggers();

    void Initialize(ros::NodeHandle* hdl_node,WAIOpenAuditorium* wai_open_auditorium,float f_node_sample_frequency,float f_trigger_timeout=3.0);
    void UpdateModel();
    void UpdateView();
    void ResetBoolEventTriggersMobile();
    void ResetBoolEventTriggersVoice();
    void SetTriggerParams(float f_trigger_timeout=1.0);
    std::string GetTriggersLast();
    bool GetTriggersAvailable();

    // Summarize all triggers
    void CheckTriggersAll();

    // Keyboard
    void CheckTriggersKeyboard();
    bool KeyIsPressed(KeySym ks);

    // Joypad
    void CheckTriggersJoypad();
    tf::Vector3 GetJoypadAxesBodyInteraction();
    void TriggerJoypadSendFeedback(float f_duration=0.5,float f_intensity=0.5);
    void TriggeredJoypadButtonsAny();
    void TriggeredJoypadOptionsCursorLeft();
    void TriggeredJoypadOptionsCursorRight();
    void TriggeredJoypadOptionsCursorDown();
    void TriggeredJoypadOptionsCursorUp();
    void TriggeredJoypadJSLeftButtonCircle();
    void TriggeredJoypadJSLeftButtonCross();
    void TriggeredJoypadJSLeftButtonTriangle();
    void TriggeredJoypadJSLeftButtonSquare();
    void TriggeredJoypadJSRightButtonSquare();
    void TriggeredJoypadJSLeftButtonJSRightButton();
    void TriggeredJoypadTriggerL1TriggerR1();
    void TriggeredJoypadTriggerL2();
    void TriggeredJoypadTriggerR2();
    void TriggeredJoypadShareOptions();
    void TriggeredJoypadSharePSButton();
    void TriggeredJoypadShareCross();
    void TriggeredJoypadShareTriangle();
    void TriggeredJoypadShareSquare();
    void TriggeredJoypadShareCircle();
    void TriggeredJoypadShareJSLeftButton(); // available!
    void TriggeredJoypadPSButtonCross();
    void TriggeredJoypadPSButtonCircle();
    void TriggeredJoypadPSButtonTriangle();
    void TriggeredJoypadPSButtonRectangle();
    void TriggeredJoypadPSButtonJSLeftButton();
    void TriggeredJoypadPSButtonJSRightButton();
    void TriggeredJoypadShareJSRightAxesDown();
    void TriggeredJoypadShareJSRightAxesUp();
    void TriggeredJoypadShareJSRightAxesLeft();
    void TriggeredJoypadShareJSRightAxesRight();
    void TriggeredJoypadShareJSRightButton();
    void TriggeredJoypadOptionsJLeftAxesDown();
    void TriggeredJoypadOptionsJLeftAxesUp();
    void TriggeredJoypadOptionsJLeftAxesLeft();
    void TriggeredJoypadOptionsJLeftAxesRight();
    void TriggeredJoypadOptionsJLeftButton();
    void TriggeredJoypadOptionsPSButton();

    // HTC Vive
    void CheckTriggersHTCVive();
    void TriggerHTCViveSendFeedbackLeft(float f_duration=1.0,float f_intensity=1.0);
    void TriggerHTCViveSendFeedbackRight(float f_duration=1.0,float f_intensity=1.0);

    void TriggeredHTCViveControllerLeftTrackpadMiddle();
    void TriggeredHTCViveControllerLeftTrackpadLeft();
    void TriggeredHTCViveControllerLeftTrackpadRight();
    void TriggeredHTCViveControllerLeftTrackpadDown();
    void TriggeredHTCViveControllerLeftTrackpadUp();
    void TriggeredHTCViveControllerLeftMenu();
    void TriggeredHTCViveControllerLeftTrigger();
    void TriggeredHTCViveControllerLeftGripTrackpadMiddle();
    void TriggeredHTCViveControllerLeftGripTrackpadLeft();
    void TriggeredHTCViveControllerLeftGripTrackpadRight();
    void TriggeredHTCViveControllerLeftGripTrackpadDown();
    void TriggeredHTCViveControllerLeftGripTrackpadUp();
    void TriggeredHTCViveControllerLeftGripMenu();
    void TriggeredHTCViveControllerLeftGripTrigger();

    void TriggeredHTCViveControllerRightTrackpadMiddle();
    void TriggeredHTCViveControllerRightTrackpadLeft();
    void TriggeredHTCViveControllerRightTrackpadRight();
    void TriggeredHTCViveControllerRightTrackpadDown();
    void TriggeredHTCViveControllerRightTrackpadUp();
    void TriggeredHTCViveControllerRightMenu();
    void TriggeredHTCViveControllerRightTrigger();
    void TriggeredHTCViveControllerRightGripTrackpadMiddle();
    void TriggeredHTCViveControllerRightGripTrackpadLeft();
    void TriggeredHTCViveControllerRightGripTrackpadRight();
    void TriggeredHTCViveControllerRightGripTrackpadDown();
    void TriggeredHTCViveControllerRightGripTrackpadUp();
    void TriggeredHTCViveControllerRightGripMenu();
    void TriggeredHTCViveControllerRightGripTrigger();


    // Mobile device
    void CheckTriggersMobile();

    // Point and click
    void CheckTriggersPointAndClick();

    // Voice
    void CheckTriggersVoice();

    // Body interaction
    void CheckTriggersBodyInteraction();

    // Events
    void CheckTriggersEvents();
};


#endif //WAI_OA_TRIGGERS_H
