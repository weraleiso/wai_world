#ifndef WAI_OA_H
#define WAI_OA_H



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
#include<sys/stat.h>
#include<dirent.h>
#include<unistd.h>



/////////////////////////////////////////////////
/// Qt libraries
/////////////////////////////////////////////////
#include<QApplication>
#include<QMainWindow>
#include<QCoreApplication>
#include<QGuiApplication>
#include<QMessageBox>
#include<QInputDialog>
#include<QFileDialog>
#include<QTimer>
#include<QPushButton>
#include<QLineEdit>
#include<QSpinBox>
#include<QListWidget>
#include<QListWidgetItem>
#include<QImage>
#include<QIcon>
#include<QLabel>
#include<QVBoxLayout>
#include<QComboBox>
#include<QDialogButtonBox>



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
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<image_transport/image_transport.h>
/*
#include<gazebo_msgs/SpawnModel.h>
#include<gazebo_msgs/DeleteModel.h>
#include<gazebo_msgs/GetModelState.h>
#include<gazebo_msgs/SetModelState.h>
#include<gazebo_msgs/ModelState.h>
#include<gazebo_msgs/ModelStates.h>
#include<gazebo_msgs/LinkStates.h>
*/
#include<std_srvs/Empty.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<sound_play/sound_play.h>
#include<audio_common_msgs/AudioData.h>
#include<picovoice_msgs/GetIntentActionGoal.h>
#include<picovoice_msgs/GetTranscriptAction.h>
#include<picovoice_msgs/GetLLMResponseAction.h>



/////////////////////////////////////////////////
/// 3rd party libraries
/////////////////////////////////////////////////
#include<view_controller_msgs/CameraPlacement.h>
#include<zbar.h>



/////////////////////////////////////////////////
/// Helper libraries
/////////////////////////////////////////////////
#include<wai_oa_session_manager.h>
#include<wai_oa_rep_manager.h>
#include<wai_oa_rep_sequencer.h>
#include<wai_oa_presenter.h>
#include<wai_oa_audience.h>
#include<wai_oa_camera_rviz.h>
#include<wai_oa_projector.h>
#include<wai_oa_marvin.h>
#include<wai_oa_teleprompter.h>
#include<wai_oa_triggers.h>
#include<wai_oa_sketch.h>
#include<wai_lowpass_filter.h>
#include<wai_kalman_filter.h>
#include<wai_reps.h>



/////////////////////////////////////////////////
/// Helper definitions
/////////////////////////////////////////////////
#define GET_OBJECT_NAME(Variable) (#Variable)

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Tag
{
    std::string qrcode;
    std::vector<cv::Point> polygon;
};

class WAIOAState;



/////////////////////////////////////////////////
/// Implementation class of ros node
/////////////////////////////////////////////////
class WAIOpenAuditorium
{
    // Declare ROS parameters from launch file
    std::string S_SESSION_GROUP;
    std::string S_SESSION_TOPIC;
    std::string S_SESSION_EXPERTISE;
    std::string S_SESSION_EXPERTISE_LEVEL;
    std::string S_SESSION_NAME;
    int I_SESSION_LENGTH;
    int I_SESSION_BREAK_LENGTH;
    int I_SCENE_START_BEFORE;
    int I_SCENE_COUNT_MAX; // Initialized on startup (file count in slides directory)
    int I_AUDIENCE_COUNT_MAX;
    int I_AUDIENCE_LISTENERS_PER_ROW;
    int I_AUDIENCE_REQUEST_QUEUE_SIZE;
    float F_NODE_SAMPLE_FREQUENCY;
    float F_SCENE_TRANSITION_TIMEOUT;
    float F_SCENE_TRIGGER_TIMEOUT;
    float F_SCENE_TRIGGER_COLL_THRES;
    int I_PRESENTER_PRESENCE_MODE;
    int I_PRESENTER_PROMPT_MODE;
    std::string S_PRESENTER_WORKSPACE_MODEL;
    float F_PRESENTER_POSE_X;
    float F_PRESENTER_POSE_Y;
    float F_PRESENTER_POSE_Z;
    float F_PRESENTER_POSE_YAW;
    float F_PROJECTION_POSE_X;
    float F_PROJECTION_POSE_Y;
    float F_PROJECTION_POSE_Z;
    float F_PROJECTION_POSE_YAW;
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

    float F_CAMERA_RGBD_DEPTH_RESOLUTION_X;
    float F_CAMERA_RGBD_DEPTH_RESOLUTION_Y;
    float F_CAMERA_RGBD_DEPTH_RESOLUTION_SCALE;
    float F_CAMERA_RGBD_DEPTH_FX;
    float F_CAMERA_RGBD_DEPTH_FY;
    float F_CAMERA_RGBD_DEPTH_CX;
    float F_CAMERA_RGBD_DEPTH_CY;
    float F_CAMERA_RGBD_DEPTH_P_FX;
    float F_CAMERA_RGBD_DEPTH_P_FY;
    float F_CAMERA_RGBD_DEPTH_P_CX;
    float F_CAMERA_RGBD_DEPTH_P_CY;

    float F_CAMERA_RGBD_RANGE_MIN;
    float F_CAMERA_RGBD_RANGE_MAX;
    float F_CAMERA_RGBD_MASK_WIDTH;
    float F_CAMERA_RGBD_THRESHOLD_DIST;
    float F_CAMERA_RGBD_THRESHOLD_BOUNDS;

    // ENABLES for interactions
    bool B_ENABLE_KALMAN;
    bool B_ENABLE_TELEPROMPTER;
    bool B_ENABLE_CAMERA_RVIZ_FLY_IN;
    bool B_ENABLE_CAMERA_RVIZ_IDLE;
    bool B_ENABLE_LECTERN;
    bool B_ENABLE_TABLE;
    bool B_ENABLE_AUDIO;
    bool B_ENABLE_SLIDE_INTERACTIONS;
    bool B_ENABLE_SESSION_SCHEDULER;

    // EVENTS that are NON-input device specific
    bool B_EVENT_STARTUP_FINISHED;

private:
    // Declare main ROS handles
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    sound_play::SoundClient sc;

    // Declare timers
    ros::Timer tmr_init_delayed;
    ros::Timer tmr_long_interval;
    ros::Timer tmr_setup_force_view;

    // ROS TF Listener
    // NOTE: Never EVER lookup transforms if possible (perf./faulty behav. TF vs TF2, aso.)!
    // We leave this member as a reminder!
    //tf::TransformListener lst_tranforms;

    // ROS service clients
    ros::ServiceClient gazebo_spawn_model_client;
    ros::ServiceClient gazebo_delete_model_client;
    /*
    ros::ServiceClient gazebo_set_model_state;
    ros::ServiceClient gazebo_get_model_state;
    ros::ServiceClient gazebo_delete_model_client;
    ros::ServiceClient gazebo_engage_robot;
    ros::ServiceClient gazebo_disengage_robot;
    */

    // Declare hints
    image_transport::TransportHints hints;
    image_transport::TransportHints hints_depth;

    // ROS PUBLISHERS
    ros::Publisher pub_pst_robot_reference;
    ros::Publisher pub_mrk_oa_arrow_force;
    ros::Publisher pub_emp_robot_takeoff;
    ros::Publisher pub_emp_robot_land;
    ros::Publisher pub_f64_robot_rotor_middle_command;
    ros::Publisher pub_f64_robot_rotor_top_command;
    ros::Publisher pub_col_light_ambient;
    ros::Publisher pub_col_light_diff_spec;
    //std::vector<ros::Publisher> vec_pub_aud_audio_captured_presenter;
    ros::Publisher pub_pic_intent_action_goal;
    ros::Publisher pub_pic_transcript_action_goal;
    ros::Publisher pub_str_transcript_action_goal;
    ros::Publisher pub_hea_audience_request_to_panel;

    // ROS SUBSCRIBERS
    //ros::Subscriber sub_aud_audio_captured_presenter;

    // AUDIO messages
    //audio_common_msgs::AudioData msg_aud_audio_captured_presenter;

    // TF BROADCASTER
    tf::TransformBroadcaster tfb_transforms;

    //Declare colors
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
    std_msgs::ColorRGBA col_light_off,col_light_dark,col_light_default,col_light_bright;
    std_msgs::ColorRGBA col_light_cold,col_light_warm;

    zbar::ImageScanner scn_zbar;

    picovoice_msgs::GetIntentActionGoal msg_pic_intent_action_goal;
    picovoice_msgs::GetTranscriptActionGoal msg_pic_transcript_action_goal;
    std_msgs::String msg_str_transcript_action_goal;
    picovoice_msgs::GetLLMResponseActionGoal msg_pic_llm_response_action_goal;

    std::vector<std_msgs::Header> vec_hea_audience_request;
    std_msgs::Header msg_hea_audience_request_incoming;

    WAIReps* wai_oa_rep_trigger_scene_prev;
    WAIReps* wai_oa_rep_trigger_scene_next;
    WAIReps* wai_oa_rep_trigger_basic;
    WAIReps* wai_oa_rep_trigger_audience;
    WAIReps* wai_oa_rep_graph_3d_function;
    WAIReps* wai_oa_rep_graph_3d_eval;
    WAIReps* wai_oa_rep_graph_3d_stats;
    WAIReps* wai_oa_rep_wim;

    WAIRvizMarkers* mrk_oa_arrow_force;

    // Declare time objects
    ros::Time tim_startup;
    ros::Time tim_trigger_activated;
    ros::Time tim_event_audience_request;


    // OTHER helper members

    // Global path to ROS-node
    std::string s_path_nodename;

    // Scene Preview
    QImage img_scene_select_preview;

    // Audience Listener by Number Preview
    QImage img_audience_number_preview;

    // Interactions
    std::stringstream sst_wai_oa_rep_wim_cursor_label;
    std::string s_setup_hand_text;
    std::string s_setup_rep_respawn_2d3d;
    std::string s_setup_camera_rviz_frame;
    std::string s_slide_interaction_cmd;
    std::string s_force_name;
    int i_audience_id_selected;
    float f_oa_audience_stats_ping[255];
    tf::Vector3 vc3_oa_offset;
    tf::Vector3 vc3_oa_spacing;
    tf::Vector3 vc3_oa_offset_wim;
    tf::Vector3 vc3_oa_spacing_wim;
    tf::Vector3 vc3_oa_selected_wim;

    // TF and GEOMETRY helper members
    // tf::Transform tf_world_wrt_presenter_head;
    tf::Vector3 vc3_world_wrt_force_origin;
    tf::Vector3 vc3_world_wrt_force_endpoint;
    tf::Vector3 vc3_camera_rviz_wrt_detail;
    tf::Quaternion qua_world_wrt_projection;
    geometry_msgs::PointStamped msg_pts_rviz_clicked;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_home;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_space;
    geometry_msgs::PoseStamped msg_pst_marvin_ref_introduction;

    // SESSION MANAGER (Intelligence)
    WAIOASessionManager wai_oa_session_manager;

    // TRIGGERS (Intelligence)
    WAIOATriggers wai_oa_triggers;

    // REP MANAGER (Intelligence)
    WAIOARepManager wai_oa_rep_manager;

    // REP SEQUENCER (Learning Sequences)
    WAIOARepSequencer wai_oa_rep_sequencer;
    // PRESENTER (Natural)
    WAIOAPresenter wai_oa_presenter;

    // AUDIENCE (Natural(s))
    WAIOAAudience wai_oa_audience;

    // SKETCH (Display/Symbology)
    WAIOASketch wai_oa_sketch;

    // VIRTUAL CAMERA (Robot->Moveable)
    WAIOACameraRviz wai_oa_camera_rviz;
    std::vector<double> vec_camera_rviz_startup;
    std::vector<double> vec_camera_rviz_default;
    std::vector<double> vec_camera_rviz_default_fpv;
    std::vector<double> vec_camera_rviz_on_presenter;
    std::vector<double> vec_camera_rviz_on_projection;

    // PROJECTOR (Robot->Moveable)
    WAIOAProjector wai_oa_projector;

    // TELEPROMPTER (Display/Symbology)
    WAIOATeleprompter wai_oa_teleprompter;

    // MARVIN (Robot)
    WAIOAMarvin wai_oa_marvin;

    // AUDITORIUM (Workspace)
    static WAIOpenAuditorium* instance;
    WAIOAState* wai_oa_state;
    WAIOpenAuditorium();
    ~WAIOpenAuditorium();

    // Callback method definitions
    void cb_tmr_init_delayed(const ros::TimerEvent& event);
    void cb_tmr_long_interval(const ros::TimerEvent& event);
    void cb_tmr_setup_force_view(const ros::TimerEvent& event);
    //void cb_sub_aud_audio_captured_presenter(const audio_common_msgs::AudioDataPtr&);

public:
    // OA (Presenter/Server) singleton instance
    static WAIOpenAuditorium* getInstance();

    // STATE MACHINE helper methods
    void StateTransitionTo(WAIOAState* state,bool b_invoke_all_requests=false);
    void RequestSetupModel();
    void RequestSetupView();
    void RequestLeaveState();

    // SETUP scripted interactions and helper methods
    void SetupProjectorStateFromScript();
    void SetupProjectorFromScript();
    void SetupCameraRvizFromScript();
    void SetupCameraRvizFromAudienceSelected();
    void SetupCameraRvizFromRepSelected();
    void SetupCameraRvizCycle(bool b_back_forth);
    void SetupCameraRvizRestorePreviousView();
    void SetupTeleprompterFromScript();
    void SetupBrowserLinkFromScript();
    void SetupSoundFromScript();
    void PlaySound(std::string s_name_sound);
    void SetupLecternFromScript();
    void SetupTableFromScript();
    void SetupVirtualPresenterFromScript();
    void SetupGraph3DFromScript();
    void SetupGraphStatsFromScript();
    void SetupRespawn2D3DFromScript();
    void PrepareRespawn2D3DFromScript();
    void SetupHandTextFromScript();
    void SetupLightFromScript();
    void SetupRepresentativeFromScript();
    void SetupMarvinAsPresenterFromScript();
    void SetupMarvinHologramFromScript();
    void SetupLearningModeFromScript();
    std::stringstream sst_setup_rep_name;
    std::stringstream sst_setup_rep_interaction;
    std::stringstream sst_setup_rep_interaction_switch;
    std::stringstream sst_setup_rep_sequence_duration;
    std::stringstream sst_setup_rep_sequence_state;
    std::stringstream sst_setup_rep_sequence_twist;
    std::stringstream sst_setup_rep_sequence_force;
    std::stringstream sst_setup_rep_respawn;

    // Hand pointer based force interaction
    bool SetupForceOrigin();
    void SetupForceView(bool b_enabled);
    void SetupForceFromTrigger();

    // MODEL VIEW CONTROLLER helper methods invoked by SM (setup model, view and leave state)
    void SetupReceivingRequestAudienceModel(); void SetupReceivingRequestAudienceView();
    void SetupHandlingRequestAudienceAcceptModel(); void SetupHandlingRequestAudienceAcceptView(); void SetupHandlingRequestAudienceAcceptLeaveState();
    void SetupHandlingRequestAudienceRejectModel(); void SetupHandlingRequestAudienceRejectView();
    void SetupScenePrevModel(); void SetupScenePrevView();
    void SetupSceneNextModel(); void SetupSceneNextView();
    void SetupSceneSelectModel(); void SetupSceneSelectView();
    void SetupCameraRvizDefaultModel(); void SetupCameraRvizDefaultView();
    void SetupCameraRvizCycleModel(); void SetupCameraRvizCycleView();
    void SetupCameraRvizSelectModel(); void SetupCameraRvizSelectView();
    void SetupCameraRvizEnforceModel(); void SetupCameraRvizEnforceView();
    void SetupCameraRvizFollowModel(); void SetupCameraRvizFollowView(); bool ToggleTriggerCameraRvizFollow();
    void SetupCameraRvizIdleModel(); void SetupCameraRvizIdleView(); void SetupCameraRvizIdleLeaveState();
    void SetupPresenceModeModel(); void SetupPresenceModeView();
    void SetupBodyInteractionModel(); void SetupBodyInteractionView(); bool ToggleTriggerBodyInteraction();
    void SetupAvatarModel(); void SetupAvatarView(); bool ToggleTriggerAvatar();
    void SetupLecternModel(); void SetupLecternView(); bool ToggleTriggerLectern();
    void SetupTableModel(); void SetupTableView(); bool ToggleTriggerTable();
    void SetupAudioModel(); void SetupAudioView(); bool ToggleTriggerAudio();
    void SetupSketchModel(); void SetupSketchView();
    void SetupLearningModeCooperativeModel(); void SetupLearningModeCooperativeView();
    void SetupLearningModePlenumModel(); void SetupLearningModePlenumView();
    void SetupEvalModel(); void SetupEvalView();
    void SetupVoiceListenModel(); void SetupVoiceListenView();
    void SetupVoicePromptModel(); void SetupVoicePromptView();
    void SetupEvalGraphInspectModel(); void SetupEvalGraphInspectView();

    void SetupEvalBowlModel(); void SetupEvalBowlView(); void SetupEvalBowlLeaveState();
    void SetupEvalBalanceModel(); void SetupEvalBalanceView(); void SetupEvalBalanceLeaveState();
    void SetupEvalMarvinModel(); void SetupEvalMarvinView(); void SetupEvalMarvinLeaveState();

    void SetupLogAudienceIDParticipationModel(int i_log_part); void SetupLogAudienceIDParticipationView();
    void SetupLogAudienceIDExaminationModel(int i_log_exam); void SetupLogAudienceIDExaminationView();
    int SetupWIMAudienceNumberModel();
    void SetupWIMAudienceSelectionModel(bool b_enable_screenshare=false); void SetupWIMAudienceSelectionView(); void SetupWIMAudienceSelectionLeaveState();
    void SetupWIMAudienceMessageModel(); void SetupWIMAudienceMessageView();
    void SetUpWIMAudienceListenerMessageModel(); void SetUpWIMAudienceListenerMessageView();
    void SetupWIMAudienceKickModel(); void SetupWIMAudienceKickView();
    void SetupWIMRepSelectionModel(); void SetupWIMRepSelectionView(); void SetupWIMRepSelectionLeaveState();
    void SetupMarvinModel(); void SetupMarvinView(); void SetupMarvinLeaveState();
    void SetupMarvinHologramModel(); void SetupMarvinHologramView(); void SetupMarvinHologramLeaveState();
    void SetupIntroductionModel(); void SetupIntroductionView(); void SetupIntroductionLeaveState();

    // OTHER helper methods
    void SendInfoToOAPanel(std::string s_info_to_panel);
    void SpawnEvalMetaphor(int i_count_score,
                          int i_count_overall,
                          std::string s_model_name_score,
                          std::string s_model_name_overall,
                          std::string s_model_name_diff,
                          std::string s_eval_metaphor_name);
    void CleanupEvalMetaphor(std::string s_model_name_score,
                            std::string s_model_name_overall,
                            std::string s_model_name_diff);
    std::vector< std::vector<float> > GetData3DFromFile(std::string filename,int* dim_x,int* dim_y);
    std::string RemoveNumbers(std::string s_string);
    std::string GetStateNameCurrent();
    std::string GetStateNameCurrentTTS();
    std::string GetEventNameInpDevLast();
    std::string GetCameraFrameNames();
    std::string GetRepSelectedClean();
    std::string GetRepDetailSelectedClean();
    std::string CleanupStringRep(std::string s_string);
    void SetTimeTriggerActivated(ros::Time tim_trigger);
    void SetEventNameInpDevLast();
    void SendInfoToAudienceID(int i_id,std::string s_msg);
    void SendInfoToAudience(std::string s_msg);
    std_msgs::Header GetAudienceRequestIncoming();
    void SetAudienceRequestIncoming(std_msgs::Header hea_audience_request_incoming);
    void SetRvizPointClicked(geometry_msgs::PointStamped pts_rviz_clicked);
    void SpinAndWaitForCondition(bool* b_cond,bool b_state);
    void SpinAndWaitForSeconds(float f_seconds);
    void ShutdownMarvin();
    void EngageMarvin();
    void UpdateAudienceIDSelected(int i_id_select);
    void UpdateWIMFromAudienceSelected();
    //void UpdateRepAndDetailSelected(int i_rep_select,std::string s_rep_select="",std::string s_detail_select="");
    void UpdateRepAndDetailSelected(bool b_use_dialog,std::string s_rep_select="workspace_presenter",std::string s_detail_select="link_base");
    int GetAudienceIDFromRequestMsg(std::string);
    ros::Time GetTimeTriggerActivated();
    inline bool GetFileExists(const std::string& s_path_file)
    {
        struct stat buffer;
        return (stat (s_path_file.c_str(), &buffer) == 0);
    }
    inline int getWeekNumber()
    {
        time_t t = time(nullptr);
        struct tm tm_info;
        localtime_r(&t, &tm_info);
        char buffer[3];
        strftime(buffer, sizeof(buffer), "%V", &tm_info);  // ISO 8601 week number
        return std::atoi(buffer);
    }
    int GetSceneCountCurrent();
    int GetSceneCountMax();
    bool GetCameraRvizEnabled();
    bool GetSessionStartupFinished();
    void ToggleRepDetailRotation();
    void RvizCameraIdleStop();
    bool CheckCollisionTriggerScenePrev();
    bool CheckCollisionTriggerSceneNext();
    bool CheckCollisionTriggerRepSelect();
    bool CheckCollisionTriggerRespawn2D3D();

    // INPUT DEVICE helper methods
    bool CheckCollision(tf::Vector3 vc3_current,tf::Vector3 vc3_reference,float f_threshold_radius);

    void run();
};



#endif //WAI_OA_H
