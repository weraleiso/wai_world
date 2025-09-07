#ifndef WAI_OA_SESSION_MANAGER_H
#define WAI_OA_SESSION_MANAGER_H



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
#include<stdio.h>
#include<sys/types.h>
#include<dirent.h>
#include<unistd.h>
#include<boost/filesystem.hpp>

#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<sound_play/sound_play.h>

#include<view_controller_msgs/CameraPlacement.h>

#include<wai_oa_rep_manager.h>
#include<wai_oa_audience.h>
#include<wai_oa_presenter.h>
#include<wai_oa_projector.h>
#include<wai_oa_camera_rviz.h>
#include<wai_reps.h>



////////////////////////////////////////////////////
/// Class definition of WAIOASessionManager
////////////////////////////////////////////////////


class WAIOASessionManager
{
    ros::NodeHandle* m_hdl_node;
    sound_play::SoundClient* m_hdl_snd_client;

    WAIOARepManager* m_oa_rep_manager;
    WAIOAAudience* m_oa_audience;
    WAIOAPresenter* m_oa_presenter;
    WAIOAProjector* m_oa_projector;
    WAIOACameraRviz* m_oa_cam_rviz;
    WAIReps* m_wai_oa_rep_wim;

    float m_f_node_sample_frequency;

    ros::Publisher pub_s_oa_status;
    ros::Publisher pub_s_session_status;
    ros::Publisher pub_emp_rviz_models_reload;

    std::string m_s_session_group;
    std::string m_s_session_topic;
    std::string m_s_session_name;
    std::string m_s_session_expertise;
    std::string m_s_session_expertise_level;

    std::string m_s_path_nodename;
    std::string m_s_path_resources;
    std::string m_s_path_resources_evals_grp_and_sub;
    std::string m_s_path_resources_aliases_grp_and_sub;
    std::string m_s_path_resources_aliases_grp_and_id_preview;
    std::string m_s_path_resources_descriptions;
    std::string m_s_path_resources_multiplots;
    std::string m_s_path_resources_reps;
    std::string m_s_path_resources_logo;
    std::string m_s_path_resources_session;
    std::string m_s_path_resources_session_prev;
    std::string m_s_path_resources_configfile;
    std::string m_s_path_resources_slides;
    inline bool GetFileExists(const std::string& s_path_file)
    {
        struct stat buffer;
        return (stat (s_path_file.c_str(), &buffer) == 0);
    }
    std::string m_s_session_workspace_presenter;
    std::stringstream m_sst_session_status;
    std::string m_s_oa_status;
    std::vector<std::string> m_vec_s_setup_camera_rviz_predefined;
    std::vector< std::vector<double>> m_vec_camera_rviz_views_predefined;

    bool m_b_enable_session_scheduler;
    bool m_b_enable_session_status_label;
    bool m_b_session_ends_notified;

    int m_i_session_scene_start_before;
    int m_i_session_scene_count_current;
    int m_i_session_scene_count_max;
    int m_i_session_length;
    int m_i_session_break_length;

    //ros::Time m_tim_session_duration;
    ros::Time m_tim_session_time_start;
    float m_f_session_time_left;
    int m_i_session_slot_current;
    int m_i_session_slots_max;
    bool m_b_session_slot_found;
    float m_f_session_duration;
    float m_f_session_length;
    float m_f_session_pace_th;
    float m_f_ses_pac_scene;
    float m_f_ses_pac_time;
    float m_f_session_pace;


    ros::Timer m_tmr_session_scheduler;
    ros::Timer tmr_session_status;

    void cb_tmr_session_scheduler(const ros::TimerEvent& event);
    void cb_tmr_session_status(const ros::TimerEvent& event);

public:
    WAIOASessionManager();
    ~WAIOASessionManager();

    void Initialize(ros::NodeHandle* hdl_node,
                    sound_play::SoundClient* hdl_snd_client,
                    float f_node_sample_frequency);
    void UpdateModel(std::string s_path_nodename,
                     std::string s_path_resources,
                     std::string s_session_group,
                     std::string s_session_topic,
                     std::string s_session_name,
                     int i_session_scene_start_before,
                     int i_session_length,
                     int i_session_break_length,
                     bool b_enable_session_scheduler);
    void UpdateView();
    void ConnectRepsAndLoadSession(WAIOARepManager* wai_oa_rep_manager,
                                   WAIOAAudience* wai_oa_audience,
                                   WAIOAPresenter* wai_oa_presenter,
                                   WAIOAProjector* wai_oa_projector,
                                   WAIOACameraRviz* wai_oa_cam_rviz,
                                   WAIReps* wai_oa_rep_wim);
    void UpdateSessionFromSchedule();
    void UpdateSessionSlidesConfig();
    void LoadSession();
    void LoadWorkspacePresenter(std::string s_workspace_name);
    void RespawnRepFromTemplate(std::string s_workspace_name);
    int GetSessionSlotCurrent();
    int GetSessionSceneCountCurrent();
    int GetSessionSceneCountMax();
    std::string GetSessionPace(bool b_scheduler_enabled);
    void SetSessionSceneCountCurrent(bool b_increment_decrement,int i_scene_select=-1,bool b_scene_update_cfg=true);
    void SchedulerStart();
    void SchedulerStop();
    std::string GetOAStatusLabel();
    void SetOAStatusLabel(std::string s_oa_state);
    void EnableUpdateSessionStatusLabel();
    void PreparePlenumMode();
    void PrepareCooperativeMode(std::string s_rep_name,
                               tf::Vector3 vc3_rep_position,
                               float f_rep_yaw=0.0,
                               float f_dist_audience=2.5,
                               int i_mode_audience=0);
    std::vector<std::string> GetCameraRVizViewLabels();
    std::vector< std::vector<double>> GetCameraRVizViews();
    std::string GetPathResourcesRoot();
    std::string GetPathResourcesDescriptions();
    std::string GetPathResourcesMultiplots();
    std::string GetPathResourcesReps();
    std::string GetPathResourcesLogo();
    std::string GetPathResourcesSlides();
    std::string GetPathResourcesEvalsGroupAndSubject();
    std::string GetPathResourcesAliasesGroupAndSubject();
    std::string GetPathResourcesAliasesGroupAndIDPreview();
    std::string GetSessionGroup();
    std::string GetSessionTopic();
    std::string GetSessionName();
    std::string GetSessionExpertise();
    std::string GetSessionExpertiseLevel();
    std::string GetSessionWorkspacePresenter();
    std::string GetSessionStatusLabel();
    void Say(std::string s_say);
    void SpinAndWaitForSeconds(float f_seconds);
};

#endif //WAI_OA_SESSION_MANAGER_H
