#include<wai_oa_session_manager.h>



/////////////////////////////////////////////////
/// Implementation of WAIOASessionManager
/////////////////////////////////////////////////

WAIOASessionManager::WAIOASessionManager()
{
}

WAIOASessionManager::~WAIOASessionManager()
{
}

void WAIOASessionManager::cb_tmr_session_scheduler(const ros::TimerEvent& event)
{
    // Update GROUP, TOPIC and NAME of session
    UpdateSessionFromSchedule();

    // Update scheduler for new config only
    if(m_s_path_resources_session.compare(m_s_path_resources_session_prev)==0)
    {
        return;
    }
    else
    {
        LoadSession();
        Say("IntelSessionManagerSessionLoaded");
    }
}

void WAIOASessionManager::Initialize(ros::NodeHandle* hdl_node,
                                             sound_play::SoundClient* hdl_snd_client,
                                             float f_node_sample_frequency)
{
    m_hdl_node=hdl_node;
    m_hdl_snd_client=hdl_snd_client;

    m_f_node_sample_frequency=f_node_sample_frequency;

    m_s_session_group="group_default";
    m_s_session_topic="topic_default";
    m_s_session_name="break";
    m_s_session_expertise="expertise_default";
    m_s_session_expertise_level="level_default";
    m_s_session_workspace_presenter="default";

    m_s_path_nodename="nodename";
    m_s_path_resources="resources";
    m_s_path_resources_descriptions=m_s_path_resources+"descriptions/";
    m_s_path_resources_multiplots=m_s_path_resources+"multiplots/";
    m_s_path_resources_reps=m_s_path_resources+"reps/";
    m_s_path_resources_logo=m_s_path_resources+"icons/open_auditorium_logo.png";

    m_s_path_resources_session=m_s_path_resources+"sessions/"+m_s_session_name;
    m_s_path_resources_session_prev="";
    m_s_path_resources_slides=m_s_path_resources_session+"/"+m_s_session_name+"/";
    m_s_path_resources_configfile=m_s_path_resources_session+"/"+m_s_session_name+".yaml";
    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";

    m_i_session_scene_start_before=1;
    m_i_session_scene_count_current=0;
    m_i_session_scene_count_max=1;
    m_i_session_length=50;
    m_i_session_break_length=2;
    m_i_session_slot_current=0;
    m_i_session_slots_max=11;
    m_b_session_slot_found=false;

    m_tim_session_time_start=ros::Time::now();
    m_f_session_time_left=0.0;

    m_f_session_pace_th=0.1;
    m_f_ses_pac_scene=0.0;
    m_f_ses_pac_time=0.0;
    m_f_session_pace=0.0;

    m_b_enable_session_scheduler=false;
    m_b_enable_session_status_label=false;
    m_b_session_ends_notified=false;

    // Init session status and workspace state label (pub to RViz)
    pub_s_oa_status=m_hdl_node->advertise<std_msgs::String>("/wai_world/world/oa_status",1);
    pub_s_session_status=m_hdl_node->advertise<std_msgs::String>("/wai_world/world/session_status",1);
    pub_emp_rviz_models_reload=m_hdl_node->advertise<std_msgs::Empty>("/wai_world/world/rviz_models_reload_request",1);

    // Separate status data for OA and participants in session
    std_msgs::String msg_s_oa_status;
    //msg_s_oa_status.data="Welcome to OPEN AUDITORIUM!\nAn open-source extended reality tool for knowledge transfer.";
    msg_s_oa_status.data="WELCOME TO W.A.I. WORLD! Intel is busy with preparing your workspace...";
    pub_s_oa_status.publish(msg_s_oa_status);
    std_msgs::String msg_s_session_status;
    msg_s_session_status.data="You are about to join the session...";
    pub_s_session_status.publish(msg_s_session_status);
    ros::spinOnce();

    m_tmr_session_scheduler=m_hdl_node->createTimer(ros::Duration(5.0),&WAIOASessionManager::cb_tmr_session_scheduler,this,false,false);
    tmr_session_status=m_hdl_node->createTimer(ros::Duration(0.2),&WAIOASessionManager::cb_tmr_session_status,this,false,false);
}

void WAIOASessionManager::UpdateModel(std::string s_path_nodename,
                                              std::string s_path_resources,
                                              std::string s_session_group,
                                              std::string s_session_topic,
                                              std::string s_session_name,
                                              int i_session_scene_start_before,
                                              int i_session_length,
                                              int i_session_break_length,
                                              bool b_enable_session_scheduler)
{
    m_s_session_group=s_session_group;
    m_s_session_topic=s_session_topic;
    m_s_session_name=s_session_name;
    m_s_session_expertise="expertise_default"; // This param is set when config is loaded from *.yaml!
    m_s_session_expertise_level="3"; // 5 Expertise levels (1-5)

    m_s_path_nodename=s_path_nodename;
    m_s_path_resources=s_path_resources;
    m_s_path_resources_descriptions=m_s_path_resources+"descriptions/";
    m_s_path_resources_multiplots=m_s_path_resources+"multiplots/";
    m_s_path_resources_reps=m_s_path_resources+"reps/";
    m_s_path_resources_logo=m_s_path_resources+"icons/open_auditorium_logo.png";

    m_s_path_resources_session=m_s_path_resources+"sessions/"+m_s_session_name;
    m_s_path_resources_session_prev="";
    m_s_path_resources_slides=m_s_path_resources_session+"/"+m_s_session_name+"/";
    m_s_path_resources_configfile=m_s_path_resources_session+"/"+m_s_session_name+".yaml";
    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";

    m_i_session_scene_start_before=i_session_scene_start_before;
    m_i_session_scene_count_current=0;
    m_i_session_scene_count_max=1; // needs uptodate m_s_path_resources_slides!
    m_i_session_length=i_session_length;
    m_i_session_break_length=i_session_break_length;

    m_b_enable_session_scheduler=b_enable_session_scheduler;
}

void WAIOASessionManager::UpdateView()
{

}

void WAIOASessionManager::ConnectRepsAndLoadSession(WAIOARepManager* wai_oa_rep_manager,
                                                            WAIOAAudience* wai_oa_audience,
                                                            WAIOAPresenter* wai_oa_presenter,
                                                            WAIOAProjector* wai_oa_projector,
                                                            WAIOACameraRviz* wai_oa_cam_rviz,
                                                            WAIReps* wai_oa_rep_wim)
{
    m_oa_rep_manager=wai_oa_rep_manager;
    m_oa_audience=wai_oa_audience;
    m_oa_presenter=wai_oa_presenter;
    m_oa_projector=wai_oa_projector;
    m_oa_cam_rviz=wai_oa_cam_rviz;
    m_wai_oa_rep_wim=wai_oa_rep_wim;
    m_b_session_ends_notified=false;

    if(m_b_enable_session_scheduler)
    {
        UpdateSessionFromSchedule();
        LoadSession();
        SchedulerStart();
    }
    else
    {
        LoadSession();
    }
}

void WAIOASessionManager::UpdateSessionFromSchedule()
{
    std::vector<std::string> s_session_parameters;
    std::stringstream sst_param_path_session;
    std::stringstream sst_param_path_schedule;

    // First part of param ns for session NAME
    sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/";

    // First part of sessions schedule (TIMES!)
    sst_param_path_schedule << sst_param_path_session.str() << "sessions_schedule/";

    // Parse weekday into parameter namespace
    std::time_t time_now;
    time(&time_now);
    const std::tm* time_weekday = std::localtime(&time_now);
    //Sunday == 0, Monday == 1, and so on ...
    if(time_weekday->tm_wday==0) sst_param_path_session<< "sessions_sunday/";
    else if(time_weekday->tm_wday==1) sst_param_path_session<< "sessions_monday/";
    else if(time_weekday->tm_wday==2) sst_param_path_session<< "sessions_tuesday/";
    else if(time_weekday->tm_wday==3) sst_param_path_session<< "sessions_wednesday/";
    else if(time_weekday->tm_wday==4) sst_param_path_session<< "sessions_thursday/";
    else if(time_weekday->tm_wday==5) sst_param_path_session<< "sessions_friday/";
    else if(time_weekday->tm_wday==6) sst_param_path_session<< "sessions_saturday/";
    else
    {
        ROS_DEBUG_STREAM("Session Manager: Parsed invalid WEEKDAY, loading default session!");
        sst_param_path_session.str("");
        sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
        m_hdl_node->getParam(sst_param_path_session.str(), s_session_parameters);
        //return default session parameters
        m_s_session_group=s_session_parameters[0];
        m_s_session_topic=s_session_parameters[1];
        //*s_session_expertise=s_session_parameters[2];
        //*s_session_expertise_level=s_session_parameters[3];
        m_s_session_name=s_session_parameters[2];
    }
    ROS_DEBUG_STREAM("Session Manager: PARAM path for SCHEDULER: "<<sst_param_path_schedule.str());
    ROS_DEBUG_STREAM("Session Manager: PARAM path for SESSION: "<< sst_param_path_session.str());


    // Parse all session times into parameter namespace
    std::vector<std::vector<int> > vec_i_session_slots;
    std::vector<struct tm> vec_tm_session_slot_start;
    std::vector<struct tm> vec_tm_session_slot_end;

    // Automatically load new session, including X min of break each session!
    // In between sessions, autoload default (e.g., break) session!
    m_f_session_duration=0.0;
    m_f_session_time_left=0.0;
    m_f_session_length=m_i_session_length*60.0; // Default setting, parameter in min to secs

    m_i_session_slot_current=0;
    m_b_session_slot_found=false;

    for(int i=1;i<=m_i_session_slots_max;i++)
    {
        std::vector<int> vec_i_session_slot;
        m_hdl_node->getParam(sst_param_path_schedule.str()+"session_"+std::to_string(i),vec_i_session_slot);
        vec_i_session_slots.push_back(vec_i_session_slot);

        struct tm tm_session_slot_start=*std::localtime(&time_now);
        tm_session_slot_start.tm_hour=vec_i_session_slot[0];
        tm_session_slot_start.tm_min=vec_i_session_slot[1];
        tm_session_slot_start.tm_sec=0;
        vec_tm_session_slot_start.push_back(tm_session_slot_start);
        struct tm tm_session_slot_end=*std::localtime(&time_now);
        tm_session_slot_end.tm_hour=vec_i_session_slot[2];
        tm_session_slot_end.tm_min=vec_i_session_slot[3];
        tm_session_slot_end.tm_sec=0;
        vec_tm_session_slot_end.push_back(tm_session_slot_end);

        if(std::difftime(time_now,std::mktime(&tm_session_slot_start))>m_i_session_break_length*60.0
            && std::difftime(std::mktime(&tm_session_slot_end),time_now)>m_i_session_break_length*60.0
                && m_b_session_slot_found==false)
        {
            m_b_session_slot_found=true;
            m_i_session_slot_current=i;
            //ROS_WARN_STREAM("Session Manager: Session slot" << m_i_session_slot_current << " found between " << std::endl << std::put_time(&tm_session_slot_start, "%c %Z") << " and " << std::endl << std::put_time(&tm_session_slot_end, "%c %Z") << ".");
            sst_param_path_session << "session_"+std::to_string(i)+"/";
            tm_session_slot_start.tm_sec+=m_i_session_break_length*60;
            m_f_session_duration=std::difftime(time_now,std::mktime(&tm_session_slot_start));
            tm_session_slot_end.tm_sec-=m_i_session_break_length*60;
            m_f_session_time_left=std::difftime(std::mktime(&tm_session_slot_end),time_now);
            m_f_session_length=std::difftime(std::mktime(&tm_session_slot_end),std::mktime(&tm_session_slot_start));
        }
    }
    if(m_b_session_slot_found==false)
    {
        m_i_session_slot_current=0;
        sst_param_path_session.str("");
        sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
        m_hdl_node->getParam(sst_param_path_session.str(), s_session_parameters);
        m_s_session_group=s_session_parameters[0];
        m_s_session_topic=s_session_parameters[1];
        //*s_session_expertise=s_session_parameters[2];
        //*s_session_expertise_level=s_session_parameters[3];
        m_s_session_name=s_session_parameters[2];
        ROS_DEBUG_STREAM("Session Manager: Parsed invalid TIME or BREAK! Loading default session: " << s_session_parameters[0] << ", " << s_session_parameters[1] << ", " << s_session_parameters[2]);
    }

    // NOW ACTUALLY PARSE SESSION NAME
    if(ros::param::has(sst_param_path_session.str()) &&
            m_hdl_node->getParam(sst_param_path_session.str(), s_session_parameters))
    {
        ROS_DEBUG_STREAM("Session Manager: Parsed SESSION NAME from PARAM: " << s_session_parameters[2] );

        if(s_session_parameters[2].compare("")==0)
        {
            ROS_DEBUG_STREAM("Session Manager: Empty SESSION NAME from PARAM, loading default session!");
            sst_param_path_session.str("");
            sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
            m_hdl_node->getParam(sst_param_path_session.str(), s_session_parameters);
        }
    }
    else
    {
        ROS_DEBUG_STREAM("Session Manager: Invalid SESSION NAME from PARAM, loading default session!");
        sst_param_path_session.str("");
        sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
        m_hdl_node->getParam(sst_param_path_session.str(), s_session_parameters);
    }

    // Update all session parameters
    // --> Group, Topic and Session Name are def. in auditorium-*.yaml file
    // --> Expertise and Expertise level are bound to session-*.yaml file!
    m_s_session_group=s_session_parameters[0];
    m_s_session_topic=s_session_parameters[1];
    //*s_session_expertise=s_session_parameters[2];
    //*s_session_expertise_level=s_session_parameters[3];
    m_s_session_name=s_session_parameters[2];

    m_s_path_resources_session=m_s_path_resources+"sessions/"+m_s_session_name;
    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";
}

void WAIOASessionManager::LoadWorkspacePresenter(std::string s_workspace_name)
{
    int i_retval=system( ("rosparam load "+
                          GetPathResourcesDescriptions()+
                          "workspace_presenter_"+s_workspace_name+".xacro /wai_world/workspace_presenter/robot_description").c_str() );
    std_msgs::Empty msg_emp_rviz_models_reload;
    pub_emp_rviz_models_reload.publish(msg_emp_rviz_models_reload);
}

void WAIOASessionManager::RespawnRepFromTemplate(std::string s_template_name)
{
    int i_retval=system( ("rosparam load "+GetPathResourcesDescriptions()+s_template_name+".xacro /wai_world/"+s_template_name+"/robot_description").c_str() );
    std_msgs::Empty msg_emp_rviz_models_reload;
    pub_emp_rviz_models_reload.publish(msg_emp_rviz_models_reload);
}

void WAIOASessionManager::UpdateSessionSlidesConfig()
{
    m_s_path_resources_configfile=m_s_path_resources_session+"/"+m_s_session_name+".yaml";
    if(GetFileExists(m_s_path_resources_configfile)==true)
    {
        // Load the *.yaml file using the external rosparam command
        // Load each session config. into a SEPARATE NAMESPACE
        // --> Be careful for same SESSION_NAMEs on multiple different SESSIONS:
        ROS_INFO_STREAM("Session Manager: LOADING session config (*.yaml)-file! " << m_s_path_resources_configfile);
    }
    else
    {
        m_s_path_resources_configfile=m_s_path_resources+"sessions/session_config_template.yaml";
        ROS_WARN_STREAM("Session Manager: Error LOADING session config (*.yaml)-file! Loading template! " << m_s_path_resources_configfile);
    }
    std::string s_sys_cmd_cfg="rosparam load "+m_s_path_resources_configfile+" "+m_s_path_nodename+"/"+m_s_session_name+"/";
    int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
    if(i_sys_retval_cfg!=0)
    {
        m_s_path_resources_configfile=m_s_path_resources+"sessions/session_config_template.yaml";
        ROS_WARN_STREAM("Session Manager: Error PARSING session config (*.yaml)-file! Loading template! "<< m_s_path_resources_session);
    }
}

void WAIOASessionManager::LoadSession()
{
    m_f_session_duration=0.0;
    m_f_session_time_left=0.0;
    m_f_session_length=m_i_session_length*60.0;

    // Once resource path updated get path to SESSION CONFIG FILE and SLIDES
    m_s_path_resources_slides=m_s_path_resources_session+"/"+m_s_session_name+"/";

    // Update the sessions slide (*.yaml)-config file
    UpdateSessionSlidesConfig();

    // Load expertise parameters
    std::stringstream sst_param_path_session_expertise;
    std::stringstream sst_param_path_session_expertise_level;
    sst_param_path_session_expertise << m_s_path_nodename+"/"+m_s_session_name << "/setup_session_expertise/session_expertise";
    sst_param_path_session_expertise_level << m_s_path_nodename+"/"+m_s_session_name << "/setup_session_expertise/session_expertise_level";
    if(ros::param::has(sst_param_path_session_expertise.str()) &&
            ros::param::has(sst_param_path_session_expertise_level.str()))
    {
        m_hdl_node->getParam(sst_param_path_session_expertise.str(), m_s_session_expertise);
        m_hdl_node->getParam(sst_param_path_session_expertise_level.str(), m_s_session_expertise_level);
    }
    else
    {
        ROS_DEBUG_STREAM("Session Manager: No expertise setup found, setting default expertise!");
        m_s_session_expertise="default";
        m_s_session_expertise_level="3";
    }


    // WORKSPACE PRESENTER - Load SESSION-specific SPACE
    std::stringstream sst_setup_session_defaults;
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/workspace_presenter/default";
    if(ros::param::has(sst_setup_session_defaults.str()))
    {
        m_hdl_node->getParam(sst_setup_session_defaults.str(), m_s_session_workspace_presenter);
        LoadWorkspacePresenter(m_s_session_workspace_presenter);
    }


    // PRESENTER - Load SESSION-specific POSE
    std::vector<double> vec_d_setup_presenter_pos;
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/presenter/default";
    if(ros::param::has(sst_setup_session_defaults.str()))
    {
        m_hdl_node->getParam(sst_setup_session_defaults.str(), vec_d_setup_presenter_pos);
        m_oa_presenter->SetPresenterPose(
                    vec_d_setup_presenter_pos[0],
                    vec_d_setup_presenter_pos[1],
                    vec_d_setup_presenter_pos[2],
                    vec_d_setup_presenter_pos[3]);
    }


    // AUDIENCE - Load aliases of current group
    std::string s_sys_cmd= "rosparam load " + m_s_path_resources_aliases_grp_and_sub + " " + m_s_path_nodename + "/" + m_s_session_group;
    int i_sys_retval = std::system(s_sys_cmd.c_str());
    if(i_sys_retval!=0)
    {
        ROS_DEBUG_STREAM("Session Manager: ERROR LOADING aliases config (*.yaml)-file!" << m_s_path_resources_aliases_grp_and_sub << " !");
    }
    std::stringstream sst_setup_session_aliases;
    std::string s_setup_session_aliases;
    for(int i=0;i<m_oa_audience->GetAudienceCountMax();i++)
    {
        sst_setup_session_aliases.str("");
        sst_setup_session_aliases << m_s_path_nodename + "/" + m_s_session_group << "/id_"<< i<< "/alias";
        if(ros::param::has(sst_setup_session_aliases.str()))
        {
            m_hdl_node->getParam(sst_setup_session_aliases.str(),s_setup_session_aliases);
            m_oa_audience->SetAliasID(i,s_setup_session_aliases,m_s_session_group); // Done
        }
    }



    // PROJECTOR - Load SESSION-specific PROJECTION and POSE
    std::vector<double> vec_d_setup_projection_pos;
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/projection/default";
    if(ros::param::has(sst_setup_session_defaults.str()))
    {
        m_hdl_node->getParam(sst_setup_session_defaults.str(), vec_d_setup_projection_pos);
        m_oa_projector->SetProjectionPose(
                    vec_d_setup_projection_pos[0],
                    vec_d_setup_projection_pos[1],
                    vec_d_setup_projection_pos[2],
                    vec_d_setup_projection_pos[3]);
        m_oa_projector->UpdateModel(cv::imread("empty", cv::IMREAD_COLOR),GetPathResourcesLogo());
        m_oa_projector->UpdateView();
    }

    // CAMERA RVIZ - Load SESSION-specific VIEWS
    m_vec_s_setup_camera_rviz_predefined.clear();
    m_vec_camera_rviz_views_predefined.clear();
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera";
    XmlRpc::XmlRpcValue lst_session_defaults;
    m_hdl_node->getParam(sst_setup_session_defaults.str(), lst_session_defaults);
    for(XmlRpc::XmlRpcValue::ValueStruct:: const_iterator it = lst_session_defaults.begin();
        it != lst_session_defaults.end(); ++it)
    {
        m_vec_s_setup_camera_rviz_predefined.push_back(it->first);
    }
    // Remove UNUSED camera default views
    m_vec_s_setup_camera_rviz_predefined.erase(std::remove(m_vec_s_setup_camera_rviz_predefined.begin(), m_vec_s_setup_camera_rviz_predefined.end(), "startup"), m_vec_s_setup_camera_rviz_predefined.end());
    m_vec_s_setup_camera_rviz_predefined.erase(std::remove(m_vec_s_setup_camera_rviz_predefined.begin(), m_vec_s_setup_camera_rviz_predefined.end(), "break"), m_vec_s_setup_camera_rviz_predefined.end());
    // Store all RViz camera default views in double vector
    for(int i=0;i<int(m_vec_s_setup_camera_rviz_predefined.size());i++)
    {
        std::string s_setup_camera_rviz=m_vec_s_setup_camera_rviz_predefined[i];
        std::vector<double> vec_camera_rviz_predefined;
        std::stringstream sst_setup_camera_rviz_defaults;
        sst_setup_camera_rviz_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/" << s_setup_camera_rviz;
        m_hdl_node->getParam(sst_setup_camera_rviz_defaults.str(), vec_camera_rviz_predefined);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
    }
    m_oa_cam_rviz->UpdateCameraRvizViews(m_vec_camera_rviz_views_predefined);
    // CAMERA RVIZ - Load SESSION-specific VIEW at index 0, store first camera cycle in OLD view
    m_oa_cam_rviz->UpdateModel(m_vec_camera_rviz_views_predefined[0],false,false,-1,3.0,"world",0,true);
    m_oa_cam_rviz->UpdateView();

    // WIM - Load SESSION-specific workspace model
    // ((WAIRepWIM*)m_wai_oa_rep_wim)->UpdateModel(GetSessionWorkspacePresenter(),GetPathResourcesReps());
    ((WAIRepWIM*)m_wai_oa_rep_wim)->UpdateModel("wim",GetPathResourcesReps());
        // --> Larger Models might not fit into workspace even if scaled down heavily, thus load individual WIM model!
    m_wai_oa_rep_wim->UpdateView();

    // OTHER REPS - Set SESSION-specific STATE
    std::vector<std::string> vec_s_reps_state;
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/reps_state";
    XmlRpc::XmlRpcValue lst_session_reps_state_default;
    m_hdl_node->getParam(sst_setup_session_defaults.str(), lst_session_reps_state_default);
    for(XmlRpc::XmlRpcValue::ValueStruct:: const_iterator it = lst_session_reps_state_default.begin();
        it != lst_session_reps_state_default.end(); ++it)
    {
        vec_s_reps_state.push_back(it->first);
    }
    for(int i=0;i<int(vec_s_reps_state.size());i++)
    {
        std::vector<double> vec_d_reps_state;
        std::stringstream sst_setup_reps_state;
        sst_setup_reps_state << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/reps_state/" << vec_s_reps_state[i];
        if(ros::param::has(sst_setup_reps_state.str()))
        {
            m_hdl_node->getParam(sst_setup_reps_state.str(), vec_d_reps_state);
            m_oa_rep_manager->SetRepStateViaService(vec_s_reps_state[i],
                                                    tf::Vector3(vec_d_reps_state[0],vec_d_reps_state[1],vec_d_reps_state[2]),
                                                    tf::Vector3(0.0,0.0,vec_d_reps_state[3]));
        }
        else
        {
            ROS_WARN_STREAM("Session Manager: Wrong config for setting REP state at session startup!");
        }
    }

    // Get number of scenes/slides from current session directory
    m_i_session_scene_count_max=GetSessionSceneCountMax();
    if(m_i_session_scene_start_before<=m_i_session_scene_count_max)
    {
        m_i_session_scene_count_current=m_i_session_scene_start_before-1; // Start with slide index of 0, trigger to actually begin
    }
    else
    {
        m_i_session_scene_start_before=1;
        m_i_session_scene_count_current=m_i_session_scene_start_before-1; // Start with slide index of 0, trigger to actually begin
        ROS_WARN("Session Manager: Scene count start before is invalid. Starting before scene 1.");
    }

    // Update "old" session name, so that scheduler can skip uptodate configs
    m_s_path_resources_session_prev=m_s_path_resources_session;

    // Update session start time
    m_tim_session_time_start=ros::Time::now();
}

int WAIOASessionManager::GetSessionSlotCurrent()
{
    return m_i_session_slot_current;
}

int WAIOASessionManager::GetSessionSceneCountCurrent()
{
    return m_i_session_scene_count_current;
}

int WAIOASessionManager::GetSessionSceneCountMax()
{
    DIR* dp;
    int i=0;
    struct dirent* ep;
    dp=opendir(m_s_path_resources_slides.c_str());
    if(dp!=NULL)
    {
        while((ep=readdir(dp)))
        {
            i++;
        }
        (void)closedir(dp);
        ROS_DEBUG_STREAM("Session Manager: Found " << i-2 << "files in slides directory.");
        return i-2; // Exclude . and ..
    }
    else
    {
        ROS_DEBUG_STREAM("Session Manager: Couldn't get number of slides in directory! Exiting...");
        exit(1);
    }
}

std::string WAIOASessionManager::GetSessionPace(bool b_scheduler_enabled)
{
    m_f_ses_pac_scene=float(GetSessionSceneCountCurrent())/float(GetSessionSceneCountMax());

    // Overwrite duration of sesssion here, if scheduler is not enabled,
    // otherwise timer of scheduler updates this member!
    if(b_scheduler_enabled==false) m_f_session_duration=(ros::Time::now()-m_tim_session_time_start).toSec();

    m_f_ses_pac_time=m_f_session_duration/m_f_session_length;
    m_f_session_pace=m_f_ses_pac_scene-m_f_ses_pac_time;
    /* ROS_WARN("DUR/LEN: %3.3f, %3.3f | Pac-Scene: %3.3f, Pac-Time: %3.3f, PACE: %3.3f ",
             m_f_session_duration,
             m_f_session_length,
             m_f_ses_pac_scene,
             m_f_ses_pac_time,
             m_f_session_pace); */
    if(m_f_session_pace<-3.0*m_f_session_pace_th) // 30% too slow
    {
        return "(---)";
    }
    else if(m_f_session_pace>=-3.0*m_f_session_pace_th
            && m_f_session_pace<-2.0*m_f_session_pace_th) // 20% too slow
    {
        return "(--)";
    }
    else if(m_f_session_pace>=-2.0*m_f_session_pace_th
            && m_f_session_pace<-1.0*m_f_session_pace_th) // 10% too slow
    {
        return "(-)";
    }
    else if(m_f_session_pace>=-m_f_session_pace_th && m_f_session_pace<=m_f_session_pace_th)
    {
        return "(~)";
    }
    else if(m_f_session_pace>1.0*m_f_session_pace_th
            && m_f_session_pace<=2.0*m_f_session_pace_th) // 10% too fast
    {
        return "(+)";
    }
    else if(m_f_session_pace>2.0*m_f_session_pace_th
            && m_f_session_pace<=3.0*m_f_session_pace_th) // 20% too fast
    {
        return "(++)";
    }
    else if(m_f_session_pace>3.0*m_f_session_pace_th) // 30% too fast
    {
        return "(+++)";
    }
    else
    {
        return "(.)";
    }
}

void WAIOASessionManager::SetSessionSceneCountCurrent(bool b_increment_decrement,int i_scene_select,bool b_scene_update_cfg)
{
    // Update/reload current session slide config from *.yaml file
    if(b_scene_update_cfg==true)
    {
        UpdateSessionSlidesConfig();
    }
    else
    {
        // Do nothing...
    }

    // Either directly select sesssion via scene number or simply increment/decrement
    if(i_scene_select==-1)
    {
        if(b_increment_decrement) m_i_session_scene_count_current++;
        else m_i_session_scene_count_current--;
    }
    else
    {
        m_i_session_scene_count_current=i_scene_select;
    }
}

std::vector<std::string> WAIOASessionManager::GetCameraRVizViewLabels()
{
    return m_vec_s_setup_camera_rviz_predefined;
}

std::vector< std::vector<double>> WAIOASessionManager::GetCameraRVizViews()
{
    return m_vec_camera_rviz_views_predefined;
}

std::string WAIOASessionManager::GetPathResourcesRoot()
{
    return m_s_path_resources;
}
std::string WAIOASessionManager::GetPathResourcesDescriptions()
{
    return m_s_path_resources_descriptions;
}
std::string WAIOASessionManager::GetPathResourcesMultiplots()
{
    return m_s_path_resources_multiplots;
}
std::string WAIOASessionManager::GetPathResourcesReps()
{
    return m_s_path_resources_reps;
}
std::string WAIOASessionManager::GetPathResourcesLogo()
{
    return m_s_path_resources_logo;
}
std::string WAIOASessionManager::GetPathResourcesSlides()
{
    return m_s_path_resources_slides;
}
std::string WAIOASessionManager::GetPathResourcesEvalsGroupAndSubject()
{
    return m_s_path_resources_evals_grp_and_sub;
}
std::string WAIOASessionManager::GetPathResourcesAliasesGroupAndSubject()
{
    return m_s_path_resources_aliases_grp_and_sub;
}
std::string WAIOASessionManager::GetPathResourcesAliasesGroupAndIDPreview()
{
    return m_s_path_resources_aliases_grp_and_id_preview;
}

std::string WAIOASessionManager::GetSessionGroup()
{
    return m_s_session_group;
}
std::string WAIOASessionManager::GetSessionTopic()
{
    return m_s_session_topic;
}
std::string WAIOASessionManager::GetSessionName()
{
    return m_s_session_name;
}
std::string WAIOASessionManager::GetSessionExpertise()
{
    return m_s_session_expertise;
}
std::string WAIOASessionManager::GetSessionExpertiseLevel()
{
    return m_s_session_expertise_level;
}
std::string WAIOASessionManager::GetSessionWorkspacePresenter()
{
    return m_s_session_workspace_presenter;
}
std::string WAIOASessionManager::GetSessionStatusLabel()
{
    m_sst_session_status.str("");
    if(m_b_enable_session_scheduler==true)
    {
        // Update status label for sessions WITH SCHEDULER enabled!
        m_sst_session_status
            << "SCD: (ON)"<< ", "
            << GetSessionName() << " ("
            << GetSessionExpertise() << ", "
            << GetSessionExpertiseLevel() << "), "
            << "SLT: " << GetSessionSlotCurrent() << ", "
            << "GRP: " << GetSessionGroup() << ", "
            << "TPC: " << GetSessionTopic() << ", "
            << "SCN: " << GetSessionSceneCountCurrent()<< "/" << m_i_session_scene_count_max << ", "
            << "REM: " << int(m_f_session_time_left)/60 <<"/"<< int(m_f_session_length)/60 << "min "
            << GetSessionPace(m_b_enable_session_scheduler);

        // Break already included in m_f_session_time_left
        // end - 3min - 2min = time for notification!
        if(m_f_session_time_left<=120.0
                && m_f_session_time_left>0.0
                && m_b_session_ends_notified==false)
        {
            Say("IntelSessionManagerSessionEnds");
            m_b_session_ends_notified=true;
        }
    }
    else
    {
        // Update status label for session without SCHEDULER!
        m_sst_session_status
            << "SCD: (OFF)"<< ", "
            << GetSessionName() << " ("
            << GetSessionExpertise() << ", "
            << GetSessionExpertiseLevel() << "), "
            << "GRP: " << GetSessionGroup() << ", "
            << "TPC: " << GetSessionTopic() << ", "
            << "SCN: " << GetSessionSceneCountCurrent()<< "/" << m_i_session_scene_count_max << ", "
            << "LEN: " << int((ros::Time::now()-m_tim_session_time_start).toSec())/60 << "/" << m_i_session_length <<"min "
            << GetSessionPace(m_b_enable_session_scheduler);

        if(m_f_session_time_left<=120.0
            && m_f_session_time_left>0.0
            && m_b_session_ends_notified==false)
        {
            Say("IntelSessionManagerSessionEnds");
            m_b_session_ends_notified=true;
        }
    }

    return m_sst_session_status.str();
}

void WAIOASessionManager::SchedulerStart()
{
    m_tmr_session_scheduler.start();
}

void WAIOASessionManager::SchedulerStop()
{
    m_tmr_session_scheduler.stop();
}

void WAIOASessionManager::EnableUpdateSessionStatusLabel()
{
    tmr_session_status.start();
}

void WAIOASessionManager::PreparePlenumMode()
{
    for(int i=0;i<m_oa_audience->GetAudienceCountMax();i++)
    {
        tf::Vector3 vc3_audience=m_oa_audience->GetAudiencePosFromID(i);
        m_oa_rep_manager->SetRepStateViaService("workspace_audience_"+std::to_string(i),
                                                vc3_audience,
                                                tf::Vector3(0.0,0.0,3.141592654));
        SpinAndWaitForSeconds(0.25);
        ROS_WARN("SPINANDWAITFORSECS!!!");
    }
}

void WAIOASessionManager::PrepareCooperativeMode(std::string s_rep_name,
                                                        tf::Vector3 vc3_rep_position,
                                                        float f_rep_yaw,
                                                        float f_dist_audience,
                                                        int i_mode_audience)
{
    m_oa_rep_manager->SetRepStateViaService(s_rep_name,
                                            vc3_rep_position,
                                            tf::Vector3(0.0,0.0,f_rep_yaw));

    if(i_mode_audience==0) // Circular arrangement
    {
        float f_delta_rot=2.0*M_PI/m_oa_audience->GetAudienceCountMax();
        for(int i=0;i<m_oa_audience->GetAudienceCountMax();i++)
        {
            tf::Vector3 vc3_audience;
            float f_audience_yaw=f_delta_rot*i;
            vc3_audience.setX(vc3_rep_position.getX()+f_dist_audience*cos(f_delta_rot*i));
            vc3_audience.setY(vc3_rep_position.getY()+f_dist_audience*sin(f_delta_rot*i));
            vc3_audience.setZ(vc3_rep_position.getZ());

            m_oa_rep_manager->SetRepStateViaService("workspace_audience_"+std::to_string(i),
                                                    vc3_audience,
                                                    tf::Vector3(0.0,0.0,f_audience_yaw));
            SpinAndWaitForSeconds(0.25);
            ROS_WARN("SPINANDWAITFORSECS!!!");
        }
    }
    else if(i_mode_audience==1)
    {

    }
    else
    {
        // Do nothing...
    }
}

void WAIOASessionManager::SpinAndWaitForSeconds(float f_seconds)
{
    ros::Rate r_sleep(m_f_node_sample_frequency);
    ros::Time tim_start=ros::Time::now();
    while((ros::Time::now()-tim_start).toSec()<f_seconds)
    {
        ros::spinOnce();
        r_sleep.sleep();
    }
}

std::string WAIOASessionManager::GetOAStatusLabel()
{
    return m_s_oa_status;
}
void WAIOASessionManager::SetOAStatusLabel(std::string s_oa_status)
{
    m_s_oa_status="PRESENTER - "+s_oa_status;
}

void WAIOASessionManager::cb_tmr_session_status(const ros::TimerEvent& event)
{
    std_msgs::String msg_s_oa_status;
    msg_s_oa_status.data=GetOAStatusLabel();
    pub_s_oa_status.publish(msg_s_oa_status);

    std_msgs::String msg_s_session_status;
    msg_s_session_status.data=GetSessionStatusLabel();//GetOAStatusLabel()+"\n"+GetSessionStatusLabel();
    pub_s_session_status.publish(msg_s_session_status);
}

void WAIOASessionManager::Say(std::string s_say)
{
    sound_play::Sound snd_play=m_hdl_snd_client->waveSoundFromPkg("wai_oa_gazebo","resources/sounds/"+s_say+".wav");
    snd_play.play();
}
