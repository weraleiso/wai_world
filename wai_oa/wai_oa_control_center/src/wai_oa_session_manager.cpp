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

    // Update only if actually new session was loaded
    if(m_s_session_name.compare(m_s_session_name_prev)==0)
    {
        return;
    }
    else
    {
        LoadSession();
        InteractionVoiceFromFile("IntelSessionManagerSessionLoaded");
    }
}

void WAIOASessionManager::Initialize(ros::NodeHandle* hdl_node,float f_node_sample_frequency)
{
    m_hdl_node=hdl_node;

    m_f_node_sample_frequency=f_node_sample_frequency;

    m_s_session_group="group_default";
    m_s_session_topic="topic_default";
    m_s_session_filename="/home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/sessions/break.odp";
    m_s_session_name=GetSessionNameFromFilename(m_s_session_filename);
    m_s_session_name_prev="";
    m_s_session_expertise="expertise_default";
    m_s_session_expertise_level="level_default";
    m_s_session_workspace_presenter="default";

    m_s_path_nodename="nodename";
    m_s_path_resources="resources";
    m_s_path_resources_descriptions=m_s_path_resources+"descriptions/";
    m_s_path_resources_multiplots=m_s_path_resources+"multiplots/";
    m_s_path_resources_reps=m_s_path_resources+"reps/";
    m_s_path_resources_logo=m_s_path_resources+"icons/open_auditorium_logo.png";

    m_s_path_resources_sessions=m_s_path_resources+"sessions/";
    m_s_path_resources_scene_preview=m_s_path_resources_sessions+"preview/";
    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";

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
                                              std::string s_session_filename,
                                              int i_session_length,
                                              int i_session_break_length,
                                              bool b_enable_session_scheduler)
{
    m_s_session_filename=s_session_filename;
    m_s_session_name=GetSessionNameFromFilename(m_s_session_filename);
    m_s_session_name_prev="";

    m_s_session_group=s_session_group;
    m_s_session_topic=s_session_topic;

    m_s_session_expertise="expertise_default"; // This param is set when config is loaded from *.yaml!
    m_s_session_expertise_level="3"; // 5 Expertise levels (1-5)

    m_s_path_nodename=s_path_nodename;
    m_s_path_resources=s_path_resources;
    m_s_path_resources_descriptions=m_s_path_resources+"descriptions/";
    m_s_path_resources_multiplots=m_s_path_resources+"multiplots/";
    m_s_path_resources_reps=m_s_path_resources+"reps/";
    m_s_path_resources_logo=m_s_path_resources+"icons/open_auditorium_logo.png";

    m_s_path_resources_sessions=m_s_path_resources+"sessions/";
    m_s_path_resources_scene_preview=m_s_path_resources_sessions+"preview/";
    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";

    m_i_session_scene_count_current=0;
    m_i_session_scene_count_max=1;
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
                                                            WAISymbol* wai_oa_rep_wim)
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
        ROS_DEBUG_STREAM("SESSION MANAGER: Parsed invalid WEEKDAY, loading default session!");
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
    ROS_DEBUG_STREAM("SESSION MANAGER: PARAM path for SCHEDULER: "<<sst_param_path_schedule.str());
    ROS_DEBUG_STREAM("SESSION MANAGER: PARAM path for SESSION: "<< sst_param_path_session.str());


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
            ROS_DEBUG_STREAM("SESSION MANAGER: Scheduler found valid time SLOT #" << m_i_session_slot_current << " between " << std::endl << std::put_time(&tm_session_slot_start, "%c %Z") << " and " << std::endl << std::put_time(&tm_session_slot_end, "%c %Z") << ".");
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
        ROS_DEBUG_STREAM("SESSION MANAGER: Parsed invalid TIME or BREAK! Loading default session: " << s_session_parameters[0] << ", " << s_session_parameters[1] << ", " << s_session_parameters[2]);
    }

    // NOW ACTUALLY PARSE SESSION NAME
    if(ros::param::has(sst_param_path_session.str()) &&
            m_hdl_node->getParam(sst_param_path_session.str(),s_session_parameters))
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: Parsed SESSION NAME from PARAM: " << s_session_parameters[2] );

        if(s_session_parameters[2].compare("")==0)
        {
            ROS_DEBUG_STREAM("SESSION MANAGER: Empty SESSION NAME from PARAM, loading default session!");
            sst_param_path_session.str("");
            sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
            m_hdl_node->getParam(sst_param_path_session.str(),s_session_parameters);
        }
    }
    else
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: Invalid SESSION NAME from PARAM, loading default session!");
        sst_param_path_session.str("");
        sst_param_path_session << m_s_path_nodename << "/setup_sessions_scheduler/sessions_default/";
        m_hdl_node->getParam(sst_param_path_session.str(),s_session_parameters);
    }

    // Update all session parameters
    // --> Group, Topic and Session Name are def. in auditorium-*.yaml file
    // --> Expertise and Expertise level are bound to session-*.yaml file!
    m_s_session_group=s_session_parameters[0];
    m_s_session_topic=s_session_parameters[1];
    //*s_session_expertise=s_session_parameters[2];
    //*s_session_expertise_level=s_session_parameters[3];
    m_s_session_name=s_session_parameters[2];

    m_s_path_resources_evals_grp_and_sub=m_s_path_resources+"groups/evals_"+m_s_session_group+"_"+m_s_session_topic+".log";
    m_s_path_resources_aliases_grp_and_sub=m_s_path_resources+"groups/aliases_"+m_s_session_group+"_"+m_s_session_topic+".yaml";
    m_s_path_resources_aliases_grp_and_id_preview=m_s_path_resources+"groups/"+m_s_session_group+"/";

    // Reset session filename to make UpdateSessionSlidesConfigFromExport()
    // use files from default session path in resources folder:
    m_s_session_filename="";
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

void WAIOASessionManager::UpdateSessionSlidesConfigFromExport()
{
    // Check for valid *.ODP or *.PPTX presentation file (path includes file extension)
    // or HTML-based session (folder path only!)
    m_b_enable_html_session=false;
    if(m_s_session_filename.find(".pptx")==std::string::npos
        && m_s_session_filename.find(".odp")==std::string::npos)
    {
        QDir qdr_session_path_html(QString::fromStdString(m_s_session_filename));
        if(qdr_session_path_html.exists())
        {
            m_b_enable_html_session=true;
            UpdateSessionFilename(m_s_session_filename);
            ROS_INFO_STREAM("SESSION MANAGER: Loading HTML session from '" << m_s_session_filename << "'!");
        }
        else
        {
            m_b_enable_html_session=false;
            UpdateSessionFilename(m_s_path_resources_sessions+"break.odp");
            ROS_WARN_STREAM("SESSION MANAGER: Error loading HTML session from '" << m_s_session_filename << "'! Loading BREAK session!");
        }
    }
    else
    {
        m_b_enable_html_session=false;
        if(GetFileExists(m_s_session_filename))
        {
            UpdateSessionFilename(m_s_session_filename);
            ROS_INFO_STREAM("SESSION MANAGER: Loading PRESENTATION FILE session from '" << m_s_session_filename << "'!");
        }
        else
        {
            UpdateSessionFilename(m_s_path_resources_sessions+"break.odp");
            ROS_WARN_STREAM("SESSION MANAGER: Error loading PRESENTATION FILE session from '" << m_s_session_filename << "'! Loading BREAK session!");
        }
    }

    // Export SESSION config and SCENE config directly from *.PPTX or *.ODP presentation file
    if(m_b_enable_html_session==false)
    {
        ROS_INFO("SESSION MANAGER: Exporting scene IMAGE from *.PPTX/*.ODP...");

        std::string s_sys_cmd_cfg="rosparam delete "+m_s_path_nodename+"/"+m_s_session_name+"/setup_scene_"+std::to_string(m_i_session_scene_count_current);
        int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

        s_sys_cmd_cfg="python3 "+m_s_path_resources_sessions+"sessionexport.py -i "+m_s_session_filename+" -x "+std::to_string(m_i_session_scene_count_current-1)+" -o "+m_s_path_resources_sessions;
        i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

        ROS_INFO("SESSION MANAGER: Exporting scene IMAGE from *.PPTX/*.ODP DONE...");
    }
    else if(m_b_enable_html_session==true)
    {
        ROS_WARN("SESSION MANAGER: Exporting scene IMAGE from HTML...");

        // Bash example: cutycapt --url=file:///home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/sessions/template/layout_one_column.html --out=/home/ias/test.png
        std::string s_sys_cmd_cfg="cutycapt --url=file://"+m_s_session_filename+"/"+std::to_string(m_i_session_scene_count_current)+".html --out="+m_s_path_resources_sessions+"scene.png";
        int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

        // Bash example: xmlstarlet sel -t -v "//notes" 1.html
        s_sys_cmd_cfg="xmlstarlet sel -t -v \"//notes\" "+m_s_session_filename+"/"+std::to_string(m_i_session_scene_count_current)+".html > "+m_s_path_resources_sessions+"session.yaml";
        i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

        ROS_WARN("SESSION MANAGER: Exporting scene IMAGE from HTML... DONE!");
    }
    else
    {
        // Do nothing...
    }

    if(m_i_session_scene_count_current==0) // If slide/index 0, then load global session config in the SESSION_NAME namespace!
    {
        ROS_WARN("SESSION MANAGER: Loading global SESSION CONFIG from scene %d...",m_i_session_scene_count_current);
        std::string s_sys_cmd_cfg="rosparam delete "+m_s_path_nodename+"/"+m_s_session_name;
        int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
        s_sys_cmd_cfg="rosparam load "+m_s_path_resources_sessions+"session.yaml "+m_s_path_nodename+"/"+m_s_session_name;
        i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
        ROS_WARN("SESSION MANAGER: Loading global SESSION CONFIG from scene %d... DONE!",m_i_session_scene_count_current);
    }
    else // otherwise, load the SCENE-SPECIFIC config, as usual, in a seperate SESSION_NAME/SETUP_SCENE_CURRENT namespace!
    {
        ROS_WARN("SESSION MANAGER: Loading CONFIG for scene %d...",m_i_session_scene_count_current);
        std::string s_sys_cmd_cfg="rosparam delete "+m_s_path_nodename+"/"+m_s_session_name+"/setup_scene_current";
        int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
        s_sys_cmd_cfg="rosparam load "+m_s_path_resources_sessions+"session.yaml "+m_s_path_nodename+"/"+m_s_session_name+"/setup_scene_current";
        i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
        ROS_WARN("SESSION MANAGER: Loading CONFIG for scene %d... DONE!",m_i_session_scene_count_current);
    }
}

void WAIOASessionManager::LoadSession()
{
    m_f_session_duration=0.0;
    m_f_session_time_left=0.0;
    m_f_session_length=m_i_session_length*60.0;
    m_i_session_scene_count_current=0;

    // Indicate SESSION is loading in status label
    /*
    SetOAStatusLabel("<b>Loading SESSION...</b>");
    UpdateSessionStatus();
    ros::spinOnce();
    ros::Rate(m_f_node_sample_frequency).sleep();
    ros::spinOnce();
    ros::Rate(m_f_node_sample_frequency).sleep();
    */

    // Update the sessions slide (*.yaml)-config file
    UpdateSessionSlidesConfigFromExport();

    // EXPERTISE - Load Parameters
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
        ROS_DEBUG_STREAM("SESSION MANAGER: No expertise setup found, setting default expertise!");
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
    else
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: No workspace setup found, setting default workspace!");
        LoadWorkspacePresenter("default");
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
    else
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: No presenter pose setup found, setting default pose!");
        m_oa_presenter->SetPresenterPose(0.0,0.0,1.0,0.0);
    }

    // AUDIENCE - Load aliases of current group
    std::string s_sys_cmd= "rosparam load " + m_s_path_resources_aliases_grp_and_sub + " " + m_s_path_nodename + "/" + m_s_session_group;
    int i_sys_retval = std::system(s_sys_cmd.c_str());
    if(i_sys_retval==0)
    {
        std::stringstream sst_setup_session_aliases;
        std::string s_setup_session_aliases;
        for(int i=0;i<m_oa_audience->GetAudienceCountMax();i++)
        {
            sst_setup_session_aliases.str("");
            sst_setup_session_aliases << m_s_path_nodename + "/" + m_s_session_group << "/id_"<< i<< "/alias";
            if(ros::param::has(sst_setup_session_aliases.str()))
            {
                m_hdl_node->getParam(sst_setup_session_aliases.str(),s_setup_session_aliases);
                m_oa_audience->SetAliasID(i,s_setup_session_aliases,m_s_session_group);
            }
        }
    }
    else
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: ERROR LOADING aliases config (*.yaml)-file!" << m_s_path_resources_aliases_grp_and_sub << " !");
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
        m_oa_projector->UpdateModel(cv::imread("empty",cv::IMREAD_COLOR),GetPathResourcesLogo());
        m_oa_projector->UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: No projection pose setup found, setting default pose!");
        m_oa_projector->SetProjectionPose(2.49,3.0,1.0,0.0);
        m_oa_projector->UpdateModel(cv::imread("empty",cv::IMREAD_COLOR),GetPathResourcesLogo());
        m_oa_projector->UpdateView();
    }

    // CAMERA RVIZ - Load SESSION-specific VIEWS
    m_vec_s_setup_camera_rviz_predefined.clear();
    m_vec_camera_rviz_views_predefined.clear();
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera";
    XmlRpc::XmlRpcValue lst_session_defaults;
    m_hdl_node->getParam(sst_setup_session_defaults.str(), lst_session_defaults);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=lst_session_defaults.begin(); it!=lst_session_defaults.end();++it)
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
    if(m_vec_s_setup_camera_rviz_predefined.size()==0)
    {
        ROS_DEBUG_STREAM("SESSION MANAGER: No camera default views found, adding set of default views!");
        //m_vec_s_setup_camera_rviz_predefined.push_back("break");
        m_vec_s_setup_camera_rviz_predefined.push_back("default");
        m_vec_s_setup_camera_rviz_predefined.push_back("expand_projection");
        m_vec_s_setup_camera_rviz_predefined.push_back("fill");
        m_vec_s_setup_camera_rviz_predefined.push_back("graph_3d");
        m_vec_s_setup_camera_rviz_predefined.push_back("graph_eval");
        m_vec_s_setup_camera_rviz_predefined.push_back("graph_stats");
        m_vec_s_setup_camera_rviz_predefined.push_back("overview");
        m_vec_s_setup_camera_rviz_predefined.push_back("presenter");
        m_vec_s_setup_camera_rviz_predefined.push_back("presenter_fpv");
        m_vec_s_setup_camera_rviz_predefined.push_back("startup");
        m_vec_s_setup_camera_rviz_predefined.push_back("wim");

        std::vector<double> vec_camera_rviz_predefined;
        std::stringstream sst_setup_camera_defaults;
        // BREAK
        /*
        vec_camera_rviz_predefined.push_back(-4.7026); vec_camera_rviz_predefined.push_back(-12.244); vec_camera_rviz_predefined.push_back(9.9361);
        vec_camera_rviz_predefined.push_back(1.352); vec_camera_rviz_predefined.push_back(-1.1921); vec_camera_rviz_predefined.push_back(-1.8079);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        */
        // DEFAULT
        vec_camera_rviz_predefined.push_back(-1.8312); vec_camera_rviz_predefined.push_back(1.4215); vec_camera_rviz_predefined.push_back(1.8032);
        vec_camera_rviz_predefined.push_back(0.96589); vec_camera_rviz_predefined.push_back(1.4216); vec_camera_rviz_predefined.push_back(0.77907);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/default";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // EXPAND PROJECTION
        vec_camera_rviz_predefined.push_back(0.5); vec_camera_rviz_predefined.push_back(3.0); vec_camera_rviz_predefined.push_back(1.05);
        vec_camera_rviz_predefined.push_back(1.0); vec_camera_rviz_predefined.push_back(3.0); vec_camera_rviz_predefined.push_back(1.05);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/expand_projection";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // FILL
        vec_camera_rviz_predefined.push_back(-6.0); vec_camera_rviz_predefined.push_back(1.5); vec_camera_rviz_predefined.push_back(3.5);
        vec_camera_rviz_predefined.push_back(1.0); vec_camera_rviz_predefined.push_back(1.5); vec_camera_rviz_predefined.push_back(0.75);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/fill";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // GRAPH-3D
        vec_camera_rviz_predefined.push_back(12.502); vec_camera_rviz_predefined.push_back(-2.0836); vec_camera_rviz_predefined.push_back(9.3688);
        vec_camera_rviz_predefined.push_back(11.158); vec_camera_rviz_predefined.push_back(2.5593); vec_camera_rviz_predefined.push_back(3.2247);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/graph_3d";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // GRAPH-EVAL
        vec_camera_rviz_predefined.push_back(2.6221); vec_camera_rviz_predefined.push_back(-5.4203); vec_camera_rviz_predefined.push_back(20.995);
        vec_camera_rviz_predefined.push_back(6.2576); vec_camera_rviz_predefined.push_back(8.8025); vec_camera_rviz_predefined.push_back(8.9597);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/graph_eval";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // GRAPH-STATS
        vec_camera_rviz_predefined.push_back(2.5); vec_camera_rviz_predefined.push_back(-1.0); vec_camera_rviz_predefined.push_back(4.5);
        vec_camera_rviz_predefined.push_back(5.0); vec_camera_rviz_predefined.push_back(-1.0); vec_camera_rviz_predefined.push_back(4.5);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/graph_stats";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // OVERVIEW
        vec_camera_rviz_predefined.push_back(-9.3404); vec_camera_rviz_predefined.push_back(-17.831); vec_camera_rviz_predefined.push_back(29.44);
        vec_camera_rviz_predefined.push_back(3.0501); vec_camera_rviz_predefined.push_back(1.7788); vec_camera_rviz_predefined.push_back(3.1549);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/overview";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // PRESENTER
        vec_camera_rviz_predefined.push_back(0.08393); vec_camera_rviz_predefined.push_back(0.47313); vec_camera_rviz_predefined.push_back(1.1179);
        vec_camera_rviz_predefined.push_back(1.0389); vec_camera_rviz_predefined.push_back(0.23996); vec_camera_rviz_predefined.push_back(0.93473);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/presenter";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // PRESENTER FPV
        vec_camera_rviz_predefined.push_back(2.3768); vec_camera_rviz_predefined.push_back(-0.040192); vec_camera_rviz_predefined.push_back(1.4326);
        vec_camera_rviz_predefined.push_back(-0.53616); vec_camera_rviz_predefined.push_back(0.34137); vec_camera_rviz_predefined.push_back(1.1204);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/presenter_fpv";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // STARTUP
        vec_camera_rviz_predefined.push_back(-250.0); vec_camera_rviz_predefined.push_back(-50.0); vec_camera_rviz_predefined.push_back(25.0);
        vec_camera_rviz_predefined.push_back(0.0); vec_camera_rviz_predefined.push_back(0.0); vec_camera_rviz_predefined.push_back(0.0);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/startup";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
        // WIM
        vec_camera_rviz_predefined.push_back(0.12969); vec_camera_rviz_predefined.push_back(0.5321); vec_camera_rviz_predefined.push_back(1.116);
        vec_camera_rviz_predefined.push_back(0.69693); vec_camera_rviz_predefined.push_back(0.88327); vec_camera_rviz_predefined.push_back(0.89466);
        m_vec_camera_rviz_views_predefined.push_back(vec_camera_rviz_predefined);
        sst_setup_camera_defaults.str("");
        sst_setup_camera_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/camera/wim";
        m_hdl_node->setParam(sst_setup_camera_defaults.str(),vec_camera_rviz_predefined);
        vec_camera_rviz_predefined.clear();
    }
    m_oa_cam_rviz->UpdateCameraRvizViews(m_vec_camera_rviz_views_predefined);
    // CAMERA RVIZ - Load SESSION-specific VIEW with index 0
    m_oa_cam_rviz->UpdateModel(m_vec_camera_rviz_views_predefined[0],false,false,-1,3.0,"world",0,true);
    m_oa_cam_rviz->UpdateView();

    // WIM - Load SESSION-specific WIM model for workspace
    // ((WAIRepWIM*)m_wai_oa_rep_wim)->UpdateModel(GetSessionWorkspacePresenter(),GetPathResourcesReps());
    ((WAIRepWIM*)m_wai_oa_rep_wim)->UpdateModel("wim",GetPathResourcesReps());
    // --> Larger Models might not fit into workspace even if scaled down heavily, thus load individual WIM model!
    m_wai_oa_rep_wim->UpdateView();

    // REP - Defaults
    std::stringstream sst_setup_rep_defaults;
    sst_setup_rep_defaults.str("");
    sst_setup_rep_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/reps/rep_name";
    if(!ros::param::has(sst_setup_rep_defaults.str())) m_hdl_node->setParam(sst_setup_rep_defaults.str(),"marvin");
    sst_setup_rep_defaults.str("");
    sst_setup_rep_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/reps/rep_interaction";
    if(!ros::param::has(sst_setup_rep_defaults.str())) m_hdl_node->setParam(sst_setup_rep_defaults.str(),"land");

    // OTHER REPS - Set SESSION-specific STATE
    std::vector<std::string> vec_s_reps_state;
    sst_setup_session_defaults.str("");
    sst_setup_session_defaults << m_s_path_nodename + "/" + m_s_session_name << "/setup_session_defaults/reps_state";
    XmlRpc::XmlRpcValue lst_session_reps_state_default;
    m_hdl_node->getParam(sst_setup_session_defaults.str(), lst_session_reps_state_default);
    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=lst_session_reps_state_default.begin(); it!=lst_session_reps_state_default.end();++it)
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
            ROS_WARN_STREAM("SESSION MANAGER: Wrong config for setting REP state at session startup!");
        }
    }

    // Get number of scenes/slides from exported session config
    if(m_b_enable_html_session==false)
    {
        std::stringstream sst_param_path_session_scene_count_max;
        sst_param_path_session_scene_count_max << m_s_path_nodename+"/"+m_s_session_name << "/setup_session_scene_count_max";
        m_hdl_node->getParam(sst_param_path_session_scene_count_max.str(),m_i_session_scene_count_max);
    }
    else
    {
        QDir qdi_scenes(QString::fromStdString(m_s_path_resources_sessions+m_s_session_name));
        if(qdi_scenes.exists())
        {
            QStringList qst_list=qdi_scenes.entryList(QStringList() << "*.html",QDir::Files);
            m_i_session_scene_count_max=qst_list.length()-1;
        }
        else
        {
            m_i_session_scene_count_max=1;
        }
    }

    // Update "previous" session name, so that scheduler stops with updating session config
    m_s_session_name_prev=m_s_session_name;

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
    return m_i_session_scene_count_max;
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

std::string WAIOASessionManager::GetSessionNameFromFilename(std::string s_session_filepath)
{
    if(s_session_filepath.find(".odp")==std::string::npos
        && s_session_filepath.find(".pptx")==std::string::npos)
    {
        size_t pos=s_session_filepath.find_last_of('/');
        std::string s_session_name_html=s_session_filepath.substr(pos+1);
        return s_session_name_html;
    }
    else
    {
        QFileInfo qfi_fileinfo(QString::fromStdString(s_session_filepath));
        std::string s_session_name_presentation=qfi_fileinfo.baseName().toStdString();
        return s_session_name_presentation;
    }
}

void WAIOASessionManager::UpdateSessionFilename(std::string s_session_filename)
{
    m_s_session_filename=s_session_filename;
    m_s_session_name=GetSessionNameFromFilename(m_s_session_filename);
}

void WAIOASessionManager::SetSessionSceneCountCurrent(bool b_increment_decrement,int i_scene_select)
{
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
    if(m_i_session_scene_count_current<1) m_i_session_scene_count_current=1;
    if(m_i_session_scene_count_current>m_i_session_scene_count_max) m_i_session_scene_count_current=m_i_session_scene_count_max;

    // Indicate SCENE is loading in status label
    SetOAStatusLabel("<b>Loading SCENE...</b>");
    UpdateSessionStatus();
    ros::spinOnce();
    ros::Rate(m_f_node_sample_frequency).sleep();
    ros::spinOnce();
    ros::Rate(m_f_node_sample_frequency).sleep();
    /*
    // Load camera RViz "fill" view
    std::vector<double> vec_d_rviz_camera_fill;
    vec_d_rviz_camera_fill.push_back(-6.0);
    vec_d_rviz_camera_fill.push_back(1.5);
    vec_d_rviz_camera_fill.push_back(3.5);
    vec_d_rviz_camera_fill.push_back(1.0);
    vec_d_rviz_camera_fill.push_back(1.5);
    vec_d_rviz_camera_fill.push_back(0.75);
    m_oa_cam_rviz->UpdateModel(vec_d_rviz_camera_fill,false,false,-1,3.0,"world",0,true);
    m_oa_cam_rviz->UpdateView();
    */

    // Do not update full session with LoadSession() here,
    // but update next/previous/selected scene only!
    UpdateSessionSlidesConfigFromExport();
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
std::string WAIOASessionManager::GetPathResourcesSessions()
{
    return m_s_path_resources_sessions;
}
std::string WAIOASessionManager::GetPathResourcesScenePreview()
{
    return m_s_path_resources_scene_preview;
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
            << "SCD: (ON)"<< ", <b>"
            << GetSessionName() << "</b> ("
            << GetSessionExpertise() << ", "
            << GetSessionExpertiseLevel() << "), "
            << "SLT: " << GetSessionSlotCurrent() << ", "
            << "GRP: " << GetSessionGroup() << ", "
            << "TPC: " << GetSessionTopic() << ", "
            << "SCN: <b>" << GetSessionSceneCountCurrent()<< "</b>/" << m_i_session_scene_count_max << ", "
            << "REM: " << int(m_f_session_time_left)/60 <<"/"<< int(m_f_session_length)/60 << "min "
            << GetSessionPace(m_b_enable_session_scheduler);

        // Break already included in m_f_session_time_left
        // end - 3min - 2min = time for notification!
        if(m_f_session_time_left<=120.0
                && m_f_session_time_left>0.0
                && m_b_session_ends_notified==false)
        {
            InteractionVoiceFromFile("IntelSessionManagerSessionEnds");
            m_b_session_ends_notified=true;
        }
    }
    else
    {
        // Update status label for session without SCHEDULER!
        m_sst_session_status
            << "SCD: (OFF)"<< ", <b>"
            << GetSessionName() << "</b> ("
            << GetSessionExpertise() << ", "
            << GetSessionExpertiseLevel() << "), "
            << "GRP: " << GetSessionGroup() << ", "
            << "TPC: " << GetSessionTopic() << ", "
            << "SCN: <b>" << GetSessionSceneCountCurrent()<< "</b>/" << m_i_session_scene_count_max << ", "
            << "LEN: " << int((ros::Time::now()-m_tim_session_time_start).toSec())/60 << "/" << m_i_session_length <<"min "
            << GetSessionPace(m_b_enable_session_scheduler);

        if(m_f_session_time_left<=120.0
            && m_f_session_time_left>0.0
            && m_b_session_ends_notified==false)
        {
            InteractionVoiceFromFile("IntelSessionManagerSessionEnds");
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

void WAIOASessionManager::PrepareScaffoldingMode(std::string s_rep_name,tf::Vector3 vc3_rep_position)
{
    m_oa_rep_manager->SetRepStateViaService(s_rep_name,
                                            vc3_rep_position,
                                            tf::Vector3(0.0,0.0,0.0));

    m_oa_rep_manager->SetRepStateViaService("workspace_audience_0",
                        tf::Vector3(vc3_rep_position.getX()-2.0,vc3_rep_position.getY()+0.0,vc3_rep_position.getZ()),
                        tf::Vector3(0.0,0.0,3.1415));
    SpinAndWaitForSeconds(0.25);
    m_oa_rep_manager->SetRepStateViaService("workspace_audience_1",
                        tf::Vector3(vc3_rep_position.getX()-2.0,vc3_rep_position.getY()+1.0,vc3_rep_position.getZ()),
                        tf::Vector3(0.0,0.0,3.1415));
    SpinAndWaitForSeconds(0.25);

    m_oa_rep_manager->SetRepStateViaService("workspace_audience_2",
                        tf::Vector3(vc3_rep_position.getX()-2.0,vc3_rep_position.getY()+3.0,vc3_rep_position.getZ()),
                        tf::Vector3(0.0,0.0,3.1415));
    SpinAndWaitForSeconds(0.25);
    m_oa_rep_manager->SetRepStateViaService("workspace_audience_3",
                        tf::Vector3(vc3_rep_position.getX()-2.0,vc3_rep_position.getY()+4.0,vc3_rep_position.getZ()),
                        tf::Vector3(0.0,0.0,3.1415));
    SpinAndWaitForSeconds(0.25);
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
void WAIOASessionManager::UpdateSessionStatus()
{
    std_msgs::String msg_s_oa_status;
    msg_s_oa_status.data=GetOAStatusLabel();
    pub_s_oa_status.publish(msg_s_oa_status);

    std_msgs::String msg_s_session_status;
    msg_s_session_status.data=GetSessionStatusLabel();
    pub_s_session_status.publish(msg_s_session_status);
}

void WAIOASessionManager::cb_tmr_session_status(const ros::TimerEvent& event)
{
    UpdateSessionStatus();
}

void WAIOASessionManager::InteractionVoiceFromFile(std::string s_filename,bool b_blocking)
{
    std::string s_command="";
    int i_retval=0;
    if(b_blocking==true) s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/"+s_filename+".wav";
    else s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/"+s_filename+".wav &";
    i_retval=system(s_command.c_str());
}
