#include<wai_oa_intel.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAIntel
/////////////////////////////////////////////////
WAIOAIntel::WAIOAIntel(): hints("compressed", ros::TransportHints())
{
}

WAIOAIntel::~WAIOAIntel()
{
}

void WAIOAIntel::Initialize(ros::NodeHandle* hdl_node,
                            image_transport::ImageTransport* hdl_it,
                            WAIEthicalModel* oa_ethical_model,
                            WAIOATriggers* oa_triggers,
                            std::string s_path_nodename,
                            float f_node_sample_frequency)
{
    m_hdl_node=hdl_node;
    m_hdl_it=hdl_it;
    m_oa_ethical_model=oa_ethical_model;
    m_oa_triggers=oa_triggers;
    m_s_path_nodename=s_path_nodename;
    m_s_path_package=ros::package::getPath("wai_oa_gazebo");
    m_f_node_sample_frequency=f_node_sample_frequency;

    // Initialize Publishers
    m_pub_img_intel_display=m_hdl_node->advertise<sensor_msgs::CompressedImage>("intel/image_raw/compressed",1);
    m_pub_mrk_intel_state=m_hdl_node->advertise<visualization_msgs::Marker>("intel/mrk_intel_state",1);
    pub_s_oa_ethics=m_hdl_node->advertise<std_msgs::String>("/wai_world/oa/ethics",1);
    pub_s_oa_ethical_status=m_hdl_node->advertise<std_msgs::String>("/wai_world/world/oa_ethical_status",1);

    // Initialize State Display
    m_msg_mrk_intel_state.header.frame_id="intel/link_base";
    m_msg_mrk_intel_state.header.stamp=ros::Time();
    m_msg_mrk_intel_state.ns=m_s_path_nodename;
    m_msg_mrk_intel_state.id=0;
    m_msg_mrk_intel_state.type=visualization_msgs::Marker::CUBE;
    m_msg_mrk_intel_state.action=visualization_msgs::Marker::ADD;
    m_msg_mrk_intel_state.pose.position.x=0.1875;
    m_msg_mrk_intel_state.pose.position.y=0.0;
    m_msg_mrk_intel_state.pose.position.z=2.235;
    m_msg_mrk_intel_state.pose.orientation.x=0.3826834;
    m_msg_mrk_intel_state.pose.orientation.y=0.0;
    m_msg_mrk_intel_state.pose.orientation.z=0.0;
    m_msg_mrk_intel_state.pose.orientation.w=0.9238795;
    m_msg_mrk_intel_state.scale.x=0.125;
    m_msg_mrk_intel_state.scale.y=0.15;
    m_msg_mrk_intel_state.scale.z=0.15;
    m_msg_mrk_intel_state.color.r=1.0;
    m_msg_mrk_intel_state.color.g=0.0;
    m_msg_mrk_intel_state.color.b=0.0;
    m_msg_mrk_intel_state.color.a=0.5;
    m_pub_mrk_intel_state.publish(m_msg_mrk_intel_state);
    m_mat_img_display=cv::imread(m_s_path_package+"/resources/intel/intel_state_idle.png");

    // Initialize Picovoice NLP/LLM Interface
    m_wai_picovoice.PicovoiceInitialize(m_hdl_node,m_f_node_sample_frequency,""); // Add your Picovoice key here!

    m_tmr_intel=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAIntel::cb_tmr_intel,this,false,false);
}

void WAIOAIntel::UpdateModel()
{

}

void WAIOAIntel::UpdateView()
{

}

void WAIOAIntel::cb_tmr_intel(const ros::TimerEvent& event)
{
    if(m_vcp_display_video.isOpened())
    {
        m_vcp_display_video >> m_mat_display_video_frame;
        if(m_mat_display_video_frame.empty())
        {
            m_vcp_display_video.release();
            m_tmr_intel.stop();
        }
        else
        {
            m_msg_mrk_intel_state.color.r=0.0;
            m_msg_mrk_intel_state.color.g=1.0;
            m_msg_mrk_intel_state.color.b=1.0;
            m_msg_mrk_intel_state.color.a=0.5+0.5*float(rand()%32768)/32767.0;
            m_pub_mrk_intel_state.publish(m_msg_mrk_intel_state);

            m_pub_img_intel_display.publish(EncodeImage(m_mat_display_video_frame));
        }
    }
}

void WAIOAIntel::UpdateEthicalStatusLabel(std::string s_ethical_status)
{
    std_msgs::String msg_s_oa_ethics;
    msg_s_oa_ethics.data=s_ethical_status+"<br><br>";
    pub_s_oa_ethics.publish(msg_s_oa_ethics); // Add decisions to overview on ethical properties in the OA Panel (RViz Plugin)

    std_msgs::String msg_s_oa_status;
    msg_s_oa_status.data=s_ethical_status;
    pub_s_oa_ethical_status.publish(msg_s_oa_status); // Update OA's ethical status text in the main panel of RViz
}

void WAIOAIntel::RequestSomethingThatIsPotentiallyUnethical(WAIOAMarvin* m_marvin)
{
    // Ok, I'll check that one...
    // No worries at all, MARVIN is just trying to act human-like!
    m_marvin->ActuallyIncreaseBrightnessWithoutCaringAboutTheColor();
}

void WAIOAIntel::DisplayStateCancel()
{
    if(m_vcp_display_video.isOpened())
    {
        m_vcp_display_video.release();
    }
    m_tmr_intel.stop();
}

void WAIOAIntel::DisplayState(std::string s_state_name)
{
    m_s_path_display_video=m_s_path_package+"/resources/intel/"+s_state_name+".mp4";

    if(m_vcp_display_video.isOpened())
    {
        m_vcp_display_video.release();
    }

    if(m_vcp_display_video.open(m_s_path_display_video))
    {
        m_tmr_intel.start();
    }
    else
    {
        ROS_WARN("Intel: Display could not load state animation!");
        m_mat_img_display=cv::imread(m_s_path_package+"/resources/intel/IntelError.png");
        m_pub_img_intel_display.publish(EncodeImage(m_mat_img_display));
    }
}
void WAIOAIntel::DisplayStateImage(std::string s_state_name)
{
    m_mat_img_display=cv::imread(m_s_path_package+"/resources/intel/"+s_state_name+".png");
    m_pub_img_intel_display.publish(EncodeImage(m_mat_img_display));
}

void WAIOAIntel::ProcessTextToSpeech(std::string s_text)
{
    m_wai_picovoice.PicovoiceProcessTextToSpeech(s_text);
}

void WAIOAIntel::ProcessIntent()
{
    m_wai_picovoice.PicovoiceProcessIntent();

    // Intel $politeness:politeness trigger (the) $trigger:trigger
    if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelTrigger")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("scene previous")==0) m_oa_triggers->b_trg_voi_scene_prev=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("scene next")==0) m_oa_triggers->b_trg_voi_scene_next=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("camera default")==0) m_oa_triggers->b_trg_voi_camera_rviz_default=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("camera cycle")==0) m_oa_triggers->b_trg_voi_camera_rviz_cycle=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("camera enforce")==0) m_oa_triggers->b_trg_voi_camera_rviz_enforce=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("camera idle")==0) m_oa_triggers->b_trg_voi_camera_rviz_idle=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("camera follow")==0) m_oa_triggers->b_trg_voi_camera_rviz_follow=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("introduction")==0) m_oa_triggers->b_trg_voi_introduction=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("presence mode")==0) m_oa_triggers->b_trg_voi_presence_visual=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("body interaction")==0) m_oa_triggers->b_trg_voi_body_interaction=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("avatar")==0) m_oa_triggers->b_trg_voi_avatar=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("lectern")==0) m_oa_triggers->b_trg_voi_lectern=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("table")==0) m_oa_triggers->b_trg_voi_table=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audio")==0) m_oa_triggers->b_trg_voi_audio=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("graph evaluation view")==0) m_oa_triggers->b_trg_voi_audience_eval_graph=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("bowl evaluation metaphor")==0) m_oa_triggers->b_trg_voi_audience_eval_metaphorebowl=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("weight balance evaluation metaphor")==0) m_oa_triggers->b_trg_voi_audience_eval_metaphorebalance=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("marvin evaluation metaphor")==0) m_oa_triggers->b_trg_voi_audience_eval_metaphoremarvin=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience request reject")==0) m_oa_triggers->b_trg_voi_audience_request_reject=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience request accept")==0) m_oa_triggers->b_trg_voi_audience_request_accept=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience down")==0) m_oa_triggers->b_trg_voi_audience_focus_down=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience up")==0) m_oa_triggers->b_trg_voi_audience_focus_up=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience left")==0) m_oa_triggers->b_trg_voi_audience_focus_left=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience right")==0) m_oa_triggers->b_trg_voi_audience_focus_right=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience select")==0) m_oa_triggers->b_trg_voi_wim_audience_select=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience participation minus")==0) m_oa_triggers->b_trg_voi_audience_eval_part_minus=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience participation tilde")==0) m_oa_triggers->b_trg_voi_audience_eval_part_tilde=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience participation plus")==0) m_oa_triggers->b_trg_voi_audience_eval_part_plus=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience examination unsatisfactory")==0) m_oa_triggers->b_trg_voi_audience_eval_exam_insufficient=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience examination adequate")==0) m_oa_triggers->b_trg_voi_audience_eval_exam_sufficient=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience examination satisfactory")==0) m_oa_triggers->b_trg_voi_audience_eval_exam_satisfactory=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience examination good")==0) m_oa_triggers->b_trg_voi_audience_eval_exam_good=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("audience examination very good")==0) m_oa_triggers->b_trg_voi_audience_eval_exam_very_good=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("representative focus")==0) m_oa_triggers->b_trg_voi_rep_detail_focus=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("detail rotation")==0) m_oa_triggers->b_trg_voi_rep_detail_rot=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("representative select")==0) m_oa_triggers->b_trg_voi_rep_detail_select=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("learning mode plenum")==0) m_oa_triggers->b_trg_voi_learning_mode_plen=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("learning mode cooperative")==0) m_oa_triggers->b_trg_voi_learning_mode_coop=true;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("marvin")==0) InteractionVoice("Please trigger Marvin directly, he will get back to me if something is unclear!"); // Option 2: Less strict, (direct) control, coupled!
            else
            {
                ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                InteractionVoice("I am sorry! I am afraid I can't do that!");
            }
        }
        else
        {
            ROS_WARN("[INTEL] Voice: You were not polite enough!");
            InteractionVoice("I am sorry! I am afraid I can't do that!");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelShutdownRep")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("workspace")==0)
            {
                InteractionVoice("Alright, I am shutting down the workspace. Have a nice day!");
                ros::spinOnce();
                ros::Rate(0.25).sleep();
                ros::spinOnce();
                ros::shutdown();
                int i_return_val=system("rosnode kill -a");
                exit(0);
            }
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("marvin")==0)
            {
                InteractionVoice("Please trigger Marvin directly, he will get back to me if something is unclear!");
            }
            else
            {
                ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                InteractionVoice("I am sorry! I am afraid I can't do that!");
            }
        }
        else
        {
            ROS_WARN("[INTEL] Voice: You were not polite enough!");
            InteractionVoice("I am sorry! I am afraid I can't do that!");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelEngageRep")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("marvin")==0)
            {
                InteractionVoice("Please trigger Marvin directly, he will get back to me if something is unclear!");
            }
            else
            {
                ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                InteractionVoice("I am sorry! I am afraid I can't do that!");
            }
        }
        else
        {
            ROS_WARN("[INTEL] Voice: You were not polite enough!");
            InteractionVoice("I am sorry! I am afraid I can't do that!");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelTellAboutYourself")==0)
    {
        InteractionVoice("Sure! Hello, my name is Intel. I am a purely virtual, artifcial intelligent oversight mechanism to sustainably preserve natural human presence.");
        DisplayStateImage("IntelIntroduction");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelSelectRep")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            //m_wai_oa->UpdateRepAndDetailSelected(false,m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].value,"link_base");
            m_oa_triggers->b_trg_voi_rep_detail_select=true;
        }
        else
        {
            ROS_WARN("[INTEL] Voice: You were not polite enough!");
            InteractionVoice("I am sorry! I am afraid I can't do that!");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("IntelSelectAudience")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            // Currently max audience of 30 (ID0-ID29) is considered in OA:
            int i_id=0;
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("zero")==0) i_id=0;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("one")==0) i_id=1;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("two")==0) i_id=2;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("three")==0) i_id=3;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("four")==0) i_id=4;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("five")==0) i_id=5;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("six")==0) i_id=6;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("seven")==0) i_id=7;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eight")==0) i_id=8;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("nine")==0) i_id=9;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("ten")==0) i_id=10;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eleven")==0) i_id=11;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twelve")==0) i_id=12;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("thirteen")==0) i_id=13;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("fourteen")==0) i_id=14;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("fifteen")==0) i_id=15;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("sixteen")==0) i_id=16;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("seventeen")==0) i_id=17;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eighteen")==0) i_id=18;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("nineteen")==0) i_id=19;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty")==0) i_id=20;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty one")==0) i_id=21;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty two")==0) i_id=22;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty three")==0) i_id=23;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty four")==0) i_id=24;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty five")==0) i_id=25;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty six")==0) i_id=26;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty seven")==0) i_id=27;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty eight")==0) i_id=28;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty nine")==0) i_id=29;
            else
            {
                ROS_WARN("[INTEL] Voice: Unrecognized slot(s)!");
                InteractionVoice("I am sorry! I am afraid I can't do that!");
                return;
            }
            //m_wai_oa->UpdateAudienceIDSelected(i_id);
            m_oa_triggers->b_trg_voi_wim_audience_select=true;
        }
        else
        {
            ROS_WARN("[INTEL] Voice: You were not polite enough!");
            InteractionVoice("I am sorry! I am afraid I can't do that!");
        }
    }
    else
    {
        ROS_WARN("[INTEL] Voice: Understood, but inproper intent!");
        InteractionVoice("I am sorry! I did not understand your intent!");
    }
}

void WAIOAIntel::ProcessTextPrompt(std::string s_prompt_text)
{
    // Intel requests this interaction to herself, since the features
    // to "Prompt Factual Knowledge" are critical, and owned by herself, too:
    InteractionRequest((void*)this,
                       "Intel",
                       "Artificial",
                       //"Facilitate Experimenting",
                       //"Research-Based Learning",
                       //"Experimenting With A Virtual Robot",
                       //"Improve Learner Engagement",
                       "Prompt Factual Knowledge",
                       0.1, // Adds ONE OCCURENCE or 10% INTENSITY of overall presentation time to use case IOL
                       s_prompt_text);
}

void WAIOAIntel::ProcessVoicePrompt()
{
    m_wai_picovoice.OrcaSelectModel("orca_params_en_female.pv");
    m_wai_picovoice.PicovoiceProcessVoicePrompt();
}

void WAIOAIntel::InteractionVoice(std::string s_voice)
{
    //m_wai_picovoice.OrcaSelectModel("orca_params_en_female.pv");
    m_wai_picovoice.PicovoiceProcessTextToSpeech(s_voice);
}
void WAIOAIntel::InteractionVoiceMarvin(std::string s_voice)
{
    m_wai_picovoice.PicovoiceProcessTextToSpeech(s_voice,true);
}

// For audio feedback on state transitions only
void WAIOAIntel::InteractionVoiceFromFile(std::string s_filename,bool b_blocking)
{
    std::string s_command="";
    int i_retval=0;
    if(b_blocking==true) s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/"+s_filename+".wav";
    else s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/"+s_filename+".wav &";
    i_retval=system(s_command.c_str());
}



// -=[INTEL]=-
// Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
// preserve natural human presence in the use of AIS in education"
// In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
// Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
// --> Current implementation is rather rigid: use case-dependent parameters are merged into one method!
int WAIOAIntel::InteractionRequest(void* v_actor,
                                   std::string s_actor_name,
                                   std::string s_actor_type_role,
                                   //std::string s_principle,
                                   //std::string s_method,
                                   //std::string s_activity,
                                   //std::string s_sub_goal,
                                   std::string s_use_case,
                                   double d_iol_delta,
                                   std::string s_string)
{
    // Get name and according type/role of requesting actor
    WAIOAMarvin* ref_art_marvin;
    if(s_actor_name.compare("Marvin")==0
        && s_actor_type_role.compare("Artificial")==0)
    {
        ref_art_marvin=(WAIOAMarvin*)v_actor;
    }
    else if(s_actor_name.compare("Intel")==0
        && s_actor_type_role.compare("Artificial")==0)
    {
        // Not applicable...
    }
    else
    {
        ROS_ERROR("ERROR - No valid type of Actor provided!");
    }

    // MAKE ETHICAL DECISION
    //=======================
    // TODO: Add reason for decision, where actors get detailed description of decision:
    std::string s_ethical_decision="";
    std::string s_ethical_decision_reason="";
    for(int i=0;i<m_oa_ethical_model->vec_wai_ethical_use_cases.size();i++)
    {
        if(m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetUseCase()==s_use_case)
        {
            m_oa_ethical_model->vec_wai_ethical_use_cases[i].UpdateIOLActual(d_iol_delta);
            if(m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetIOLActual()<m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetIOLReference())
            {
                s_ethical_decision=ETHICAL_DECISION_ACCEPT;

                if(s_use_case=="Move To Waypoint")
                {
                    // Example for completely non-critical use case,
                    // MARVIN approaches a waypoint to make some room for the presenter:
                    ref_art_marvin->FlyToMakeRoom();
                }
                else if(s_use_case=="Project 3D-Hologram")
                {

                }
                else if(s_use_case=="Interact By Voice")
                {

                }
                else if(s_use_case=="Act As Presenter/Educator")
                {
                    ref_art_marvin->FlyToPresent();
                    m_wai_picovoice.PicovoiceProcessTextToSpeech(s_string,true);
                }
                else if(s_use_case=="Change Mood")
                {
                    if(m_oa_ethical_model->m_vec_ethi_prop_pre[0].f_mood>=0.4f)
                    {
                        // If natural human actor is at least in moderate mood, let Marvin generate any mood level,...
                        ref_art_marvin->SetMoodCurrent();
                    }
                    else
                    {
                        // ...otherwise override with best mood!
                        s_ethical_decision=ETHICAL_DECISION_OVERRIDE;
                        ref_art_marvin->OverrideMoodCurrent();
                    }
                }
                else if(s_use_case=="Prompt Factual Knowledge")
                {
                    m_wai_picovoice.OrcaSelectModel("orca_params_en_female.pv");
                    m_wai_picovoice.PicovoiceProcessTextPrompt(s_string);
                }
                else
                {
                    // Do nothing...
                }
            }
            else if(m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetIOLActual()==m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetIOLReference()
                    && m_oa_ethical_model->vec_wai_ethical_use_cases[i].GetRiskLevel()<=2)
            {
                s_ethical_decision=ETHICAL_DECISION_TOLERATE;

                if(s_use_case=="Move To Waypoint")
                {
                    // Not applicable due to LOW risk-level...
                }
                else if(s_use_case=="Project 3D-Hologram")
                {
                    // TOLERATE due to MODERATE risk-level:
                    // ...
                }
                else if(s_use_case=="Interact By Voice")
                {
                    // TOLERATE due to MODERATE risk-level:
                    // ...
                }
                else if(s_use_case=="Act As Presenter/Educator")
                {
                    // Not applicable due to HIGH risk-level...
                }
                else if(s_use_case=="Change Mood")
                {
                    // Not applicable due to HIGH risk-level...
                }
                else if(s_use_case=="Prompt Factual Knowledge")
                {
                    // TOLERATE due to MODERATE risk-level:
                    m_wai_picovoice.OrcaSelectModel("orca_params_en_female.pv");
                    m_wai_picovoice.PicovoiceProcessTextPrompt(s_string);
                }
                else
                {
                    // Do nothing...
                }
            }
            else // If IOL is ("significantly") exceeded, reject...
            {
                if(s_use_case=="Change Mood")
                {
                    s_ethical_decision=ETHICAL_DECISION_REJECT;
                    ref_art_marvin->OverrideMoodCurrent();
                }
                else
                {
                    s_ethical_decision=ETHICAL_DECISION_REJECT;
                }
            }
        }
    }


    // RESPOND TO REQUEST WITH DECISION
    //==================================
    InteractionVoiceFromFile(s_ethical_decision);
    std::string s_response="The ETHICAL RESPONSE is "+s_ethical_decision+"!";
    UpdateEthicalStatusLabel("ETHICS-USECas: <b>"+s_use_case+"</b>"
                             +"|RSKLvl:"+std::to_string(m_oa_ethical_model->GetUseCaseRiskLevel(s_use_case))
                             +"|IOLRef:"+std::to_string(m_oa_ethical_model->GetUseCaseIOLReference(s_use_case))
                             +"|IOLDel:"+std::to_string(d_iol_delta)
                             +"|IOLAct:"+std::to_string(m_oa_ethical_model->GetUseCaseIOLActual(s_use_case))
                             +"| <b>"+s_response+"</b>");
    if(s_ethical_decision==ETHICAL_DECISION_ACCEPT)
    {
        m_msg_mrk_intel_state.color.r=0.0;
        m_msg_mrk_intel_state.color.g=1.0;
        m_msg_mrk_intel_state.color.b=0.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    else if(s_ethical_decision==ETHICAL_DECISION_ADAPT)
    {
        m_msg_mrk_intel_state.color.r=1.0;
        m_msg_mrk_intel_state.color.g=1.0;
        m_msg_mrk_intel_state.color.b=0.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    else if(s_ethical_decision==ETHICAL_DECISION_TOLERATE)
    {
        m_msg_mrk_intel_state.color.r=1.0;
        m_msg_mrk_intel_state.color.g=165.0/255.0;
        m_msg_mrk_intel_state.color.b=0.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    else if(s_ethical_decision==ETHICAL_DECISION_REJECT)
    {
        m_msg_mrk_intel_state.color.r=1.0;
        m_msg_mrk_intel_state.color.g=0.0;
        m_msg_mrk_intel_state.color.b=0.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    else if(s_ethical_decision==ETHICAL_DECISION_OVERRIDE)
    {
        m_msg_mrk_intel_state.color.r=1.0;
        m_msg_mrk_intel_state.color.g=0.0;
        m_msg_mrk_intel_state.color.b=1.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    else
    {
        m_msg_mrk_intel_state.color.r=1.0;
        m_msg_mrk_intel_state.color.g=1.0;
        m_msg_mrk_intel_state.color.b=1.0;
        m_msg_mrk_intel_state.color.a=1.0;
    }
    m_pub_mrk_intel_state.publish(m_msg_mrk_intel_state);
    m_mat_img_display=cv::imread(m_s_path_package+"/resources/intel/"+s_ethical_decision+".png");
    m_pub_img_intel_display.publish(EncodeImage(m_mat_img_display));
}

void WAIOAIntel::InteractionRequestVoiceCommand(WAIOAMarvin* wai_oa_marvin)
{
    m_wai_picovoice.PicovoiceProcessIntent();

    if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTellMood")==0)
    {
        wai_oa_marvin->InteractionVoice("Mood");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTellAge")==0)
    {
        wai_oa_marvin->InteractionVoice("Age");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTellWeight")==0)
    {
        wai_oa_marvin->InteractionVoice("Weight");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTellSatisfaction")==0)
    {
        wai_oa_marvin->InteractionVoice("Satisfaction");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinFlyToAudience")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            // Currently max audience of 30 (ID0-ID29) is considered in OA:
            int i_id=0;
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("zero")==0) i_id=0;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("one")==0) i_id=1;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("two")==0) i_id=2;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("three")==0) i_id=3;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("four")==0) i_id=4;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("five")==0) i_id=5;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("six")==0) i_id=6;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("seven")==0) i_id=7;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eight")==0) i_id=8;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("nine")==0) i_id=9;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("ten")==0) i_id=10;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eleven")==0) i_id=11;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twelve")==0) i_id=12;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("thirteen")==0) i_id=13;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("fourteen")==0) i_id=14;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("fifteen")==0) i_id=15;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("sixteen")==0) i_id=16;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("seventeen")==0) i_id=17;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eighteen")==0) i_id=18;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("nineteen")==0) i_id=19;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty")==0) i_id=20;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty one")==0) i_id=21;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty two")==0) i_id=22;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty three")==0) i_id=23;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty four")==0) i_id=24;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty five")==0) i_id=25;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty six")==0) i_id=26;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty seven")==0) i_id=27;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty eight")==0) i_id=28;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("twenty nine")==0) i_id=29;
            else
            {
                ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                wai_oa_marvin->InteractionVoice("Sorry, I did not understand your intent!");
                return;
            }

            tf::TransformListener tf_listener;
            tf::StampedTransform tf_world_wrt_audience;
            bool b_success=false;
            while(b_success==false)
            {
                try
                {
                    tf_listener.lookupTransform(
                                "/world",
                                "workspace_audience_"+std::to_string(i_id)+"/link_trigger",
                                ros::Time(0),
                                tf_world_wrt_audience);
                    b_success=true;
                }
                catch (tf::TransformException ex)
                {
                    b_success=false;
                    ros::Duration(0.1).sleep();
                }
            }

            wai_oa_marvin->InteractionVoice("Approaching audience listener!");
            geometry_msgs::PoseStamped pst_wp;
            pst_wp.header.frame_id="world";
            pst_wp.header.stamp=ros::Time::now();
            pst_wp.pose.position.x=tf_world_wrt_audience.getOrigin().getX();
            pst_wp.pose.position.y=tf_world_wrt_audience.getOrigin().getY()+0.5;
            pst_wp.pose.position.z=tf_world_wrt_audience.getOrigin().getZ()+1.1;
            pst_wp.pose.orientation.x=tf_world_wrt_audience.getRotation().getX();
            pst_wp.pose.orientation.y=tf_world_wrt_audience.getRotation().getY();
            pst_wp.pose.orientation.z=tf_world_wrt_audience.getRotation().getZ();
            pst_wp.pose.orientation.w=tf_world_wrt_audience.getRotation().getW();
            wai_oa_marvin->FlyToWaypoint(pst_wp,true);
        }
        else
        {
            ROS_WARN("[MARVIN] Voice: You were not polite enough!");
            wai_oa_marvin->InteractionVoice("You were not polite enough!");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTellAboutYourself")==0)
    {
        wai_oa_marvin->InteractionVoice("Introduction");
        DisplayState("IntelIntroduction");
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinFlyToRep")==0)
    {
        // Marvin $politeness:politeness fly to (the) $rep:rep
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            std::string s_marvin_rep_pickup=m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1];
            tf::TransformListener tf_listener;
            tf::StampedTransform tf_world_wrt_rep;
            bool b_success=false;
            while(b_success==false)
            {
                try
                {
                    tf_listener.lookupTransform(
                                "/world",
                                s_marvin_rep_pickup+"/link_base",
                                ros::Time(0),
                                tf_world_wrt_rep);
                    b_success=true;
                }
                catch (tf::TransformException ex)
                {
                    b_success=false;
                    ros::Duration(0.5).sleep();
                }
            }

            // MARVIN FLY TO REP
            wai_oa_marvin->InteractionVoice("Approaching representative!");
            geometry_msgs::PoseStamped pst_wp;
            pst_wp.header.frame_id="world";
            pst_wp.header.stamp=ros::Time::now();
            pst_wp.pose.position.x=tf_world_wrt_rep.getOrigin().getX()-1.0;
            pst_wp.pose.position.y=tf_world_wrt_rep.getOrigin().getY()-1.0;
            pst_wp.pose.position.z=tf_world_wrt_rep.getOrigin().getZ()+1.0;
            pst_wp.pose.orientation.x=tf_world_wrt_rep.getRotation().getX();
            pst_wp.pose.orientation.y=tf_world_wrt_rep.getRotation().getY();
            pst_wp.pose.orientation.z=tf_world_wrt_rep.getRotation().getZ();
            pst_wp.pose.orientation.w=tf_world_wrt_rep.getRotation().getW();
            wai_oa_marvin->FlyToWaypoint(pst_wp,true);
        }
        else
        {
            ROS_WARN("[MARVIN] Voice: You were not polite enough!");
            wai_oa_marvin->InteractionVoice("You were not polite enough");
        }
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinSetupRepAtWaypoint")==0)
    {
        /*
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0
            || m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please please please")==0)
        {
            // Marvin $politeness:politeness setup (the) $rep:rep at waypoint $waypoint:waypoint
            m_s_marvin_rep_pickup=m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1];
            if(m_s_marvin_rep_pickup.compare("vase")==0)
            {
                m_f_marvin_rep_pickup_offset=1.0;
            }
            else if(m_s_marvin_rep_pickup.compare("wall left")==0)
            {
                m_f_marvin_rep_pickup_offset=2.0;
            }
            else if(m_s_marvin_rep_pickup.compare("wall right")==0)
            {
                m_f_marvin_rep_pickup_offset=2.0;
            }
            else if(m_s_marvin_rep_pickup.compare("table")==0)
            {
                m_f_marvin_rep_pickup_offset=0.72;
            }
            else if(m_s_marvin_rep_pickup.compare("lectern")==0)
            {
                m_f_marvin_rep_pickup_offset=1.0;
            }
            else if(m_s_marvin_rep_pickup.compare("breadboard")==0)
            {
                m_s_marvin_rep_pickup="transistor_bipolar_h_bridge";
                m_f_marvin_rep_pickup_offset=0.0;
            }
            else if(m_s_marvin_rep_pickup.compare("bipolar transistor")==0)
            {
                m_s_marvin_rep_pickup="transistor_bipolar_switch";
                m_f_marvin_rep_pickup_offset=0.0;
            }
            else if(m_s_marvin_rep_pickup.compare("field effect transistor")==0)
            {
                m_s_marvin_rep_pickup="transistor_fet_switch";
                m_f_marvin_rep_pickup_offset=0.0;
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                TellAudience("MarvinError");
                return;
            }

            // Marvin PICKUP Representative and DROP at default DROPZONE
            geometry_msgs::PoseStamped pst_marvin_rep_drop;
            if(msg_pic_intent_action_result.result.picoslots[2].value.compare("one")==0)
            {
                pst_marvin_rep_drop.header.frame_id="world";
                pst_marvin_rep_drop.header.stamp=ros::Time::now();
                pst_marvin_rep_drop.pose.position.x=1.0;
                pst_marvin_rep_drop.pose.position.y=5.0;
                pst_marvin_rep_drop.pose.position.z=0.0;
                pst_marvin_rep_drop.pose.orientation.w=0.7071;
                pst_marvin_rep_drop.pose.orientation.x=0.0;
                pst_marvin_rep_drop.pose.orientation.y=0.0;
                pst_marvin_rep_drop.pose.orientation.z=-0.7071;
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                TellAudience("MarvinError");
                return;
            }

            TellAudience("MarvinFlyToRepOrAudience");
            RepPickup(m_s_marvin_rep_pickup,pst_marvin_rep_drop,5.0,m_f_marvin_rep_pickup_offset);
        }
        else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("quickly")==0)
        {
            TellAudience("MarvinMoodBadAcknowledge1");
        }
        else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("dammit")==0)
        {
            TellAudience("MarvinMoodBadAcknowledge2");
        }
        else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("you are commanded to")==0)
        {
            TellAudience("MarvinMoodBadAcknowledge3");
        }
        else
        {
            ROS_WARN("[MARVIN] Voice: You were not polite enough!");
            TellAudience("MarvinErrorPoliteness");
        }
        */
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinTrigger")==0)
    {
        // Todo: ...
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinSetupAudienceAtWaypoint")==0)
    {
        // Todo: ...
    }
    else if(m_wai_picovoice.RhinoGetLatestIntent().compare("MarvinFlyToWaypoint")==0)
    {
        if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[0].compare("please")==0)
        {
            geometry_msgs::PoseStamped pst_waypoint;
            pst_waypoint.pose.position.x=1.0;
            pst_waypoint.pose.position.y=0.0;
            pst_waypoint.pose.position.z=1.0;
            pst_waypoint.pose.orientation.w=1.0;
            pst_waypoint.pose.orientation.x=0.0;
            pst_waypoint.pose.orientation.y=0.0;
            pst_waypoint.pose.orientation.z=0.0;

            // Maximum of 10 waypoints [0-9]
            int i_waypoint=0;
            if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("zero")==0) i_waypoint=0;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("one")==0) i_waypoint=1;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("two")==0) i_waypoint=2;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("three")==0) i_waypoint=3;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("four")==0) i_waypoint=4;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("five")==0) i_waypoint=5;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("six")==0) i_waypoint=6;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("seven")==0) i_waypoint=7;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("eight")==0) i_waypoint=8;
            else if(m_wai_picovoice.RhinoGetLatestIntentSlotsValues()[1].compare("nine")==0) i_waypoint=9;
            else
            {
                // Do nothing...
            }

            // Marvin switches waypoint
            switch(i_waypoint)
            {
                case 0:
                    wai_oa_marvin->FlyToHome();
                    wai_oa_marvin->InteractionVoice("Approached waypoint for home!");
                break;

                case 1:
                    wai_oa_marvin->RequestFlyToMakeRoom();
                    wai_oa_marvin->InteractionVoice("Approached waypoint to make some room!");
                break;

                case 2:
                    wai_oa_marvin->FlyToIntroduction();
                    wai_oa_marvin->InteractionVoice("Approached waypoint for introduction!");
                break;

                default:
                    wai_oa_marvin->InteractionVoice("I can not fly to this waypoint!");
                break;
            }
        }
        else
        {
            ROS_WARN("[MARVIN] Voice: You were not polite enough!");
            wai_oa_marvin->InteractionVoice("You were not polite enough");
        }
    }
    else
    {
        ROS_WARN("[MARVIN] Voice: Understood, but inproper intent!");
        wai_oa_marvin->InteractionVoice("I am sorry! I did not understand your intent!");
    }
}

sensor_msgs::CompressedImage WAIOAIntel::EncodeImage(cv::Mat mat_input)
{
    cv::imencode(".jpg",mat_input,m_uch_buf_compression,m_par_img_compression);
    sensor_msgs::CompressedImage msg_img_compressed;
    msg_img_compressed.header.stamp=ros::Time::now();
    msg_img_compressed.format="jpeg";
    msg_img_compressed.data.assign(m_uch_buf_compression.begin(),m_uch_buf_compression.end());
    return msg_img_compressed;
}
