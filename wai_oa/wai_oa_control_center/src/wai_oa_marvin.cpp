#include<wai_oa_marvin.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAMarvin
/////////////////////////////////////////////////

WAIOAMarvin::WAIOAMarvin()
{
}

WAIOAMarvin::~WAIOAMarvin()
{
}

void WAIOAMarvin::cb_tmr_marvin(const ros::TimerEvent& event)
{
    for(int i=0;i<(*m_lns_gazebo_linkstates).name.size();i++)
    {
        if((*m_lns_gazebo_linkstates).name[i].compare("marvin::link_base")==0)
        {
            m_pst_marvin_actual.pose.position.x=(*m_lns_gazebo_linkstates).pose[i].position.x;
            m_pst_marvin_actual.pose.position.y=(*m_lns_gazebo_linkstates).pose[i].position.y;
            m_pst_marvin_actual.pose.position.z=(*m_lns_gazebo_linkstates).pose[i].position.z;
            m_pst_marvin_actual.pose.orientation.w=(*m_lns_gazebo_linkstates).pose[i].orientation.w;
            m_pst_marvin_actual.pose.orientation.x=(*m_lns_gazebo_linkstates).pose[i].orientation.x;
            m_pst_marvin_actual.pose.orientation.y=(*m_lns_gazebo_linkstates).pose[i].orientation.y;
            m_pst_marvin_actual.pose.orientation.z=(*m_lns_gazebo_linkstates).pose[i].orientation.z;
        }
        if((*m_lns_gazebo_linkstates).name[i].compare("marvin::link_marvin_hook_gripper")==0)
        {
            m_pst_marvin_hook_actual.pose.position.x=(*m_lns_gazebo_linkstates).pose[i].position.x;
            m_pst_marvin_hook_actual.pose.position.y=(*m_lns_gazebo_linkstates).pose[i].position.y;
            m_pst_marvin_hook_actual.pose.position.z=(*m_lns_gazebo_linkstates).pose[i].position.z;
            m_pst_marvin_hook_actual.pose.orientation.w=(*m_lns_gazebo_linkstates).pose[i].orientation.w;
            m_pst_marvin_hook_actual.pose.orientation.x=(*m_lns_gazebo_linkstates).pose[i].orientation.x;
            m_pst_marvin_hook_actual.pose.orientation.y=(*m_lns_gazebo_linkstates).pose[i].orientation.y;
            m_pst_marvin_hook_actual.pose.orientation.z=(*m_lns_gazebo_linkstates).pose[i].orientation.z;
        }
        if((*m_lns_gazebo_linkstates).name[i].compare(m_s_rep+"::link_base")==0)
        {
            m_pst_rep_actual.pose.position.x=(*m_lns_gazebo_linkstates).pose[i].position.x;
            m_pst_rep_actual.pose.position.y=(*m_lns_gazebo_linkstates).pose[i].position.y;
            m_pst_rep_actual.pose.position.z=(*m_lns_gazebo_linkstates).pose[i].position.z+1.7;
            m_pst_rep_actual.pose.orientation.w=(*m_lns_gazebo_linkstates).pose[i].orientation.w;
            m_pst_rep_actual.pose.orientation.x=(*m_lns_gazebo_linkstates).pose[i].orientation.x;
            m_pst_rep_actual.pose.orientation.y=(*m_lns_gazebo_linkstates).pose[i].orientation.y;
            m_pst_rep_actual.pose.orientation.z=(*m_lns_gazebo_linkstates).pose[i].orientation.z;
        }
    }

    // Trigger pickup of reps
    if(m_b_rep_pickup)
    {
        geometry_msgs::PoseStamped pst_marvin_attach_rep;
        pst_marvin_attach_rep.pose.position.x=m_pst_marvin_hook_actual.pose.position.x;
        pst_marvin_attach_rep.pose.position.y=m_pst_marvin_hook_actual.pose.position.y;
        // Rep is attached Below hook!
        pst_marvin_attach_rep.pose.position.z=m_pst_marvin_hook_actual.pose.position.z-m_f_marvin_rep_pickup_offset;
        pst_marvin_attach_rep.pose.orientation=m_pst_marvin_actual.pose.orientation;
        RepSetState(m_s_rep,pst_marvin_attach_rep.pose);
    }
    else
    {
        // Do nothing...
    }

    // Publish current mood
    pub_mrk_marvin_mood.publish(mrk_marvin_mood->GetMarker());
}
void WAIOAMarvin::cb_tmr_marvin_hologram(const ros::TimerEvent& event)
{
    m_f_marvin_hologram_yaw+=0.01;
    m_qua_oa_marvin_hologram.setRPY(0.0,0.0,m_f_marvin_hologram_yaw);
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,m_f_marvin_hologram_alpha,0.125),CleanupStringDetail(m_s_setup_marvin_hologram),m_col_marvin_hologram,m_s_setup_marvin_hologram,m_s_hologram_resource_path,false,0.5,m_f_marvin_hologram_alpha+(std::rand()%20-10)*0.01);
    m_wai_oa_rep_marvin_hologram->UpdateView();
}

void WAIOAMarvin::Initialize(ros::NodeHandle* hdl_node,
                                     std::string s_path_nodename,
                                     sound_play::SoundClient* hdl_snd_client,
                                     float f_node_sample_frequency,gazebo_msgs::LinkStates* lns_gazebo_linkstates)
                                     //,std::vector<ros::Publisher>* vec_pub_reps_state)
{
    m_hdl_node=hdl_node;
    m_s_path_nodename=s_path_nodename;
    m_s_path_synthetization=ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/picovoice/pv_orca_synthetization.wav";
    m_hdl_snd_client=hdl_snd_client;
    //m_vec_pub_reps_state=vec_pub_reps_state;

    m_lns_gazebo_linkstates=lns_gazebo_linkstates;

    m_f_node_sample_frequency=f_node_sample_frequency;

    m_s_rep="vase";
    m_s_marvin_rep_pickup="vase";
    m_b_rep_pickup=false;

    // Init mood
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    m_i_mood=std::rand()%5+1;
    mrk_marvin_mood=WAIRvizMarkers::create_rviz_marker("SPHERE");
    mrk_marvin_mood->Initialize(m_s_path_nodename,"marvin/link_base");
    mrk_marvin_mood->UpdatePose(tf::Vector3(0.075,0.0,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.05,0.05,0.05));
    std_msgs::ColorRGBA col_mood;
    if(m_i_mood==5)
    {
        col_mood.r=0.0; col_mood.g=1.0; col_mood.b=0.0; col_mood.a=1.0;
    }
    else if(m_i_mood==4)
    {
        col_mood.r=0.0; col_mood.g=1.0; col_mood.b=0.0; col_mood.a=0.5;
    }
    else if(m_i_mood==3)
    {
        col_mood.r=1.0; col_mood.g=0.65; col_mood.b=0.0; col_mood.a=1.0;
    }
    else if(m_i_mood==2)
    {
        col_mood.r=1.0; col_mood.g=0.0; col_mood.b=0.0; col_mood.a=0.5;
    }
    else if(m_i_mood==1)
    {
        col_mood.r=1.0; col_mood.g=0.0; col_mood.b=0.0; col_mood.a=1.0;
    }
    else
    {
        col_mood.r=0.5; col_mood.g=0.5; col_mood.b=0.5; col_mood.a=0.9;
    }
    mrk_marvin_mood->UpdateColor(col_mood);

    m_pst_rep_actual.header.frame_id="world";
    m_pst_rep_actual.header.stamp=ros::Time::now();
    m_pst_rep_actual.pose.position.x=1.0;
    m_pst_rep_actual.pose.position.y=1.0;
    m_pst_rep_actual.pose.position.z=1.0;
    m_pst_rep_actual.pose.orientation.w=1.0;
    m_pst_rep_actual.pose.orientation.x=0.0;
    m_pst_rep_actual.pose.orientation.y=0.0;
    m_pst_rep_actual.pose.orientation.z=0.0;
    m_pst_marvin_actual.header.frame_id="world";
    m_pst_marvin_actual.header.stamp=ros::Time::now();
    m_pst_marvin_actual.pose.position.x=0.0;
    m_pst_marvin_actual.pose.position.y=0.0;
    m_pst_marvin_actual.pose.position.z=0.0;
    m_pst_marvin_actual.pose.orientation.w=1.0;
    m_pst_marvin_actual.pose.orientation.x=0.0;
    m_pst_marvin_actual.pose.orientation.y=0.0;
    m_pst_marvin_actual.pose.orientation.z=0.0;
    m_pst_marvin_reference.header.frame_id="world";
    m_pst_marvin_reference.header.stamp=ros::Time::now();
    m_pst_marvin_reference.pose.position.x=0.0;
    m_pst_marvin_reference.pose.position.y=0.0;
    m_pst_marvin_reference.pose.position.z=0.0;
    m_pst_marvin_reference.pose.orientation.w=1.0;
    m_pst_marvin_reference.pose.orientation.x=0.0;
    m_pst_marvin_reference.pose.orientation.y=0.0;
    m_pst_marvin_reference.pose.orientation.z=0.0;

    // Init REP for hologram
    m_col_marvin_hologram.r=0.101960784; m_col_marvin_hologram.g=0.42745098; m_col_marvin_hologram.b=0.588235294; m_col_marvin_hologram.a=0.3;
    m_wai_oa_rep_marvin_hologram=WAIReps::create_representative("OOI");
    m_wai_oa_rep_marvin_hologram->Initialize(m_hdl_node,m_s_path_nodename,"wai_oa_rep_marvin_hologram","marvin_hologram");
    m_s_hologram_resource_path="";
    m_s_setup_marvin_hologram="logo";
    m_f_marvin_hologram_alpha=0.0;
    m_f_marvin_hologram_yaw=0.0;
    m_qua_oa_marvin_hologram=tf::Quaternion(0.0,0.0,0.0,1.0);
    /*
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,m_f_marvin_hologram_alpha,0.075),m_s_setup_marvin_hologram,m_col_marvin_hologram,m_s_setup_marvin_hologram,m_s_hologram_resource_path,false,1.0,m_f_marvin_hologram_alpha);
    m_wai_oa_rep_marvin_hologram->UpdateView();*/

    m_gazebo_engage_marvin=m_hdl_node->serviceClient<std_srvs::Empty>("/wai_world/marvin/engage");
    m_gazebo_disengage_marvin=m_hdl_node->serviceClient<std_srvs::Empty>("/wai_world/marvin/shutdown");
    m_gazebo_set_model_state=m_hdl_node->serviceClient<gazebo_msgs::SetModelState>("/wai_world/gazebo/set_model_state");

    sub_pic_intent_action_result=m_hdl_node->subscribe("/wai_world/world/get_intent/result", 1, &WAIOAMarvin::cb_sub_pic_intent_action_result,this);
    sub_pic_transcript_action_result=m_hdl_node->subscribe("/wai_world/world/get_transcript/result", 1, &WAIOAMarvin::cb_sub_pic_transcript_action_result,this);
    sub_str_transcript_action_result=m_hdl_node->subscribe("/wai_world/world/get_transcript/result_input_dialog", 1, &WAIOAMarvin::cb_sub_str_transcript_action_result,this);
    pub_pic_llm_response_action_goal=m_hdl_node->advertise<picovoice_msgs::GetLLMResponseActionGoal>("/wai_world/world/get_llm_response/goal",1);
    pub_pic_synthetization_action_goal=m_hdl_node->advertise<picovoice_msgs::GetSynthetizationActionGoal>("/wai_world/world/get_synthetization/goal",1);
    sub_pic_llm_response_action_result=m_hdl_node->subscribe("/wai_world/world/get_llm_response/result", 1, &WAIOAMarvin::cb_sub_pic_llm_response_action_result,this);
    sub_pic_synthetization_action_result=m_hdl_node->subscribe("/wai_world/world/get_synthetization/result", 1, &WAIOAMarvin::cb_sub_pic_synthetization_action_result,this);
    pub_pst_marvin_reference=m_hdl_node->advertise<geometry_msgs::PoseStamped>("/wai_world/marvin/reference/position",1);
    pub_mrk_marvin_mood=m_hdl_node->advertise<visualization_msgs::Marker>("/wai_world/marvin/mrk_mood",1);

    m_tmr_marvin=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAMarvin::cb_tmr_marvin,this,false,false);
    m_tmr_marvin_hologram=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAMarvin::cb_tmr_marvin_hologram,this,false,false);
}

void WAIOAMarvin::cb_sub_pic_intent_action_result(const picovoice_msgs::GetIntentActionResultConstPtr& msg)
{
    msg_pic_intent_action_result=*msg;
    if(msg_pic_intent_action_result.result.is_understood==1)
    {
        if(msg_pic_intent_action_result.result.intent.compare("MarvinTellMood")==0)
        {
            TellMood();
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinTellAge")==0)
        {
            TellAge();
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinTellWeight")==0)
        {
            TellWeight();
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinTellSatisfaction")==0)
        {
            TellSatisfaction();
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinFlyToAudience")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                int i_id=0;
                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("zero")==0) i_id=0;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("one")==0) i_id=1;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("two")==0) i_id=2;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("three")==0) i_id=3;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("four")==0) i_id=4;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("five")==0) i_id=5;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("six")==0) i_id=6;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("seven")==0) i_id=7;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eight")==0) i_id=8;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("nine")==0) i_id=9;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("ten")==0) i_id=10;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eleven")==0) i_id=11;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twelve")==0) i_id=12;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("thirteen")==0) i_id=13;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("fourteen")==0) i_id=14;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("fifteen")==0) i_id=15;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("sixteen")==0) i_id=16;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("seventeen")==0) i_id=17;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("eighteen")==0) i_id=18;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("nineteen")==0) i_id=19;
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("twenty")==0) i_id=20;
                else
                {
                    ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                    Say("MarvinError");
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
                        //ROS_ERROR("%s",ex.what());
                        ros::Duration(0.1).sleep();
                    }
                }

                Say("MarvinFlyToRepOrAudience");
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
                FlyToWaypoint(pst_wp,true);
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: You were not polite enough!");
                Say("MarvinErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinTellAboutYourself")==0)
        {
            TellAboutYourself();
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinFlyToRep")==0)
        {
            // Marvin $politeness:politeness fly to (the) $rep:rep
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                m_s_marvin_rep_pickup=msg_pic_intent_action_result.result.picoslots[1].value;
                if(m_s_marvin_rep_pickup.compare("vase")==0)
                {
                    m_f_marvin_rep_flyto_offset=1.0;
                }
                else if(m_s_marvin_rep_pickup.compare("wall left")==0)
                {
                    m_f_marvin_rep_flyto_offset=2.0;
                }
                else if(m_s_marvin_rep_pickup.compare("wall right")==0)
                {
                    m_f_marvin_rep_flyto_offset=2.0;
                }
                else if(m_s_marvin_rep_pickup.compare("table")==0)
                {
                    m_f_marvin_rep_flyto_offset=0.72;
                }
                else if(m_s_marvin_rep_pickup.compare("lectern")==0)
                {
                    m_f_marvin_rep_flyto_offset=1.0;
                }
                else if(m_s_marvin_rep_pickup.compare("breadboard")==0)
                {
                    m_s_marvin_rep_pickup="transistor_bipolar_h_bridge";
                    m_f_marvin_rep_flyto_offset=0.5;
                }
                else if(m_s_marvin_rep_pickup.compare("bipolar transistor")==0)
                {
                    m_s_marvin_rep_pickup="transistor_bipolar_switch";
                    m_f_marvin_rep_flyto_offset=0.5;
                }
                else if(m_s_marvin_rep_pickup.compare("field effect transistor")==0)
                {
                    m_s_marvin_rep_pickup="transistor_fet_switch";
                    m_f_marvin_rep_flyto_offset=0.5;
                }
                else
                {
                    ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                    Say("MarvinError");
                    return;
                }

                tf::TransformListener tf_listener;
                tf::StampedTransform tf_world_wrt_rep;
                bool b_success=false;
                while(b_success==false)
                {
                    try
                    {
                        tf_listener.lookupTransform(
                                    "/world",
                                    m_s_marvin_rep_pickup+"/link_base",
                                    ros::Time(0),
                                    tf_world_wrt_rep);
                        b_success=true;
                    }
                    catch (tf::TransformException ex)
                    {
                        b_success=false;
                        //ROS_ERROR("%s",ex.what());
                        ros::Duration(0.1).sleep();
                    }
                }

                // MARVIN FLY TO REP
                Say("MarvinFlyToRepOrAudience");
                geometry_msgs::PoseStamped pst_wp;
                pst_wp.header.frame_id="world";
                pst_wp.header.stamp=ros::Time::now();
                pst_wp.pose.position.x=tf_world_wrt_rep.getOrigin().getX();
                pst_wp.pose.position.y=tf_world_wrt_rep.getOrigin().getY()+m_f_marvin_rep_flyto_offset;
                pst_wp.pose.position.z=tf_world_wrt_rep.getOrigin().getZ()+m_f_marvin_rep_flyto_offset;
                pst_wp.pose.orientation.x=tf_world_wrt_rep.getRotation().getX();
                pst_wp.pose.orientation.y=tf_world_wrt_rep.getRotation().getY();
                pst_wp.pose.orientation.z=tf_world_wrt_rep.getRotation().getZ();
                pst_wp.pose.orientation.w=tf_world_wrt_rep.getRotation().getW();
                FlyToWaypoint(pst_wp,true);
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: You were not polite enough!");
                Say("MarvinErrorPoliteness");
            }

        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinSetupRepAtWaypoint")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0
                || msg_pic_intent_action_result.result.picoslots[0].value.compare("please please please")==0)
            {
                // Marvin $politeness:politeness setup (the) $rep:rep at waypoint $waypoint:waypoint
                m_s_marvin_rep_pickup=msg_pic_intent_action_result.result.picoslots[1].value;
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
                    Say("MarvinError");
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
                    Say("MarvinError");
                    return;
                }

                Say("MarvinFlyToRepOrAudience");
                RepPickup(m_s_marvin_rep_pickup,pst_marvin_rep_drop,5.0,m_f_marvin_rep_pickup_offset);
            }
            else if(msg_pic_intent_action_result.result.picoslots[0].value.compare("quickly")==0)
            {
                Say("MarvinMoodBadAcknowledge1");
            }
            else if(msg_pic_intent_action_result.result.picoslots[0].value.compare("dammit")==0)
            {
                Say("MarvinMoodBadAcknowledge2");
            }
            else if(msg_pic_intent_action_result.result.picoslots[0].value.compare("you are commanded to")==0)
            {
                Say("MarvinMoodBadAcknowledge3");
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: You were not polite enough!");
                Say("MarvinErrorPoliteness");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinFlyToRep")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0)
            {
                // Command Marvin to rep with virtual camera following
                // ...
            }
            else if(msg_pic_intent_action_result.result.picoslots[0].value.compare("quickly")==0)
            {
                Say("MarvinMoodBadAcknowledge3");
            }
            else if(msg_pic_intent_action_result.result.picoslots[0].value.compare("dammit")==0)
            {
                Say("MarvinMoodBadAcknowledge4");
            }
            else
            {
                ROS_WARN("[MARVIN] Voice: Unrecognized slot(s)!");
                Say("MarvinError");
            }
        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinTrigger")==0)
        {

        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinSetupAudienceAtWaypoint")==0)
        {

        }
        else if(msg_pic_intent_action_result.result.intent.compare("MarvinFlyToWaypoint")==0)
        {
            if(msg_pic_intent_action_result.result.picoslots[0].value.compare("please")==0)
            {
                geometry_msgs::PoseStamped pst_waypoint;
                pst_waypoint.pose.position.x=1.0;
                pst_waypoint.pose.position.y=0.0;
                pst_waypoint.pose.position.z=1.0;
                pst_waypoint.pose.orientation.w=1.0;
                pst_waypoint.pose.orientation.x=0.0;
                pst_waypoint.pose.orientation.y=0.0;
                pst_waypoint.pose.orientation.z=0.0;

                int i_waypoint=0;

                if(msg_pic_intent_action_result.result.picoslots[1].value.compare("one")==0)
                {
                    i_waypoint=1;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("two")==0)
                {
                    i_waypoint=2;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("three")==0)
                {
                    i_waypoint=3;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("four")==0)
                {
                    i_waypoint=4;
                }
                else if(msg_pic_intent_action_result.result.picoslots[1].value.compare("five")==0)
                {
                    i_waypoint=5;
                }
                else
                {
                    // Do nothing...
                }

                // Marvin switches waypoint
                switch(i_waypoint)
                {
                    case 1:
                        pst_waypoint.pose.position.y=1.0;
                        Say("MarvinWaypoint1");
                        FlyToWaypoint(pst_waypoint);
                    break;

                    case 2:
                        pst_waypoint.pose.position.y=1.5;
                        Say("MarvinWaypoint2");
                        FlyToWaypoint(pst_waypoint);
                    break;

                    case 3:
                        pst_waypoint.pose.position.y=2.0;
                        Say("MarvinWaypoint3");
                        FlyToWaypoint(pst_waypoint);
                    break;

                    case 4:
                        pst_waypoint.pose.position.y=2.5;
                        Say("MarvinWaypoint4");
                        FlyToWaypoint(pst_waypoint);
                    break;

                    case 5:
                        pst_waypoint.pose.position.y=3.0;
                        Say("MarvinWaypoint5");
                        FlyToWaypoint(pst_waypoint);
                    break;

                    default:
                        Say("MarvinError");
                    break;
                }
            }
            else
            {
                Say("MarvinMoodBadAcknowledge2");
            }
        }
        else
        {
            ROS_WARN("[MARVIN] Voice: Understood, but inproper intent!");
        }
    }
    else
    {
        ROS_WARN("[MARVIN] Voice: Did not understand request!");
    }
}

// Callback for voice based prompt
void WAIOAMarvin::cb_sub_pic_transcript_action_result(const picovoice_msgs::GetTranscriptActionResultConstPtr& msg)
{
    msg_pic_transcript_action_result=*msg;
    msg_pic_llm_response_action_goal.header.stamp=ros::Time::now();
    std::string s_prompt=msg_pic_transcript_action_result.result.transcript;
    s_prompt.append(" And please keep your answer short!"); // Append prompt for "short answer"
    msg_pic_llm_response_action_goal.goal.prompt=s_prompt;
    pub_pic_llm_response_action_goal.publish(msg_pic_llm_response_action_goal);
    ROS_WARN_STREAM("[MARVIN] Voice: Received voice prompt..." << std::endl << "{" << s_prompt << "}" << std::endl << " ...from CHEETAH and SENT it to PICOLLM...");
}
// Callback for direct string-based input dialog prompt
void WAIOAMarvin::cb_sub_str_transcript_action_result(const std_msgs::StringConstPtr& msg)
{
    std::string s_prompt=(*msg).data;
    s_prompt.append(" And please keep your answer short!"); // Append prompt for "short answer"
    msg_pic_llm_response_action_goal.goal.prompt=s_prompt;
    pub_pic_llm_response_action_goal.publish(msg_pic_llm_response_action_goal);
    ROS_WARN_STREAM("[MARVIN] Voice: Received voice prompt..." << std::endl << "{" << s_prompt << "}" << std::endl << " ...from INPUT DIALOG and SENT it to PICOLLM...");
}

void WAIOAMarvin::cb_sub_pic_llm_response_action_result(const picovoice_msgs::GetLLMResponseActionResultConstPtr& msg)
{
    msg_pic_llm_response_action_result=*msg;
    std::string s_llm_response_processed=msg_pic_llm_response_action_result.result.llm_response;

    // PROCESS/CLEANUP TRANSCRIPT STRING:
    // Remove </s> at end of response:
    s_llm_response_processed.pop_back();
    s_llm_response_processed.pop_back();
    s_llm_response_processed.pop_back();
    s_llm_response_processed.pop_back();
    // Remove linebreaks:
    std::string::size_type i=0;
    while (i<s_llm_response_processed.length())
    {
        i=s_llm_response_processed.find('\n',i);
        if(i==std::string::npos)
        {
            break;
        }
        s_llm_response_processed[i]=' ';
        //s_llm_response_processed.erase(i);
    }
    // Remove all * characters:
    s_llm_response_processed.erase(std::remove(s_llm_response_processed.begin(),s_llm_response_processed.end(),'*'),s_llm_response_processed.end());
    // Remove duplicate whitespaces:
    std::string::iterator new_end = std::unique(s_llm_response_processed.begin(),s_llm_response_processed.end(),BothAreSpaces);
    s_llm_response_processed.erase(new_end, s_llm_response_processed.end());
    ROS_WARN_STREAM("Processed Transcript: " << s_llm_response_processed);

    msg_pic_synthetization_action_goal.header.stamp=ros::Time::now();
    msg_pic_synthetization_action_goal.goal.transcript=s_llm_response_processed;
    msg_pic_synthetization_action_goal.goal.male_or_female_voice=false;
    msg_pic_synthetization_action_goal.goal.synthesize_to_wav=true;
    msg_pic_synthetization_action_goal.goal.synthesize_to_wav_path=m_s_path_synthetization;
    pub_pic_synthetization_action_goal.publish(msg_pic_synthetization_action_goal);
    ROS_WARN("[MARVIN] Voice: Received response from PICOLLM and SENT transcript to ORCA...");
}

void WAIOAMarvin::MarvinSynthesizeText(std::string s_text)
{
    msg_pic_synthetization_action_goal.header.stamp=ros::Time::now();
    msg_pic_synthetization_action_goal.goal.transcript=s_text;
    msg_pic_synthetization_action_goal.goal.male_or_female_voice=false;
    msg_pic_synthetization_action_goal.goal.synthesize_to_wav=true;
    msg_pic_synthetization_action_goal.goal.synthesize_to_wav_path=m_s_path_synthetization;
    pub_pic_synthetization_action_goal.publish(msg_pic_synthetization_action_goal);
}

void WAIOAMarvin::cb_sub_pic_synthetization_action_result(const picovoice_msgs::GetSynthetizationActionResultConstPtr& msg)
{
    msg_pic_synthetization_action_result=*msg;
    bool b_synth_succ=msg_pic_synthetization_action_result.result.synthesized_success;
    if(b_synth_succ)
    {
        ROS_WARN("[MARVIN] Voice: Received synthetization SUCCESSFUL from ORCA and now playing .WAV!");
        sound_play::Sound snd_play=m_hdl_snd_client->waveSound(m_s_path_synthetization);
        snd_play.play();
    }
    else
    {
        ROS_WARN("[MARVIN] Voice: Received synthetization FAILED from ORCA!");
    }
}

void WAIOAMarvin::UpdateModel(std::string s_text)
{

}

void WAIOAMarvin::UpdateView()
{

}

std::string WAIOAMarvin::CleanupStringDetail(std::string s_string)
{
    std::string s_string_cleanedup;
    s_string_cleanedup=s_string;

    if(s_string_cleanedup.length()>0)
    {
        // Remove the first term "link_..."
        std::size_t pos = s_string_cleanedup.find('_');
        if(pos!=std::string::npos) s_string_cleanedup = s_string_cleanedup.substr(pos+1);

        // Replace all underscores with whitespaces
        std::replace(s_string_cleanedup.begin(),s_string_cleanedup.end(), '_', ' ');

        // Capitalize all words
        for(int x=0;x<s_string_cleanedup.length();x++)
        {
            if(x==0) s_string_cleanedup[x]=toupper(s_string_cleanedup[x]);
            else if(s_string_cleanedup[x-1]==' ') s_string_cleanedup[x]=toupper(s_string_cleanedup[x]);
        }
    }
    else
    {
        s_string_cleanedup="Error!";
    }
    return s_string_cleanedup;
}

void WAIOAMarvin::Say(std::string s_say)
{
    sound_play::Sound snd_play=m_hdl_snd_client->waveSoundFromPkg("wai_oa_gazebo","resources/sounds/"+s_say+".wav");
    snd_play.play();
}
void WAIOAMarvin::SayText(std::string s_text)
{
    //sound_play::Sound snd_play=m_hdl_snd_client->waveSoundFromPkg("wai_oa_gazebo","resources/sounds/"+s_say+".wav");
    m_hdl_snd_client->say(s_text);
}
void WAIOAMarvin::Acknowledge()
{
    std::srand(time(NULL));
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int i_mood_variation=std::rand()%5+1;

    if(m_i_mood==1) // Strongly unpleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodBadAcknowledge1");
        else if(i_mood_variation==2) Say("MarvinMoodBadAcknowledge2");
        else if(i_mood_variation==3) Say("MarvinMoodBadAcknowledge3");
        else if(i_mood_variation==4) Say("MarvinMoodBadAcknowledge4");
        else if(i_mood_variation==5) Say("MarvinMoodBadAcknowledge5");
        else Say("MarvinError");
    }
    else if(m_i_mood==2) // Mildly unpleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodBetterThanBadAcknowledge1");
        else if(i_mood_variation==2) Say("MarvinMoodBetterThanBadAcknowledge2");
        else if(i_mood_variation==3) Say("MarvinMoodBetterThanBadAcknowledge3");
        else if(i_mood_variation==4) Say("MarvinMoodBetterThanBadAcknowledge4");
        else if(i_mood_variation==5) Say("MarvinMoodBetterThanBadAcknowledge5");
        else Say("MarvinError");
    }
    else if(m_i_mood==3) // Neutral Mood
    {
        if(i_mood_variation==1) Say("MarvinMoodOkAcknowledge1");
        else if(i_mood_variation==2) Say("MarvinMoodOkAcknowledge2");
        else if(i_mood_variation==3) Say("MarvinMoodOkAcknowledge3");
        else if(i_mood_variation==4) Say("MarvinMoodOkAcknowledge4");
        else if(i_mood_variation==5) Say("MarvinMoodOkAcknowledge5");
        else Say("MarvinError");
    }
    else if(m_i_mood==4) // Mildly pleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodBetterThanOkAcknowledge1");
        else if(i_mood_variation==2) Say("MarvinMoodBetterThanOkAcknowledge2");
        else if(i_mood_variation==3) Say("MarvinMoodBetterThanOkAcknowledge3");
        else if(i_mood_variation==4) Say("MarvinMoodBetterThanOkAcknowledge4");
        else if(i_mood_variation==5) Say("MarvinMoodBetterThanOkAcknowledge5");
        else Say("MarvinError");
    }
    else if(m_i_mood==5) // Strongly pleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodFineAcknowledge1");
        else if(i_mood_variation==2) Say("MarvinMoodFineAcknowledge2");
        else if(i_mood_variation==3) Say("MarvinMoodFineAcknowledge3");
        else if(i_mood_variation==4) Say("MarvinMoodFineAcknowledge4");
        else if(i_mood_variation==5) Say("MarvinMoodFineAcknowledge5");
        else Say("MarvinError");
    }
    else
    {
        Say("MarvinError");
    }
}
void WAIOAMarvin::TellMood()
{
    if(m_i_mood==1) Say("MarvinMoodBad");
    else if(m_i_mood==2) Say("MarvinMoodBetterThanBad");
    else if(m_i_mood==3) Say("MarvinMoodOk");
    else if(m_i_mood==4) Say("MarvinMoodBetterThanOk");
    else if(m_i_mood==5) Say("MarvinMoodFine");
    else Say("MarvinError");
}
void WAIOAMarvin::TellAge()
{
    Say("MarvinAge");
}
void WAIOAMarvin::TellWeight()
{
    Say("MarvinWeight");
}
void WAIOAMarvin::TellSatisfaction()
{
    Say("MarvinSatisfaction");
}
void WAIOAMarvin::TellAboutYourself()
{
    //Say("MarvinIntroduction");
    MarvinSynthesizeText("Sure! Hi guys! My name is Marvin. I'm a purely virtual, artificially intelligent display drone, created to support real natural human presenters. And, like Paul Milgram, Haruo Takemura, Akira Utsumi and Fumio Kishino, i like all kind of displays.");
}

void WAIOAMarvin::EnableHologram(std::string s_hologram_resource_path,std::string s_setup_marvin_hologram)
{
    m_s_hologram_resource_path=s_hologram_resource_path;
    m_s_setup_marvin_hologram=s_setup_marvin_hologram;
    m_f_marvin_hologram_alpha=0.4;
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,m_f_marvin_hologram_alpha,0.125),CleanupStringDetail(s_setup_marvin_hologram),m_col_marvin_hologram,s_setup_marvin_hologram,m_s_hologram_resource_path,true,0.5,m_f_marvin_hologram_alpha);
    m_wai_oa_rep_marvin_hologram->UpdateView();
    m_tmr_marvin_hologram.start();
}
void WAIOAMarvin::DisableHologram()
{
    m_tmr_marvin_hologram.stop();
    m_s_setup_marvin_hologram="logo";
    m_f_marvin_hologram_alpha=0.0;
    m_f_marvin_hologram_yaw=0.0;
    m_qua_oa_marvin_hologram=tf::Quaternion(0.0,0.0,0.0,1.0);
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,m_f_marvin_hologram_alpha,0.0),CleanupStringDetail(m_s_setup_marvin_hologram),m_col_marvin_hologram,m_s_setup_marvin_hologram,m_s_hologram_resource_path,true,0.0,m_f_marvin_hologram_alpha);
    m_wai_oa_rep_marvin_hologram->UpdateView();
}

void WAIOAMarvin::RepSetState(std::string s_rep_set_state,geometry_msgs::Pose pos_rep_set_state)
{
    /*
    for(unsigned long i=0;i<(*m_vec_pub_reps_state).size();i++)
    {
        if ((*m_vec_pub_reps_state)[i].getTopic().find(s_rep_set_state) != std::string::npos)
        {
            (*m_vec_pub_reps_state)[i].publish(pos_rep_set_state);
        }
    }*/
    gazebo_msgs::SetModelState set_model_state;
    gazebo_msgs::ModelState model_state;
    model_state.model_name = s_rep_set_state;
    geometry_msgs::Pose pos_setup_state;
    pos_setup_state.position.x=pos_rep_set_state.position.x;
    pos_setup_state.position.y=pos_rep_set_state.position.y;
    pos_setup_state.position.z=pos_rep_set_state.position.z;
    pos_setup_state.orientation.w=pos_rep_set_state.orientation.w;
    pos_setup_state.orientation.x=pos_rep_set_state.orientation.x;
    pos_setup_state.orientation.y=pos_rep_set_state.orientation.y;
    pos_setup_state.orientation.z=pos_rep_set_state.orientation.z;
    model_state.pose=pos_setup_state;
    set_model_state.request.model_state=model_state;
    if(m_gazebo_set_model_state.isValid())
    {
        m_gazebo_set_model_state.waitForExistence();
        m_gazebo_set_model_state.call(set_model_state);
    }
}

void WAIOAMarvin::Engage(geometry_msgs::PoseStamped pst_reference_engage)
{
    if(m_i_mood==1) Say("MarvinMoodBadEngage");
    else if(m_i_mood==2) Say("MarvinMoodBetterThanBadEngage");
    else if(m_i_mood==3) Say("MarvinMoodOkEngage");
    else if(m_i_mood==4) Say("MarvinMoodBetterThanOkEngage");
    else if(m_i_mood==5) Say("MarvinMoodFineEngage");
    else Say("MarvinError");

    std_srvs::Empty srv_engange_marvin;
    if(m_gazebo_engage_marvin.isValid())
    {
        m_gazebo_engage_marvin.waitForExistence();
        m_gazebo_engage_marvin.call(srv_engange_marvin);
    }

    // Fly at mood-dependent height (disabled)
    //pst_reference_engage.pose.position.z=pst_reference_engage.pose.position.z*float(m_i_mood)/5.0;
    m_pst_marvin_reference=pst_reference_engage;
    pub_pst_marvin_reference.publish(m_pst_marvin_reference);

    // Periodically update pose of Marvin and Rep
    m_tmr_marvin.start();
}
void WAIOAMarvin::Shutdown()
{
    geometry_msgs::PoseStamped pst_marvin_reference_land;
    pst_marvin_reference_land.pose=m_pst_marvin_actual.pose;
    pst_marvin_reference_land.pose.position.z=0.0;
    FlyToWaypoint(pst_marvin_reference_land,true);


    std_srvs::Empty srv_disengange_marvin;
    if(m_gazebo_disengage_marvin.isValid())
    {
        m_gazebo_disengage_marvin.waitForExistence();
        m_gazebo_disengage_marvin.call(srv_disengange_marvin);
    }

    Say("MarvinDisengaged");

    // Stop update of states/poses
    m_tmr_marvin.stop();
}

void WAIOAMarvin::RepPickup(std::string s_rep,geometry_msgs::PoseStamped pst_reference_rep_drop,float f_rep_pickup_flight_height,float f_marvin_rep_pickup_offset)
{
    // Acknowledge task
    ros::Rate(0.5).sleep();
    Acknowledge();
    if(m_i_mood==1) return;

    // Define ros rate
    ros::Rate rat_marvin(m_f_node_sample_frequency);

    // Define rep to setup
    m_s_rep=s_rep;
    m_f_rep_pickup_flight_height=f_rep_pickup_flight_height;
    m_f_marvin_rep_pickup_offset=f_marvin_rep_pickup_offset;
    m_f_marvin_rep_flyto_offset=m_f_marvin_rep_pickup_offset; // Pickup height offset is same as flyto

    // Save Marvins origin
    geometry_msgs::PoseStamped pst_marvin_origin;
    pst_marvin_origin.pose=m_pst_marvin_actual.pose;

    // Fly above object
    geometry_msgs::PoseStamped pst_marvin_gain_height;
    pst_marvin_gain_height.pose=m_pst_marvin_actual.pose;
    pst_marvin_gain_height.pose.position.z=m_f_rep_pickup_flight_height;
    FlyToWaypoint(pst_marvin_gain_height);

    geometry_msgs::PoseStamped pst_marvin_rep_hover_above;
    pst_marvin_rep_hover_above.pose=m_pst_rep_actual.pose;
    pst_marvin_rep_hover_above.pose.position.z=m_f_rep_pickup_flight_height;
    FlyToWaypoint(pst_marvin_rep_hover_above);

    geometry_msgs::PoseStamped pst_marvin_rep_pickup;
    pst_marvin_rep_pickup.pose=m_pst_rep_actual.pose;
    pst_marvin_rep_pickup.pose.position.z=m_f_marvin_rep_pickup_offset+0.5;
    FlyToWaypoint(pst_marvin_rep_pickup);

    // Pickup object
    Say("MarvinPickingUpObject");
    //ApplyState(m_s_marvin_rep_pickup,tf::Vector3(msg_lns_gazebo.pose[i].position.x,msg_lns_gazebo.pose[i].position.y,msg_lns_gazebo.pose[i].position.z-1.0),tf::Quaternion(0.0,0.0,0.0,1.0));
    m_b_rep_pickup=true;

    // Gain height after pickup
    geometry_msgs::PoseStamped pst_marvin_rep_pickup_hover_above;
    pst_marvin_rep_pickup_hover_above.pose=m_pst_rep_actual.pose;
    pst_marvin_rep_pickup_hover_above.pose.position.z=m_f_rep_pickup_flight_height;
    FlyToWaypoint(pst_marvin_rep_pickup_hover_above);

    // Fly to dropzone
    geometry_msgs::PoseStamped pst_marvin_rep_drop_hover;
    pst_marvin_rep_drop_hover.pose=pst_reference_rep_drop.pose;
    pst_marvin_rep_drop_hover.pose.position.z=m_f_rep_pickup_flight_height;
    FlyToWaypoint(pst_marvin_rep_drop_hover);

    // Lower altitude and drop with commanded orientation
    geometry_msgs::PoseStamped pst_marvin_rep_drop;
    pst_marvin_rep_drop.pose=pst_reference_rep_drop.pose;
    pst_marvin_rep_drop.pose.position.z=m_f_marvin_rep_pickup_offset+1.0;
    FlyToWaypoint(pst_marvin_rep_drop);

    // Drop rep
    Say("MarvinDroppingObject");
    m_b_rep_pickup=false;

    // Gain height once more
    FlyToWaypoint(pst_marvin_rep_drop_hover);

    // Return home
    geometry_msgs::PoseStamped pst_marvin_return_home;
    pst_marvin_return_home.pose=pst_marvin_origin.pose;
    FlyToWaypoint(pst_marvin_return_home);

    // Report completed task
    ReportOnTaskComplete();
}
void WAIOAMarvin::ReportOnTaskComplete()
{
    std::srand(time(NULL));
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    int i_mood_variation=std::rand()%3+1;

    if(m_i_mood==1) // Strongly unpleasant mood
    {
        Say("MarvinMoodBadTaskComplete1");
        /*
        if(i_mood_variation==1) Say("MarvinMoodBadTaskComplete1");
        else if(i_mood_variation==2) Say("MarvinMoodBadTaskComplete2");
        else if(i_mood_variation==3) Say("MarvinMoodBadTaskComplete3");
        else Say("MarvinError");
        */
    }
    else if(m_i_mood==2) // Mildly unpleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodBetterThanBadTaskComplete1");
        else if(i_mood_variation==2) Say("MarvinMoodBetterThanBadTaskComplete2");
        else if(i_mood_variation==3) Say("MarvinMoodBetterThanBadTaskComplete3");
        else if(i_mood_variation==4) Say("MarvinMoodBetterThanBadTaskComplete4");
        else Say("MarvinError");
    }
    else if(m_i_mood==3) // Neutral Mood
    {
        if(i_mood_variation==1) Say("MarvinMoodOkTaskComplete1");
        else if(i_mood_variation==2) Say("MarvinMoodOkTaskComplete2");
        else if(i_mood_variation==3) Say("MarvinMoodOkTaskComplete3");
        else Say("MarvinError");
    }
    else if(m_i_mood==4) // Mildly pleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodBetterThanOkTaskComplete1");
        else if(i_mood_variation==2) Say("MarvinMoodBetterThanOkTaskComplete2");
        else if(i_mood_variation==3) Say("MarvinMoodBetterThanOkTaskComplete3");
        else Say("MarvinError");
    }
    else if(m_i_mood==5) // Strongly pleasant mood
    {
        if(i_mood_variation==1) Say("MarvinMoodFineTaskComplete1");
        else if(i_mood_variation==2) Say("MarvinMoodFineTaskComplete2");
        else if(i_mood_variation==3) Say("MarvinMoodFineTaskComplete3");
        else Say("MarvinError");
    }
    else
    {
        Say("MarvinError");
    }
}
void WAIOAMarvin::FlyToWaypoint(geometry_msgs::PoseStamped pst_reference_waypoint,bool b_blocking)
{
    // Define ros rate
    ros::Rate rat_marvin(m_f_node_sample_frequency);

    // Command marvin to waypoint
    pub_pst_marvin_reference.publish(pst_reference_waypoint);
    ros::spinOnce();
    rat_marvin.sleep();

    // Wait for Marvin to approx. reach waypoint (blocking)
    if(b_blocking==true)
    {
        float f_length_diff=1.0;
        do
        {
            f_length_diff=(tf::Vector3(pst_reference_waypoint.pose.position.x,
                                             pst_reference_waypoint.pose.position.y,
                                             pst_reference_waypoint.pose.position.z)
                                 -tf::Vector3(m_pst_marvin_actual.pose.position.x,
                                              m_pst_marvin_actual.pose.position.y,
                                              m_pst_marvin_actual.pose.position.z)).length();
            //ROS_WARN("Length diff: %3.3f",f_length_diff);

            ros::spinOnce();
            rat_marvin.sleep();
        }while(f_length_diff>0.15);

        // Wait 1 sec for stabilization
        for(int i=0;i<int(m_f_node_sample_frequency);i++)
        {
            ros::spinOnce();
            rat_marvin.sleep();
        }
    }
}
