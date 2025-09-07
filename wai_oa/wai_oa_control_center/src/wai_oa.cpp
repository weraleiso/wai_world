#include<wai_oa_state.h>



/////////////////////////////////////////////////
/// Allow only one object to be instantiated
/////////////////////////////////////////////////
WAIOpenAuditorium* WAIOpenAuditorium::instance = 0;
WAIOpenAuditorium* WAIOpenAuditorium::getInstance()
{
    if(instance==0)
    {
        instance=new WAIOpenAuditorium();
    }
    return instance;
}


/////////////////////////////////////////////////
/// Delete instance
/////////////////////////////////////////////////
WAIOpenAuditorium::~WAIOpenAuditorium()
{
    delete wai_oa_state;
    delete instance;
}


//////////////////////////////////////////////////
/// Construct object including all initialization
//////////////////////////////////////////////////
WAIOpenAuditorium::WAIOpenAuditorium():it(nh),
    hints("compressed", ros::TransportHints()),
    hints_depth("compressedDepth", ros::TransportHints())
{
    // Resolve parameters from launchfile
    s_path_nodename=ros::this_node::getName();
    nh.getParam(s_path_nodename+"/"+"S_SESSION_GROUP", S_SESSION_GROUP);
    nh.getParam(s_path_nodename+"/"+"S_SESSION_TOPIC", S_SESSION_TOPIC);
    nh.getParam(s_path_nodename+"/"+"S_SESSION_EXPERTISE", S_SESSION_EXPERTISE);
    nh.getParam(s_path_nodename+"/"+"S_SESSION_EXPERTISE_LEVEL", S_SESSION_EXPERTISE_LEVEL);
    nh.getParam(s_path_nodename+"/"+"S_SESSION_NAME", S_SESSION_NAME);
    nh.getParam(s_path_nodename+"/"+"I_SESSION_LENGTH", I_SESSION_LENGTH);
    nh.getParam(s_path_nodename+"/"+"I_SESSION_BREAK_LENGTH", I_SESSION_BREAK_LENGTH);
    nh.getParam(s_path_nodename+"/"+"I_SCENE_START_BEFORE", I_SCENE_START_BEFORE);
    nh.getParam(s_path_nodename+"/"+"I_AUDIENCE_COUNT_MAX", I_AUDIENCE_COUNT_MAX);
    nh.getParam(s_path_nodename+"/"+"I_AUDIENCE_LISTENERS_PER_ROW", I_AUDIENCE_LISTENERS_PER_ROW);
    nh.getParam(s_path_nodename+"/"+"I_AUDIENCE_REQUEST_QUEUE_SIZE", I_AUDIENCE_REQUEST_QUEUE_SIZE);
    nh.getParam(s_path_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY", F_NODE_SAMPLE_FREQUENCY);
    nh.getParam(s_path_nodename+"/"+"F_SCENE_TRANSITION_TIMEOUT", F_SCENE_TRANSITION_TIMEOUT);
    nh.getParam(s_path_nodename+"/"+"F_SCENE_TRIGGER_TIMEOUT", F_SCENE_TRIGGER_TIMEOUT);
    nh.getParam(s_path_nodename+"/"+"F_SCENE_TRIGGER_COLL_THRES", F_SCENE_TRIGGER_COLL_THRES);
    nh.getParam(s_path_nodename+"/"+"I_PRESENTER_PRESENCE_MODE", I_PRESENTER_PRESENCE_MODE);
    nh.getParam(s_path_nodename+"/"+"I_PRESENTER_PROMPT_MODE", I_PRESENTER_PROMPT_MODE);
    nh.getParam(s_path_nodename+"/"+"S_PRESENTER_WORKSPACE_MODEL", S_PRESENTER_WORKSPACE_MODEL);
    nh.getParam(s_path_nodename+"/"+"F_PRESENTER_POSE_X", F_PRESENTER_POSE_X);
    nh.getParam(s_path_nodename+"/"+"F_PRESENTER_POSE_Y", F_PRESENTER_POSE_Y);
    nh.getParam(s_path_nodename+"/"+"F_PRESENTER_POSE_Z", F_PRESENTER_POSE_Z);
    nh.getParam(s_path_nodename+"/"+"F_PRESENTER_POSE_YAW", F_PRESENTER_POSE_YAW);
    nh.getParam(s_path_nodename+"/"+"F_PROJECTION_POSE_X", F_PROJECTION_POSE_X);
    nh.getParam(s_path_nodename+"/"+"F_PROJECTION_POSE_Y", F_PROJECTION_POSE_Y);
    nh.getParam(s_path_nodename+"/"+"F_PROJECTION_POSE_Z", F_PROJECTION_POSE_Z);
    nh.getParam(s_path_nodename+"/"+"F_PROJECTION_POSE_YAW", F_PROJECTION_POSE_YAW);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_RESOLUTION_X", F_CAMERA_RGBD_RESOLUTION_X);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_RESOLUTION_Y", F_CAMERA_RGBD_RESOLUTION_Y);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_RESOLUTION_SCALE", F_CAMERA_RGBD_RESOLUTION_SCALE);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_FX", F_CAMERA_RGBD_FX);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_FY", F_CAMERA_RGBD_FY);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_CX", F_CAMERA_RGBD_CX);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_CY", F_CAMERA_RGBD_CY);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_RESOLUTION_X", F_CAMERA_RGBD_DEPTH_RESOLUTION_X);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_RESOLUTION_Y", F_CAMERA_RGBD_DEPTH_RESOLUTION_Y);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_RESOLUTION_SCALE", F_CAMERA_RGBD_DEPTH_RESOLUTION_SCALE);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_FX", F_CAMERA_RGBD_DEPTH_FX);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_FY", F_CAMERA_RGBD_DEPTH_FY);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_CX", F_CAMERA_RGBD_DEPTH_CX);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_DEPTH_CY", F_CAMERA_RGBD_DEPTH_CY);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_RANGE_MIN", F_CAMERA_RGBD_RANGE_MIN);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_RANGE_MAX", F_CAMERA_RGBD_RANGE_MAX);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_MASK_WIDTH", F_CAMERA_RGBD_MASK_WIDTH);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_THRESHOLD_DIST", F_CAMERA_RGBD_THRESHOLD_DIST);
    nh.getParam(s_path_nodename+"/"+"F_CAMERA_RGBD_THRESHOLD_BOUNDS", F_CAMERA_RGBD_THRESHOLD_BOUNDS);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_KALMAN", B_ENABLE_KALMAN);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_TELEPROMPTER", B_ENABLE_TELEPROMPTER);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_CAMERA_RVIZ_FLY_IN", B_ENABLE_CAMERA_RVIZ_FLY_IN);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_CAMERA_RVIZ_IDLE", B_ENABLE_CAMERA_RVIZ_IDLE);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_LECTERN", B_ENABLE_LECTERN);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_TABLE", B_ENABLE_TABLE);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_AUDIO", B_ENABLE_AUDIO);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_SLIDE_INTERACTIONS", B_ENABLE_SLIDE_INTERACTIONS);
    nh.getParam(s_path_nodename+"/"+"B_ENABLE_SESSION_SCHEDULER", B_ENABLE_SESSION_SCHEDULER);

    // SRAND Initialization
    std::srand(time(NULL));
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    // Init ROS Subscribers and Publishers
    pub_hea_audience_request_to_panel=nh.advertise<std_msgs::Header>("audience_request_to_panel",1);
    pub_pic_intent_action_goal=nh.advertise<picovoice_msgs::GetIntentActionGoal>("/wai_world/world/get_intent/goal",1);
    if(I_PRESENTER_PROMPT_MODE==1) // If prompt mode is 1, use VOICE prompt
    {
        pub_pic_transcript_action_goal=nh.advertise<picovoice_msgs::GetTranscriptActionGoal>("/wai_world/world/get_transcript/goal",1);
    }
    else
    {
        pub_str_transcript_action_goal=nh.advertise<std_msgs::String>("/wai_world/world/get_transcript/result_input_dialog",1);
    }
    pub_mrk_oa_arrow_force=nh.advertise<visualization_msgs::Marker>("mrk_oa_arrow_force",1);
    //pub_hea_eval_outgoing=nh.advertise<std_msgs::Header>("eval_outgoing",1);
    //pub_hea_scene_outgoing=nh.advertise<std_msgs::Header>("scene_outgoing",1);
    //sub_hea_scene_confirmed=nh.subscribe("scene_confirmed", 1, &WAIOpenAuditorium::cb_sub_hea_scene_confirmed,this);
    //sub_aud_audio_captured_presenter=nh.subscribe("/audio_capture/audio", 1, &WAIOpenAuditorium::cb_sub_aud_audio_captured_presenter,this);
    gazebo_spawn_model_client=nh.serviceClient<gazebo_msgs::SpawnModel>("/wai_world/gazebo/spawn_urdf_model");
    gazebo_delete_model_client=nh.serviceClient<gazebo_msgs::DeleteModel>("/wai_world/gazebo/delete_model");

    // Init colors
    col_invisible.r=0.0; col_invisible.g=0.0; col_invisible.b=0.0; col_invisible.a=0.0;
    col_white.r=1.0; col_white.g=1.0; col_white.b=1.0; col_white.a=1.0;
    col_black.r=0.0; col_black.g=0.0; col_black.b=0.0; col_black.a=1.0;
    col_grey.r=0.5; col_grey.g=0.5; col_grey.b=0.5; col_grey.a=0.5;
    col_red_trans.r=1.0; col_red_trans.g=0.0; col_red_trans.b=0.0; col_red_trans.a=0.3;
    col_red.r=1.0; col_red.g=0.0; col_red.b=0.0; col_red.a=0.9;
    col_red_opaque.r=1.0; col_red_opaque.g=0.0; col_red_opaque.b=0.0; col_red_opaque.a=1.0;
    col_green_trans.r=0.0; col_green_trans.g=1.0; col_green_trans.b=0.0; col_green_trans.a=0.3;
    col_green.r=0.0; col_green.g=1.0; col_green.b=0.0; col_green.a=0.9;
    col_green_opaque.r=0.0; col_green_opaque.g=1.0; col_green_opaque.b=0.0; col_green_opaque.a=1.0;
    col_blue_trans.r=0.0; col_blue_trans.g=0.0; col_blue_trans.b=1.0; col_blue_trans.a=0.3;
    col_blue.r=0.0; col_blue.g=0.0; col_blue.b=1.0; col_blue.a=0.9;
    col_blue_opaque.r=0.0; col_blue_opaque.g=0.0; col_blue_opaque.b=1.0; col_blue_opaque.a=1.0;
    col_red_light_opaque.r=219.0/255.0; col_red_light_opaque.g=68.0/255.0; col_red_light_opaque.b=55.0/255.0; col_red_light_opaque.a=1.0;
    col_cyan_trans.r=0.0; col_cyan_trans.g=1.0; col_cyan_trans.b=1.0; col_cyan_trans.a=0.3;
    col_cyan.r=0.0; col_cyan.g=1.0; col_cyan.b=1.0; col_cyan.a=0.9;
    col_cyan_opaque.r=0.0; col_cyan_opaque.g=1.0; col_cyan_opaque.b=1.0; col_cyan_opaque.a=1.0;
    col_orange.r=1.0; col_orange.g=0.65; col_orange.b=0.0; col_orange.a=0.9;
    col_orange_trans.r=1.0; col_orange_trans.g=0.65; col_orange_trans.b=0.0; col_orange_trans.a=0.3;
    col_oa_trans.r=0.101960784; col_oa_trans.g=0.42745098; col_oa_trans.b=0.588235294; col_oa_trans.a=0.3;
    col_oa_shiny.r=0.101960784; col_oa_shiny.g=0.42745098; col_oa_shiny.b=0.588235294; col_oa_shiny.a=0.1;
    col_oa.r=0.101960784; col_oa.g=0.42745098; col_oa.b=0.588235294; col_oa.a=0.9;
    col_oa_opaque.r=0.101960784; col_oa_opaque.g=0.42745098; col_oa_opaque.b=0.588235294; col_oa_opaque.a=1.0;
    // Define colors for RViz light
    col_light_off.r=0.0; col_light_off.g=0.0; col_light_off.b=0.0; col_light_off.a=255.0;
    col_light_dark.r=64.0; col_light_dark.g=64.0; col_light_dark.b=64.0; col_light_dark.a=255.0;
    col_light_default.r=136.0; col_light_default.g=138.0; col_light_default.b=133.0; col_light_default.a=255.0;
    col_light_bright.r=192.0; col_light_bright.g=192.0; col_light_bright.b=192.0; col_light_bright.a=255.0;
    col_light_cold.r=114.0; col_light_cold.g=159.0; col_light_cold.b=207.0; col_light_cold.a=255.0;
    col_light_warm.r=233.0; col_light_warm.g=185.0; col_light_warm.b=110.0; col_light_warm.a=255.0;

    // Init triggers and events
    B_EVENT_STARTUP_FINISHED=false;

    // Init time related members
    tim_startup=ros::Time::now();
    tim_trigger_activated=tim_startup;
    tim_event_audience_request=tim_startup;

    // SESSION MANAGER - Initialization
    wai_oa_session_manager.Initialize(&nh,&sc,F_NODE_SAMPLE_FREQUENCY);
    wai_oa_session_manager.UpdateModel(
                s_path_nodename,
                ros::package::getPath("wai_oa_gazebo")+"/resources/",
                S_SESSION_GROUP,
                S_SESSION_TOPIC,
                S_SESSION_NAME,
                I_SCENE_START_BEFORE,
                I_SESSION_LENGTH,
                I_SESSION_BREAK_LENGTH,
                B_ENABLE_SESSION_SCHEDULER);

    // PRESENTER - Initialization
    //tf_world_wrt_presenter_head=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    wai_oa_presenter.Initialize( &nh,
                                 &it,
                                 s_path_nodename,
                                 wai_oa_session_manager.GetPathResourcesReps(),
                                 &tfb_transforms,
                                 F_NODE_SAMPLE_FREQUENCY,
                                 F_CAMERA_RGBD_RESOLUTION_X,
                                 F_CAMERA_RGBD_RESOLUTION_Y,
                                 F_CAMERA_RGBD_RESOLUTION_SCALE,
                                 F_CAMERA_RGBD_FX,
                                 F_CAMERA_RGBD_FY,
                                 F_CAMERA_RGBD_CX,
                                 F_CAMERA_RGBD_CY,
                                 F_CAMERA_RGBD_RANGE_MIN,
                                 F_CAMERA_RGBD_RANGE_MAX,
                                 F_CAMERA_RGBD_MASK_WIDTH,
                                 F_CAMERA_RGBD_THRESHOLD_DIST,
                                 F_CAMERA_RGBD_THRESHOLD_BOUNDS);

    /* Presence Modes:
     * 0 ... NO Presence
     * 1 ... 2D-AVATAR
     * 2 ... 2D-CAMERA
     * 3 ... 3D-AVATAR (unused)
     * 4 ... 3D-POINTCLOUD
    */
    wai_oa_presenter.UpdateModel(I_PRESENTER_PRESENCE_MODE);
    wai_oa_presenter.SetPresenterPose(F_PRESENTER_POSE_X,F_PRESENTER_POSE_Y,F_PRESENTER_POSE_Z,F_PRESENTER_POSE_YAW);

    // PROJECTOR - Initialization
    wai_oa_projector.Initialize(&nh,
                                &it,
                                &tfb_transforms,
                                F_NODE_SAMPLE_FREQUENCY,
                                wai_oa_session_manager.GetPathResourcesLogo());
    wai_oa_projector.SetProjectionPose(F_PROJECTION_POSE_X,F_PROJECTION_POSE_Y,F_PROJECTION_POSE_Z,F_PROJECTION_POSE_YAW);

    // CAMERA RViz - Initialization
    nh.getParam(s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()+"/setup_session_defaults/camera/default",vec_camera_rviz_default);
    wai_oa_camera_rviz.Initialize( &nh,
                                   &tfb_transforms,
                                   s_path_nodename,
                                   wai_oa_session_manager.GetPathResourcesReps(),
                                   F_NODE_SAMPLE_FREQUENCY,
                                   F_SCENE_TRIGGER_TIMEOUT,
                                   "world",
                                   0);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,
                                   false,
                                   false,
                                   -1,
                                   F_SCENE_TRANSITION_TIMEOUT,
                                   "world",
                                   0,
                                   true);

    // Prepare AUDIENCE and WIM - Initialization
    i_audience_id_selected=0;
    vc3_oa_offset=tf::Vector3(0.0,-2.25,0.0);
    vc3_oa_spacing=tf::Vector3(-2.5,1.5,0.0);
    vc3_oa_offset_wim=vc3_oa_offset+tf::Vector3(0.0,0.0,0.75);
    vc3_oa_spacing_wim=vc3_oa_spacing+tf::Vector3(0.0,0.0,0.0);
    vc3_oa_selected_wim=vc3_oa_offset_wim;

    // AUDIENCE - Initialization
    wai_oa_audience.Initialize( &nh,
                                I_AUDIENCE_COUNT_MAX,
                                I_AUDIENCE_LISTENERS_PER_ROW,
                                vc3_oa_offset,
                                vc3_oa_spacing,
                                wai_oa_session_manager.GetSessionGroup(),
                                EVAL_MODEL_FRACTIONS_MOSAIC,
                                EVAL_PART_FRACTION_SIZE,
                                EVAL_EXAM_FRACTION_SIZE);

    // TELEPROMPTER - Initialization
    if(B_ENABLE_TELEPROMPTER)
    {
        wai_oa_teleprompter.Initialize(&nh,F_NODE_SAMPLE_FREQUENCY,22);
    }

    // SKETCH - Initialization
    wai_oa_sketch.Initialize(&nh,&it,&sc,F_NODE_SAMPLE_FREQUENCY,1.0);

    // TRIGGERS - Initialization
    wai_oa_triggers.Initialize(&nh,this,F_NODE_SAMPLE_FREQUENCY,F_SCENE_TRIGGER_TIMEOUT);

    // SYMBOLS - Initialization
    wai_oa_rep_trigger_scene_prev=WAIReps::create_representative("TRIGGER");
    wai_oa_rep_trigger_scene_prev->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_trigger_scene_prev),"world");
    ((WAIRepTrigger*)wai_oa_rep_trigger_scene_prev)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,-1.0,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "SCENE Previous",col_orange,false,true,F_SCENE_TRIGGER_TIMEOUT);
    wai_oa_rep_trigger_scene_next=WAIReps::create_representative("TRIGGER");
    wai_oa_rep_trigger_scene_next->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_trigger_scene_next),"world");
    ((WAIRepTrigger*)wai_oa_rep_trigger_scene_next)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "SCENE Next",col_green,false,true,F_SCENE_TRIGGER_TIMEOUT);
    wai_oa_rep_trigger_basic=WAIReps::create_representative("TRIGGER");
    wai_oa_rep_trigger_basic->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_trigger_basic),"world");
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "ACTION",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    wai_oa_rep_trigger_audience=WAIReps::create_representative("TRIGGER");
    wai_oa_rep_trigger_audience->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_trigger_audience),"world");
    ((WAIRepTrigger*)wai_oa_rep_trigger_audience)->UpdateModel(
                tf::Vector3(2.0,0.9,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "AUDIENCE",col_red,false,true,F_SCENE_TRIGGER_TIMEOUT);

    wai_oa_rep_graph_3d_function=WAIReps::create_representative("GRAPH_3D");
    wai_oa_rep_graph_3d_function->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_graph_3d_function),"graph_3d_function");
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateModel(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1),tf::Vector3(7.0,7.0,3.5));
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateAxes(0.04,0.04,0.0,0.03, 0.0,6.28,0.0,6.28,-3.0,3.0);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateLabelsAndColors("X","Y","Z","f(x,y)","",col_red,col_green,col_blue);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->InitGrid(14,14,0.5,0.5);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateGrid(0.01,col_white);

    wai_oa_rep_graph_3d_eval=WAIReps::create_representative("GRAPH_3D");
    wai_oa_rep_graph_3d_eval->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_graph_3d_eval),"graph_3d_eval");
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateModel(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1),tf::Vector3((I_AUDIENCE_COUNT_MAX+1)*0.75,(EVAL_SUBPERIODS+1)*0.5,3));
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateAxes(0.75,0.5,0.25,0.1,
                                                            0.0,(I_AUDIENCE_COUNT_MAX+1)*0.75,
                                                            0.0,(EVAL_SUBPERIODS+1)*0.5,
                                                            -3.0,3.0);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateLabelsAndColors("ID","M","Evals","Evaluation","",col_cyan,col_orange,col_oa);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->InitGrid(I_AUDIENCE_COUNT_MAX,12,0.75,0.5);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateGrid(0.01,col_white);
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateGraphCursor(tf::Vector3(0.0,0.0,10.0),"Cursor");

    wai_oa_rep_graph_3d_stats=WAIReps::create_representative("GRAPH_STATS");
    wai_oa_rep_graph_3d_stats->Initialize(&nh,s_path_nodename,GET_OBJECT_NAME(wai_oa_rep_graph_3d_stats),"graph_3d_stats");
    ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->UpdateModel(tf::Vector3(0,0,0),tf::Quaternion(0,0,0,1),tf::Vector3(5.0,3.0,7.0));
    ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->UpdateAxes(1.0,1.0,0.0,0.1, 0.0,4.0,0.0,2.0,0.0,6.0);
    ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->UpdateLabelsAndColors("Cnd","Evl","Min,Max,\nMu,Sigma","STATS","",col_oa_opaque,col_oa_opaque,col_oa_shiny);
    ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->InitGrid(4,2,1.0,1.0);
    ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->UpdateGrid(0.01,col_white);

    // Init default positions of TF vectors
    mrk_oa_arrow_force=WAIRvizMarkers::create_rviz_marker("ARROW");
    mrk_oa_arrow_force->Initialize(s_path_nodename,"world");
    mrk_oa_arrow_force->UpdatePose(tf::Vector3(0.0,0.0,0.0),tf::Quaternion(0,0,0,1),tf::Vector3(0.025,0.05,0.125));


    // NEW STARTUP QUEUE
    //-------------------

    // WIM - Initialization
    wai_oa_rep_wim=WAIReps::create_representative("WIM");
    ((WAIRepWIM*)wai_oa_rep_wim)->InitializeWithAudienceCount(&nh,
                                                             s_path_nodename,
                                                             GET_OBJECT_NAME(wai_oa_rep_wim),
                                                             "wim",
                                                             I_AUDIENCE_COUNT_MAX,
                                                             tf::Vector3(1.0/25.0,1.0/25.0,1.0/25.0));
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                "",
                col_oa_shiny,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                true); // All coords given in true scale!

    for(int i=0;i<I_AUDIENCE_COUNT_MAX;i++) f_oa_audience_stats_ping[i]=0.0;
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateStats(f_oa_audience_stats_ping);
    wai_oa_rep_wim->UpdateView();

    // REP MANAGER - Initialization (takes time to receive spawned reps and details!)
    wai_oa_rep_manager.Initialize(&nh,
                                  s_path_nodename,
                                  wai_oa_session_manager.GetPathResourcesReps(),
                                  F_NODE_SAMPLE_FREQUENCY);
    // Wait couple of seconds after init, using tmr_init_delayed!
    // Gazebo takes some time to spawn all REPs...

    // REP SEQUENCER - Initialization
    wai_oa_rep_sequencer.Initialize(&nh,&wai_oa_rep_manager,F_NODE_SAMPLE_FREQUENCY);

    // MARVIN - Initialization
    wai_oa_marvin.Initialize(&nh,
                             s_path_nodename,
                             &sc,
                             F_NODE_SAMPLE_FREQUENCY,
                             wai_oa_rep_manager.GetRepLinkStates());
    msg_pst_marvin_ref_home.header.frame_id="world";
    msg_pst_marvin_ref_home.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_home.pose.position.x=2.5;
    msg_pst_marvin_ref_home.pose.position.y=0.0;
    msg_pst_marvin_ref_home.pose.position.z=0.75;
    msg_pst_marvin_ref_home.pose.orientation.w=0.0;
    msg_pst_marvin_ref_home.pose.orientation.x=0.0;
    msg_pst_marvin_ref_home.pose.orientation.y=0.0;
    msg_pst_marvin_ref_home.pose.orientation.z=1.0;
    msg_pst_marvin_ref_space.header.frame_id="world";
    msg_pst_marvin_ref_space.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_space.pose.position.x=2.5;
    msg_pst_marvin_ref_space.pose.position.y=0.5;
    msg_pst_marvin_ref_space.pose.position.z=2.5;
    msg_pst_marvin_ref_space.pose.orientation.w=0.0;
    msg_pst_marvin_ref_space.pose.orientation.x=0.0;
    msg_pst_marvin_ref_space.pose.orientation.y=0.0;
    msg_pst_marvin_ref_space.pose.orientation.z=1.0;
    msg_pst_marvin_ref_introduction.header.frame_id="world";
    msg_pst_marvin_ref_introduction.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_introduction.pose.position.x=1.5;
    msg_pst_marvin_ref_introduction.pose.position.y=3.0;
    msg_pst_marvin_ref_introduction.pose.position.z=1.0;
    msg_pst_marvin_ref_introduction.pose.orientation.w=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.x=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.y=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.z=1.0;

    // TRIGGERS - Init virtual triggers for hand interaction
    wai_oa_rep_trigger_scene_prev->UpdateView();
    wai_oa_rep_trigger_scene_next->UpdateView();
    wai_oa_rep_trigger_basic->UpdateView();
    wai_oa_rep_trigger_audience->UpdateView();

    // TIMERS Initialization
    tmr_init_delayed=nh.createTimer(ros::Duration(5.0),&WAIOpenAuditorium::cb_tmr_init_delayed,this,true,true);
    tmr_long_interval=nh.createTimer(ros::Duration(10.0),&WAIOpenAuditorium::cb_tmr_long_interval,this,false,true);
    tmr_setup_force_view=nh.createTimer(ros::Duration(1.0/F_NODE_SAMPLE_FREQUENCY),&WAIOpenAuditorium::cb_tmr_setup_force_view,this,false,false);

    // CAMERAs Initialization
    // Left as example code to dynamically change parameters during runtime (e.g., camera settings)
    //int i_sys_rval=system("rosrun dynamic_reconfigure dynparam set /wai_world/oa/camera_rgbd_in/driver ir_mode 8 &");
    //int i_sys_rval=system("rosrun dynamic_reconfigure dynparam set /wai_world/oa/camera_rgbd_in/driver color_mode 8 &");
    //int i_sys_rval=system("rosrun dynamic_reconfigure dynparam set /wai_world/oa/camera_rgbd_in/driver depth_mode 8 &");

    // --- Previously triggered by delayed startup-timer ---
    // wai_oa_rep_manager.FinalizeRepsDetailsAvailable();

    // SESSION MANAGER POST-Initialization
    // "Connect" other REPs, after they were initialized properly and enable updating status label
    wai_oa_session_manager.ConnectRepsAndLoadSession(&wai_oa_rep_manager,
                                                     &wai_oa_audience,
                                                     &wai_oa_presenter,
                                                     &wai_oa_projector,
                                                     &wai_oa_camera_rviz,
                                                     wai_oa_rep_wim);
    wai_oa_session_manager.EnableUpdateSessionStatusLabel();

    PlaySound("IntelWelcome"); // Intel welcomes Presenter and Audience
}



/////////////////////////////////////////////////
/// Callbacks for timers
/////////////////////////////////////////////////

void WAIOpenAuditorium::cb_tmr_init_delayed(const ros::TimerEvent& event)
{
    wai_oa_projector.UpdateModel(cv::Mat(),"");
    wai_oa_projector.UpdateView("");
    SendInfoToAudience("PRE-WORKSPACE READY"); // Notify audience
    SpinAndWaitForSeconds(2.0);
    wai_oa_marvin.Engage(msg_pst_marvin_ref_home); // Let Marvin takeoff
    SpinAndWaitForSeconds(5.0);
    PlaySound("IntelWelcomeDone"); // Let Intel report that startup is done
    B_EVENT_STARTUP_FINISHED=true; // Signal startup finished
}

void WAIOpenAuditorium::cb_tmr_long_interval(const ros::TimerEvent& event)
{
    for(int i=0;i<I_AUDIENCE_COUNT_MAX;i++)
    {
        f_oa_audience_stats_ping[i]=wai_oa_audience.GetStatsAudienceID(i);
    }
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateStats(f_oa_audience_stats_ping);
    wai_oa_rep_wim->UpdateView();

    for(int j=0;j<I_AUDIENCE_COUNT_MAX;j++)
    {
        wai_oa_audience.ResetStatsAudienceID(j);
        wai_oa_audience.GetAudienceID(j).SendPing();
    }

    // Update basic trigger markers (audience/listeners rejoining etc.)
    wai_oa_rep_trigger_scene_prev->UpdateView();
    wai_oa_rep_trigger_scene_next->UpdateView();
    wai_oa_rep_trigger_basic->UpdateView();
    wai_oa_rep_trigger_audience->UpdateView();
}

void WAIOpenAuditorium::cb_tmr_setup_force_view(const ros::TimerEvent& event)
{
        vc3_world_wrt_force_endpoint=wai_oa_presenter.GetHandRightPosition();
        mrk_oa_arrow_force->UpdateColor(col_oa_opaque);
        ((ArrowMarker*)mrk_oa_arrow_force)->UpdateVector(vc3_world_wrt_force_origin,vc3_world_wrt_force_endpoint);
        pub_mrk_oa_arrow_force.publish(mrk_oa_arrow_force->GetMarker());

        float f_force_mag=(vc3_world_wrt_force_endpoint-vc3_world_wrt_force_origin).length()*10.0;
        int i_force_mag=int(f_force_mag);
        std::stringstream sst_force_mag;
        sst_force_mag << "F ("<<s_force_name<< "): "<< i_force_mag << "N";
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    sst_force_mag.str(),col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
        wai_oa_rep_trigger_basic->UpdateView();
}



/////////////////////////////////////////////////
/// Callback to receive captured audio data
/////////////////////////////////////////////////
/*
void WAIOpenAuditorium::cb_sub_aud_audio_captured_presenter(const audio_common_msgs::AudioDataPtr& msg)
{
    for(int i=0;i<I_AUDIENCE_COUNT_MAX;i++)
    {
        vec_pub_aud_audio_captured_presenter[i].publish(*msg);
    }
}
*/



/////////////////////////////////////////////////
/// Helper methods
/////////////////////////////////////////////////
std::string WAIOpenAuditorium::RemoveNumbers(std::string s_string)
{
    size_t first_index = s_string.find_first_not_of("0123456789");
    return s_string.substr(first_index);
}

void WAIOpenAuditorium::SetupLogAudienceIDParticipationModel(int i_log_part)
{
    QMessageBox::warning(NULL,"EVALUATION Disclaimer","<b>Before you continue, please keep in mind that this is open source and untested software!</b><br>Under any circumstances, the authors <i>DO NOT GUARANTEE FOR CORRECTNESS and ARE NOT RESPONSIBLE OF<br></i>partial results, overall results, or any derived results, from this evaluation!<br>Thus, the authors <b>strictly do not recommend to use this feature of OA on its own</b>,<br>particularly for grading! Additionally, under any circumstances, USE YOUR OWN DATA and DERIVE<br>YOUR OWN RESULTS of any evaulation as a sanity check!");

    std::string s_part_score="cancel";
    std::string s_comment="no_comment";

    // i_audience_id_selected --> Idx. starts at zero!
    if(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalModel()==EVAL_MODEL_CALCULATOR)
    {
        if(i_log_part==0)
        {
            s_part_score="0.0";
        }
        else if(i_log_part==1)
        {
            s_part_score="0.5";
        }
        else if(i_log_part==2)
        {
            s_part_score="1.0";
        }
        else
        {
            ROS_DEBUG("Participation log: Invalid trigger!");
            return;
        }
    }
    else if(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalModel()==EVAL_MODEL_FRACTIONS_MOSAIC)
    {
        if(i_log_part==3) //Log PART SCORE
        {
            bool ok_score=false;
            bool ok_comment=false;

            do
            {
                double d_eval = QInputDialog::getDouble(
                    NULL,
                    QString::fromStdString("EVAL Participation [ID"+std::to_string(i_audience_id_selected)+"]"),
                    QString::fromStdString("Input score (Max. "+std::to_string(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax())+"):"),
                    0.0,
                    0.0,
                    wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax(),
                    2,
                    &ok_score,
                    Qt::WindowFlags());
                s_part_score=std::to_string(d_eval);

                QString qst_comment=QInputDialog::getText(
                    NULL,
                    QString::fromStdString("EVAL Participation [ID"+std::to_string(i_audience_id_selected)+"]"),
                    "Comment:",
                    QLineEdit::Normal,
                    "no_comment",
                    &ok_comment,
                    Qt::WindowFlags());
                s_comment=qst_comment.toStdString();
                std::replace(s_comment.begin(),s_comment.end(),' ', '_');

            }while(!ok_score || !ok_comment);

            int ret = QMessageBox::warning(NULL,QString::fromStdString("EVAL Participation [ID"+std::to_string(i_audience_id_selected)+"]"),
                                           "Do your really want to log the evaluation?",
                                            QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);
            if(ret==QMessageBox::No)
            {
                s_part_score="cancel";
            }
            else
            {
                // Do nothing...
            }
        }
        else if(i_log_part==0)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax()*0.0;
            s_part_score=sst_f_to_s.str();

            bool ok_comment=false;
            QString qst_comment=QInputDialog::getText(
                NULL,
                QString::fromStdString("EVAL Participation MINUS [ID"+std::to_string(i_audience_id_selected)+"]"),
                "Comment:",
                QLineEdit::Normal,
                "no_comment",
                &ok_comment,
                Qt::WindowFlags());
            s_comment=qst_comment.toStdString();
            std::replace(s_comment.begin(),s_comment.end(),' ', '_');
        }
        else if(i_log_part==1)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax()*0.6875;
            s_part_score=sst_f_to_s.str();

            bool ok_comment=false;
            QString qst_comment=QInputDialog::getText(
                NULL,
                QString::fromStdString("EVAL Participation TILDE [ID"+std::to_string(i_audience_id_selected)+"]"),
                "Comment:",
                QLineEdit::Normal,
                "no_comment",
                &ok_comment,
                Qt::WindowFlags());
            s_comment=qst_comment.toStdString();
            std::replace(s_comment.begin(),s_comment.end(),' ', '_');
        }
        else if(i_log_part==2)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax()*1.0;
            s_part_score=sst_f_to_s.str();

            bool ok_comment=false;
            QString qst_comment=QInputDialog::getText(
                NULL,
                QString::fromStdString("EVAL Participation PLUS [ID"+std::to_string(i_audience_id_selected)+"]"),
                "Comment:",
                QLineEdit::Normal,
                "no_comment",
                &ok_comment,
                Qt::WindowFlags());
            s_comment=qst_comment.toStdString();
            std::replace(s_comment.begin(),s_comment.end(),' ', '_');
        }
        else
        {
            ROS_DEBUG("Participation log: Invalid trigger!");
            return;
        }
    }
    else
    {
        ROS_DEBUG("Participtation log: Invalid eval model!");
        return;
    }

    // Inform audience
    wai_oa_audience.GetAudienceID(i_audience_id_selected).SendInfo("EVAL Participation:"+s_part_score);

    // Actually log PARTICIPATION to file
    if(!(s_part_score.compare("cancel")==0))
    {
        std::ofstream f_out_log_aud_id;
        f_out_log_aud_id.open(wai_oa_session_manager.GetPathResourcesEvalsGroupAndSubject().c_str() , std::ios::out | std::ios::app);
        std::string s_separator=";";

        // Get weekday
        std::string s_weekday="invalid";
        std::time_t t = std::time(0);   // get time now
        std::tm* now = std::localtime(&t);
        if(now->tm_wday==0) s_weekday="sunday";
        else if(now->tm_wday==1) s_weekday="monday";
        else if(now->tm_wday==2) s_weekday="tuesday";
        else if(now->tm_wday==3) s_weekday="wednesday";
        else if(now->tm_wday==4) s_weekday="thursday";
        else if(now->tm_wday==5) s_weekday="friday";
        else if(now->tm_wday==6) s_weekday="saturday";
        else ROS_DEBUG_STREAM("Participation log: Invalid weekday!");

        // Get calendar week
        int i_calendar_week=getWeekNumber();

        // Write to outfile stream
        f_out_log_aud_id
            << (now->tm_year + 1900) << s_separator // YEAR
            << (now->tm_mon + 1) << s_separator // MONTH
            << now->tm_mday << s_separator // DAY
            << i_calendar_week << s_separator // CALENDAR WEEK
            << s_weekday << s_separator// WEEKDAY
            << now->tm_hour << s_separator // HOUR
            << now->tm_min << s_separator // MINUTE
            << now->tm_sec << s_separator // SECOND
            << i_audience_id_selected << s_separator // ID
            << wai_oa_audience.GetAliasID(i_audience_id_selected) << s_separator // ALIAS
            << wai_oa_session_manager.GetSessionGroup() << s_separator // GROUP
            << wai_oa_session_manager.GetSessionTopic() << s_separator // TOPIC
            << wai_oa_session_manager.GetSessionExpertise() << s_separator // EXPERTISE
            << wai_oa_session_manager.GetSessionExpertiseLevel() << s_separator // EXPERTISE LEVEL
            << wai_oa_session_manager.GetSessionName() << s_separator // SESSION NAME
            << "part" << s_separator // EVAL TYPE
            << "eval_label" << s_separator // EVAL LABEL
            << s_part_score << s_separator // EVAL SCORE
            << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax() << s_separator // EVAL SCORE MAX
            << std::atof(s_part_score.c_str())/wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartScoreMax() << s_separator // RELATION
            << s_comment << s_separator // COMMENT
            << std::endl;
        f_out_log_aud_id.close();
    }

    // Update data of Cursor label
    sst_wai_oa_rep_wim_cursor_label.str("");
    sst_wai_oa_rep_wim_cursor_label << "ID" << i_audience_id_selected << "-Part: " << s_part_score;
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                sst_wai_oa_rep_wim_cursor_label.str(),
                col_red,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                false); // All coords given in true scale!
}
void WAIOpenAuditorium::SetupLogAudienceIDParticipationView()
{
    // Setup camera on WIM
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,true,8,F_SCENE_TRIGGER_TIMEOUT);
    wai_oa_camera_rviz.UpdateView();

    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                sst_wai_oa_rep_wim_cursor_label.str(),
                col_red,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                false); // All coords given in true scale!
    wai_oa_rep_wim->UpdateView();
}



void WAIOpenAuditorium::SetupLogAudienceIDExaminationModel(int i_log_exam)
{
    QMessageBox::warning(NULL,"EVALUATION Disclaimer","<b>Before you continue, please keep in mind that this is open source and untested software!</b><br>Under any circumstances, the authors <i>DO NOT GUARANTEE FOR CORRECTNESS and ARE NOT RESPONSIBLE OF</i><br>partial results, overall results, or any derived results, from this evaluation!<br>Thus, the authors <b>strictly do not recommend to use this feature of OA on its own, particularly for grading!<br>Additionally, under any circumstances, USE YOUR OWN DATA and DERIVE<br>YOUR OWN RESULTS of any evaulation as a sanity check!");

    std::string s_exam_score="cancel";
    std::string s_comment="no_comment";

    if(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalModel()==EVAL_MODEL_CALCULATOR)
    {
        if(i_log_exam==0)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.4375;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==1)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.5625;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==2)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.6875;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==3)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.8125;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==4)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.9375;
            s_exam_score=sst_f_to_s.str();
        }
        else
        {
            ROS_DEBUG("Examination log: Invalid trigger!");
            return;
        }
    }
    else if(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalModel()==EVAL_MODEL_FRACTIONS_MOSAIC)
    {
        if(i_log_exam==0)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.4375;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==1)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.5625;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==2)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.6875;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==3)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.8125;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==4)
        {
            std::stringstream sst_f_to_s;
            sst_f_to_s << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax()*0.9375;
            s_exam_score=sst_f_to_s.str();
        }
        else if(i_log_exam==5)
        {
            bool ok_score=false;
            bool ok_comment=false;

            do
            {
                double d_eval = QInputDialog::getDouble(
                    NULL,
                    QString::fromStdString("EVAL Examination [ID"+std::to_string(i_audience_id_selected)+"]"),
                    QString::fromStdString("Input score (Max. "+std::to_string(wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax())+"):"),
                    0.0,
                    0.0,
                    wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax(),
                    2,
                    &ok_score,
                    Qt::WindowFlags());
                s_exam_score=std::to_string(d_eval);

                QString qst_comment=QInputDialog::getText(
                    NULL,
                    QString::fromStdString("EVAL Examination [ID"+std::to_string(i_audience_id_selected)+"]"),
                    "Comment:",
                    QLineEdit::Normal,
                    "no_comment",
                    &ok_comment,
                    Qt::WindowFlags());
                s_comment=qst_comment.toStdString();
                std::replace(s_comment.begin(),s_comment.end(),' ', '_');

            }while(!ok_score || !ok_comment);

            int ret = QMessageBox::warning(NULL,QString::fromStdString("EVAL Examination [ID"+std::to_string(i_audience_id_selected)+"]"),
                                           "Do your really want to log the evaluation?",
                                            QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);
            if(ret==QMessageBox::No)
            {
                s_exam_score="cancel";
            }
            else
            {
                // Do nothing...
            }
        }
        else
        {
            ROS_DEBUG("Examination log: Invalid trigger!");
            return;
        }
    }
    else
    {
        ROS_DEBUG("Examination log: Invalid eval model!");
        return;
    }

    // Inform audience/listener
    wai_oa_audience.GetAudienceID(i_audience_id_selected).SendInfo("EVAL Examination:"+s_exam_score);

    // Actually log EXAMINATION to file
    if(!(s_exam_score.compare("cancel")==0))
    {
        std::ofstream f_out_log_aud_id;
        f_out_log_aud_id.open((wai_oa_session_manager.GetPathResourcesEvalsGroupAndSubject()).c_str() , std::ios::out | std::ios::app);
        std::string s_separator=";";

        // Get weekday
        std::string s_weekday="invalid";
        std::time_t t = std::time(0);   // get time now
        std::tm* now = std::localtime(&t);
        if(now->tm_wday==0) s_weekday="sunday";
        else if(now->tm_wday==1) s_weekday="monday";
        else if(now->tm_wday==2) s_weekday="tuesday";
        else if(now->tm_wday==3) s_weekday="wednesday";
        else if(now->tm_wday==4) s_weekday="thursday";
        else if(now->tm_wday==5) s_weekday="friday";
        else if(now->tm_wday==6) s_weekday="saturday";
        else ROS_DEBUG_STREAM("Participation log: Invalid weekday!");

        // Get calendar week
        int i_calendar_week=-1;
        constexpr int DAYS_PER_WEEK = 7 ;
        const int wday = now->tm_wday ;
        const int delta = wday ? wday-1 : DAYS_PER_WEEK-1 ;
        i_calendar_week=( now->tm_yday + DAYS_PER_WEEK - delta ) / DAYS_PER_WEEK ;
        i_calendar_week=i_calendar_week-1; // Corrected

        // Write to outfile stream
        f_out_log_aud_id
            << (now->tm_year + 1900) << s_separator // YEAR
            << (now->tm_mon + 1) << s_separator // MONTH
            << now->tm_mday << s_separator // DAY
            << i_calendar_week << s_separator // CALENDAR WEEK
            << s_weekday << s_separator// WEEKDAY
            << now->tm_hour << s_separator // HOUR
            << now->tm_min << s_separator // MINUTE
            << now->tm_sec << s_separator // SECOND
            << i_audience_id_selected << s_separator // ID
            << wai_oa_audience.GetAliasID(i_audience_id_selected) << s_separator // ALIAS
            << wai_oa_session_manager.GetSessionGroup() << s_separator // GROUP
            << wai_oa_session_manager.GetSessionTopic() << s_separator // TOPIC
            << wai_oa_session_manager.GetSessionExpertise() << s_separator // EXPERTISE
            << wai_oa_session_manager.GetSessionExpertiseLevel() << s_separator // EXPERTISE LEVEL
            << wai_oa_session_manager.GetSessionName() << s_separator // SESSION NAME
            << "exam" << s_separator // EVAL TYPE
            << "eval_label" << s_separator // EVAL LABEL
            << s_exam_score << s_separator // EVAL SCORE
            << wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax() << s_separator // EVAL SCORE MAX
            << std::atof(s_exam_score.c_str())/wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalExamScoreMax() << s_separator // RELATION
            << s_comment << s_separator // COMMENT
            << std::endl;
        f_out_log_aud_id.close();
    }

    // Update WIM cursor label
    sst_wai_oa_rep_wim_cursor_label.str("");
    sst_wai_oa_rep_wim_cursor_label << "ID" << i_audience_id_selected << "-Exam: " << s_exam_score;
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                sst_wai_oa_rep_wim_cursor_label.str(),
                col_red,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                false); // All coords given in true scale!
    wai_oa_rep_wim->UpdateView();
}
void WAIOpenAuditorium::SetupLogAudienceIDExaminationView()
{
    // Setup camera on WIM
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,true,8,F_SCENE_TRIGGER_TIMEOUT);
    wai_oa_camera_rviz.UpdateView();

    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                sst_wai_oa_rep_wim_cursor_label.str(),
                col_red,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                false); // All coords given in true scale!
    wai_oa_rep_wim->UpdateView();
}


int WAIOpenAuditorium::SetupWIMAudienceNumberModel()
{
    QVBoxLayout* vbox = new QVBoxLayout();

    QDialog* d = new QDialog();
    d->setWindowTitle("AUDIENCE Number");
    QLabel* lbl_audience_number=new QLabel();
    lbl_audience_number->setText("Select an Audience Listener by number:");
    QLabel* lbl_audience_number_preview=new QLabel();
    img_audience_number_preview.load(QString::fromStdString(wai_oa_session_manager.GetPathResourcesLogo()));
    lbl_audience_number_preview->setPixmap(QPixmap::fromImage(img_audience_number_preview.scaled(256,144)));

    QSpinBox* spb_audience_number = new QSpinBox();
    spb_audience_number->setMinimum(0);
    spb_audience_number->setMaximum(wai_oa_audience.GetAudienceCountMax()-1);
    spb_audience_number->setSingleStep(1);
    spb_audience_number->setValue(0);

    QObject::connect(spb_audience_number,QOverload<int>::of(&QSpinBox::valueChanged),[=](int i_audience_number)
    {
        std::string s_audience_number_preview_img_path=wai_oa_session_manager.GetPathResourcesAliasesGroupAndIDPreview()+"id_"+std::to_string(i_audience_number)+".png";
        img_audience_number_preview.load(QString::fromStdString(s_audience_number_preview_img_path));
        lbl_audience_number_preview->setPixmap(QPixmap::fromImage(img_audience_number_preview.scaled(256,144)));
    });

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
    vbox->addWidget(lbl_audience_number);
    vbox->addWidget(lbl_audience_number_preview);
    vbox->addWidget(spb_audience_number);
    vbox->addWidget(buttonBox);
    d->setLayout(vbox);
    int result = d->exec();
    int i_audience_id_by_number=spb_audience_number->value();

    if(result == QDialog::Accepted)
    {
        return i_audience_id_by_number;
    }
    else
    {
        return i_audience_id_selected;
    }
}

void WAIOpenAuditorium::SetupWIMAudienceSelectionModel(bool b_enable_screenshare)
{
    ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                "WIM",
                "",
                col_green,
                vc3_oa_offset_wim,
                tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                vc3_oa_spacing_wim,
                vc3_oa_selected_wim,
                i_audience_id_selected,
                S_PRESENTER_WORKSPACE_MODEL,
                wai_oa_session_manager.GetPathResourcesReps(),
                false); // All coords given in true scale!

    if(b_enable_screenshare==true)
    {
        // Use rqt_image_view instance to show listeners screen
        std::string s_sys_cmd="rqt_image_view /wai_world/oa"+std::to_string(i_audience_id_selected)+"/screenshare/image_raw/theora &";
        int retval=system(s_sys_cmd.c_str());
    }
}
void WAIOpenAuditorium::SetupWIMAudienceSelectionView()
{
    wai_oa_rep_wim->UpdateView();
}
void WAIOpenAuditorium::SetupWIMAudienceSelectionLeaveState()
{
    // Cleanup instances of rqt_imgage_view
    std::string s_sys_cmd="killall rqt_image_view";
    int retval=system(s_sys_cmd.c_str());

    SetupCameraRvizRestorePreviousView();
}

void WAIOpenAuditorium::SetupWIMAudienceMessageModel()
{
    bool ok_aud_msg=false;
    QString qst_audience_msg;
    do
    {
        qst_audience_msg=QInputDialog::getText(
            NULL,
            "MESSAGE",
            "Input a short message\n(Up to 100 characters!):",
            QLineEdit::Normal,
            "Type in your message here...",
            &ok_aud_msg,
            Qt::WindowFlags());
    }while(qst_audience_msg.length()>100);

    if(ok_aud_msg)
    {
        QMessageBox::information(NULL,"MESSAGE","The message will be sent to the Audience!");
        SendInfoToAudience("MESSAGE:"+qst_audience_msg.toStdString());
        SendInfoToOAPanel("PRE-MESSAGE:"+qst_audience_msg.toStdString());

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "MESSAGE",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        QMessageBox::information(NULL,"MESSAGE","The message will be dismissed!");

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "MESSAGE (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
}
void WAIOpenAuditorium::SetupWIMAudienceMessageView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetUpWIMAudienceListenerMessageModel()
{
    bool ok_aud_msg=false;
    QString qst_audience_msg;
    do
    {
        qst_audience_msg=QInputDialog::getText(
            NULL,
            "MESSAGE TO LISTENER",
            "Input a short message\n(Up to 100 characters!):",
            QLineEdit::Normal,
            "Type in your message here...",
            &ok_aud_msg,
            Qt::WindowFlags());
    }while(qst_audience_msg.length()>100);

    if(ok_aud_msg)
    {
        QMessageBox::information(NULL,QString::fromStdString("MESSAGE TO LISTENER (ID"+std::to_string(i_audience_id_selected)+")"),"The message will be sent to this specific Listener in the Audience!");
        SendInfoToAudienceID(i_audience_id_selected,"MESSAGE:"+qst_audience_msg.toStdString());
        //SendInfoToOAPanel("PRE-MSG LIST:"+qst_audience_msg.toStdString());

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "MSG LIST",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        QMessageBox::information(NULL,QString::fromStdString("MESSAGE TO LISTENER (ID"+std::to_string(i_audience_id_selected)+")"),"The message will be dismissed!");

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "MSG LIST (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
}
void WAIOpenAuditorium::SetUpWIMAudienceListenerMessageView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupWIMAudienceKickModel()
{
    int ret = QMessageBox::warning(NULL,QString::fromStdString("KICK (ID"+std::to_string(i_audience_id_selected)+")"),
                                   "Do your really want to kick\nthis listener from the audience?",
                                    QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);
    if(ret==QMessageBox::Yes)
    {
        std::string s_command="rosnode kill /wai_world/oa"+std::to_string(i_audience_id_selected)+"/wai_oa_audience_listener_control_center_node &";
        int i_sys_rval=system(s_command.c_str());

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "KICK (ID"+std::to_string(i_audience_id_selected)+")",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {   
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "KICK (ID"+std::to_string(i_audience_id_selected)+") (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
}
void WAIOpenAuditorium::SetupWIMAudienceKickView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}


void WAIOpenAuditorium::SetupWIMRepSelectionModel()
{
    wai_oa_rep_manager.EnableRepAndDetail();
}
void WAIOpenAuditorium::SetupWIMRepSelectionView()
{
    // Do nothing...
}
void WAIOpenAuditorium::SetupWIMRepSelectionLeaveState()
{
    wai_oa_rep_manager.DisableRepAndDetail();
    SetupCameraRvizRestorePreviousView();
}



/////////////////////////////////////////////////
/// OTHER helper methods
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetAudienceRequestIncoming(std_msgs::Header hea_audience_request_incoming)
{
    msg_hea_audience_request_incoming=hea_audience_request_incoming;
}
void WAIOpenAuditorium::SendInfoToAudienceID(int i_id,std::string s_msg)
{
    wai_oa_audience.GetAudienceID(i_id).SendInfo(s_msg);
}
void WAIOpenAuditorium::SendInfoToAudience(std::string s_msg)
{
    for(int i=0;i<wai_oa_audience.GetAudienceCountMax();i++)
    {
        wai_oa_audience.GetAudienceID(i).SendInfo(s_msg); // ID info added by audience
    }
}


std_msgs::Header WAIOpenAuditorium::GetAudienceRequestIncoming()
{
    return msg_hea_audience_request_incoming;
}

void WAIOpenAuditorium::SetRvizPointClicked(geometry_msgs::PointStamped pts_rviz_clicked)
{
    msg_pts_rviz_clicked=pts_rviz_clicked;
}

void WAIOpenAuditorium::SpinAndWaitForCondition(bool* b_cond,bool b_state)
{
    ros::Rate r_sleep(F_NODE_SAMPLE_FREQUENCY);
    while(*b_cond != b_state)
    {
        ros::spinOnce();
        r_sleep.sleep();
    }
}

void WAIOpenAuditorium::SpinAndWaitForSeconds(float f_seconds)
{
    ros::Rate r_sleep(F_NODE_SAMPLE_FREQUENCY);
    ros::Time tim_start=ros::Time::now();
    while((ros::Time::now()-tim_start).toSec()<f_seconds)
    {
        ros::spinOnce();
        r_sleep.sleep();
    }
}

std::string WAIOpenAuditorium::GetStateNameCurrent()
{
    //return RemoveNumbers(typeid(*this->wai_oa_state).name());

    std::string s_current_state=RemoveNumbers(typeid(*this->wai_oa_state).name());
    std::string s_erase="State";
    size_t pos = s_current_state.find(s_erase);
    if (pos != std::string::npos)
    {
        s_current_state.erase(pos,s_erase.length());
    }
    return s_current_state;
}
std::string WAIOpenAuditorium::GetStateNameCurrentTTS()
{
    std::string s_state_name_tts=RemoveNumbers(typeid(*this->wai_oa_state).name());
    std::string s_erase="State";
    size_t pos = s_state_name_tts.find(s_erase);
    if (pos != std::string::npos)
    {
        s_state_name_tts.erase(pos,s_erase.length());
    }
    //s_state_name_tts=s_state_name_tts+"TTS";
    s_state_name_tts="Intel"+s_state_name_tts;
    return s_state_name_tts;
}

std::string WAIOpenAuditorium::GetEventNameInpDevLast()
{
    return wai_oa_triggers.GetTriggersLast();
}

std::string WAIOpenAuditorium::GetCameraFrameNames()
{
    return wai_oa_camera_rviz.GetCurrentFrame();
}

std::string WAIOpenAuditorium::GetRepSelectedClean()
{
    return wai_oa_rep_manager.GetRepSelectedClean();
}

std::string WAIOpenAuditorium::GetRepDetailSelectedClean()
{
    return wai_oa_rep_manager.GetRepDetailSelectedClean();
}
std::string WAIOpenAuditorium::CleanupStringRep(std::string s_string)
{
    std::string s_string_cleanedup;
    s_string_cleanedup=s_string;

    if(s_string_cleanedup.length()>0)
    {
        // Do not remove the first term "link_...", like for the details since its not given typically!

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
        s_string_cleanedup="NONE";
    }
    return s_string_cleanedup;
}

bool WAIOpenAuditorium::ToggleTriggerLectern()
{
    return B_ENABLE_LECTERN=!B_ENABLE_LECTERN;
}
bool WAIOpenAuditorium::ToggleTriggerTable()
{
    return B_ENABLE_TABLE=!B_ENABLE_TABLE;
}
bool WAIOpenAuditorium::ToggleTriggerAudio()
{
    return B_ENABLE_AUDIO=!B_ENABLE_AUDIO;
}


void WAIOpenAuditorium::ShutdownMarvin()
{
    wai_oa_marvin.Shutdown();
}
void WAIOpenAuditorium::EngageMarvin()
{
    wai_oa_marvin.Engage(msg_pst_marvin_ref_home);
}

void WAIOpenAuditorium::UpdateWIMFromAudienceSelected()
{
    vc3_oa_selected_wim=
            wai_oa_audience.GetAudiencePosFromID(i_audience_id_selected)
            +tf::Vector3(0.0,0.0,0.75);
}
void WAIOpenAuditorium::UpdateAudienceIDSelected(int i_id_select)
{
    if(i_id_select>=0) // Directly select/update ID (IDs start at index 0!)
    {
        i_audience_id_selected=i_id_select;
    }
    else if(i_id_select==-1)
    {
        if(i_audience_id_selected<=(I_AUDIENCE_COUNT_MAX-1-I_AUDIENCE_LISTENERS_PER_ROW)) i_audience_id_selected+=I_AUDIENCE_LISTENERS_PER_ROW;
        else i_audience_id_selected=I_AUDIENCE_COUNT_MAX-1;
    }
    else if(i_id_select==-2)
    {
        if(i_audience_id_selected>(I_AUDIENCE_LISTENERS_PER_ROW-1)) i_audience_id_selected-=I_AUDIENCE_LISTENERS_PER_ROW;
        else i_audience_id_selected=0;
    }
    else if(i_id_select==-3)
    {
        if(i_audience_id_selected<I_AUDIENCE_COUNT_MAX-1) i_audience_id_selected++;
        else;
    }
    else if(i_id_select==-4)
    {
        if(i_audience_id_selected>0) i_audience_id_selected--;
        else;
    }
    else
    {
        // Do nothing...
    }
}


void WAIOpenAuditorium::UpdateRepAndDetailSelected(bool b_use_dialog,std::string s_rep_select,std::string s_detail_select)
{
    if(b_use_dialog==true)
    {
        /*
        QMessageBox::information(NULL,"REP DETAILS Select","Updating index of REPs and DETAILS!\nThis will take a couple of seconds...");
        // Create select dialog for REPS
        QDialog* dlg_rep_select=new QDialog();
        dlg_rep_select->setWindowTitle("REP Select");
        QLabel* lbl_rep_select=new QLabel();
        lbl_rep_select->setText("Select a REP:");
        QVBoxLayout* lay_box_rep_select=new QVBoxLayout();
        QComboBox* cmb_rep_select=new QComboBox();

        std::vector<std::string> vec_s_rep_select_labels;
        vec_s_rep_select_labels=wai_oa_rep_manager.GetCurrentRepsIndex();
        for(int i=0;i<vec_s_rep_select_labels.size();i++)
        {
            cmb_rep_select->addItem(QString::fromStdString(vec_s_rep_select_labels[i]));
        }
        cmb_rep_select->model()->sort(0,Qt::AscendingOrder);

        QDialogButtonBox* btn_box_rep_select=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
        QObject::connect(btn_box_rep_select,SIGNAL(accepted()),dlg_rep_select,SLOT(accept()));
        QObject::connect(btn_box_rep_select,SIGNAL(rejected()),dlg_rep_select,SLOT(reject()));
        lay_box_rep_select->addWidget(lbl_rep_select);
        lay_box_rep_select->addWidget(cmb_rep_select);
        lay_box_rep_select->addWidget(btn_box_rep_select);
        dlg_rep_select->setLayout(lay_box_rep_select);
        int res_rep_select=dlg_rep_select->exec();
        */


        // Create select dialog for DETAILS
        QDialog* dlg_rep_detail_select=new QDialog();
        dlg_rep_detail_select->setWindowTitle("REP DETAILS Select");
        QLabel* lbl_rep_detail_select=new QLabel();
        lbl_rep_detail_select->setText("Select a DETAIL:");
        QVBoxLayout* lay_box_vertical=new QVBoxLayout();
        QComboBox* cmb_rep_detail_select=new QComboBox();

        std::vector<std::string> vec_s_rep_detail_select_labels;
        vec_s_rep_detail_select_labels=wai_oa_rep_manager.GetCurrentRepsDetailsIndex();
        for(int i=0;i<vec_s_rep_detail_select_labels.size();i++)
        {
            cmb_rep_detail_select->addItem(QString::fromStdString(vec_s_rep_detail_select_labels[i]));
        }
        cmb_rep_detail_select->model()->sort(0,Qt::AscendingOrder);

        QDialogButtonBox* btn_box_rep_detail_select=new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
        QObject::connect(btn_box_rep_detail_select,SIGNAL(accepted()),dlg_rep_detail_select,SLOT(accept()));
        QObject::connect(btn_box_rep_detail_select,SIGNAL(rejected()),dlg_rep_detail_select,SLOT(reject()));
        lay_box_vertical->addWidget(lbl_rep_detail_select);
        lay_box_vertical->addWidget(cmb_rep_detail_select);
        lay_box_vertical->addWidget(btn_box_rep_detail_select);
        dlg_rep_detail_select->setLayout(lay_box_vertical);
        int res_rep_detail_select=dlg_rep_detail_select->exec();

        // Update REP and DETAIL selection
        if(res_rep_detail_select==QDialog::Accepted)
        {
            std::string s_selection=cmb_rep_detail_select->currentText().toStdString();
            std::size_t pos=s_selection.find("/");
            if(pos!=std::string::npos)
            {
                s_rep_select=s_selection.substr(0,pos);
                s_detail_select=s_selection.substr(pos+1);
            }
            else
            {
                s_rep_select="workspace_presenter";
                s_detail_select="link_base";
            }

            ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                        tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.025,0.25,0.125),
                        "REP Select",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
        }
        else
        {
            ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                        tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.025,0.25,0.125),
                        "REP Select (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
        }
    }
    wai_oa_rep_manager.UpdateModel(s_rep_select,s_detail_select);
}


bool WAIOpenAuditorium::CheckCollision(tf::Vector3 vc3_current,tf::Vector3 vc3_reference,float f_threshold_radius)
{
    if((vc3_reference-vc3_current).length()<f_threshold_radius) return true;
    else return false;
}

bool WAIOpenAuditorium::CheckCollisionTriggerScenePrev()
{
    return (CheckCollision(
                tf::Vector3(msg_pts_rviz_clicked.point.x,msg_pts_rviz_clicked.point.y,msg_pts_rviz_clicked.point.z),
                ((WAIRepTrigger*)wai_oa_rep_trigger_scene_prev)->GetPosition(),
                F_SCENE_TRIGGER_COLL_THRES)
            ||
            CheckCollision(
                wai_oa_presenter.GetHandLeftPosition(),
                ((WAIRepTrigger*)wai_oa_rep_trigger_scene_prev)->GetPosition(),
                F_SCENE_TRIGGER_COLL_THRES)
            );
}

bool WAIOpenAuditorium::CheckCollisionTriggerSceneNext()
{
    return (CheckCollision(
                tf::Vector3(msg_pts_rviz_clicked.point.x,msg_pts_rviz_clicked.point.y,msg_pts_rviz_clicked.point.z),
                ((WAIRepTrigger*)wai_oa_rep_trigger_scene_next)->GetPosition(),
                F_SCENE_TRIGGER_COLL_THRES)
            ||
            CheckCollision(wai_oa_presenter.GetHandLeftPosition(),
                ((WAIRepTrigger*)wai_oa_rep_trigger_scene_next)->GetPosition(),
                F_SCENE_TRIGGER_COLL_THRES)
            );
}

bool WAIOpenAuditorium::CheckCollisionTriggerRespawn2D3D()
{
    return CheckCollision(
                wai_oa_presenter.GetHandRightPosition(),
                tf::Vector3(2.5,2.0,1.0),
                F_SCENE_TRIGGER_COLL_THRES*3.0);
}

bool WAIOpenAuditorium::CheckCollisionTriggerRepSelect()
{
    tf::Vector3 vc3_point_clicked(msg_pts_rviz_clicked.point.x,
                                  msg_pts_rviz_clicked.point.y,
                                  msg_pts_rviz_clicked.point.z);

    for(unsigned long i=0;i<wai_oa_rep_manager.GetRepLinkStates()->pose.size();i++)
    {
        tf::Vector3 vc3_gazebo_pos;
        vc3_gazebo_pos.setX(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.x);
        vc3_gazebo_pos.setY(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.y);
        vc3_gazebo_pos.setZ(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.z);

        if(CheckCollision( vc3_point_clicked,
                           vc3_gazebo_pos,
                           F_SCENE_TRIGGER_COLL_THRES))
        {
            // Get name of selected rep
            std::string s_key = "::";
            std::string s_force_name_dirty=wai_oa_rep_manager.GetRepLinkStates()->name[i];
            std::string::size_type index = s_force_name_dirty.find(s_key);
            if (index == std::string::npos)
            {
                // Separator not contained, do nothing...
            }
            std::string s_rep_sel_point_and_click = s_force_name_dirty.substr(0, index);
            if(s_rep_sel_point_and_click.compare("ground_plane")==0)
            {
                s_rep_sel_point_and_click="workspace_presenter";
            }
            ROS_DEBUG_STREAM("Point and Click: REP selected is " << s_rep_sel_point_and_click << ".");

            wai_oa_rep_manager.UpdateModel(s_rep_sel_point_and_click,"link_base");

            ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                        tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.025,0.25,0.125),
                        "REP Select",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);

            return true;
        }
    }

    return false;
}





// SCRIPTED INTERACTIONS:
//------------------------


/////////////////////////////////////////////////
/// Setup PROJECTOR from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupProjectorStateFromScript()
{
    std::vector<double> vec_setup_projector_state;
    std::stringstream s_setup_projector_state;
    s_setup_projector_state << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/projector_state";
    if(ros::param::has(s_setup_projector_state.str()))
    {
        nh.getParam(s_setup_projector_state.str(),vec_setup_projector_state);
        wai_oa_projector.SetProjectionPose(vec_setup_projector_state[0],vec_setup_projector_state[1],vec_setup_projector_state[2],vec_setup_projector_state[3]);
        wai_oa_projector.UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("PROJECTOR State: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup PROJECTOR from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupProjectorFromScript()
{   
    std::string s_setup_projector;
    std::string s_setup_projector_transition;
    std::stringstream sst_setup_projector;
    std::stringstream sst_setup_projector_transition;
    sst_setup_projector << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/projector";
    sst_setup_projector_transition << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/projector_transition";

    // Disable all other timer-based scripted interactions on change to next slide:
    wai_oa_projector.DisablePlayVideo();
    wai_oa_projector.DisableRobotcam();
    wai_oa_projector.DisableLivecam();

    // Check if parameter is given in config otherwise use default config!
    if(ros::param::has(sst_setup_projector.str()))
    {
        nh.getParam(sst_setup_projector.str(), s_setup_projector);
    }
    else
    {
        sst_setup_projector.str("");
        sst_setup_projector << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_session_defaults/projector/default";
        nh.getParam(sst_setup_projector.str(),s_setup_projector);
        ROS_DEBUG_STREAM("PROJECTOR: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << ", using '" << s_setup_projector << "' as default!");
    }

    if(s_setup_projector.find(".png")!=std::string::npos
            || s_setup_projector.find(".PNG")!=std::string::npos) // Load picture from file
    {
        std::stringstream sst_pictures_path;
        sst_pictures_path << wai_oa_session_manager.GetPathResourcesRoot() << "pictures/" << s_setup_projector;
        wai_oa_projector.UpdateModel(cv::imread(sst_pictures_path.str(), cv::IMREAD_COLOR),wai_oa_session_manager.GetPathResourcesLogo());
        wai_oa_projector.UpdateView("");
        //ros::spinOnce(); // Testing for picture not updated on projection
    }
    else if(s_setup_projector.find(".mp4")!=std::string::npos
            || s_setup_projector.find(".MP4")!=std::string::npos) // Load video from file
    {
        std::stringstream sst_pictures_path;
        sst_pictures_path << wai_oa_session_manager.GetPathResourcesRoot() << "videos/" << s_setup_projector;
        wai_oa_projector.EnablePlayVideo(sst_pictures_path.str(),wai_oa_session_manager.GetPathResourcesLogo());
    }
    else if(s_setup_projector.compare("livecam")==0)
    {
        wai_oa_projector.EnableLivecam();
    }
    else if(s_setup_projector.compare("repcam")==0)
    {
        std::stringstream sst_setup_rep_name;
        std::string s_setup_rep_name;
        sst_setup_rep_name << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/rep_name";

        // Retrieve REP NAME either from script of current scene or from default setting
        if(ros::param::has(sst_setup_rep_name.str()))
        {
            nh.getParam(sst_setup_rep_name.str(),s_setup_rep_name);
        }
        else
        {
            // Get default setting
            sst_setup_rep_name.str("");
            sst_setup_rep_name << s_path_nodename+"/"+wai_oa_session_manager.GetSessionName() << "/setup_session_defaults/reps/rep_name";
            nh.getParam(sst_setup_rep_name.str(),s_setup_rep_name);
        }
        wai_oa_projector.EnableRobotcam(s_setup_rep_name);
    }
    else // If setup is 'slide' or anything else
    {
        // Check for slide transition
        if(ros::param::has(sst_setup_projector_transition.str()))
        {
            nh.getParam(sst_setup_projector_transition.str(), s_setup_projector_transition);
            wai_oa_projector.UpdateView(s_setup_projector_transition,F_SCENE_TRANSITION_TIMEOUT);
        }
        else
        {
            // Check for multiple file name formats: PPT --> SlideX.PNG, PDF Exporter --> presentation-X.png
            std::stringstream sst_slides_path,sst_slides_path_1,sst_slides_path_2;
            sst_slides_path_1 << wai_oa_session_manager.GetPathResourcesSlides() << "Slide" << wai_oa_session_manager.GetSessionSceneCountCurrent() << ".PNG";
            sst_slides_path_2 << wai_oa_session_manager.GetPathResourcesSlides() << "presentation-" << wai_oa_session_manager.GetSessionSceneCountCurrent() << ".png";
            if(GetFileExists(sst_slides_path_1.str()) )
            {
                sst_slides_path.str(sst_slides_path_1.str());
            }
            else if(GetFileExists(sst_slides_path_2.str()) )
            {
                sst_slides_path.str(sst_slides_path_2.str());
            }
            wai_oa_projector.UpdateModel(cv::imread(sst_slides_path.str(), cv::IMREAD_COLOR),wai_oa_session_manager.GetPathResourcesLogo());
            wai_oa_projector.UpdateView("");
        }

        // DEPRECATED QR-Code based interface
        // (No slide interactions for intro and outro slide!)
        /*
        if(B_ENABLE_SLIDE_INTERACTIONS
            && wai_oa_session_manager.GetSessionSceneCountCurrent()>1
            && wai_oa_session_manager.GetSessionSceneCountCurrent()<wai_oa_session_manager.GetSessionSceneCountMax())
        {
            // Load as grayscale image as input for QR code scanner
            cv::Mat grayImg=cv::imread(sst_slides_path.str(), cv::IMREAD_GRAYSCALE);
            zbar::Image img(grayImg.cols, grayImg.rows, "Y800", grayImg.data, grayImg.cols * grayImg.rows);
            scn_zbar.scan(img);

            std::vector<Tag> tags;
            for (zbar::SymbolIterator s = img.symbol_begin(); s != img.symbol_end(); ++s)
            {
                Tag tag;
                tag.qrcode = s->get_data();
                ROS_DEBUG_STREAM("Projector: Decoded tag is " << tag.qrcode);
                if(tag.qrcode.compare("lectern:1")==0)
                {
                    SetupLecternModel();
                    SetupLecternView();
                }
                if(tag.qrcode.compare("3dmode:1")==0)
                {
                    SetupPresenceModeModel();
                    SetupPresenceModeView();
                }
                for(int i = 0; i < s->get_location_size(); i++)
                {
                    tag.polygon.push_back(cv::Point(s->get_location_x(i), s->get_location_y(i)));
                }
                tags.push_back(tag);
            }
        }
        */
    }
}


/////////////////////////////////////////////////
/// Setup LIGHT from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupLightFromScript()
{
    std::string s_setup_light;
    std::string s_setup_light_name="light_1"; // Default name of first light in rviz_plugin_wai_lights
    std::vector<double> vec_light_from_script;
    std::stringstream sst_setup_light;
    sst_setup_light << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/light";

    if(ros::param::has(sst_setup_light.str()))
    {
        pub_col_light_ambient=nh.advertise<std_msgs::ColorRGBA>("/wai_world/world/"+s_setup_light_name+"/color_ambient",1);
        pub_col_light_diff_spec=nh.advertise<std_msgs::ColorRGBA>("/wai_world/world/"+s_setup_light_name+"/color_diff_spec",1);
        SpinAndWaitForSeconds(0.5);
        std_msgs::ColorRGBA msg_col_lights;

        // Init vector for custom light settings
        vec_light_from_script.push_back(128); // R ambient
        vec_light_from_script.push_back(128); // G ambient
        vec_light_from_script.push_back(128); // B ambient
        vec_light_from_script.push_back(128); // R spec,diff
        vec_light_from_script.push_back(128); // G spec,diff
        vec_light_from_script.push_back(128); // B spec,diff

        nh.getParam(sst_setup_light.str(),s_setup_light);
        if(s_setup_light.compare("off")==0)
        {
            msg_col_lights=col_light_off;
            pub_col_light_ambient.publish(msg_col_lights);
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else if(s_setup_light.compare("dark")==0)
        {
            msg_col_lights=col_light_dark;
            pub_col_light_ambient.publish(msg_col_lights);
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else if(s_setup_light.compare("default")==0)
        {
            msg_col_lights=col_light_default;
            pub_col_light_ambient.publish(msg_col_lights);
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else if(s_setup_light.compare("bright")==0)
        {
            msg_col_lights=col_light_bright;
            pub_col_light_ambient.publish(msg_col_lights);
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else if(s_setup_light.compare("cold")==0) // With the following settings, diffuse and specular are changed only!
        {
            msg_col_lights=col_light_cold;
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else if(s_setup_light.compare("warm")==0)
        {
            msg_col_lights=col_light_warm;
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
        else
        {
            nh.getParam(sst_setup_light.str(), vec_light_from_script);
            msg_col_lights.r=vec_light_from_script[0];
            msg_col_lights.g=vec_light_from_script[1];
            msg_col_lights.b=vec_light_from_script[2];
            msg_col_lights.a=255.0;
            pub_col_light_ambient.publish(msg_col_lights);
            msg_col_lights.r=vec_light_from_script[3];
            msg_col_lights.g=vec_light_from_script[4];
            msg_col_lights.b=vec_light_from_script[5];
            msg_col_lights.a=255.0;
            pub_col_light_diff_spec.publish(msg_col_lights);
        }
    }
    else
    {
        ROS_DEBUG_STREAM("LIGHT: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup CAMERA from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupCameraRvizFromScript()
{
    std::stringstream sst_setup_camera_rviz;
    std::stringstream sst_setup_camera_rviz_link;
    std::stringstream sst_setup_camera_rviz_rep_follow;
    std::string s_setup_camera_rviz;
    std::string s_setup_camera_rviz_rep_follow;
    s_setup_camera_rviz_frame="world";
    std::vector<double> vec_camera_rviz_from_script;

    sst_setup_camera_rviz << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/camera";
    sst_setup_camera_rviz_link << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/camera_link";
    sst_setup_camera_rviz_rep_follow << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/camera_follow";

    // Disable follow views
    wai_oa_camera_rviz.CameraFollowPresenterStop();
    wai_oa_camera_rviz.CameraFollowRepStop();

    if( ros::param::has(sst_setup_camera_rviz.str()) &&
            !ros::param::has(sst_setup_camera_rviz_link.str()) ) // Camera setup and NO link existing
    {
        if(nh.getParam(sst_setup_camera_rviz.str(), s_setup_camera_rviz)) // Parameter given as string, load default vectors according to name of camera setting!
        {
            std::stringstream sst_setup_camera_rviz_defaults;
            sst_setup_camera_rviz_defaults << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_session_defaults/camera/" << s_setup_camera_rviz;
            nh.getParam(sst_setup_camera_rviz_defaults.str(), vec_camera_rviz_from_script);
        }
        else // Parameter given as double array
        {
            nh.getParam(sst_setup_camera_rviz.str(), vec_camera_rviz_from_script);
        }
    }
    else if( ros::param::has(sst_setup_camera_rviz.str()) &&
             ros::param::has(sst_setup_camera_rviz_link.str()) ) // Setup WITH link existing
    {
        nh.getParam(sst_setup_camera_rviz.str(), vec_camera_rviz_from_script);
        nh.getParam(sst_setup_camera_rviz_link.str(), s_setup_camera_rviz_frame);
    }
    else if(ros::param::has(sst_setup_camera_rviz_rep_follow.str())) // Follow REP with camera view
    {
        nh.getParam(sst_setup_camera_rviz_rep_follow.str(),s_setup_camera_rviz_rep_follow);
        wai_oa_camera_rviz.CameraFollowRepStart(s_setup_camera_rviz_rep_follow);
        return;
    }
    else
    {
        sst_setup_camera_rviz.str("");
        sst_setup_camera_rviz << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_session_defaults/camera/default";
        nh.getParam(sst_setup_camera_rviz.str(),vec_camera_rviz_from_script);
        ROS_DEBUG_STREAM("CAMERA: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << ", using default!");
    }

    if(B_ENABLE_CAMERA_RVIZ_FLY_IN && wai_oa_session_manager.GetSessionSceneCountCurrent()==1)
    {
        std::vector<double> vec_setup_camera_rviz_startup;
        std::stringstream sst_setup_camera_rviz_startup;
        sst_setup_camera_rviz_startup << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_session_defaults/camera/startup";
        nh.getParam(sst_setup_camera_rviz_startup.str(), vec_setup_camera_rviz_startup);

        wai_oa_camera_rviz.UpdateModel(vec_setup_camera_rviz_startup,false,false,-1,1.0);
        wai_oa_camera_rviz.UpdateView();
        SpinAndWaitForSeconds(2.0);
    }

    //wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,false,-1);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_from_script,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,s_setup_camera_rviz_frame);
    wai_oa_camera_rviz.UpdateView();
}

void WAIOpenAuditorium::SetupCameraRvizFromAudienceSelected()
{
    std::vector<double> vec_camera_rviz_audience;
    /*
    Birds eye view:
    vec_camera_rviz_audience.push_back(-0.5);
    vec_camera_rviz_audience.push_back(-0.75);
    vec_camera_rviz_audience.push_back(1.75);
    vec_camera_rviz_audience.push_back(1.0);
    vec_camera_rviz_audience.push_back(0.0);
    vec_camera_rviz_audience.push_back(0.5);
    Look over shoulder:
    */
    vec_camera_rviz_audience.push_back(1.0);
    vec_camera_rviz_audience.push_back(0.65);
    vec_camera_rviz_audience.push_back(1.75);
    vec_camera_rviz_audience.push_back(0.25);
    vec_camera_rviz_audience.push_back(0.0);
    vec_camera_rviz_audience.push_back(0.35);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_audience,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"workspace_audience_"+std::to_string(i_audience_id_selected)+"/link_base",0);
    wai_oa_camera_rviz.UpdateView();
}

void WAIOpenAuditorium::SetupCameraRvizFromRepSelected()
{
    // Get predefined view from REP's details
    std::stringstream sst_details;
    std::vector<double> vec_d_details_camera_rviz;
    sst_details.str("");
    sst_details << "/wai_world/" << wai_oa_rep_manager.GetRepSelected() << "/details/" << wai_oa_rep_manager.GetRepDetailSelected() << "/camera_rviz";
    if(ros::param::has(sst_details.str()))
    {
        nh.getParam(sst_details.str(),vec_d_details_camera_rviz);
    }
    else
    {
        vec_d_details_camera_rviz.push_back(-1.5); // If no details are available from script
        vec_d_details_camera_rviz.push_back(-1.5);
        vec_d_details_camera_rviz.push_back(3.0);
    }

    // Update vector to display detail mesh
    tf::Vector3 vc3_link_focus(0.0,0.0,0.0);
    tf::Vector3 vc3_link_eye(vec_d_details_camera_rviz[0],vec_d_details_camera_rviz[1],vec_d_details_camera_rviz[2]);
    //vc3_link_focus.setValue(0.0,0.0,0.0);
    //vc3_link_eye.setValue(-1.0,2.0,2.0);
    double d_length_eye=vc3_link_eye.length()/2.0;
    //vc3_camera_rviz_wrt_detail.setValue(d_length_eye/2.0,0.0,d_length_eye);
    wai_oa_camera_rviz.SetTransformDetail(
                tf::Transform(
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(d_length_eye/2.0,0.0,d_length_eye)
                    ));

    std::vector<double> vec_camera_rviz_rep_from_detail;
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_eye.getX()); // eye
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_eye.getY());
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_eye.getZ());
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_focus.getX()); // focus
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_focus.getY());
    vec_camera_rviz_rep_from_detail.push_back(vc3_link_focus.getZ());
    wai_oa_camera_rviz.UpdateModel(
                vec_camera_rviz_rep_from_detail,
                false,
                false,
                -1,
                1.0,
                wai_oa_rep_manager.GetRepSelected()+"/"+wai_oa_rep_manager.GetRepDetailSelected());
    wai_oa_camera_rviz.UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizCycle(bool b_back_forth)
{
    if(b_back_forth==true)
    {
        wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,true,-1,F_SCENE_TRANSITION_TIMEOUT,"world",0,true);
    }
    else
    {
        wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,false,-1,F_SCENE_TRANSITION_TIMEOUT,"world",0,true);
    }
    wai_oa_camera_rviz.UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizRestorePreviousView()
{
    wai_oa_camera_rviz.RestorePreviousView();
}


/////////////////////////////////////////////////
/// Setup TELEPROMPTER from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupTeleprompterFromScript()
{
    std::string s_setup_teleprompter;
    std::stringstream sst_setup_teleprompter;
    sst_setup_teleprompter << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/teleprompter";

    if(ros::param::has(sst_setup_teleprompter.str()))
    {
        nh.getParam(sst_setup_teleprompter.str(),s_setup_teleprompter);
        wai_oa_teleprompter.UpdateModel(s_setup_teleprompter);
    }
    else
    {
        ROS_DEBUG_STREAM("TELEPROMPTER: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup BROWSER LINK from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupBrowserLinkFromScript()
{
    std::string s_setup_browser_link;
    std::stringstream sst_setup_browser_link;
    sst_setup_browser_link << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/browser_link";

    if(ros::param::has(sst_setup_browser_link.str()))
    {
        nh.getParam(sst_setup_browser_link.str(), s_setup_browser_link);
        std::string s_syscmd="google-chrome "+s_setup_browser_link+" &";
        int i_retval=system(s_syscmd.c_str());
    }
    else
    {
        ROS_DEBUG_STREAM("BROWSER LINK: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup SOUND from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupSoundFromScript()
{
    std::string s_setup_sound;
    std::stringstream sst_setup_sound;
    sst_setup_sound << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/sound";
    if(ros::param::has(sst_setup_sound.str()))
    {
        nh.getParam(sst_setup_sound.str(), s_setup_sound);
        PlaySound(s_setup_sound);
    }
    else
    {
        ROS_DEBUG_STREAM("SOUND: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}
void WAIOpenAuditorium::PlaySound(std::string s_name_sound)
{
    sound_play::Sound snd_play=sc.waveSoundFromPkg("wai_oa_gazebo","resources/sounds/"+s_name_sound+".wav");
    snd_play.play();
}


/////////////////////////////////////////////////
/// Setup LECTERN from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupLecternFromScript()
{
    bool b_setup_lectern;
    std::stringstream sst_setup_lectern;
    sst_setup_lectern << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/lectern";
    if(ros::param::has(sst_setup_lectern.str()))
    {
        nh.getParam(sst_setup_lectern.str(), b_setup_lectern);
        B_ENABLE_LECTERN=b_setup_lectern;

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "LECTERN",col_cyan,B_ENABLE_LECTERN,true,F_SCENE_TRIGGER_TIMEOUT);

        if(B_ENABLE_LECTERN)
        {
            wai_oa_rep_manager.SetRepStateViaService("lectern",tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,0.0,0.0));
        }
        else
        {
            wai_oa_rep_manager.SetRepStateViaService("lectern",tf::Vector3(2.45,-2.0,0.0));
        }

        wai_oa_rep_trigger_basic->UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("LECTERN: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup TABLE from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupTableFromScript()
{
    bool b_setup_table;
    std::stringstream sst_setup_table;
    sst_setup_table << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/table";
    if(ros::param::has(sst_setup_table.str()))
    {
        nh.getParam(sst_setup_table.str(), b_setup_table);
        B_ENABLE_TABLE=b_setup_table;

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "TABLE",col_cyan,B_ENABLE_TABLE,true,F_SCENE_TRIGGER_TIMEOUT);
        if(B_ENABLE_TABLE)
        {
            wai_oa_rep_manager.SetRepStateViaService("table",tf::Vector3(0.05,0.0,0.0));
        }
        else
        {
            wai_oa_rep_manager.SetRepStateViaService("table",tf::Vector3(1.75,-2.0,0.0));
        }

        wai_oa_rep_trigger_basic->UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("TABLE: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup VIRTUAL PRESENTER from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupVirtualPresenterFromScript()
{
    std::string s_setup_virtual_presenter;
    std::stringstream sst_setup_virtual_presenter;
    sst_setup_virtual_presenter << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/virtual_presenter";
    if(ros::param::has(sst_setup_virtual_presenter.str()))
    {
        nh.getParam(sst_setup_virtual_presenter.str(), s_setup_virtual_presenter);

        std::stringstream sst_sys_com_presenter_virtual;
        sst_sys_com_presenter_virtual
                << "rosbag play -q "
                << wai_oa_session_manager.GetPathResourcesRoot()
                << "rosbags/"
                << s_setup_virtual_presenter
                << ".bag"
                << " &";

        //Example command: int i_sys_rval=int i_sys_rval=system("rosbag play ~/wai_auditorium_presenter_virtual.bag --duration=30 &");
        int i_sys_rval=system(sst_sys_com_presenter_virtual.str().c_str());
    }
    else
    {
        ROS_DEBUG_STREAM("VIRTUAL PRESENTER: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup 3D-GRAPH from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupGraph3DFromScript()
{
    std::string s_setup_graph_3d;
    //std::vector<int> vec_i_setup_graph_3d_dims;
    std::stringstream sst_setup_graph_3d,sst_setup_graph_3d_dims,sst_setup_graph_3d_grid;
    float f_setup_graph_3d_grid=0.04;
    sst_setup_graph_3d << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/graph_3d";
    //sst_setup_graph_3d_dims << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/graph_3d_dims";
    sst_setup_graph_3d_grid << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/graph_3d_grid";
    if(ros::param::has(sst_setup_graph_3d.str())
        //&& ros::param::has(sst_setup_graph_3d_dims.str())
        && ros::param::has(sst_setup_graph_3d_grid.str()))
    {
        nh.getParam(sst_setup_graph_3d.str(),s_setup_graph_3d);
        //nh.getParam(sst_setup_graph_3d_dims.str(),vec_i_setup_graph_3d_dims);
        nh.getParam(sst_setup_graph_3d_grid.str(),f_setup_graph_3d_grid);

        int i_data_size_x=0;//vec_i_setup_graph_3d_dims[0];
        int i_data_size_y=0;//vec_i_setup_graph_3d_dims[1];
        int i_data_size_z=1;
        std::vector< std::vector<float> > vec_temp=GetData3DFromFile(wai_oa_session_manager.GetPathResourcesRoot()+"graphs/"+s_setup_graph_3d+".csv",&i_data_size_x,&i_data_size_y);

        float* graph_data = new float[i_data_size_x*i_data_size_y*i_data_size_z];

        for(int x=0;x<i_data_size_x;x++)
        {
            for(int y=0;y<i_data_size_y;y++)
            {
                for(int z=0;z<i_data_size_z;z++)
                {
                    *(graph_data+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=vec_temp[x][y];
                }
            }
        }

        ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateAxes(f_setup_graph_3d_grid,f_setup_graph_3d_grid,0.0,0.03, 0.0,6.28,0.0,6.28,-3.0,3.0);
        ((WAIRepGraph3D*)wai_oa_rep_graph_3d_function)->UpdateGraphDataFunction(graph_data,i_data_size_x,i_data_size_y,i_data_size_z);
        wai_oa_rep_graph_3d_function->UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("GRAPH 3D: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}
std::vector< std::vector<float> > WAIOpenAuditorium::GetData3DFromFile(std::string filename,int* dim_x,int* dim_y)
{
    std::ifstream input_check(filename.c_str());
    std::string s_check;

    // Check for dimensions
    int i=0,j=0;
    while(std::getline(input_check,s_check))
    {
        i++;
        j=0;
        std::istringstream iss(s_check);
        std::string num;
        while(std::getline(iss,num,','))
        {
            j++;
        }
    }
    j=j-1; // Last terminating character per line is "," --> so reduce j by minus one!
    input_check.close();
    ROS_WARN("3D-GRAPH: Dimensions are X=%d,Y=%d",i,j);
    *dim_x=i;
    *dim_y=j;

    std::ifstream input(filename.c_str());
    std::string s;
    std::vector< std::vector<float> > result(i);
    for(int n=0;n<i;n++)
    {
        std::getline(input,s);
        std::istringstream iss(s);
        std::string num;
        for(int k=0;k<j;k++)
        {
            std::getline(iss,num,',');
            // Be careful with atof/stof/stod conversion, use stringstream instead!
            std::stringstream ss_num;
            float f_num=0.0;
            ss_num << num;
            ss_num >> f_num;
            result[n].push_back(f_num);
        }
    }

    return result;
}


/////////////////////////////////////////////////
/// Setup STATS-GRAPH from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupGraphStatsFromScript()
{
    std::vector<std::vector<std::vector<double> >> vec_vec_d_stats(10);
    std::string s_setup_graph_stats;
    std::stringstream sst_setup_graph_stats;
    sst_setup_graph_stats << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/graph_stats";
    if(ros::param::has(sst_setup_graph_stats.str()))
    {
        nh.getParam(sst_setup_graph_stats.str(), s_setup_graph_stats);
        int i_sys_rval=system( ("rosparam load "+wai_oa_session_manager.GetPathResourcesRoot()+"graphs/"+s_setup_graph_stats+".yaml "+s_path_nodename+"/graph_stats").c_str() );

        std::vector<std::string> vec_s_setup_evals;
        std::stringstream sst_setup_evals;
        sst_setup_evals << s_path_nodename + "/graph_stats/evaluations";
        XmlRpc::XmlRpcValue lst_setup_evals;
        nh.getParam(sst_setup_evals.str(),lst_setup_evals);
        for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=lst_setup_evals.begin();it!=lst_setup_evals.end();++it)
        {
            vec_s_setup_evals.push_back(it->first);
        }

        for(int i=0;i<vec_s_setup_evals.size();i++)
        {
            std::stringstream sst_setup_conds;
            sst_setup_conds << s_path_nodename + "/graph_stats/evaluations/"+vec_s_setup_evals[i];
            XmlRpc::XmlRpcValue lst_setup_conds;
            nh.getParam(sst_setup_conds.str(),lst_setup_conds);
            for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it=lst_setup_conds.begin();it!=lst_setup_conds.end();++it)
            {
                std::vector<double> vec_d_data;
                std::stringstream sst_setup_data;
                sst_setup_data << s_path_nodename + "/graph_stats/evaluations/"+vec_s_setup_evals[i]+"/"+it->first;
                nh.getParam(sst_setup_data.str(),vec_d_data);
                //ROS_WARN_STREAM("Data: " << vec_d_data[0] << ";"<< vec_d_data[1] << ";"<< vec_d_data[2] << ";"<< vec_d_data[3] << ";");
                vec_vec_d_stats[i].push_back(vec_d_data);
            }
        }

        /*
        for(int y=0;y<2;y++)
        {
            for(int x=0;x<4;x++)
            {
                ROS_WARN_STREAM("Row "<<y<<": "<<vec_vec_d_stats[y][x][0] << " ; "<<vec_vec_d_stats[y][x][1] << " ; "<<vec_vec_d_stats[y][x][2] << " ; "<<vec_vec_d_stats[y][x][3] << " ; ");
            }
        }*/

        // Fill stats graph data
        // Init stats graph
        int i_data_size_x=vec_vec_d_stats[0].size(); // CONDITIONS per Eval (constant)
        int i_data_size_y=vec_s_setup_evals.size(); // EVALS
        int i_data_size_z=1;
        float* graph_stats_data_min = new float[i_data_size_x*i_data_size_y*i_data_size_z];
        float* graph_stats_data_max = new float[i_data_size_x*i_data_size_y*i_data_size_z];
        float* graph_stats_data_mean = new float[i_data_size_x*i_data_size_y*i_data_size_z];
        float* graph_stats_data_stddev = new float[i_data_size_x*i_data_size_y*i_data_size_z];
        for(int x=0;x<i_data_size_x;x++)
        {
            for(int y=0;y<i_data_size_y;y++)
            {
                for(int z=0;z<i_data_size_z;z++)
                {
                    *(graph_stats_data_min+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=vec_vec_d_stats[y][x][0];//x+1.0;
                    *(graph_stats_data_max+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=vec_vec_d_stats[y][x][1];//x+2.0+0.1*x;
                    *(graph_stats_data_mean+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=vec_vec_d_stats[y][x][2];//x+1.5+0.025*x;
                    *(graph_stats_data_stddev+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=vec_vec_d_stats[y][x][3];//0.125*(x+1);
                }
            }
        }
        ((WAIRepGraphStats*)wai_oa_rep_graph_3d_stats)->UpdateGraphDataStats(
                    graph_stats_data_min,
                    graph_stats_data_max,
                    graph_stats_data_mean,
                    graph_stats_data_stddev,
                    4,2,1);
        wai_oa_rep_graph_3d_stats->UpdateView();
    }
    else
    {
        ROS_DEBUG_STREAM("GRAPH STATS: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Prepare 2D/3D RESPAWNING from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::PrepareRespawn2D3DFromScript()
{
    std::stringstream sst_setup_respawn_2d3d;
    sst_setup_respawn_2d3d << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/respawn_2d3d";
    if(ros::param::has(sst_setup_respawn_2d3d.str()))
    {
        nh.getParam(sst_setup_respawn_2d3d.str(),s_setup_rep_respawn_2d3d);
        wai_oa_presenter.SetupHandText(CleanupStringRep(s_setup_rep_respawn_2d3d));
    }
    else
    {
        // Reset hand text from respawn interaction
        s_setup_rep_respawn_2d3d="";
        wai_oa_presenter.SetupHandText("Dave");
        ROS_DEBUG_STREAM("2D/3D RESPAWN: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}
void WAIOpenAuditorium::SetupRespawn2D3DFromScript() // Actual interaction triggered with virtual hand pointer
{
    if(s_setup_rep_respawn_2d3d.compare("")!=0)
    {
        wai_oa_presenter.SetupRespawn2D3D(s_setup_rep_respawn_2d3d);
    }
    else
    {
        wai_oa_presenter.SetupRespawn2D3D("hand_right");
        wai_oa_presenter.SetupHandText("Dave");
        ROS_DEBUG_STREAM("2D/3D RESPAWN: No resource prepared to respawn!");
    }
}


/////////////////////////////////////////////////
/// Setup HAND TEXT from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupHandTextFromScript()
{
    std::stringstream sst_setup_hand_text;
    sst_setup_hand_text << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/hand_text";
    if(ros::param::has(sst_setup_hand_text.str()))
    {
        nh.getParam(sst_setup_hand_text.str(), s_setup_hand_text);
        wai_oa_presenter.SetupHandText(s_setup_hand_text);
    }
    else
    {
        wai_oa_presenter.SetupHandText("Dave");
        ROS_DEBUG_STREAM("HAND TEXT: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup REPRESENTATIVE (REP) from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupRepresentativeFromScript()
{
    sst_setup_rep_name.str("");
    sst_setup_rep_interaction.str("");
    sst_setup_rep_interaction_switch.str("");
    sst_setup_rep_sequence_duration.str("");
    sst_setup_rep_sequence_state.str("");
    sst_setup_rep_sequence_twist.str("");
    sst_setup_rep_sequence_force.str("");

    sst_setup_rep_name<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_name";
    sst_setup_rep_interaction<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_interaction";
    sst_setup_rep_interaction_switch<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_interaction_switch";
    sst_setup_rep_sequence_duration<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_sequence_duration";
    sst_setup_rep_sequence_state<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_sequence_state";
    sst_setup_rep_sequence_twist<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_sequence_twist";
    sst_setup_rep_sequence_force<<s_path_nodename+"/"+wai_oa_session_manager.GetSessionName()<<"/setup_scene_"<<wai_oa_session_manager.GetSessionSceneCountCurrent()<<"/rep_sequence_force";

    if(ros::param::has(sst_setup_rep_name.str()) && ros::param::has(sst_setup_rep_interaction.str()))
    {
        std::string s_setup_rep_name=""; nh.getParam(sst_setup_rep_name.str(),s_setup_rep_name);
        std::string s_setup_rep_interaction=""; nh.getParam(sst_setup_rep_interaction.str(),s_setup_rep_interaction);

        if(s_setup_rep_interaction.compare("state")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::vector<double> vec_d_setup_rep_interaction_switch; nh.getParam(sst_setup_rep_interaction_switch.str(),vec_d_setup_rep_interaction_switch);

            if(s_setup_rep_name.compare("presenter")==0)
            {
                // Update Presenter REPs pose, default settings from startup are overwritten...
                wai_oa_presenter.SetPresenterPose(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2],vec_d_setup_rep_interaction_switch[3]);
            }
            else if(s_setup_rep_name.compare("projection")==0)
            {
                wai_oa_projector.SetProjectionPose(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2],vec_d_setup_rep_interaction_switch[3]);
                wai_oa_projector.UpdateView();
            }
            else
            {
                wai_oa_rep_manager.SetRepStateViaService(s_setup_rep_name,
                                                         tf::Vector3(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2]),
                                                         tf::Vector3(0.0,0.0,vec_d_setup_rep_interaction_switch[3]));
            }
        }
        else if(s_setup_rep_interaction.compare("force")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::vector<double> vec_d_setup_rep_interaction_switch; nh.getParam(sst_setup_rep_interaction_switch.str(),vec_d_setup_rep_interaction_switch);
            std::size_t pos=s_setup_rep_name.find("/");
            std::string s_setup_link_name="";
            if(pos!=std::string::npos)
            {
                s_setup_link_name=s_setup_rep_name.substr(pos+1);
            }
            else s_setup_link_name="link_base";
            wai_oa_rep_manager.SetRepForceViaService(s_setup_rep_name,s_setup_link_name,tf::Vector3(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2]));
        }
        else if(s_setup_rep_interaction.compare("reference")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::vector<double> vec_d_setup_rep_interaction_switch; nh.getParam(sst_setup_rep_interaction_switch.str(),vec_d_setup_rep_interaction_switch);

            pub_pst_robot_reference=nh.advertise<geometry_msgs::PoseStamped>("/wai_world/"+s_setup_rep_name+"/reference/position",1);
            SpinAndWaitForSeconds(1.0/F_NODE_SAMPLE_FREQUENCY*10.0);
            geometry_msgs::PoseStamped msg_pst_robot_setpoint;
            msg_pst_robot_setpoint.header.stamp=ros::Time::now();
            msg_pst_robot_setpoint.header.frame_id="world";
            msg_pst_robot_setpoint.pose.position.x=vec_d_setup_rep_interaction_switch[0];
            msg_pst_robot_setpoint.pose.position.y=vec_d_setup_rep_interaction_switch[1];
            msg_pst_robot_setpoint.pose.position.z=vec_d_setup_rep_interaction_switch[2];
            tf::Quaternion qua_robot_rotation;
            qua_robot_rotation.setRPY(0.0,0.0,vec_d_setup_rep_interaction_switch[3]);
            qua_robot_rotation=qua_robot_rotation.normalize();
            msg_pst_robot_setpoint.pose.orientation.w=qua_robot_rotation.getW();
            msg_pst_robot_setpoint.pose.orientation.x=qua_robot_rotation.getX();
            msg_pst_robot_setpoint.pose.orientation.y=qua_robot_rotation.getY();
            msg_pst_robot_setpoint.pose.orientation.z=qua_robot_rotation.getZ();
            pub_pst_robot_reference.publish(msg_pst_robot_setpoint);
        }
        else if(s_setup_rep_interaction.compare("startup")==0)
        {
            double d_setup_rep_interaction_switch=0.0;
            if(ros::param::has(sst_setup_rep_interaction_switch.str()))
            {
                nh.getParam(sst_setup_rep_interaction_switch.str(),d_setup_rep_interaction_switch);
            }
            else
            {
                d_setup_rep_interaction_switch=30.0;
            }
            // Startup interface specifically for INGENUITY to spin up the co-axial rotors
            pub_f64_robot_rotor_middle_command=nh.advertise<std_msgs::Float64>("/wai_world/"+s_setup_rep_name+"/joint_"+s_setup_rep_name+"_rotor_middle_velocity_controller/command",1,true);
            pub_f64_robot_rotor_top_command=nh.advertise<std_msgs::Float64>("/wai_world/"+s_setup_rep_name+"/joint_"+s_setup_rep_name+"_rotor_top_velocity_controller/command",1,true);
            std_msgs::Float64 msg_ingenuity_rot_mid;
            std_msgs::Float64 msg_ingenuity_rot_top;
            msg_ingenuity_rot_mid.data=d_setup_rep_interaction_switch;
            msg_ingenuity_rot_top.data=-d_setup_rep_interaction_switch;
            pub_f64_robot_rotor_middle_command.publish(msg_ingenuity_rot_mid);
            pub_f64_robot_rotor_top_command.publish(msg_ingenuity_rot_top);
        }
        else if(s_setup_rep_interaction.compare("takeoff")==0)
        {
            pub_emp_robot_takeoff=nh.advertise<std_msgs::Empty>("/wai_world/"+s_setup_rep_name+"/takeoff",1,true);
            std_msgs::Empty msg_emp_robot_takeoff;
            pub_emp_robot_takeoff.publish(msg_emp_robot_takeoff);
        }
        else if(s_setup_rep_interaction.compare("land")==0)
        {
            pub_emp_robot_land=nh.advertise<std_msgs::Empty>("/wai_world/"+s_setup_rep_name+"/land",1,true);
            std_msgs::Empty msg_emp_robot_land;
            pub_emp_robot_land.publish(msg_emp_robot_land);
        }
        else if(s_setup_rep_interaction.compare("shutdown")==0)
        {
            // Shutdown interface specifically for INGENUITY to stop the co-axial rotors
            pub_f64_robot_rotor_middle_command=nh.advertise<std_msgs::Float64>("/wai_world/"+s_setup_rep_name+"/joint_"+s_setup_rep_name+"_rotor_middle_velocity_controller/command",1,true);
            pub_f64_robot_rotor_top_command=nh.advertise<std_msgs::Float64>("/wai_world/"+s_setup_rep_name+"/joint_"+s_setup_rep_name+"_rotor_top_velocity_controller/command",1,true);
            std_msgs::Float64 msg_ingenuity_rot_mid;
            std_msgs::Float64 msg_ingenuity_rot_top;
            msg_ingenuity_rot_mid.data=0.0;
            msg_ingenuity_rot_top.data=0.0;
            pub_f64_robot_rotor_middle_command.publish(msg_ingenuity_rot_mid);
            pub_f64_robot_rotor_top_command.publish(msg_ingenuity_rot_top);
        }
        else if(s_setup_rep_interaction.compare("respawn")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            // Advanced method (fix!)
            std::vector<double> vec_d_setup_rep_interaction_switch;
            nh.getParam(sst_setup_rep_interaction_switch.str(),vec_d_setup_rep_interaction_switch);

            // Currently the model is Spawned but not showing in RViz due to missing robot_description PARAMETER!
            // --> cycle through reps in directory and preapre description script based (moved to init-method in Constrcutor!):
            //int i_sys_rval=system( ("rosparam load "+ros::package::getPath("wai_oa_gazebo")+"/descriptions/"+s_setup_spawn_rep+".xacro /wai_world/"+s_setup_spawn_rep+"/robot_description").c_str() );
            //int i_sys_rval=system( ("rosparam set /wai_world/"+s_setup_spawn_rep+"/tf_prefix "+s_setup_spawn_rep).c_str() );

            // Double check XACRO to URDF conversion, e.g.:
            // rosrun xacro xacro --inorder -o darth_vader_helmet.urdf darth_vader_helmet.xacro

            // First make sure that entity is not existing already
            if(wai_oa_rep_manager.GetRepExists(s_setup_rep_name))
            {
                ROS_WARN_STREAM("REPRESENTATIVE: Rep already exists, skipped respawn! (Scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << ")!");
                return;
            }
            /* Do not simply delete model, since loaded plugins remain!
            gazebo_msgs::DeleteModel gaz_delete_model;
            gaz_delete_model.request.model_name=s_setup_rep_name;
            if(gazebo_delete_model_client.isValid())
            {
                gazebo_delete_model_client.waitForExistence();
                gazebo_delete_model_client.call(gaz_delete_model);
            }
            */
            // Then respawn
            gazebo_msgs::SpawnModel model;
            std::ifstream ifs;
            ifs.open((ros::package::getPath("wai_oa_gazebo")+"/resources/descriptions/"+s_setup_rep_name+".xacro").c_str());
            std::string xml((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            model.request.model_xml=xml; // Be carefule with xml/urdf VS .xacro -> double check!
            model.request.model_name=s_setup_rep_name;//"template";
            model.request.reference_frame="world";
            model.request.robot_namespace="/wai_world/"+s_setup_rep_name;//"template";
            geometry_msgs::Pose pos_spawn_repres;
            pos_spawn_repres.position.x=vec_d_setup_rep_interaction_switch[0];
            pos_spawn_repres.position.y=vec_d_setup_rep_interaction_switch[1];
            pos_spawn_repres.position.z=vec_d_setup_rep_interaction_switch[2];
            tf::Quaternion qua_spawn_rotation;
            qua_spawn_rotation.setRPY(0.0,0.0,vec_d_setup_rep_interaction_switch[3]);
            qua_spawn_rotation=qua_spawn_rotation.normalize();
            pos_spawn_repres.orientation.w=qua_spawn_rotation.getW();
            pos_spawn_repres.orientation.x=qua_spawn_rotation.getX();
            pos_spawn_repres.orientation.y=qua_spawn_rotation.getY();
            pos_spawn_repres.orientation.z=qua_spawn_rotation.getZ();
            model.request.initial_pose = pos_spawn_repres;
            if(gazebo_spawn_model_client.isValid())
            {
                gazebo_spawn_model_client.waitForExistence();
                gazebo_spawn_model_client.call(model);
            }

            // Respawn rep via RepManager
            wai_oa_session_manager.RespawnRepFromTemplate(s_setup_rep_name);

            //wai_oa_rep_manager.SetRepStateViaService("template",tf::Vector3(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2]),tf::Vector3(0.0,0.0,vec_d_setup_rep_interaction_switch[3]));
            wai_oa_rep_manager.SetRepStateViaService(s_setup_rep_name,tf::Vector3(vec_d_setup_rep_interaction_switch[0],vec_d_setup_rep_interaction_switch[1],vec_d_setup_rep_interaction_switch[2]),tf::Vector3(0.0,0.0,vec_d_setup_rep_interaction_switch[3]));
        }
        /* Old setup LIGHT interface, now implemented as individual interaction!
        else if(s_setup_rep_interaction.compare("light_color")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::string s_setup_rep_interaction_switch;
            nh.getParam(sst_setup_rep_interaction_switch.str(),s_setup_rep_interaction_switch);

            // Rep name is e.g., "light_1"
            pub_col_light_diff_spec=nh.advertise<std_msgs::ColorRGBA>("/wai_world/world/"+s_setup_rep_name+"_color_diff_spec",1);
            SpinAndWaitForSeconds(0.5);
            std_msgs::ColorRGBA msg_col_lights;
            if(s_setup_rep_interaction_switch.compare("default")==0)
            {
                msg_col_lights=col_light_default;
                pub_col_light_diff_spec.publish(msg_col_lights);
            }
            if(s_setup_rep_interaction_switch.compare("cold")==0)
            {
                msg_col_lights=col_light_cold;
                pub_col_light_diff_spec.publish(msg_col_lights);
            }
            else if(s_setup_rep_interaction_switch.compare("warm")==0)
            {
                msg_col_lights=col_light_warm;
                pub_col_light_diff_spec.publish(msg_col_lights);
            }
            else
            {
                // Do nothing...
            }
        }
        // Old setup MARVIN HOLOGRAM interface, now implemented as individual interaction!
        else if(s_setup_rep_interaction.compare("hologram")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::string s_setup_rep_interaction_switch=""; nh.getParam(sst_setup_rep_interaction_switch.str(),s_setup_rep_interaction_switch);
            wai_oa_marvin.EnableHologram(wai_oa_session_manager.GetPathResourcesReps(),s_setup_rep_interaction_switch);
        }
        */
        else if(s_setup_rep_interaction.compare("multiplot")==0 && ros::param::has(sst_setup_rep_interaction_switch.str()))
        {
            std::string s_setup_rep_interaction_switch; nh.getParam(sst_setup_rep_interaction_switch.str(),s_setup_rep_interaction_switch);
            std::string s_syscmd="killall rqt_multiplot";
            int retval=system(s_syscmd.c_str());
            s_syscmd="rqt_multiplot --multiplot-config "+wai_oa_session_manager.GetPathResourcesMultiplots()+s_setup_rep_interaction_switch+".xml --multiplot-run-all &";
            retval=system(s_syscmd.c_str());
        }
        else if(s_setup_rep_interaction.compare("sequence")==0)
        {
            if( ros::param::has(sst_setup_rep_sequence_duration.str())
                && ros::param::has(sst_setup_rep_sequence_state.str())
                && ros::param::has(sst_setup_rep_sequence_twist.str())
                && ros::param::has(sst_setup_rep_sequence_force.str()) )
            {
                PlaySound("IntelSetupRepInitializing");
                wai_oa_session_manager.LoadWorkspacePresenter("testlab");

                std::string s_setup_seq_rep_name="";
                float f_setup_seq_rep_duration=0.0;
                std::vector<double> vec_d_setup_rep_state;
                std::vector<double> vec_d_setup_rep_twist;
                std::vector<double> vec_d_setup_rep_force;
                nh.getParam(sst_setup_rep_name.str(),s_setup_seq_rep_name);
                nh.getParam(sst_setup_rep_sequence_duration.str(),f_setup_seq_rep_duration);
                nh.getParam(sst_setup_rep_sequence_state.str(),vec_d_setup_rep_state);
                nh.getParam(sst_setup_rep_sequence_twist.str(),vec_d_setup_rep_twist);
                nh.getParam(sst_setup_rep_sequence_force.str(),vec_d_setup_rep_force);
                wai_oa_rep_sequencer.UpdateModel(GetSceneCountCurrent(),
                                                 s_setup_seq_rep_name,
                                                 f_setup_seq_rep_duration,
                                                 vec_d_setup_rep_state,
                                                 vec_d_setup_rep_twist,
                                                 vec_d_setup_rep_force);

                PlaySound("IntelSetupRepFinished");
                wai_oa_session_manager.LoadWorkspacePresenter("default");
            }
            else
            {
                ROS_DEBUG_STREAM("REPRESENTATIVE: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("REPRESENTATIVE: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
        }
    }
    else
    {
        ROS_DEBUG_STREAM("REPRESENTATIVE: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup MARVIN As PRESENTER from Script (RESTRICTED!)
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupMarvinAsPresenterFromScript()
{
    std::string s_setup_marvin_as_presenter;
    std::stringstream sst_setup_marvin_as_presenter;
    sst_setup_marvin_as_presenter << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/marvin_presenter";

    if(ros::param::has(sst_setup_marvin_as_presenter.str()))
    {
        nh.getParam(sst_setup_marvin_as_presenter.str(),s_setup_marvin_as_presenter);
        wai_oa_marvin.MarvinSynthesizeText(s_setup_marvin_as_presenter);
    }
    else
    {
        ROS_DEBUG_STREAM("MARVIN AS PRESENTER: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup MARVIN HOLOGRAM from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupMarvinHologramFromScript()
{
    std::string s_setup_marvin_hologram;
    std::stringstream sst_setup_marvin_hologram;
    sst_setup_marvin_hologram << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/marvin_hologram";

    // Disable holograms from previous scenes
    wai_oa_marvin.DisableHologram();

    if(ros::param::has(sst_setup_marvin_hologram.str()))
    {
        nh.getParam(sst_setup_marvin_hologram.str(),s_setup_marvin_hologram);
        wai_oa_marvin.EnableHologram(wai_oa_session_manager.GetPathResourcesReps(),s_setup_marvin_hologram);
    }
    else
    {
        ROS_DEBUG_STREAM("MARVIN HOLOGRAM: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}


/////////////////////////////////////////////////
/// Setup LEARNING MODE from Script
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupLearningModeFromScript()
{
    std::stringstream sst_setup_learning_mode;
    std::string s_setup_learning_mode;

    sst_setup_learning_mode << s_path_nodename + "/" + wai_oa_session_manager.GetSessionName() << "/setup_scene_" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "/learning_mode";
    if(ros::param::has(sst_setup_learning_mode.str()))
    {
        nh.getParam(sst_setup_learning_mode.str(),s_setup_learning_mode);
        if(s_setup_learning_mode.compare("cooperative")==0)
        {
            wai_oa_rep_manager.UpdateModel(s_setup_learning_mode,"link_base");
            SetupLearningModeCooperativeModel();
            SetupLearningModeCooperativeView();
        }
        else if(s_setup_learning_mode.compare("plenum")==0)
        {
            SetupLearningModePlenumModel();
            SetupLearningModePlenumView();
        }
        else
        {
            // Do nothing...
        }
    }
    else
    {
        ROS_DEBUG_STREAM("LEARNING MODE: No setup found for scene #" << wai_oa_session_manager.GetSessionSceneCountCurrent() << "!");
    }
}





// INPUT DEVICE INTERACTIONS:
//----------------------------



/////////////////////////////////////////////////
/// Setup FORCE
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupForceFromTrigger()
{
    tf::Vector3 vc3_force_resulting=vc3_world_wrt_force_endpoint-vc3_world_wrt_force_origin;
    vc3_force_resulting=vc3_force_resulting*10;

    wai_oa_rep_manager.SetRepForceViaService(s_force_name,"link_base",vc3_force_resulting);
    wai_oa_triggers.TriggerJoypadSendFeedback(1.0,1.0); // PS5 controller settings 1.0,0.5

    mrk_oa_arrow_force->UpdateColor(col_grey);
    ((ArrowMarker*)mrk_oa_arrow_force)->UpdateVector(tf::Vector3(0.0,0.0,0.0),tf::Vector3(0.0,0.0,0.0));
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                            tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                            tf::Quaternion(0.0,0.0,0.0,1.0),
                            tf::Vector3(0.025,0.25,0.125),
                            "FORCE",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    pub_mrk_oa_arrow_force.publish(mrk_oa_arrow_force->GetMarker());
}
bool WAIOpenAuditorium::SetupForceOrigin()
{
    vc3_world_wrt_force_origin=tf::Vector3(0.0,0.0,0.0);
    vc3_world_wrt_force_endpoint=tf::Vector3(0.0,0.0,0.0);
    s_force_name="";
    tf::Vector3 vc3_hand_right=wai_oa_presenter.GetHandRightPosition();

    for(unsigned long i=0;i<wai_oa_rep_manager.GetRepLinkStates()->pose.size();i++)
    {
        tf::Vector3 vc3_gazebo_pos;
        vc3_gazebo_pos.setX(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.x);
        vc3_gazebo_pos.setY(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.y);
        vc3_gazebo_pos.setZ(wai_oa_rep_manager.GetRepLinkStates()->pose[i].position.z);

        if(CheckCollision( vc3_hand_right,
                           vc3_gazebo_pos,
                           F_SCENE_TRIGGER_COLL_THRES))
        {
            // Get force origin of selected rep
            vc3_world_wrt_force_origin=vc3_hand_right;

            // Get name of selected rep
            std::string s_key = "::";
            std::string s_force_name_dirty=wai_oa_rep_manager.GetRepLinkStates()->name[i];
            std::string::size_type index = s_force_name_dirty.find(s_key);
            if (index == std::string::npos)
            {
                // Separator not contained, do nothing...
            }
            s_force_name = s_force_name_dirty.substr(0, index);
            ROS_DEBUG_STREAM("REP: Selected is " << s_force_name << ".");

            ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                        tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.025,0.25,0.125),
                        "F ("+s_force_name+")",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);

            return true;
        }
    }

    return false;
}
void WAIOpenAuditorium::SetupForceView(bool b_enabled)
{
    if(b_enabled) tmr_setup_force_view.start();
    else tmr_setup_force_view.stop();
}



/////////////////////////////////////////////////
/// Setup MODELS AND VIEWS as part of SM
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupReceivingRequestAudienceModel()
{
    if(int(vec_hea_audience_request.size())<I_AUDIENCE_REQUEST_QUEUE_SIZE)
    {
        std::stringstream sst_audience_request_queue("AUDIENCE");

        // If connect or shutdown request, just notify panel but dont add to request vector
        if(msg_hea_audience_request_incoming.frame_id.find("CONNECT")!=std::string::npos
            || msg_hea_audience_request_incoming.frame_id.find("SHUTDOWN")!=std::string::npos)
        {
            SendInfoToOAPanel(msg_hea_audience_request_incoming.frame_id);
            return;
        }

        // Check if request from same ID is already queued
        for(int i=0;i<int(vec_hea_audience_request.size());i++)
        {
            if(GetAudienceIDFromRequestMsg(vec_hea_audience_request[i].frame_id)==GetAudienceIDFromRequestMsg(msg_hea_audience_request_incoming.frame_id))
            {
                int i_request_audience_id=GetAudienceIDFromRequestMsg(msg_hea_audience_request_incoming.frame_id);
                wai_oa_audience.GetAudienceID(i_request_audience_id).SendInfo(msg_hea_audience_request_incoming.frame_id+" (SAT)");
                SendInfoToOAPanel(msg_hea_audience_request_incoming.frame_id+" (SAT)");
                return;
            }
            else
            {
                sst_audience_request_queue << vec_hea_audience_request[i].frame_id << std::endl;
            }
        }

        // Requests not saturated yet, add to vector and marker vis. and send notification to panel
        SendInfoToOAPanel(msg_hea_audience_request_incoming.frame_id+" (OK)");

        // Process message types before adding to request vector
        if(msg_hea_audience_request_incoming.frame_id.find("MESSAGE")!=std::string::npos)
        {
            // Remove chat message for visualization in marker (full message is shown in panel session log!)
            std::string s_temp=msg_hea_audience_request_incoming.frame_id;
            size_t pos=s_temp.find(":");
            s_temp.erase(pos);
            msg_hea_audience_request_incoming.frame_id=s_temp;
        }
        else
        {
            // Do nothing...
        }
        vec_hea_audience_request.push_back(msg_hea_audience_request_incoming);
        sst_audience_request_queue << msg_hea_audience_request_incoming.frame_id << std::endl;


        // Update marker accordingly
        ((WAIRepTrigger*)wai_oa_rep_trigger_audience)->UpdateModel(
                    tf::Vector3(2.0,0.9,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    sst_audience_request_queue.str(),col_red,true,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        // If too many requests, dont add to vector, however notify audience and panel...
        int i_request_audience_id=GetAudienceIDFromRequestMsg(msg_hea_audience_request_incoming.frame_id);
        wai_oa_audience.GetAudienceID(i_request_audience_id).SendInfo(msg_hea_audience_request_incoming.frame_id+" (MAX)");
        SendInfoToOAPanel(msg_hea_audience_request_incoming.frame_id+" (MAX)");
    }
}
void WAIOpenAuditorium::SetupReceivingRequestAudienceView()
{
    wai_oa_rep_trigger_audience->UpdateView();
}

void WAIOpenAuditorium::SendInfoToOAPanel(std::string s_info_to_panel)
{
    std_msgs::Header msg_hea_info_to_panel;
    msg_hea_info_to_panel.stamp=ros::Time::now();
    msg_hea_info_to_panel.frame_id=s_info_to_panel;
    while(pub_hea_audience_request_to_panel.getNumSubscribers()==0)
    {
        ros::spinOnce();
        ros::Rate(5.0).sleep();
    }
    pub_hea_audience_request_to_panel.publish(msg_hea_info_to_panel);
}

void WAIOpenAuditorium::SetupHandlingRequestAudienceAcceptModel()
{
    std::stringstream sst_audience_request_queue("AUDIENCE");

    if(!vec_hea_audience_request.empty())
    {
        // Send ACCEPTED confirmation to audience
        int i_request_audience_id=GetAudienceIDFromRequestMsg(vec_hea_audience_request.front().frame_id);
        wai_oa_audience.GetAudienceID(i_request_audience_id).SendInfo(vec_hea_audience_request.front().frame_id+" (ACC)");
        SendInfoToOAPanel(vec_hea_audience_request.front().frame_id+" (ACC)");
        vec_hea_audience_request.erase(vec_hea_audience_request.begin());

        for(int i=0;i<int(vec_hea_audience_request.size());i++)
        {
            sst_audience_request_queue << vec_hea_audience_request[i].frame_id << std::endl;
        }

        // NOW ACTUALLY SELECT FOCUSED ID FROM AUDIENCE:
        // Update audience selected id as member of context
        UpdateAudienceIDSelected(i_request_audience_id);
        // Update model of camera
        std::vector<double> vec_camera_rviz_audience;
        vec_camera_rviz_audience.push_back(-0.5);
        vec_camera_rviz_audience.push_back(-0.75);
        vec_camera_rviz_audience.push_back(1.75);
        vec_camera_rviz_audience.push_back(1.0);
        vec_camera_rviz_audience.push_back(0.0);
        vec_camera_rviz_audience.push_back(0.5);
        wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_audience,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"workspace_audience_"+std::to_string(i_audience_id_selected)+"/link_base",0);
        // Update model of WIM
        UpdateWIMFromAudienceSelected();
        ((WAIRepWIM*)wai_oa_rep_wim)->UpdateModel(
                    tf::Vector3(0.0,0.0,0.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    "WIM",
                    "",
                    col_green,
                    vc3_oa_offset_wim,
                    tf::Vector3(I_AUDIENCE_COUNT_MAX/I_AUDIENCE_LISTENERS_PER_ROW,I_AUDIENCE_LISTENERS_PER_ROW,1),
                    vc3_oa_spacing_wim,
                    vc3_oa_selected_wim,
                    i_audience_id_selected,
                    S_PRESENTER_WORKSPACE_MODEL,
                    wai_oa_session_manager.GetPathResourcesReps(),
                    false); // All coords given in true scale!
        // Channel audio comm. (audio_play) to according audience ID
        std::string s_cmd_kill="rosnode kill /wai_world/world/audio_common/audio_play &";
        int i_sys_rval_kill=system(s_cmd_kill.c_str());
        std::string s_cmd="roslaunch audio_play play.launch ns:=/wai_world/oa"+std::to_string(i_audience_id_selected)+"/audio_common &";
        int i_sys_rval_launch=system(s_cmd.c_str());
    }
    else
    {
        // Do nothing...
    }

    // Update marker accordingly
    ((WAIRepTrigger*)wai_oa_rep_trigger_audience)->UpdateModel(
                tf::Vector3(2.0,0.9,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                sst_audience_request_queue.str(),col_red,true,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupHandlingRequestAudienceAcceptView()
{
    wai_oa_camera_rviz.UpdateView();
    wai_oa_rep_wim->UpdateView();
    wai_oa_rep_trigger_audience->UpdateView();
}
void WAIOpenAuditorium::SetupHandlingRequestAudienceAcceptLeaveState()
{
    // Channel audio comm (Play NODE) to audience ID
    std::string s_cmd_kill="rosnode kill /wai_world/oa"+std::to_string(i_audience_id_selected)+"/audio_common/audio_play &";
    int i_sys_rval_kill=system(s_cmd_kill.c_str());
    std::string s_cmd_launch="roslaunch audio_play play.launch ns:=/wai_world/world/audio_common/audio_play &";
    int i_sys_rval_launch=system(s_cmd_launch.c_str());

    SetupCameraRvizRestorePreviousView();
}

void WAIOpenAuditorium::SetupHandlingRequestAudienceRejectModel()
{
    std::stringstream sst_audience_request_queue("AUDIENCE");

    if(!vec_hea_audience_request.empty())
    {
        // Send ACCEPTED confirmation to audience
        int i_request_audience_id=GetAudienceIDFromRequestMsg(vec_hea_audience_request.front().frame_id);
        wai_oa_audience.GetAudienceID(i_request_audience_id).SendInfo(vec_hea_audience_request.front().frame_id+" (ACC)");
        SendInfoToOAPanel(vec_hea_audience_request.front().frame_id+" (ACC)");
        vec_hea_audience_request.erase(vec_hea_audience_request.begin());

        for(int i=0;i<int(vec_hea_audience_request.size());i++)
        {
            sst_audience_request_queue << vec_hea_audience_request[i].frame_id << std::endl;
        }
    }
    else
    {
        // Do nothing...
    }

    // Update marker accordingly
    ((WAIRepTrigger*)wai_oa_rep_trigger_audience)->UpdateModel(
                tf::Vector3(2.0,0.9,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                sst_audience_request_queue.str(),col_red,true,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupHandlingRequestAudienceRejectView()
{
    wai_oa_rep_trigger_audience->UpdateView();
}


/////////////////////////////////////////////////
/// Setup MARVIN as little helper robot
/////////////////////////////////////////////////
void WAIOpenAuditorium::SetupMarvinModel()
{
    // Marvin set camera
    std::vector<double> vec_camera_rviz_rep_from_detail;
    vec_camera_rviz_rep_from_detail.push_back(3.0); // eye
    vec_camera_rviz_rep_from_detail.push_back(1.5);
    vec_camera_rviz_rep_from_detail.push_back(1.5);
    vec_camera_rviz_rep_from_detail.push_back(0.0); // focus
    vec_camera_rviz_rep_from_detail.push_back(0.0);
    vec_camera_rviz_rep_from_detail.push_back(0.0);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_rep_from_detail,false,false,-1,0.0,"marvin/link_base");
}
void WAIOpenAuditorium::SetupMarvinView()
{
    wai_oa_camera_rviz.UpdateView();
}
void WAIOpenAuditorium::SetupMarvinLeaveState()
{
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default);
    wai_oa_camera_rviz.UpdateView();
}

// MARVIN - Hologram projection...
void WAIOpenAuditorium::SetupMarvinHologramModel()
{
    // Not manually triggered currently...
}
void WAIOpenAuditorium::SetupMarvinHologramView()
{
    // Not manually triggered currently...
}
void WAIOpenAuditorium::SetupMarvinHologramLeaveState()
{
    // Not manually triggered currently...
}

// INTRODUCTION - Trigger intro mode...
void WAIOpenAuditorium::SetupIntroductionModel()
{
    if(I_PRESENTER_PRESENCE_MODE==1 || I_PRESENTER_PRESENCE_MODE==2)
    {
        // Transistion MARVIN to any dedicated workspace for introduction
        wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_introduction,false);
    }
    else if(I_PRESENTER_PRESENCE_MODE==4)
    {
        // Transistion presenter to any dedicated workspace for introduction
        wai_oa_presenter.SetPresenterPose(0.0,3.0,1.0,0.0);
    }
    else
    {
        // Reject PRESENTER Introduction mode without any valid presence mode
        QMessageBox::information(NULL,"PRESENCE Mode","Select a <b>valid PRESENCE mode</b> before (Tab+O)!");
        return;
    }

    // Setup vector for details
    tf::Vector3 vc3_link_focus;
    tf::Vector3 vc3_link_eye;
    vc3_link_focus.setValue(0.0,0.0,0.0);
    vc3_link_eye.setValue(-1.0,2.0,2.0);
    double d_length_eye=vc3_link_eye.length()/2.0;
    wai_oa_camera_rviz.SetTransformDetail(
                tf::Transform(
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(d_length_eye/2.0,0.0,d_length_eye)
                    ));

    // Setup camera and show details
    wai_oa_rep_manager.SetRepStateViaService("table",tf::Vector3(0.05,3.0,0.0));
    wai_oa_camera_rviz.CameraIdleStart(2.0,2*M_PI*0.01,0,tf::Vector3(0.75,3.0,1.0));
    wai_oa_rep_manager.UpdateModel("presenter","link_base");
    wai_oa_rep_manager.EnableRepAndDetail();
}
void WAIOpenAuditorium::SetupIntroductionView()
{
    // Not necessary for intro...
}
void WAIOpenAuditorium::SetupIntroductionLeaveState()
{
    wai_oa_rep_manager.DisableRepAndDetail();
    wai_oa_camera_rviz.CameraIdleStop();
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default);
    wai_oa_camera_rviz.UpdateView();

    // Reset poses of MARVIN or Presenter
    wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_home,false);
    wai_oa_presenter.SetPresenterPose(0.0,0.0,1.0,0.0);
}



// Setup SCENE
void WAIOpenAuditorium::SetupScenePrevModel()
{
    wai_oa_session_manager.SetSessionSceneCountCurrent(false);

    ((WAIRepTrigger*)wai_oa_rep_trigger_scene_prev)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,-1.0,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "SCENE Previous",col_orange,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupScenePrevView()
{
    wai_oa_rep_trigger_scene_prev->UpdateView();
}

void WAIOpenAuditorium::SetupSceneNextModel()
{
    wai_oa_session_manager.SetSessionSceneCountCurrent(true,-1,false); // Set scene_update_cfg to TRUE, to update script "online" on each scene transition, however puts delay on scene transition!

    ((WAIRepTrigger*)wai_oa_rep_trigger_scene_next)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "SCENE Next",col_green,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupSceneNextView()
{
    wai_oa_rep_trigger_scene_next->UpdateView();
}

void WAIOpenAuditorium::SetupSceneSelectModel()
{
    QVBoxLayout* vbox = new QVBoxLayout();

    QDialog* d = new QDialog();
    d->setWindowTitle("SCENE Select");
    QLabel* lbl_scene_select=new QLabel();
    lbl_scene_select->setText("Select a scene:");
    QLabel* lbl_scene_select_preview=new QLabel();

    if(GetSceneCountCurrent()!=0)
    {
        img_scene_select_preview.load(QString::fromStdString(wai_oa_session_manager.GetPathResourcesSlides())+"Slide"+QString::number(GetSceneCountCurrent())+".PNG");
        lbl_scene_select_preview->setPixmap(QPixmap::fromImage(img_scene_select_preview.scaled(256,144)));
    }
    else
    {
        img_scene_select_preview.load(QString::fromStdString(wai_oa_session_manager.GetPathResourcesSlides())+"Slide1.PNG");
        lbl_scene_select_preview->setPixmap(QPixmap::fromImage(img_scene_select_preview.scaled(256,144)));
    }

    QSpinBox* spb_scene_select = new QSpinBox();
    spb_scene_select->setMinimum(1);
    spb_scene_select->setMaximum(wai_oa_session_manager.GetSessionSceneCountMax());
    spb_scene_select->setSingleStep(1);
    spb_scene_select->setValue(GetSceneCountCurrent());

    QObject::connect(spb_scene_select,QOverload<int>::of(&QSpinBox::valueChanged),[=](int i_scene_select)
    {
        img_scene_select_preview.load(QString::fromStdString(wai_oa_session_manager.GetPathResourcesSlides())+"Slide"+QString::number(i_scene_select)+".PNG");
        lbl_scene_select_preview->setPixmap(QPixmap::fromImage(img_scene_select_preview.scaled(256,144)));
    });

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
    vbox->addWidget(lbl_scene_select);
    vbox->addWidget(lbl_scene_select_preview);
    vbox->addWidget(spb_scene_select);
    vbox->addWidget(buttonBox);
    d->setLayout(vbox);
    int result = d->exec();
    int i_frame_id_to_scene=spb_scene_select->value();

    if(result == QDialog::Accepted)
    {
        // Get scene from GUI
        wai_oa_session_manager.SetSessionSceneCountCurrent(true,i_frame_id_to_scene-1);

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "SCENE Select ("+std::to_string(i_frame_id_to_scene)+")",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        // If cancelled, reload current scene
        i_frame_id_to_scene=wai_oa_session_manager.GetSessionSceneCountCurrent();
        wai_oa_session_manager.SetSessionSceneCountCurrent(true,i_frame_id_to_scene-1);
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "SCENE Select (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }




    /*
    int i_frame_id_to_scene=0;
    int i_scn_cnt_max=wai_oa_session_manager.GetSessionSceneCountMax();
    bool ok_scene=false;
    i_frame_id_to_scene = QInputDialog::getInt(
        NULL,
        "SCENE Select",
        QString::fromStdString("Select scene (MIN. 1, MAX. "+std::to_string(i_scn_cnt_max)+"):"),
        1,
        1,
        i_scn_cnt_max,
        1,
        &ok_scene,
        Qt::WindowFlags());

    if(ok_scene==false)
    {
        // If cancelled, reload current scene
        i_frame_id_to_scene=wai_oa_session_manager.GetSessionSceneCountCurrent();
        wai_oa_session_manager.SetSessionSceneCountCurrent(true,i_frame_id_to_scene-1);
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "SCENE Select (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        // Get scene from GUI
        wai_oa_session_manager.SetSessionSceneCountCurrent(true,i_frame_id_to_scene-1);

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "SCENE Select ("+std::to_string(i_frame_id_to_scene)+")",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    */
}
void WAIOpenAuditorium::SetupSceneSelectView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}



// Setup CAMERA RVIZ
void WAIOpenAuditorium::SetupCameraRvizDefaultModel()
{
    SetupCameraRvizCycle(false);
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "CAMERA Default",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupCameraRvizDefaultView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizCycleModel()
{
    SetupCameraRvizCycle(true);
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "CAMERA Cycle",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupCameraRvizCycleView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizSelectModel()
{
    QDialog* d = new QDialog();
    d->setWindowTitle("CAMERA Select");
    QLabel* lbl_camera_rviz_select=new QLabel();
    lbl_camera_rviz_select->setText("Select a camera view:");
    QVBoxLayout* vbox = new QVBoxLayout();
    QComboBox* cmb_camera_rviz_select = new QComboBox();

    std::vector<std::string> vec_s_view_lbls;
    vec_s_view_lbls=wai_oa_session_manager.GetCameraRVizViewLabels();
    for(int i=0;i<vec_s_view_lbls.size();i++)
    {
        cmb_camera_rviz_select->addItem(QString::fromStdString(vec_s_view_lbls[i]));
    }
    //QComboBox* comboBoxB = new QComboBox();
    //comboBoxB->addItems(QStringList() << "A" << "B" << "C");
    //QLineEdit* lineEditA = new QLineEdit();

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                                        | QDialogButtonBox::Cancel);
    QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
    QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));

    vbox->addWidget(lbl_camera_rviz_select);
    vbox->addWidget(cmb_camera_rviz_select);
    //vbox->addWidget(comboBoxB);
    vbox->addWidget(buttonBox);

    d->setLayout(vbox);

    int result = d->exec();
    if(result == QDialog::Accepted)
    {
        std::vector<double> vec_dummy;
        wai_oa_camera_rviz.UpdateModel(vec_dummy,true,true,cmb_camera_rviz_select->currentIndex());

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "CAMERA Select",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "CAMERA Select (DIS)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
}
void WAIOpenAuditorium::SetupCameraRvizSelectView()
{
     wai_oa_rep_trigger_basic->UpdateView();
     wai_oa_camera_rviz.UpdateView();
}

void WAIOpenAuditorium::SetupCameraRvizEnforceModel()
{
    for(int i=0;i<wai_oa_audience.GetAudienceCountMax();i++)
    {
        wai_oa_audience.GetAudienceID(i).SendInfo("CAMERA Enforce"); // "ID"+std::to_string(i)+"-
    }
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "CAMERA Enforce",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupCameraRvizEnforceView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupCameraRvizFollowModel()
{
    // Only follow presenter in 3D-POINTCLOUD presence mode (don't forget to also enable BODY INT. and AVATAR!)
    if(I_PRESENTER_PRESENCE_MODE==4)
    {
        wai_oa_camera_rviz.ToggleCameraFollowPresenter();

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "CAMERA Follow",
                    col_cyan,
                    wai_oa_camera_rviz.GetCameraFollowPresenterEnabled(),
                    true,
                    F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "CAMERA Follow (REJ)",
                    col_cyan,
                    false,
                    true,
                    F_SCENE_TRIGGER_TIMEOUT);
    }
}
void WAIOpenAuditorium::SetupCameraRvizFollowView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizIdleModel()
{
    if(wai_oa_session_manager.GetSessionName().compare("break")==0)
    {
        // Orbiting trajectory
        wai_oa_camera_rviz.CameraIdleStart(7.5,2*M_PI*0.02,0);
    }
    else
    {
        // Cylindric trajectory
        wai_oa_camera_rviz.CameraIdleStart(0.035,2*M_PI*0.05,1);
    }
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "CAMERA Idle",col_cyan,true,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupCameraRvizIdleView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupCameraRvizIdleLeaveState()
{
    RvizCameraIdleStop();
    SetupCameraRvizRestorePreviousView();
}
void WAIOpenAuditorium::SetupPresenceModeModel()
{
    I_PRESENTER_PRESENCE_MODE++;
    if(I_PRESENTER_PRESENCE_MODE>4) I_PRESENTER_PRESENCE_MODE=0;
    if(I_PRESENTER_PRESENCE_MODE==3) I_PRESENTER_PRESENCE_MODE++; // Skip 3D-AVATAR for now!

    // Update model of presenter
    wai_oa_presenter.UpdateModel(I_PRESENTER_PRESENCE_MODE);

    if(I_PRESENTER_PRESENCE_MODE==0) // NO PRESENCE
    {
        wai_oa_camera_rviz.CameraFollowPresenterStop();

        // Intel comments disabling human presenter
        //SpinAndWaitForSeconds(3.0);
        PlaySound("IntelPlenumSettingUpPresenceModeDisabled");

        // Marvin gets shutdown, VOI interactions to robots get disabled
        wai_oa_marvin.Shutdown();

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "PRESENCE (OFF)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else if(I_PRESENTER_PRESENCE_MODE==1) // 2D-AVATAR
    {
        wai_oa_camera_rviz.CameraFollowPresenterStop();
        wai_oa_marvin.Engage(msg_pst_marvin_ref_home); // Move marvin into presenters workspace

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "PRESENCE (2D-AVA)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT); // (bool)wai_oa_presenter.GetPresenceMode()
    }
    if(I_PRESENTER_PRESENCE_MODE==2) // 2D-CAMERA
    {
        wai_oa_camera_rviz.CameraFollowPresenterStop();
        wai_oa_marvin.Engage(msg_pst_marvin_ref_home); // Move marvin into presenters workspace

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "PRESENCE (2D-CAM)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT); // (bool)wai_oa_presenter.GetPresenceMode()
    }
    else if(I_PRESENTER_PRESENCE_MODE==3) // 3D-AVATAR (currently unused!)
    {
        // Shutdown OpenNI 2 first
        int i_sys_rval=0;
        std::string s_command="";
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/driver &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/camera_rgbd_in_nodelet_manager &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_metric &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_metric_rect &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_points &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_rectify_depth &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_registered_hw_metric_rect &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_registered_metric &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/depth_registered_rectify_depth &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/points_xyzrgb_hw_registered &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in/rgb_rectify_color &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in_base_link &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in_base_link1 &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in_base_link2 &"; i_sys_rval=system(s_command.c_str());
        s_command="rosnode kill /wai_world/oa/camera_rgbd_in_base_link3 &"; i_sys_rval=system(s_command.c_str());

        // Move MARVIN and start OpenNI 2 tracker node
        wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_space,true);
        s_command="rosrun openni_tracker openni_tracker &"; i_sys_rval=system(s_command.c_str());

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "PRESENCE (3D-AVA)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else if(I_PRESENTER_PRESENCE_MODE==4) // 3D-POINTCLOUD
    {
        // Move MARVIN
        wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_space,false);

        /* SWITCH FROM 3D-AVATAR TO 3D-POINTCLOUD
        // Shutdown tracker node first
        int i_sys_rval=0;
        std::string s_command="";
        s_command="rosnode kill /wai_world/oa/openni_tracker &"; i_sys_rval=system(s_command.c_str());

        // LAUNCH openni2
        SpinAndWaitForSeconds(5.0);
        s_command="roslaunch openni2_launch openni2.launch __ns:=/wai_world/oa &"; i_sys_rval=system(s_command.c_str());
        */

        ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                    tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.25,0.125),
                    "PRESENCE (3D-PCL)",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    }
    else
    {
        // Do nothing...xxx
    }

}
void WAIOpenAuditorium::SetupPresenceModeView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupBodyInteractionModel()
{
    wai_oa_presenter.ToggleBodyInteraction();
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "BODY INT",col_cyan,wai_oa_presenter.GetBodyInteractionEnabled(),true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupBodyInteractionView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupAvatarModel()
{
    wai_oa_presenter.ToggleAvatar();

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "AVATAR",col_cyan,wai_oa_presenter.GetAvatarEnabled(),true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupAvatarView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}
void WAIOpenAuditorium::SetupLecternModel()
{
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "LECTERN",col_cyan,ToggleTriggerLectern(),true,F_SCENE_TRIGGER_TIMEOUT);
    if(B_ENABLE_LECTERN)
    {
        wai_oa_rep_manager.SetRepStateViaService("lectern",tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-0.5,0.0,0.0));
    }
    else
    {
        wai_oa_rep_manager.SetRepStateViaService("lectern",tf::Vector3(2.45,-2.0,0.0));
    }
}
void WAIOpenAuditorium::SetupLecternView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupTableModel()
{
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "TABLE",col_cyan,ToggleTriggerTable(),true,F_SCENE_TRIGGER_TIMEOUT);
    if(B_ENABLE_TABLE)
    {
        wai_oa_rep_manager.SetRepStateViaService("table",tf::Vector3(0.05,0.0,0.0));
    }
    else
    {
        wai_oa_rep_manager.SetRepStateViaService("table",tf::Vector3(1.75,-2.0,0.0));
    }
}
void WAIOpenAuditorium::SetupTableView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupAudioModel()
{
    bool b_trigger_audio=ToggleTriggerAudio();

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "AUDIO",col_cyan,b_trigger_audio,true,F_SCENE_TRIGGER_TIMEOUT);

    if(b_trigger_audio==true)
    {
        int i_sys_rval=system("roslaunch audio_capture capture.launch ns:=/wai_world/world/audio_common &");
    }
    else
    {
        int i_sys_rval=system("rosnode kill /wai_world/world/audio_common/audio_capture &");
    }
}
void WAIOpenAuditorium::SetupAudioView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupSketchModel()
{
    wai_oa_sketch.ReopenSketch();

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "SKETCH",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupSketchView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupLearningModePlenumModel()
{
    // Reset learning mode to plenum as default
    wai_oa_session_manager.PreparePlenumMode();

    wai_oa_presenter.SetPresenterPose(0.0,0.0,1.0,0.0);
    wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_home,false);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_default,true,false,-1,F_SCENE_TRANSITION_TIMEOUT,"world",0,true);

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "MODE Plen.",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupLearningModePlenumView()
{
    wai_oa_camera_rviz.UpdateView();
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupLearningModeCooperativeModel()
{
    // Location for coop mode is given in world frame:
    // The REPs state is set to this subworkspace, if coop mode is enabled.
    tf::Vector3 vc3_rep_pos_coop(-3.5,1.0,0.0);
    tf::Quaternion qua_rep_ori_coop(0.0,0.0,0.0,1.0);

    wai_oa_session_manager.PrepareCooperativeMode(wai_oa_rep_manager.GetRepSelected(),vc3_rep_pos_coop);

    wai_oa_presenter.SetPresenterPose(vc3_rep_pos_coop.getX(),vc3_rep_pos_coop.getY(),1.0,3.141592654);

    geometry_msgs::PoseStamped pst_marvin_coop;
    pst_marvin_coop.header.frame_id="world";
    pst_marvin_coop.header.stamp=ros::Time::now();
    pst_marvin_coop.pose.position.x=vc3_rep_pos_coop.getX()-1.0;
    pst_marvin_coop.pose.position.y=vc3_rep_pos_coop.getY()-1.0;
    pst_marvin_coop.pose.position.z=1.0;
    pst_marvin_coop.pose.orientation.w=qua_rep_ori_coop.getW();
    pst_marvin_coop.pose.orientation.x=qua_rep_ori_coop.getX();
    pst_marvin_coop.pose.orientation.y=qua_rep_ori_coop.getY();
    pst_marvin_coop.pose.orientation.z=qua_rep_ori_coop.getZ();
    wai_oa_marvin.FlyToWaypoint(pst_marvin_coop,false);

    std::vector<double> vec_camera_rviz;
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getX()+2.5);
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getY()+2.5);
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getZ()+5.0);
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getX()-1.0);
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getY()-1.0);
    vec_camera_rviz.push_back(vc3_rep_pos_coop.getZ());
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"world",0,true);

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "MODE Coop.",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupLearningModeCooperativeView()
{
    wai_oa_camera_rviz.UpdateView();
    wai_oa_rep_trigger_basic->UpdateView();
}


void WAIOpenAuditorium::SetupVoiceListenModel()
{
    if(I_PRESENTER_PRESENCE_MODE!=0) // Only if human presenter is actually present!
    {
        // Initiate voice recognition (amongst others, Marvin)
        msg_pic_intent_action_goal.header.frame_id="world";
        msg_pic_intent_action_goal.header.stamp=ros::Time::now();
        //msg_pic_intent_action_goal.goal_id.id="0";
        msg_pic_intent_action_goal.goal_id.stamp=ros::Time::now();
        msg_pic_intent_action_goal.goal.context_url="open_auditorium"; // Name of *.rhn file!
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTellMood");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTellAge");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTellWeight");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTellSatisfaction");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinFlyToAudience");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTellAboutYourself");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinFlyToRep");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinSetupRepAtWaypoint");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinTrigger");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinFlyToWaypoint");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelTrigger");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelShutdownRep");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelTellAboutYourself");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelSelectRep");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelEngageRep");
        msg_pic_intent_action_goal.goal.intents.push_back("IntelSelectAudience");
        msg_pic_intent_action_goal.goal.intents.push_back("MarvinSetupAudienceAtWaypoint");
        msg_pic_intent_action_goal.goal.require_endpoint=false;
        pub_pic_intent_action_goal.publish(msg_pic_intent_action_goal);
    }
    else
    {
        //SpinAndWaitForSeconds(2.0);
        PlaySound("IntelErrorPresence");
    }

    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "VOICE",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupVoiceListenView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupVoicePromptModel()
{
    if(I_PRESENTER_PRESENCE_MODE!=0) // Only if human presenter is actually present!
    {
        if(I_PRESENTER_PROMPT_MODE==1)
        {
            // Initiate voice recognition (amongst others, Marvin)
            msg_pic_transcript_action_goal.header.frame_id="world";
            msg_pic_transcript_action_goal.header.stamp=ros::Time::now();
            msg_pic_transcript_action_goal.goal_id.stamp=ros::Time::now();
            msg_pic_transcript_action_goal.goal.enable_automatic_punctuation=true;
            pub_pic_transcript_action_goal.publish(msg_pic_transcript_action_goal);
            ROS_DEBUG("Sent msg_pic_transcript_action_goal...");
        }
        else
        {
            // Use input dialog to generate text-based prompt (e.g., if mic is not available)
            bool ok_prompt=false;
            msg_str_transcript_action_goal.data="";
            QString qst_prompt=QInputDialog::getText(
                        NULL,
                        QString::fromStdString("PROMPT Marvin"),
                        "Prompt:",
                        QLineEdit::Normal,
                        "How many kilometers is one nautical mile?",
                        &ok_prompt,
                        Qt::WindowFlags());
            if(ok_prompt)
            {
                msg_str_transcript_action_goal.data=qst_prompt.toStdString();
                pub_str_transcript_action_goal.publish(msg_str_transcript_action_goal);
            }
        }
    }
    // ...
    ((WAIRepTrigger*)wai_oa_rep_trigger_basic)->UpdateModel(
                tf::Vector3(F_CAMERA_RGBD_RANGE_MAX-1.0,-0.5,1.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "PROMPT",col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
}
void WAIOpenAuditorium::SetupVoicePromptView()
{
    wai_oa_rep_trigger_basic->UpdateView();
}

void WAIOpenAuditorium::SetupEvalModel()
{
    int i_data_size_x=I_AUDIENCE_COUNT_MAX;
    int i_data_size_y=12;
    int i_data_size_z=16;

    wai_oa_audience.ImportEvalsGroupFromFileText(wai_oa_session_manager.GetPathResourcesEvalsGroupAndSubject());

    float* graph_data_eval_part = new float[i_data_size_x*i_data_size_y*i_data_size_z];
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                *(graph_data_eval_part+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=
                        wai_oa_audience.GetAudienceID(x).GetEvalPartScoreSubperiod(y,z)/
                        wai_oa_audience.GetAudienceID(x).GetEvalPartScoreMax();
            }
        }
    }

    float* graph_data_eval_exam = new float[i_data_size_x*i_data_size_y*i_data_size_z];
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                *(graph_data_eval_exam+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=
                        wai_oa_audience.GetAudienceID(x).GetEvalExamScoreSubperiod(y,z)/
                        wai_oa_audience.GetAudienceID(x).GetEvalExamScoreMax();
            }
        }
    }

    float* graph_data_eval_overall = new float[i_data_size_x*i_data_size_y*i_data_size_z];
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                // z[0] is reserverd/not used,
                // put overall score into element z[1]!
                // all other elements remain -1.0, not used resp.!
                if(z==1)
                {
                    *(graph_data_eval_overall+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=
                            wai_oa_audience.GetAudienceID(x).GetEvalOverallStatsSubperiod(y);
                }
                else
                {
                    *(graph_data_eval_overall+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z)=-1024.0;
                }
            }
        }
    }

    float f_part_scores_overall[i_data_size_x];
    float f_part_fractions_overall[i_data_size_x];
    float f_exam_scores_overall[i_data_size_x];
    float f_exam_fractions_overall[i_data_size_x];
    float f_part_stats[i_data_size_x];
    float f_exam_stats[i_data_size_x];
    float f_overall_stats[i_data_size_x];
    std::string s_overall_standing[i_data_size_x];
    for(int x=0;x<i_data_size_x;x++)
    {
        wai_oa_audience.GetAudienceID(x).UpdateEvalStatsOverall(
                    &f_part_scores_overall[x],
                    &f_part_fractions_overall[x],
                    &f_exam_scores_overall[x],
                    &f_exam_fractions_overall[x],
                    &f_part_stats[x],
                    &f_exam_stats[x],
                    &f_overall_stats[x],
                    &s_overall_standing[x]);
    }

    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateGraphDataEval(
                graph_data_eval_part,
                graph_data_eval_exam,
                graph_data_eval_overall,
                i_data_size_x,
                i_data_size_y,
                i_data_size_z,
                f_part_scores_overall,
                f_part_fractions_overall,
                f_exam_scores_overall,
                f_exam_fractions_overall,
                f_overall_stats,
                s_overall_standing,
                i_audience_id_selected);
}
void WAIOpenAuditorium::SetupEvalView()
{
    // Setup camera on Eval-Graph
    std::vector<double> vec_camera_rviz_eval_from_id;
    vec_camera_rviz_eval_from_id.push_back(-3.0455+i_audience_id_selected*0.75);
    vec_camera_rviz_eval_from_id.push_back(1.4609);
    vec_camera_rviz_eval_from_id.push_back(3.3354);
    vec_camera_rviz_eval_from_id.push_back(0.49764+i_audience_id_selected*0.75);
    vec_camera_rviz_eval_from_id.push_back(2.8087);
    vec_camera_rviz_eval_from_id.push_back(0.010454);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_eval_from_id,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"graph_3d_eval");
    wai_oa_camera_rviz.UpdateView();

    // Update EVAL graph
    wai_oa_rep_graph_3d_eval->UpdateView();
}


void WAIOpenAuditorium::SetupEvalGraphInspectModel()
{
    //int i_audience_id=int(msg_pts_rviz_clicked.point.x/0.75);
    //int i_audience_sub=int(msg_pts_rviz_clicked.point.y/0.5);
    int i_audience_id=std::round(msg_pts_rviz_clicked.point.x/0.75);
    int i_audience_sub=std::round(msg_pts_rviz_clicked.point.y/0.5);

    /*
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateGraphCursor(
                tf::Vector3(msg_pts_rviz_clicked.point.x,msg_pts_rviz_clicked.point.y,1.25),
                wai_oa_audience.ImportEvalsIDSubperiodFromTextfile(wai_oa_session_manager.GetPathResourcesEvalsGroupAndSubject(),i_audience_id,i_audience_sub));
                */
    ((WAIRepGraph3D*)wai_oa_rep_graph_3d_eval)->UpdateGraphCursor(
                tf::Vector3(msg_pts_rviz_clicked.point.x,msg_pts_rviz_clicked.point.y,1.25),
                wai_oa_audience.GetEvalsAudienceIDSubperiod(i_audience_id,i_audience_sub));
    msg_pts_rviz_clicked.point.x=0.0;
    msg_pts_rviz_clicked.point.y=0.0;
    msg_pts_rviz_clicked.point.z=0.0;
}

int WAIOpenAuditorium::GetAudienceIDFromRequestMsg(std::string s_request_msg_frame_id)
{
    std::string::size_type pos = s_request_msg_frame_id.find('-');
    if (pos!=std::string::npos)
    {
        std::string s_id=s_request_msg_frame_id.substr(0, pos);
        size_t last_index = s_id.find_last_not_of("0123456789");
        std::string s_cid = s_id.substr(last_index + 1);
        return atoi(s_cid.c_str());
    }
    else
    {
        return 0; // Return default selection of audience/listener id 0
    }
}

void WAIOpenAuditorium::SetupEvalGraphInspectView()
{
    wai_oa_rep_graph_3d_eval->UpdateView();
}


// EVAL METAPHORS Helper Functions (ToDo: Merge Spwaning/Deleting into Rep Manager!)
void WAIOpenAuditorium::SpawnEvalMetaphor(int i_count_score,
                                         int i_count_overall,
                                         std::string s_model_name_score,
                                         std::string s_model_name_overall,
                                         std::string s_model_name_diff,
                                         std::string s_eval_metaphor_name)
{
    int i_count_diff=i_count_overall-i_count_score;
    std::string s_path_to_models=wai_oa_session_manager.GetPathResourcesDescriptions();

    std::ifstream ifs_eval_model_score,ifs_eval_model_overall,ifs_eval_model_diff;
    ifs_eval_model_score.open(s_path_to_models+s_model_name_score+".xacro");
    ifs_eval_model_overall.open(s_path_to_models+s_model_name_overall+".xacro");
    ifs_eval_model_diff.open(s_path_to_models+s_model_name_diff+".xacro");
    std::string xml_eval_model_score((std::istreambuf_iterator<char>(ifs_eval_model_score)), std::istreambuf_iterator<char>());
    std::string xml_eval_model_overall((std::istreambuf_iterator<char>(ifs_eval_model_overall)), std::istreambuf_iterator<char>());
    std::string xml_eval_model_diff((std::istreambuf_iterator<char>(ifs_eval_model_diff)), std::istreambuf_iterator<char>());

    gazebo_msgs::SpawnModel gaz_spm_eval_model_score;
    gazebo_msgs::SpawnModel gaz_spm_eval_model_overall;
    gazebo_msgs::SpawnModel gaz_spm_eval_model_diff;
    gaz_spm_eval_model_score.request.model_xml=xml_eval_model_score;
    gaz_spm_eval_model_overall.request.model_xml=xml_eval_model_overall;
    gaz_spm_eval_model_diff.request.model_xml=xml_eval_model_diff;

    tf::Vector3 vc3_eval_model_pos; tf::Quaternion qua_dummy; tf::Vector3 vc3_dummy1; tf::Vector3 vc3_dummy2; tf::Vector3 vc3_dummy3;
    wai_oa_rep_manager.GetRepStateViaService(s_eval_metaphor_name,
                                             "link_base",
                                             &vc3_eval_model_pos,
                                             &qua_dummy,
                                             &vc3_dummy1,
                                             &vc3_dummy2,
                                             &vc3_dummy3);
    geometry_msgs::Pose pos_eval_model;
    pos_eval_model.position.x=vc3_eval_model_pos.getX();
    pos_eval_model.position.y=vc3_eval_model_pos.getY();
    pos_eval_model.position.z=vc3_eval_model_pos.getZ()+3.0;
    pos_eval_model.orientation.w=1.0;
    pos_eval_model.orientation.x=0.0;
    pos_eval_model.orientation.y=0.0;
    pos_eval_model.orientation.z=0.0;

    // BOWL Metaphor
    if(s_eval_metaphor_name.compare("eval_bowl")==0)
    {
        for(int i=0;i<i_count_overall;i++)
        {
            // Spawn "how many points could have been reached"
            gaz_spm_eval_model_overall.request.model_name=(s_model_name_overall+std::to_string(i)).c_str();
            pos_eval_model.position.x=vc3_eval_model_pos.getX()-0.5;
            pos_eval_model.position.y=vc3_eval_model_pos.getY();
            gaz_spm_eval_model_overall.request.initial_pose=pos_eval_model;

            if(gazebo_spawn_model_client.isValid())
            {
                gazebo_spawn_model_client.waitForExistence();
                gazebo_spawn_model_client.call(gaz_spm_eval_model_overall);
            }

            if(i<i_count_score)
            {
                // Spawn "achieved points"
                gaz_spm_eval_model_score.request.model_name=(s_model_name_score+std::to_string(i)).c_str();
                pos_eval_model.position.x=vc3_eval_model_pos.getX()+0.5;
                pos_eval_model.position.y=vc3_eval_model_pos.getY()+0.5;
                gaz_spm_eval_model_score.request.initial_pose=pos_eval_model;
                if(gazebo_spawn_model_client.isValid())
                {
                    gazebo_spawn_model_client.waitForExistence();
                    gazebo_spawn_model_client.call(gaz_spm_eval_model_score);
                }
            }
            else
            {
                // Spawn "non-achieved points"
                gaz_spm_eval_model_diff.request.model_name=(s_model_name_diff+std::to_string(i-i_count_score)).c_str();
                pos_eval_model.position.x=vc3_eval_model_pos.getX()+0.5;
                pos_eval_model.position.y=vc3_eval_model_pos.getY()-0.5;
                gaz_spm_eval_model_diff.request.initial_pose=pos_eval_model;
                if(gazebo_spawn_model_client.isValid())
                {
                    gazebo_spawn_model_client.waitForExistence();
                    gazebo_spawn_model_client.call(gaz_spm_eval_model_diff);
                }
            }

            SpinAndWaitForSeconds(1.0/F_NODE_SAMPLE_FREQUENCY); // 0.25
        }
    }
    else if(s_eval_metaphor_name.compare("eval_weight_balance")==0)
    {
        int cnt_score=0,cnt_frac=0;

        // First spawn achievable points in middle
        for(int i=0;i<i_count_overall;i++)
        {
            gaz_spm_eval_model_overall.request.model_name=(s_model_name_overall+std::to_string(i)).c_str();
            gaz_spm_eval_model_overall.request.initial_pose=pos_eval_model;
            if(gazebo_spawn_model_client.isValid())
            {
                gazebo_spawn_model_client.waitForExistence();
                gazebo_spawn_model_client.call(gaz_spm_eval_model_overall);
            }
            SpinAndWaitForSeconds(1.0/F_NODE_SAMPLE_FREQUENCY); // 0.25
        }
        // Afterwards, switched spawning of score and diff
        for(int i=0;i<i_count_overall;i++)
        {
            if(cnt_score<i_count_score)
            {
                // Spawn "achieved points"
                gaz_spm_eval_model_score.request.model_name=(s_model_name_score+std::to_string(cnt_score)).c_str();
                pos_eval_model.position.y=vc3_eval_model_pos.getY()+2.0;
                gaz_spm_eval_model_score.request.initial_pose=pos_eval_model;
                if(gazebo_spawn_model_client.isValid())
                {
                    gazebo_spawn_model_client.waitForExistence();
                    gazebo_spawn_model_client.call(gaz_spm_eval_model_score);
                }
                SpinAndWaitForSeconds(1.0/F_NODE_SAMPLE_FREQUENCY); // 0.25
                cnt_score++;
            }
            if(cnt_frac<i_count_diff)
            {
                // Spawn "non-achieved points"
                gaz_spm_eval_model_diff.request.model_name=(s_model_name_diff+std::to_string(cnt_frac)).c_str();
                pos_eval_model.position.y=vc3_eval_model_pos.getY()-2.0;
                gaz_spm_eval_model_diff.request.initial_pose=pos_eval_model;
                if(gazebo_spawn_model_client.isValid())
                {
                    gazebo_spawn_model_client.waitForExistence();
                    gazebo_spawn_model_client.call(gaz_spm_eval_model_diff);
                }
                SpinAndWaitForSeconds(1.0/F_NODE_SAMPLE_FREQUENCY); // 0.25
                cnt_frac++;
            }
        }
    }
    else
    {
        ROS_WARN("Eval Metaphor: Metaphor not found!");
        // Do nothing...
    }
}

void WAIOpenAuditorium::CleanupEvalMetaphor(std::string s_model_name_score,
                                           std::string s_model_name_overall,
                                           std::string s_model_name_diff)
{
    gazebo_msgs::DeleteModel gaz_dem_eval_model;
    int i=0;
    do
    {
        gaz_dem_eval_model.request.model_name=s_model_name_overall+std::to_string(i);
        if(gazebo_delete_model_client.isValid())
        {
            gazebo_delete_model_client.waitForExistence();
            gazebo_delete_model_client.call(gaz_dem_eval_model);
        }
        SpinAndWaitForSeconds(2.0*1.0/F_NODE_SAMPLE_FREQUENCY);
        i++;
    }while(gaz_dem_eval_model.response.success);
    i=0;
    do
    {
        gaz_dem_eval_model.request.model_name=s_model_name_score+std::to_string(i);
        if(gazebo_delete_model_client.isValid())
        {
            gazebo_delete_model_client.waitForExistence();
            gazebo_delete_model_client.call(gaz_dem_eval_model);
        }
        SpinAndWaitForSeconds(2.0*1.0/F_NODE_SAMPLE_FREQUENCY);
        i++;
    }while(gaz_dem_eval_model.response.success);
    i=0;
    do
    {
        gaz_dem_eval_model.request.model_name=s_model_name_diff+std::to_string(i);
        if(gazebo_delete_model_client.isValid())
        {
            gazebo_delete_model_client.waitForExistence();
            gazebo_delete_model_client.call(gaz_dem_eval_model);
        }
        SpinAndWaitForSeconds(2.0*1.0/F_NODE_SAMPLE_FREQUENCY);
        i++;
    }while(gaz_dem_eval_model.response.success);
}

// Eval BOWL - Grading Metaphor
void WAIOpenAuditorium::SetupEvalBowlModel()
{
    // First setup camera
    std::vector<double> vec_camera_rviz;
    vec_camera_rviz.push_back(2.0);
    vec_camera_rviz.push_back(2.0);
    vec_camera_rviz.push_back(4.0);
    vec_camera_rviz.push_back(0.0);
    vec_camera_rviz.push_back(0.0);
    vec_camera_rviz.push_back(0.0);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"eval_bowl/link_base",0);
    wai_oa_camera_rviz.UpdateView();

    // Setup spheres marker for Rviz
    wai_oa_rep_manager.EnableEvalMetaphors();

    SetupEvalModel();
    int i_scores_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamScoresOverall();
    int i_fracs_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamFractionsOverall();
    ROS_WARN("Preparing Eval BOWL Metaphor - Score: %d, Fractions: %d",i_scores_overall,i_fracs_overall);
    SpawnEvalMetaphor(i_scores_overall,
                      i_fracs_overall,
                     "eval_sphere_green",
                     "eval_sphere_orange",
                     "eval_sphere_red",
                     "eval_bowl");
}
void WAIOpenAuditorium::SetupEvalBowlView()
{

}
void WAIOpenAuditorium::SetupEvalBowlLeaveState()
{
    // Cleanup metaphor
    ROS_WARN("Cleaning up Eval BOWL Metaphor. Please wait a second...");
    CleanupEvalMetaphor("eval_sphere_green",
                       "eval_sphere_orange",
                       "eval_sphere_red");
    wai_oa_rep_manager.DisableEvalMetaphors();
    ROS_WARN("Cleaning up Eval BOWL Metaphor. DONE!");

    // Reset camera
    SetupCameraRvizCycle(false);
}



// Eval WEIGHT BALANCE - Grading Metaphor
void WAIOpenAuditorium::SetupEvalBalanceModel()
{
    // First setup camera
    std::vector<double> vec_camera_rviz;
    vec_camera_rviz.push_back(2.5);
    vec_camera_rviz.push_back(2.5);
    vec_camera_rviz.push_back(9.0);
    vec_camera_rviz.push_back(0.0);
    vec_camera_rviz.push_back(0.0);
    vec_camera_rviz.push_back(0.0);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"eval_weight_balance/link_base",0);
    wai_oa_camera_rviz.UpdateView();

    // Setup spheres marker for Rviz
    wai_oa_rep_manager.EnableEvalMetaphors();

    SetupEvalModel();
    int i_scores_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamScoresOverall();
    int i_fracs_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamFractionsOverall();
    ROS_WARN("Preparing Eval WEIGHT BALANCE Metaphor - Score: %d, Fractions: %d",i_scores_overall,i_fracs_overall);
    SpawnEvalMetaphor(i_scores_overall,i_fracs_overall,
                     "eval_sphere_green",
                     "eval_sphere_orange",
                     "eval_sphere_red",
                     "eval_weight_balance");

    // Setup spheres marker for Rviz
    // ...
}
void WAIOpenAuditorium::SetupEvalBalanceView()
{

}
void WAIOpenAuditorium::SetupEvalBalanceLeaveState()
{
    // Cleanup metaphor
    ROS_WARN("Cleaning up Eval BALANCE Metaphor. Please wait a second...");
    CleanupEvalMetaphor("eval_sphere_green",
                       "eval_sphere_orange",
                       "eval_sphere_red");
    wai_oa_rep_manager.DisableEvalMetaphors();
    ROS_WARN("Cleaning up Eval BALANCE Metaphor. DONE!");

    // Reset camera
    SetupCameraRvizCycle(false);
}

void WAIOpenAuditorium::SetupEvalMarvinModel()
{
    // Get evals
    SetupEvalModel();
    float f_scores_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamScoresOverall();
    float f_fracs_overall=wai_oa_audience.GetAudienceID(i_audience_id_selected).GetEvalPartExamFractionsOverall();
    float f_marvin_flight_height=4.0*f_scores_overall/f_fracs_overall;
    ROS_WARN("Preparing Eval MARVIN Metaphor - Score: %3.3f, Fractions: %3.3f, Height: %3.3f[m]",
             f_scores_overall,f_fracs_overall,f_marvin_flight_height);

    // Position camera
    std::vector<double> vec_camera_rviz_eval_marvin;
    vec_camera_rviz_eval_marvin.push_back(-1.5);
    vec_camera_rviz_eval_marvin.push_back(-1.5);
    vec_camera_rviz_eval_marvin.push_back(3.0);
    vec_camera_rviz_eval_marvin.push_back(0.0);
    vec_camera_rviz_eval_marvin.push_back(0.0);
    vec_camera_rviz_eval_marvin.push_back(0.0);
    wai_oa_camera_rviz.UpdateModel(vec_camera_rviz_eval_marvin,false,false,-1,F_SCENE_TRANSITION_TIMEOUT,"marvin/link_base");
    wai_oa_camera_rviz.UpdateView();

    // Get Pose of Eval MARVIN metaphore
    tf::Vector3 vc3_eval_model_pos; tf::Quaternion qua_dummy; tf::Vector3 vc3_dummy1; tf::Vector3 vc3_dummy2; tf::Vector3 vc3_dummy3;
    wai_oa_rep_manager.GetRepStateViaService("eval_marvin",
                                             "link_base",
                                             &vc3_eval_model_pos,
                                             &qua_dummy,
                                             &vc3_dummy1,
                                             &vc3_dummy2,
                                             &vc3_dummy3);
    // Fly to starting pose
    wai_oa_marvin.Say("MarvinEvalAcknowledge");
    geometry_msgs::PoseStamped pst_eval_marvin;
    pst_eval_marvin.header.frame_id="world";
    pst_eval_marvin.header.stamp=ros::Time::now();
    pst_eval_marvin.pose.position.x=vc3_eval_model_pos.getX()+1.0;
    pst_eval_marvin.pose.position.y=vc3_eval_model_pos.getY();
    pst_eval_marvin.pose.position.z=vc3_eval_model_pos.getZ()+0.5;
    pst_eval_marvin.pose.orientation.w=0.0;
    pst_eval_marvin.pose.orientation.x=0.0;
    pst_eval_marvin.pose.orientation.y=0.0;
    pst_eval_marvin.pose.orientation.z=1.0;
    wai_oa_marvin.FlyToWaypoint(pst_eval_marvin,true);

    // Fly to dedicated metaphore height
    wai_oa_marvin.Say("MarvinEvalGainingHeight");
    pst_eval_marvin.pose.position.x=vc3_eval_model_pos.getX()+1.0;
    pst_eval_marvin.pose.position.y=vc3_eval_model_pos.getY();
    pst_eval_marvin.pose.position.z=vc3_eval_model_pos.getZ()+f_marvin_flight_height;
    wai_oa_marvin.FlyToWaypoint(pst_eval_marvin,true);

    // Fly "through obstacle" at eval height
    if((f_scores_overall/f_fracs_overall)<0.5) wai_oa_marvin.Say("MarvinEvalReachedHeightInsufficient");
    if((f_scores_overall/f_fracs_overall)>=0.5 && (f_scores_overall/f_fracs_overall)<0.625) wai_oa_marvin.Say("MarvinEvalReachedHeightSufficient");
    if((f_scores_overall/f_fracs_overall)>=0.625 && (f_scores_overall/f_fracs_overall)<0.75) wai_oa_marvin.Say("MarvinEvalReachedHeightSatisfactory");
    if((f_scores_overall/f_fracs_overall)>=0.75 && (f_scores_overall/f_fracs_overall)<0.875) wai_oa_marvin.Say("MarvinEvalReachedHeightGood");
    if((f_scores_overall/f_fracs_overall)>=0.875) wai_oa_marvin.Say("MarvinEvalReachedHeightVeryGood");
    pst_eval_marvin.pose.position.x=vc3_eval_model_pos.getX()-1.0;
    pst_eval_marvin.pose.position.y=vc3_eval_model_pos.getY();
    pst_eval_marvin.pose.position.z=vc3_eval_model_pos.getZ()+f_marvin_flight_height;
    wai_oa_marvin.FlyToWaypoint(pst_eval_marvin,true);
    wai_oa_marvin.Say("MarvinEvalComplete");
}
void WAIOpenAuditorium::SetupEvalMarvinView()
{

}
void WAIOpenAuditorium::SetupEvalMarvinLeaveState()
{
    // Get Pose of Eval MARVIN metaphore
    tf::Vector3 vc3_eval_model_pos; tf::Quaternion qua_dummy; tf::Vector3 vc3_dummy1; tf::Vector3 vc3_dummy2; tf::Vector3 vc3_dummy3;
    wai_oa_rep_manager.GetRepStateViaService("eval_marvin",
                                             "link_base",
                                             &vc3_eval_model_pos,
                                             &qua_dummy,
                                             &vc3_dummy1,
                                             &vc3_dummy2,
                                             &vc3_dummy3);
    // Gain height to return home
    geometry_msgs::PoseStamped pst_eval_marvin;
    pst_eval_marvin.header.frame_id="world";
    pst_eval_marvin.header.stamp=ros::Time::now();
    pst_eval_marvin.pose.position.x=vc3_eval_model_pos.getX()-1.0;
    pst_eval_marvin.pose.position.y=vc3_eval_model_pos.getY();
    pst_eval_marvin.pose.position.z=vc3_eval_model_pos.getZ()+5.0;
    pst_eval_marvin.pose.orientation.w=1.0;
    pst_eval_marvin.pose.orientation.x=0.0;
    pst_eval_marvin.pose.orientation.y=0.0;
    pst_eval_marvin.pose.orientation.z=0.0;
    wai_oa_marvin.FlyToWaypoint(pst_eval_marvin,true);

    // Approach home
    pst_eval_marvin.pose.position.x=msg_pst_marvin_ref_home.pose.position.x-1.0;
    pst_eval_marvin.pose.position.y=msg_pst_marvin_ref_home.pose.position.y;
    pst_eval_marvin.pose.position.z=msg_pst_marvin_ref_home.pose.position.z+3.0;
    wai_oa_marvin.FlyToWaypoint(pst_eval_marvin,true);

    // Return home
    wai_oa_marvin.FlyToWaypoint(msg_pst_marvin_ref_home,true);
    wai_oa_marvin.ReportOnTaskComplete();

    // Reset camera
    SetupCameraRvizCycle(false);
}



ros::Time WAIOpenAuditorium::GetTimeTriggerActivated()
{
    return tim_trigger_activated;
}
void WAIOpenAuditorium::SetTimeTriggerActivated(ros::Time tim_trigger)
{
    tim_trigger_activated=tim_trigger;
}
int WAIOpenAuditorium::GetSceneCountCurrent()
{
    return wai_oa_session_manager.GetSessionSceneCountCurrent();
}
int WAIOpenAuditorium::GetSceneCountMax()
{
    return wai_oa_session_manager.GetSessionSceneCountMax();
}
bool WAIOpenAuditorium::GetCameraRvizEnabled()
{
    return B_ENABLE_CAMERA_RVIZ_IDLE;
}
bool WAIOpenAuditorium::GetSessionStartupFinished()
{
    return B_EVENT_STARTUP_FINISHED;
}
void WAIOpenAuditorium::RvizCameraIdleStop()
{
    wai_oa_camera_rviz.CameraIdleStop();
}
void WAIOpenAuditorium::ToggleRepDetailRotation()
{
    wai_oa_rep_manager.ToggleRepDetailRotation();
}





/////////////////////////////////////////////////
/// Run method with ROS spin
/////////////////////////////////////////////////
void WAIOpenAuditorium::run()
{
    // Init objects
    ros::Rate ros_rate(F_NODE_SAMPLE_FREQUENCY);

    // State machine transition to StatePlenumPresenting
    this->StateTransitionTo(new StatePlenumPresenting,true);

    // ROS Loop
    while(ros::ok())
    {
        // CHECK FOR TRIGGERED INPUTS AND SPIN
        wai_oa_triggers.CheckTriggersAll();
        wai_oa_presenter.SetJoypadInputAxis(wai_oa_triggers.GetJoypadAxesBodyInteraction()); // Interface to check for joypad axis trigger on sides of presenter
        ros::spinOnce();
        ros_rate.sleep();
    }
}
