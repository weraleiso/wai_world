#include<wai_oa_audience_listener.h>



/////////////////////////////////////////////////
/// Implementation of state machine
/////////////////////////////////////////////////
void WAIOAAudienceListener::StateTransitionTo(WAIAuditState *state)
{
    ROS_WARN("State machine: Changing state to %s.",typeid(*state).name());
    if(this->m_wai_oa_audience_listener_state!=NULL)
    {
        delete this->m_wai_oa_audience_listener_state;
    }
    this->m_wai_oa_audience_listener_state=state;
    this->m_wai_oa_audience_listener_state->SetContext(this);
}

// Common requests of context amongst all(!) states
void WAIOAAudienceListener::RequestSetupModel()
{
    this->m_wai_oa_audience_listener_state->HandleSetupModel();
}
void WAIOAAudienceListener::RequestSetupView()
{
    this->m_wai_oa_audience_listener_state->HandleSetupView();
}
void WAIOAAudienceListener::RequestSetupLeaveState()
{
    this->m_wai_oa_audience_listener_state->HandleSetupLeaveState();
}



// Default state
void StateListening::HandleSetupModel()
{

}
void StateListening::HandleSetupView()
{

}
void StateListening::HandleSetupLeaveState()
{

}



/////////////////////////////////////////////////
/// Allow only one object to be instantiated
/////////////////////////////////////////////////
WAIOAAudienceListener* WAIOAAudienceListener::m_ins_node=0;
WAIOAAudienceListener* WAIOAAudienceListener::getInstance()
{
    if(m_ins_node==0)
    {
        m_ins_node=new WAIOAAudienceListener();
    }
    return m_ins_node;
}



//////////////////////////////////////////////////
/// Construct object including all initialization
//////////////////////////////////////////////////
WAIOAAudienceListener::WAIOAAudienceListener():
    m_hdl_it(m_hdl_node),
    m_wai_oa_image_processor(new HandsAndHeadDetector(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
{
    // Resolve parameters from launchfile
    m_s_path_nodename=ros::this_node::getName()+"/";
    m_hdl_node.getParam(m_s_path_nodename+"F_NODE_SAMPLE_FREQUENCY",F_NODE_SAMPLE_FREQUENCY);
    m_hdl_node.getParam(m_s_path_nodename+"F_SCENE_TRANSITION_TIMEOUT",F_SCENE_TRANSITION_TIMEOUT);
    m_hdl_node.getParam(m_s_path_nodename+"F_SCENE_TRIGGER_TIMEOUT",F_SCENE_TRIGGER_TIMEOUT);
    m_hdl_node.getParam(m_s_path_nodename+"F_SCENE_TRIGGER_COLL_THRES",F_SCENE_TRIGGER_COLL_THRES);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_X",F_CAMERA_RGBD_RESOLUTION_X);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_Y",F_CAMERA_RGBD_RESOLUTION_Y);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_SCALE",F_CAMERA_RGBD_RESOLUTION_SCALE);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_FX",F_CAMERA_RGBD_FX);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_FY",F_CAMERA_RGBD_FY);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_CX",F_CAMERA_RGBD_CX);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_CY",F_CAMERA_RGBD_CY);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_RANGE_MIN",F_CAMERA_RGBD_RANGE_MIN);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_RANGE_MAX",F_CAMERA_RGBD_RANGE_MAX);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_THRESHOLD_DIST",F_CAMERA_RGBD_THRESHOLD_DIST);
    m_hdl_node.getParam(m_s_path_nodename+"F_CAMERA_RGBD_THRESHOLD_BOUNDS",F_CAMERA_RGBD_THRESHOLD_BOUNDS);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_KALMAN",B_ENABLE_KALMAN);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_CAMERA_RVIZ_FLY_IN",B_ENABLE_CAMERA_RVIZ_FLY_IN);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW",B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW",B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_BODY_INTERACTION",B_ENABLE_BODY_INTERACTION);
    m_hdl_node.getParam(m_s_path_nodename+"B_ENABLE_AVATAR",B_ENABLE_AVATAR);
    m_hdl_node.getParam(m_s_path_nodename+"F_LISTENER_MOOD",F_LISTENER_MOOD);
    m_hdl_node.getParam(m_s_path_nodename+"F_LISTENER_PRESENCE_VISUAL",F_LISTENER_PRESENCE_VISUAL);

    // Resolve resource paths and configs for this session...
    m_s_path_resources=ros::package::getPath("wai_oa_gazebo")+"/resources/";
    m_s_path_resources_sessions_listener=m_s_path_resources+"sessions_listener/";
    m_qst_path_session=QString::fromStdString(m_s_path_resources_sessions_listener+"break/");
    m_s_path_resources_reps=m_s_path_resources+"reps/";
    m_s_path_resources_logo=m_s_path_resources+"icons/open_auditorium_logo.png";

    if(ros::get_environment_variable(m_s_audience_id,"WAI_OA_AUDIENCE_ID"))
    {
        ROS_WARN_STREAM("OA Audience - Found AUDIENCE ID as \""+m_s_audience_id+"\"");
    }
    else
    {
        ROS_WARN_STREAM("OA Audience - No AUDIENCE ID found! Exiting...");
        exit(1);
    }

    // Init colors
    m_col_invisible.r=0.0; m_col_invisible.g=0.0; m_col_invisible.b=0.0; m_col_invisible.a=0.0;
    m_col_white.r=1.0; m_col_white.g=1.0; m_col_white.b=1.0; m_col_white.a=1.0;
    m_col_black.r=0.0; m_col_black.g=0.0; m_col_black.b=0.0; m_col_black.a=1.0;
    m_col_grey.r=0.5; m_col_grey.g=0.5; m_col_grey.b=0.5; m_col_grey.a=0.5;
    m_col_red_trans.r=1.0; m_col_red_trans.g=0.0; m_col_red_trans.b=0.0; m_col_red_trans.a=0.3;
    m_col_red.r=1.0; m_col_red.g=0.0; m_col_red.b=0.0; m_col_red.a=0.9;
    m_col_red_opaque.r=1.0; m_col_red_opaque.g=0.0; m_col_red_opaque.b=0.0; m_col_red_opaque.a=1.0;
    m_col_green_trans.r=0.0; m_col_green_trans.g=1.0; m_col_green_trans.b=0.0; m_col_green_trans.a=0.3;
    m_col_green.r=0.0; m_col_green.g=1.0; m_col_green.b=0.0; m_col_green.a=0.9;
    m_col_green_opaque.r=0.0; m_col_green_opaque.g=1.0; m_col_green_opaque.b=0.0; m_col_green_opaque.a=1.0;
    m_col_blue_trans.r=0.0; m_col_blue_trans.g=0.0; m_col_blue_trans.b=1.0; m_col_blue_trans.a=0.3;
    m_col_blue.r=0.0; m_col_blue.g=0.0; m_col_blue.b=1.0; m_col_blue.a=0.9;
    m_col_blue_opaque.r=0.0; m_col_blue_opaque.g=0.0; m_col_blue_opaque.b=1.0; m_col_blue_opaque.a=1.0;
    m_col_red_light_opaque.r=219.0/255.0; m_col_red_light_opaque.g=68.0/255.0; m_col_red_light_opaque.b=55.0/255.0; m_col_red_light_opaque.a=1.0;
    m_col_cyan_trans.r=0.0; m_col_cyan_trans.g=1.0; m_col_cyan_trans.b=1.0; m_col_cyan_trans.a=0.3;
    m_col_cyan.r=0.0; m_col_cyan.g=1.0; m_col_cyan.b=1.0; m_col_cyan.a=0.9;
    m_col_cyan_opaque.r=0.0; m_col_cyan_opaque.g=1.0; m_col_cyan_opaque.b=1.0; m_col_cyan_opaque.a=1.0;
    m_col_orange.r=1.0; m_col_orange.g=0.65; m_col_orange.b=0.0; m_col_orange.a=0.9;
    m_col_orange_trans.r=1.0; m_col_orange_trans.g=0.65; m_col_orange_trans.b=0.0; m_col_orange_trans.a=0.3;
    m_col_oa_trans.r=0.101960784; m_col_oa_trans.g=0.42745098; m_col_oa_trans.b=0.588235294; m_col_oa_trans.a=0.3;
    m_col_oa_shiny.r=0.101960784; m_col_oa_shiny.g=0.42745098; m_col_oa_shiny.b=0.588235294; m_col_oa_shiny.a=0.1;
    m_col_oa.r=0.101960784; m_col_oa.g=0.42745098; m_col_oa.b=0.588235294; m_col_oa.a=0.9;
    m_col_oa_opaque.r=0.101960784; m_col_oa_opaque.g=0.42745098; m_col_oa_opaque.b=0.588235294; m_col_oa_opaque.a=1.0;

    // Init ROS Publishers
    //m_pub_img_camera_2d=m_hdl_it.advertise("camera_rgb_out/usb_cam/image_raw",1);
    //m_pub_img_projector=m_hdl_it.advertise("projector/image_raw",1);
    m_pub_img_camera_2d=m_hdl_node.advertise<sensor_msgs::CompressedImage>("camera_rgb_out/usb_cam/image_raw/compressed",10,true); // Added latched publisher!
    m_pub_img_projector=m_hdl_node.advertise<sensor_msgs::CompressedImage>("projector/image_raw/compressed",10,true); // Added latched publisher!
    m_pub_img_camera_audit_rgbd_rgb=m_hdl_it.advertise("camera_rgbd_out"+m_s_audience_id+"/rgb/image_raw",1);
    m_pub_img_camera_audit_rgbd_depth=m_hdl_it.advertise("camera_rgbd_out"+m_s_audience_id+"/depth/image_raw",1);
    m_pub_cai_img_camera_audit_rgbd_rgb=m_hdl_node.advertise<sensor_msgs::CameraInfo>("camera_rgbd_out"+m_s_audience_id+"/rgb/camera_info",1);
    m_pub_cai_img_camera_audit_rgbd_depth=m_hdl_node.advertise<sensor_msgs::CameraInfo>("camera_rgbd_out"+m_s_audience_id+"/depth/camera_info",1);
    m_pub_cpl_rviz_camera=m_hdl_node.advertise<view_controller_msgs::CameraPlacement>("camera_rviz/placement",1,true);
    m_pub_msg_hea_audience_request=m_hdl_node.advertise<std_msgs::Header>("/wai_world/oa/audience_request",1,true); // Modified: 10
    m_pub_hea_ping_to_presenter=m_hdl_node.advertise<std_msgs::Header>("/wai_world/oa/ping_from_audience_"+m_s_audience_id,1);
    m_pub_fma_ethical_properties_to_presenter=m_hdl_node.advertise<std_msgs::Float32MultiArray>("/wai_world/oa/ethical_properties_from_audience",1);
    m_pub_s_oa_status=m_hdl_node.advertise<std_msgs::String>("oa_status",1);

    // Welcome audience
    SetOAStatusLabel("WELCOME TO W.A.I. WORLD! Intel is busy with preparing your workspace...");

    // Init ROS Subscribers
    image_transport::TransportHints hints("compressed",ros::TransportHints()); // MODIFIED! disabled hints for now.
    m_sub_img_camera_rgbd_rgb.shutdown();
    m_sub_img_camera_rgbd_depth.shutdown();
    m_sub_img_camera_rgbd_rgb=m_hdl_it.subscribe("camera_rgbd_in"+m_s_audience_id+"/rgb/image_raw",1,&WAIOAAudienceListener::cb_sub_img_camera_rgbd_rgb,this);
    m_sub_img_camera_rgbd_depth=m_hdl_it.subscribe("camera_rgbd_in"+m_s_audience_id+"/depth/image",1,&WAIOAAudienceListener::cb_sub_img_camera_rgbd_depth,this);
    m_sub_cai_img_camera_rgbd_rgb=m_hdl_node.subscribe("camera_rgbd_in"+m_s_audience_id+"/rgb/camera_info",1,&WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_rgb,this);
    m_sub_cai_img_camera_rgbd_depth=m_hdl_node.subscribe("camera_rgbd_in"+m_s_audience_id+"/depth/camera_info",1,&WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_depth,this);
    m_sub_img_camera_livecam.shutdown();
    m_sub_img_camera_2d.shutdown();
    //m_sub_img_camera_livecam=m_hdl_it.subscribe("livecam/usb_cam/image_raw",1,&WAIOAAudienceListener::cb_sub_img_camera_livecam,this);//,hints);
    m_sub_img_camera_livecam=m_hdl_node.subscribe("livecam/usb_cam/image_raw/compressed",1,&WAIOAAudienceListener::cb_sub_img_camera_livecam,this);
    //m_sub_img_camera_2d=m_hdl_it.subscribe("camera_rgb_in/usb_cam/image_raw",1,&WAIOAAudienceListener::cb_sub_img_camera_2d,this);//,hints);
    m_sub_img_camera_2d=m_hdl_node.subscribe("camera_rgb_in/usb_cam/image_raw/compressed",1,&WAIOAAudienceListener::cb_sub_img_camera_2d,this);
    m_sub_cpl_rviz_camera=m_hdl_node.subscribe("/wai_world/oa/camera_rviz/placement",1,&WAIOAAudienceListener::cb_sub_cpl_rviz_camera,this);
    m_sub_joy_controller=m_hdl_node.subscribe("joy",1,&WAIOAAudienceListener::cb_sub_joy_controller,this);
    m_sub_bol_mob_camera_rviz_follow_presenter=m_hdl_node.subscribe("mob_camera_rviz_follow_presenter",1,&WAIOAAudienceListener::cb_sub_bol_mob_camera_rviz_follow_presenter,this);
    m_sub_bol_mob_3d_mode=m_hdl_node.subscribe("mob_3d_mode",1,&WAIOAAudienceListener::cb_sub_bol_mob_3d_mode,this);
    m_sub_bol_mob_body_interaction=m_hdl_node.subscribe("mob_body_interaction",1,&WAIOAAudienceListener::cb_sub_bol_mob_body_interaction,this);
    m_sub_bol_mob_avatar=m_hdl_node.subscribe("mob_avatar",1,&WAIOAAudienceListener::cb_sub_bol_mob_avatar,this);
    m_sub_bol_mob_question=m_hdl_node.subscribe("mob_question",1,&WAIOAAudienceListener::cb_sub_bol_mob_question,this);
    m_sub_bol_mob_break=m_hdl_node.subscribe("mob_break",1,&WAIOAAudienceListener::cb_sub_bol_mob_break,this);
    m_sub_bol_mob_message=m_hdl_node.subscribe("mob_message",1,&WAIOAAudienceListener::cb_sub_bol_mob_message,this);
    m_sub_hea_ping_from_presenter=m_hdl_node.subscribe("ping_to_audience",1,&WAIOAAudienceListener::cb_sub_hea_ping_from_presenter,this);
    m_sub_hea_info_from_presenter=m_hdl_node.subscribe("info_to_audience",1,&WAIOAAudienceListener::cb_sub_hea_info_from_presenter,this);

    // Send confirmation for connection attempt to presenter
    std::string s_audience_request="ID"+m_s_audience_id+"-CONNECT";
    m_msg_hea_audience_request.stamp=ros::Time::now();
    m_msg_hea_audience_request.frame_id=s_audience_request;
    while(m_pub_msg_hea_audience_request.getNumSubscribers()==0)
    {
        ROS_WARN("Audience Listener: Waiting for Presenter to accept requests...");
        ros::spinOnce();
        ros::Rate(1.0).sleep();
    }
    m_pub_msg_hea_audience_request.publish(m_msg_hea_audience_request);
    SetOAStatusLabel("Intel notified the presenter that you are there...");

    // Init X11 display for user inputs and screen properties
    m_dsp_x11_display=XOpenDisplay(NULL);

    // Init user input device events
    I_CAMERA_RVIZ_CYCLE_VIEW=0;
    B_EVENT_INPDEV_SESSION_LOAD=false;
    B_EVENT_INPDEV_SCENE_PREV=false;
    B_EVENT_INPDEV_SCENE_NEXT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_MOOD=false;
    B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    B_EVENT_INPDEV_SKETCH=false;
    B_EVENT_INPDEV_SHARE=false;
    B_EVENT_INPDEV_SELF_EVALUATE=false;
    I_EVENT_INPDEV_REQUEST=0;

    m_tim_trigger_action=ros::Time::now();
    m_i_scene_count_current=0;
    m_i_scene_count_max=1;

    // Init transforms
    m_vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);

    // Init intrinsic parameters from CameraInfo topic (however, takes longer!)
    /*
    boost::shared_ptr<sensor_msgs::CameraInfo const> shared_cai_img_camera_rgbd;
    sensor_msgs::CameraInfo m_msg_cai_img_camera_rgbd;
    shared_cai_img_camera_rgbd=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/depth/camera_info",m_hdl_node);
    if(shared_cai_img_camera_rgbd != NULL){
        m_msg_cai_img_camera_rgbd=*shared_cai_img_camera_rgbd;
    }
    F_CAMERA_RGBD_FX=m_msg_cai_img_camera_rgbd.K[0];
    F_CAMERA_RGBD_FY=m_msg_cai_img_camera_rgbd.K[4];
    F_CAMERA_RGBD_CX=m_msg_cai_img_camera_rgbd.K[2];
    F_CAMERA_RGBD_CY=m_msg_cai_img_camera_rgbd.K[5];
    */
    m_wai_oa_image_processing_strategy=new HandsAndHeadDetector(
                F_CAMERA_RGBD_RANGE_MIN,
                F_CAMERA_RGBD_RANGE_MAX,
                F_CAMERA_RGBD_THRESHOLD_DIST,
                F_CAMERA_RGBD_THRESHOLD_BOUNDS,
                F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE,
                F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE,
                F_CAMERA_RGBD_FX*F_CAMERA_RGBD_RESOLUTION_SCALE,
                F_CAMERA_RGBD_FY*F_CAMERA_RGBD_RESOLUTION_SCALE,
                F_CAMERA_RGBD_CX*F_CAMERA_RGBD_RESOLUTION_SCALE,
                F_CAMERA_RGBD_CY*F_CAMERA_RGBD_RESOLUTION_SCALE);
    m_wai_oa_image_processor.ChangeProcessingStrategy(m_wai_oa_image_processing_strategy);

    // Init images and texture plugins for 2D-webcam, livecam, and projector
    m_mat_img_camera_livecam=cv::imread(m_s_path_resources_logo);
    m_mat_img_camera_2d=cv::imread(m_s_path_resources_logo);
    m_pub_img_camera_2d.publish(EncodeImage(m_mat_img_camera_2d));
    cv::Mat mat_img_logo_resized;
    cv::resize(m_mat_img_camera_2d,mat_img_logo_resized,cv::Size(960,540));
    m_pub_img_projector.publish(EncodeImage(mat_img_logo_resized));

    // Init 2D avatar
    m_qt_qpo_mouse_cursor_pos.setX(0);
    m_qt_qpo_mouse_cursor_pos.setY(0);
    m_qt_qpo_mouse_cursor_pos_old.setX(0);
    m_qt_qpo_mouse_cursor_pos_old.setY(0);
    m_cv_pnt_mouse_cursor_vel.x=0;
    m_cv_pnt_mouse_cursor_vel.y=0;

    // Init triggers
    m_wai_oaa_trg_action=WAISymbol::create_representative("TRIGGER");
    m_wai_oaa_trg_action->Initialize(&m_hdl_node,
                                   m_s_path_nodename,
                                   "wai_sym_audience_trigger", //GET_OBJECT_NAME(m_wai_oaa_trg_action),
                                   "workspace_audience_"+m_s_audience_id+"/link_base");
    SetActionTriggerLabel("ACTION");

    m_wai_oaa_ooi_head=WAISymbol::create_representative("OOI");
    m_wai_oaa_ooi_head->Initialize(&m_hdl_node,
                                   m_s_path_nodename,
                                   "wai_sym_audience_head", //GET_OBJECT_NAME(m_wai_oaa_trg_action),
                                   "camera_rgbd_out"+m_s_audience_id+"_depth_optical_frame");
    ((WAIRepOOI*)m_wai_oaa_ooi_head)->UpdateModel(
                m_vc3_camera_rgbd_wrt_head,
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.025,0.075),"Head",m_col_oa_shiny,"head",m_s_path_resources_reps,true,1.1);

    // Init request message
    m_msg_hea_audience_request.stamp.sec=0;
    m_msg_hea_audience_request.stamp.nsec=0;
    m_msg_hea_audience_request.frame_id="Request!";

    // Init timers
    m_tmr_avatar_2d=m_hdl_node.createTimer(ros::Duration(1.0/F_NODE_SAMPLE_FREQUENCY)*10.0,&WAIOAAudienceListener::cb_tmr_avatar_2d,this,false,true);
}



/////////////////////////////////////////////////
/// Delete m_ins_node
/////////////////////////////////////////////////
WAIOAAudienceListener::~WAIOAAudienceListener()
{
    delete m_wai_oa_audience_listener_state;
    delete m_ins_node;
}



/////////////////////////////////////////////////
/// Callback to receive inputs from joypad
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_joy_controller(const sensor_msgs::JoyPtr& msg)
{
    m_msg_joy_input_dev=*msg;

    B_EVENT_INPDEV_SESSION_LOAD=false;
    B_EVENT_INPDEV_SCENE_PREV=false;
    B_EVENT_INPDEV_SCENE_NEXT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    B_EVENT_INPDEV_SKETCH=false;
    B_EVENT_INPDEV_SHARE=false;
    B_EVENT_INPDEV_MOOD=false;
    B_EVENT_INPDEV_SELF_EVALUATE=false;
    I_EVENT_INPDEV_REQUEST=0;

    // SHARE + OPTIONS
    if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[9]==1)
    {
        // PRESENTER MODE
        B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=true;
    }
    // JSLeftButton + JSRightButton
    else if(m_msg_joy_input_dev.buttons[11]==1 && m_msg_joy_input_dev.buttons[12]==1)
    {
        // BODY INTERACTION
        B_EVENT_INPDEV_BODY_INTERACTION=true;
    }

    // OPTIONS + JSLeftButton
    else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[11]==1)
    {
        // SCENE PREVIOUS
        B_EVENT_INPDEV_SCENE_PREV=true;
    }
    // OPTIONS + JSRightButton
    else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[12]==1)
    {
        // SCENE NEXT
        B_EVENT_INPDEV_SCENE_NEXT=true;
    }

    // OPTIONS + CURSOR DOWN
    else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[14]==1)
    {
        // CAMERA RVIZ DEFAULT
        B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=true;
    }
    // OPTIONS + CURSOR UP
    else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[13]==1)
    {
        // CAMERA RVIZ CYCLE
        B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=true;
    }

    // SHARE + RECTANGLE
    else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[3]==1)
    {
        // FOLLOW RVIZ CAMERA PRESENTER
        B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=true;
    }
    // SHARE + CROSS
    else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[0]==1)
    {
        // AVATAR
        B_EVENT_INPDEV_AVATAR=true;
    }
    // SHARE + TRIANGLE
    else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[2]==1)
    {
        // QUESTION
        I_EVENT_INPDEV_REQUEST=1;
    }
    // SHARE + CIRCLE
    else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[1]==1)
    {
        // BREAK
        I_EVENT_INPDEV_REQUEST=2;
    }
    // SHARE + PSButton
    else if(m_msg_joy_input_dev.buttons[8]==1 && m_msg_joy_input_dev.buttons[10]==1)
    {
        // SEND MESSAGE
        I_EVENT_INPDEV_REQUEST=3;
    }
    // SHARE + PSButton
    else if(m_msg_joy_input_dev.buttons[9]==1 && m_msg_joy_input_dev.buttons[10]==1)
    {
        // ...
        //I_EVENT_INPDEV_REQUEST=4;
    }
    else
    {
        // Do nothing...
    }
}



/////////////////////////////////////////////////
/// Callback to receive inputs from mobile dev
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_bol_mob_3d_mode(const std_msgs::BoolPtr& msg)
{
    B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=msg->data;
}
void WAIOAAudienceListener::cb_sub_bol_mob_camera_rviz_follow_presenter(const std_msgs::BoolPtr& msg)
{
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=msg->data;
}
void WAIOAAudienceListener::cb_sub_bol_mob_body_interaction(const std_msgs::BoolPtr& msg)
{
    B_EVENT_INPDEV_BODY_INTERACTION=msg->data;
}
void WAIOAAudienceListener::cb_sub_bol_mob_avatar(const std_msgs::BoolPtr& msg)
{
    B_EVENT_INPDEV_AVATAR=msg->data;
}
void WAIOAAudienceListener::cb_sub_bol_mob_question(const std_msgs::BoolPtr& msg)
{
    if(msg->data==true) I_EVENT_INPDEV_REQUEST=1;
    else I_EVENT_INPDEV_REQUEST=0;
}
void WAIOAAudienceListener::cb_sub_bol_mob_break(const std_msgs::BoolPtr& msg)
{
    if(msg->data==true) I_EVENT_INPDEV_REQUEST=2;
    else I_EVENT_INPDEV_REQUEST=0;
}
void WAIOAAudienceListener::cb_sub_bol_mob_message(const std_msgs::BoolPtr& msg)
{
    if(msg->data==true) I_EVENT_INPDEV_REQUEST=3;
    else I_EVENT_INPDEV_REQUEST=0;
}



////////////////////////////////////////////////////
void WAIOAAudienceListener::cb_tmr_avatar_2d(const ros::TimerEvent& event)
{
    m_qt_qpo_mouse_cursor_pos=QApplication::desktop()->cursor().pos();
    m_cv_pnt_mouse_cursor_vel.x=(m_qt_qpo_mouse_cursor_pos.x()-m_qt_qpo_mouse_cursor_pos_old.x())*F_NODE_SAMPLE_FREQUENCY;
    m_cv_pnt_mouse_cursor_vel.y=(m_qt_qpo_mouse_cursor_pos.y()-m_qt_qpo_mouse_cursor_pos_old.y())*F_NODE_SAMPLE_FREQUENCY;

    int i_vel_min=400;
    int i_vel_max=1200;
    int i_vel_delta=400;
    if(abs(m_cv_pnt_mouse_cursor_vel.x)>=i_vel_min && abs(m_cv_pnt_mouse_cursor_vel.x)<=i_vel_max) m_cv_pnt_mouse_cursor_vel.x=m_cv_pnt_mouse_cursor_vel.x/i_vel_delta;
    else if(m_cv_pnt_mouse_cursor_vel.x>i_vel_max) m_cv_pnt_mouse_cursor_vel.x=i_vel_max/i_vel_delta;
    else if(m_cv_pnt_mouse_cursor_vel.x<-i_vel_max) m_cv_pnt_mouse_cursor_vel.x=-i_vel_max/i_vel_delta;
    else m_cv_pnt_mouse_cursor_vel.x=0;
    if(abs(m_cv_pnt_mouse_cursor_vel.y)>=i_vel_min && abs(m_cv_pnt_mouse_cursor_vel.y)<=i_vel_max) m_cv_pnt_mouse_cursor_vel.y=m_cv_pnt_mouse_cursor_vel.y/i_vel_delta;
    else if(m_cv_pnt_mouse_cursor_vel.y>i_vel_max) m_cv_pnt_mouse_cursor_vel.y=i_vel_max/i_vel_delta;
    else if(m_cv_pnt_mouse_cursor_vel.y<-i_vel_max) m_cv_pnt_mouse_cursor_vel.y=-i_vel_max/i_vel_delta;
    else m_cv_pnt_mouse_cursor_vel.y=0;

    m_mat_img_avatar_2d=cv::imread(m_s_path_resources+
                                 "avatar2d_listener/image"+
                                 std::to_string(m_cv_pnt_mouse_cursor_vel.x)+
                                 std::to_string(m_cv_pnt_mouse_cursor_vel.y)+
                                 ".png");
    if(!m_mat_img_avatar_2d.empty())
    {
        m_pub_img_camera_2d.publish(EncodeImage(m_mat_img_avatar_2d));
    }

    m_qt_qpo_mouse_cursor_pos_old=m_qt_qpo_mouse_cursor_pos;
}



////////////////////////////////////////////////////
/// Callback to receive ping request from presenter
////////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_hea_ping_from_presenter(const std_msgs::HeaderPtr& msg)
{
    // Original interface to return ping stats
    std_msgs::Header msg_hea_ping_to_presenter;
    msg_hea_ping_to_presenter.stamp=msg->stamp; // Return timestamp of presenter
    std::stringstream sstr_ping_to_presenter;
    sstr_ping_to_presenter << "ping:from_audience_" << m_s_audience_id;
    msg_hea_ping_to_presenter.frame_id=sstr_ping_to_presenter.str();
    m_pub_hea_ping_to_presenter.publish(msg_hea_ping_to_presenter);

    // Ethical properties returned may contain "ping" in one direction only:
    ros::Time tim_now=ros::Time::now();
    ros::Time tim_ping_sent=msg_hea_ping_to_presenter.stamp;
    float f_ping_oneway=(tim_now-tim_ping_sent).toSec();


    // Use ping interface to also send ethical low-level properties to the (presenters) workspace
    std_msgs::Float32MultiArray msg_fma_ethical_properties_to_presenter;
    msg_fma_ethical_properties_to_presenter.data.clear();
    msg_fma_ethical_properties_to_presenter.data.push_back( std::stof(m_s_audience_id) ); // ACTOR's ID
    msg_fma_ethical_properties_to_presenter.data.push_back( 1.0 ); // ACTOR's POSITION X Y Z
    msg_fma_ethical_properties_to_presenter.data.push_back( 2.0 );
    msg_fma_ethical_properties_to_presenter.data.push_back( 3.0 );
    msg_fma_ethical_properties_to_presenter.data.push_back( F_LISTENER_PRESENCE_VISUAL );
    msg_fma_ethical_properties_to_presenter.data.push_back( F_LISTENER_MOOD );
    msg_fma_ethical_properties_to_presenter.data.push_back( f_ping_oneway );
    // ...
    m_pub_fma_ethical_properties_to_presenter.publish(msg_fma_ethical_properties_to_presenter);
}



/////////////////////////////////////////////////
/// Callback to receive info msgs from presenter
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_hea_info_from_presenter(const std_msgs::HeaderPtr& msg)
{
    m_msg_hea_info_from_presenter=*msg;
    I_EVENT_INFO_FROM_PRESENTER=1;
    if(m_msg_hea_info_from_presenter.frame_id.find("CAMERA Enforce")!=std::string::npos)
    {
        I_EVENT_INFO_FROM_PRESENTER=2;
    }
    else if(m_msg_hea_info_from_presenter.frame_id.find("PRE-SHUTDOWN")!=std::string::npos)
    {
        QMessageBox::warning(NULL,"SHUTDOWN","The Presenter has shut down the session!");
        ros::shutdown();
    }
    else
    {
        // Do nothing...
    }
}



///////////////////////////////////////////////////
/// Callback for receiving camera placements
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_cpl_rviz_camera(const view_controller_msgs::CameraPlacementPtr& msg)
{
    if(B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW
        || B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW)
    {
        m_pub_cpl_rviz_camera.publish(*msg);
    }
}


///////////////////////////////////////////////////
/// Callback for LinkStat messages from Gazebo
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_lns_gazebo(const gazebo_msgs::LinkStatesPtr& msg)
{
    m_msg_lns_gazebo=(*msg);
}



/////////////////////////////////////////////////
/// Callback to receive camera intrinsics
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr& msg)
{
    if(F_LISTENER_PRESENCE_VISUAL!=1.0f) return;

    // Receive intrinsics via camera_info topic
    //F_CAMERA_RGBD_FX=msg->K[0];
    //F_CAMERA_RGBD_FY=msg->K[4];
    //F_CAMERA_RGBD_CX=msg->K[2];
    //F_CAMERA_RGBD_CY=msg->K[5];
    F_CAMERA_RGBD_P_FX=msg->P[0];
    F_CAMERA_RGBD_P_FY=msg->P[5];
    F_CAMERA_RGBD_P_CX=msg->P[2];
    F_CAMERA_RGBD_P_CY=msg->P[6];
    msg->K[0]=msg->K[0]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[4]=msg->K[4]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[2]=msg->K[2]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[5]=msg->K[5]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[0]=msg->P[0]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[5]=msg->P[5]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[2]=msg->P[2]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[6]=msg->P[6]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->header.frame_id="camera_rgbd_out"+m_s_audience_id+"_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    m_pub_cai_img_camera_audit_rgbd_rgb.publish(msg);
}
void WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr& msg)
{
    if(F_LISTENER_PRESENCE_VISUAL!=1.0f) return;

    // Receive intrinsics via camera_info topic
    //F_CAMERA_RGBD_FX=msg->K[0];
    //F_CAMERA_RGBD_FY=msg->K[4];
    //F_CAMERA_RGBD_CX=msg->K[2];
    //F_CAMERA_RGBD_CY=msg->K[5];
    F_CAMERA_RGBD_P_FX=msg->P[0];
    F_CAMERA_RGBD_P_FY=msg->P[5];
    F_CAMERA_RGBD_P_CX=msg->P[2];
    F_CAMERA_RGBD_P_CY=msg->P[6];
    msg->K[0]=msg->K[0]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[4]=msg->K[4]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[2]=msg->K[2]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->K[5]=msg->K[5]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[0]=msg->P[0]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[5]=msg->P[5]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[2]=msg->P[2]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->P[6]=msg->P[6]*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg->header.frame_id="camera_rgbd_out"+m_s_audience_id+"_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    m_pub_cai_img_camera_audit_rgbd_depth.publish(msg);
}



///////////////////////////////////////////////////
/// Callback for receiving RGB camera images
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    if(F_LISTENER_PRESENCE_VISUAL!=1.0f) return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    m_mat_img_camera_rgbd_rgb=cv_ptr->image;
    if(F_CAMERA_RGBD_RESOLUTION_SCALE!=1.0)
    {
        cv::resize(cv_ptr->image,m_mat_img_camera_rgbd_rgb,cv::Size(),F_CAMERA_RGBD_RESOLUTION_SCALE,F_CAMERA_RGBD_RESOLUTION_SCALE);
    }
    m_msg_img_camera_rgbd_rgb=cv_bridge::CvImage(std_msgs::Header(),"bgr8",m_mat_img_camera_rgbd_rgb).toImageMsg();
    m_msg_img_camera_rgbd_rgb->header.stamp=ros::Time::now();
    m_msg_img_camera_rgbd_rgb->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    m_msg_img_camera_rgbd_rgb->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    m_msg_img_camera_rgbd_rgb->step=m_msg_img_camera_rgbd_rgb->width*3;
    m_msg_img_camera_rgbd_rgb->header.frame_id="camera_rgbd_out"+m_s_audience_id+"_rgb_optical_frame";

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    m_pub_img_camera_audit_rgbd_rgb.publish(m_msg_img_camera_rgbd_rgb);
}

///////////////////////////////////////////////////
/// Callback for receiving DEPTH camera images
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr& msg)
{
    m_tim_event_received_img_camera_3d=ros::Time::now();
    if(F_LISTENER_PRESENCE_VISUAL!=1.0f) return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.setTo(0, cv_ptr->image != cv_ptr->image);
        cv_ptr->image.setTo(0, cv_ptr->image > F_CAMERA_RGBD_RANGE_MAX+0.5);
        cv_ptr->image.setTo(0, cv_ptr->image < F_CAMERA_RGBD_RANGE_MIN);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    m_mat_img_camera_rgbd_depth_32fc1=cv_ptr->image;
    if(F_CAMERA_RGBD_RESOLUTION_SCALE!=1.0)
    {
        cv::resize(cv_ptr->image,m_mat_img_camera_rgbd_depth_32fc1,cv::Size(),F_CAMERA_RGBD_RESOLUTION_SCALE,F_CAMERA_RGBD_RESOLUTION_SCALE,cv::INTER_NEAREST);
    }
    m_msg_img_camera_rgbd_depth=cv_bridge::CvImage(std_msgs::Header(),"32FC1",m_mat_img_camera_rgbd_depth_32fc1).toImageMsg();
    m_msg_img_camera_rgbd_depth->header.stamp=ros::Time::now();
    m_msg_img_camera_rgbd_depth->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    m_msg_img_camera_rgbd_depth->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    m_msg_img_camera_rgbd_depth->step=m_msg_img_camera_rgbd_depth->width*4;
    m_msg_img_camera_rgbd_depth->header.frame_id="camera_rgbd_out"+m_s_audience_id+"_rgb_optical_frame"; // Still RGB frame, althouhg for depth images!

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    m_pub_img_camera_audit_rgbd_depth.publish(m_msg_img_camera_rgbd_depth);
}



///////////////////////////////////////////////////
/// Callback for receiving camera_2d
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_2d(const sensor_msgs::CompressedImageConstPtr& msg)
{
    m_tim_event_received_img_camera_2d=ros::Time::now();
    if(F_LISTENER_PRESENCE_VISUAL!=0.5f) return;

    m_pub_img_camera_2d.publish(msg);
}



///////////////////////////////////////////////////
/// Callback for receiving livecam
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_livecam(const sensor_msgs::CompressedImageConstPtr& msg)
{
    /*
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    // Convert ROS Image to OpenCV Image
    //cv::resize(m_mat_img_camera_robot_rgb,m_mat_img_camera_robot_rgb,cv::Size(),1.5,1.5); // resize to better fit projection wall
    m_mat_img_camera_livecam=cv_ptr->image;
    */
    m_mat_img_camera_livecam=cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
}



/////////////////////////////////////////////////
/// Helper methods
/////////////////////////////////////////////////
int WAIOAAudienceListener::GetSceneCountCurrent()
{
    return m_i_scene_count_current;
}

void WAIOAAudienceListener::SetOAStatusLabel(std::string s_oa_status)
{
    std_msgs::String msg_s_oa_status;
    msg_s_oa_status.data="ID"+m_s_audience_id+"-"+s_oa_status;
    m_pub_s_oa_status.publish(msg_s_oa_status);
    ros::spinOnce();
}

void WAIOAAudienceListener::SetActionTriggerLabel(std::string s_label)
{
    ((WAIRepTrigger*)m_wai_oaa_trg_action)->UpdateModel(
                tf::Vector3(0.0,0.5,0.85),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "ID"+m_s_audience_id+"-"+s_label,m_col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    m_wai_oaa_trg_action->UpdateView();
}
void WAIOAAudienceListener::InteractionVoiceFromFile(std::string s_filename,bool b_blocking)
{   
    std::string s_command="";
    int i_retval=0;
    if(b_blocking==true) s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/audience/"+s_filename+".wav";
    else s_command="canberra-gtk-play -V 0.0 -f "+ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/audience/"+s_filename+".wav &";
    i_retval=system(s_command.c_str());
}

bool WAIOAAudienceListener::KeyIsPressed(KeySym ks)
{
    char keys_return[32];
    XQueryKeymap(m_dsp_x11_display, keys_return);
    KeyCode kc2=XKeysymToKeycode(m_dsp_x11_display, ks);
    bool isPressed= !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
    return isPressed;
}
void WAIOAAudienceListener::CheckTriggersFromKeyboard()
{
    // Reset all triggers
    B_EVENT_INPDEV_SESSION_LOAD=false;
    B_EVENT_INPDEV_SCENE_PREV=false;
    B_EVENT_INPDEV_SCENE_NEXT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    I_EVENT_INPDEV_REQUEST=0;

    // MODIFIER
    bool b_keyboard_modifier=KeyIsPressed(XK_Tab);

    // BASIC Interactions
    if(b_keyboard_modifier && KeyIsPressed(XK_L))
    {
        B_EVENT_INPDEV_SESSION_LOAD=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Left))
    {
        B_EVENT_INPDEV_SCENE_PREV=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Right))
    {
        B_EVENT_INPDEV_SCENE_NEXT=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Down))
    {
        B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Up))
    {
        B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_f))
    {
        B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_o))
    {
        B_EVENT_INPDEV_MODE_PRESENCE_VISUAL=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_b))
    {
        B_EVENT_INPDEV_BODY_INTERACTION=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_v))
    {
        B_EVENT_INPDEV_AVATAR=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_p))
    {
        B_EVENT_INPDEV_SKETCH=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_s))
    {
        B_EVENT_INPDEV_SHARE=true;
    }
    else if(b_keyboard_modifier && (KeyIsPressed(XK_Page_Up)||KeyIsPressed(XK_KP_Page_Up)) )
    {
        B_EVENT_INPDEV_MOOD=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_e))
    {
        B_EVENT_INPDEV_SELF_EVALUATE=true;
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_q))
    {
        I_EVENT_INPDEV_REQUEST=1; // Send request for QUESTION
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_r))
    {
        I_EVENT_INPDEV_REQUEST=2; // Send request for BREAK
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_m))
    {
        I_EVENT_INPDEV_REQUEST=3; // Send request for MESSAGE
    }
    else
    {
        // Do nothing...
    }
}

bool WAIOAAudienceListener::CheckTriggersFromCollission(tf::Vector3 vc3_current,tf::Vector3 vc3_reference,float f_threshold)
{
    if((vc3_reference-vc3_current).length()<f_threshold) return true;
    else return false;
}

void WAIOAAudienceListener::SetupCameraRviz(int i_scene)
{
    std::vector<double> vec_camera_rviz_temp;
    geometry_msgs::Point pnt_eye, pnt_focus;

    std::stringstream sst_setup_camera_rviz_defaults;
    sst_setup_camera_rviz_defaults << m_s_path_nodename << "setup_scene_defaults/camera/default";

    m_hdl_node.getParam(sst_setup_camera_rviz_defaults.str(), vec_camera_rviz_temp);
    pnt_eye.x=vec_camera_rviz_temp[0];
    pnt_eye.y=vec_camera_rviz_temp[1];
    pnt_eye.z=vec_camera_rviz_temp[2];
    pnt_focus.x=vec_camera_rviz_temp[3];
    pnt_focus.y=vec_camera_rviz_temp[4];
    pnt_focus.z=vec_camera_rviz_temp[5];

    TransitionCameraRviz(pnt_eye,pnt_focus,3);
}

void WAIOAAudienceListener::TransitionCameraRviz(geometry_msgs::Point pnt_eye,geometry_msgs::Point pnt_focus,int i_duration,std::string s_frame,int i_mode)
{
    m_msg_cpl_rviz_camera.interpolation_mode=i_mode;
    m_msg_cpl_rviz_camera.time_from_start.sec=i_duration;
    m_msg_cpl_rviz_camera.time_from_start.nsec=0;
    m_msg_cpl_rviz_camera.target_frame=s_frame;
    m_msg_cpl_rviz_camera.up.header.stamp=ros::Time::now();
    m_msg_cpl_rviz_camera.up.header.frame_id=s_frame;
    m_msg_cpl_rviz_camera.up.vector.x=0.0;
    m_msg_cpl_rviz_camera.up.vector.y=0.0;
    m_msg_cpl_rviz_camera.up.vector.z=1.0;
    m_msg_cpl_rviz_camera.eye.header.stamp=ros::Time::now();
    m_msg_cpl_rviz_camera.eye.header.frame_id=s_frame;
    m_msg_cpl_rviz_camera.eye.point=pnt_eye;
    m_msg_cpl_rviz_camera.focus.header.stamp=ros::Time::now();
    m_msg_cpl_rviz_camera.focus.header.frame_id=s_frame;
    m_msg_cpl_rviz_camera.focus.point=pnt_focus;
    m_pub_cpl_rviz_camera.publish(m_msg_cpl_rviz_camera);
    ros::spinOnce();
}

void WAIOAAudienceListener::SetupVirtualPresenter(int i_scene)
{
    system("rosbag play ~/wai_audit_presenter_virtual.bag --duration=30 &");
}
void WAIOAAudienceListener::SetupRobot(int i_scene)
{
    // Do nothing...
}




/////////////////////////////////////////////////
/// Run method with ROS spin
/////////////////////////////////////////////////
void WAIOAAudienceListener::run()
{
    // Init objects
    ros::Rate ros_rate(F_NODE_SAMPLE_FREQUENCY);

    // State machine transition to STARTUP state
    this->StateTransitionTo(new StateListening);
    this->RequestSetupModel();
    this->RequestSetupView();

    // ROS Loop
    while(ros::ok())
    {
        CheckTriggersFromKeyboard();

        // Trigger of actions
        if((ros::Time::now()-m_tim_trigger_action).toSec()>F_SCENE_TRIGGER_TIMEOUT)
        {
            if(B_EVENT_INPDEV_SESSION_LOAD)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SESSION_LOAD=false;

                // Get path to sessions_listener subfolder in resources,
                // HTML sessions are stored separately in each folder as files (0-n).html!
                QFileDialog::Options m_qst_path_session_options;
                m_qst_path_session_options |= QFileDialog::DontUseNativeDialog;
                m_qst_path_session=QFileDialog::getExistingDirectory(
                                                        NULL,
                                                        "LOAD Session",
                                                        QString::fromStdString(m_s_path_resources_sessions_listener),
                                                        m_qst_path_session_options);
                // Reset scene counter
                m_i_scene_count_current=0;

                SetOAStatusLabel("LST TRG: [KEY] LOAD SESSION");
                SetActionTriggerLabel("LOAD SESSION");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_SCENE_PREV)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SCENE_PREV=false;

                // Get scene count max from number of HTML files and update scene count current
                QDir qdi_scenes(m_qst_path_session);
                if(qdi_scenes.exists())
                {
                    QStringList qst_list=qdi_scenes.entryList(QStringList() << "*.html",QDir::Files);
                    m_i_scene_count_max=qst_list.length()-1;
                }
                else
                {
                    m_i_scene_count_max=1;
                    break;
                }

                if(m_i_scene_count_current>1) m_i_scene_count_current--;

                // Bash example: cutycapt --url=file:///home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/sessions/template/layout_one_column.html --out=/home/ias/test.png
                std::string s_sys_cmd_cfg="cutycapt --url=file://"+m_qst_path_session.toStdString()+"/"+std::to_string(m_i_scene_count_current)+".html --out="+m_s_path_resources_sessions_listener+"scene.png";
                int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

                // Bash example: xmlstarlet sel -t -v "//notes" 1.html
                //s_sys_cmd_cfg="xmlstarlet sel -t -v \"//notes\" "+m_s_session_filename+"/"+std::to_string(m_i_session_scene_count_current)+".html > "+m_m_s_path_resources_sessions+"session.yaml";
                //i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
                m_mat_img_projector=cv::imread(m_s_path_resources_sessions_listener+"scene.png");
                m_pub_img_projector.publish(EncodeImage(m_mat_img_projector));

                SetOAStatusLabel("LST TRG: [KEY] SCENE PREV ("+std::to_string(m_i_scene_count_current)+"/"+std::to_string(m_i_scene_count_max)+")");
                SetActionTriggerLabel("PREV ("+std::to_string(m_i_scene_count_current)+"/"+std::to_string(m_i_scene_count_max)+")");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_SCENE_NEXT)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SCENE_NEXT=false;

                // Get scene count max from number of HTML files and update scene count current
                QDir qdi_scenes(m_qst_path_session);
                if(qdi_scenes.exists())
                {
                    QStringList qst_list=qdi_scenes.entryList(QStringList() << "*.html",QDir::Files);
                    m_i_scene_count_max=qst_list.length()-1;
                }
                else
                {
                    m_i_scene_count_max=1;
                    break;
                }

                // Update current session scene count
                if(m_i_scene_count_current<m_i_scene_count_max) m_i_scene_count_current++;

                // Bash example: cutycapt --url=file:///home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/sessions/template/layout_one_column.html --out=/home/ias/test.png
                std::string s_sys_cmd_cfg="cutycapt --url=file://"+m_qst_path_session.toStdString()+"/"+std::to_string(m_i_scene_count_current)+".html --out="+m_s_path_resources_sessions_listener+"scene.png";
                int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

                // Bash example: xmlstarlet sel -t -v "//notes" 1.html
                //s_sys_cmd_cfg="xmlstarlet sel -t -v \"//notes\" "+m_s_session_filename+"/"+std::to_string(m_i_session_scene_count_current)+".html > "+m_m_s_path_resources_sessions+"session.yaml";
                //i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
                m_mat_img_projector=cv::imread(m_s_path_resources_sessions_listener+"scene.png");
                m_pub_img_projector.publish(EncodeImage(m_mat_img_projector));

                SetOAStatusLabel("LST TRG: [KEY] SCENE NEXT ("+std::to_string(m_i_scene_count_current)+"/"+std::to_string(m_i_scene_count_max)+")");
                SetActionTriggerLabel("NEXT ("+std::to_string(m_i_scene_count_current)+"/"+std::to_string(m_i_scene_count_max)+")");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_MODE_PRESENCE_VISUAL)
            {
                m_tim_trigger_action=ros::Time::now();

                /*
                 * PRESENCE VISUAL (normed values):
                 * 0.00 ... No presence
                 * 0.25 ... 2D-Avatar
                 * 0.50 ... 2D-Camera
                 * 0.75 ... 3D-Avatar (e.g., with a "virtualized head")
                 * 1.00 ... 3D-Pointcloud/Mesh
                 */
                F_LISTENER_PRESENCE_VISUAL+=0.25f;
                if(F_LISTENER_PRESENCE_VISUAL==0.75f) F_LISTENER_PRESENCE_VISUAL=1.0f; // Skip 3D-Avatar for now (OpenNI Tracker required!)
                if(F_LISTENER_PRESENCE_VISUAL>1.0f)F_LISTENER_PRESENCE_VISUAL=0.0f;

                if(F_LISTENER_PRESENCE_VISUAL==0.0f)
                {
                    // Stop 2D-Avatar timer
                    m_tmr_avatar_2d.stop();

                    // Publish empty 2D webcam image to disable
                    m_mat_img_camera_2d.setTo(0);
                    m_pub_img_camera_2d.publish(EncodeImage(m_mat_img_camera_2d));
                }
                else if(F_LISTENER_PRESENCE_VISUAL==0.25f)
                {
                    m_tmr_avatar_2d.start();
                }
                else if(F_LISTENER_PRESENCE_VISUAL==0.5f)
                {
                    // Stop 2D-Avatar timer
                    m_tmr_avatar_2d.stop();
                }
                else if(F_LISTENER_PRESENCE_VISUAL==0.75f)
                {
                    m_tmr_avatar_2d.stop();
                }
                else if(F_LISTENER_PRESENCE_VISUAL==1.0f)
                {
                    // Stop 2D-Avatar timer
                    m_tmr_avatar_2d.stop();

                    // Publish empty 2D webcam image to disable
                    m_mat_img_camera_2d.setTo(0);
                    m_pub_img_camera_2d.publish(EncodeImage(m_mat_img_camera_2d));
                }
                else
                {
                    // Do nothing...
                }

                SetOAStatusLabel("LST TRG: [KEY] PRESENCE");
                std::stringstream sst_presence;
                sst_presence.precision(2);
                sst_presence << "PRESENCE(" << F_LISTENER_PRESENCE_VISUAL << ")";
                //SetActionTriggerLabel("PRESENCE("+std::to_string(F_LISTENER_PRESENCE_VISUAL)+")");
                SetActionTriggerLabel(sst_presence.str());
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_AVATAR)
            {
                m_tim_trigger_action=ros::Time::now();
                B_ENABLE_AVATAR=!B_ENABLE_AVATAR;
                if(B_ENABLE_AVATAR)
                {
                    SetOAStatusLabel("LST TRG: [KEY] AVATAR (ON)");
                    SetActionTriggerLabel("AVATAR (ON)");
                }
                else
                {
                    SetOAStatusLabel("LST TRG: [KEY] AVATAR (OFF)");
                    SetActionTriggerLabel("AVATAR (OFF)");
                }
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_BODY_INTERACTION)
            {
                m_tim_trigger_action=ros::Time::now();
                B_ENABLE_BODY_INTERACTION=!B_ENABLE_BODY_INTERACTION;
                if(B_ENABLE_BODY_INTERACTION)
                {
                    SetOAStatusLabel("LST TRG: [KEY] BODY INT (ON)");
                    SetActionTriggerLabel("BODY INT (ON)");
                }
                else
                {
                    SetOAStatusLabel("LST TRG: [KEY] BODY INT (OFF)");
                    SetActionTriggerLabel("BODY INT (OFF)");
                }
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_SKETCH)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SKETCH=false;

                std::string s_sys_cmd_cfg="roslaunch wai_sketch wai_sketch.launch &";
                int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

                SetOAStatusLabel("LST TRG: [KEY] SKETCH");
                SetActionTriggerLabel("SKETCH");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_SHARE)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SHARE=false;

                std::stringstream sst_audience_request;
                int ok_req_share=QMessageBox::No;
                sst_audience_request << "ID" << m_s_audience_id << "-SHARE";
                ok_req_share=QMessageBox::warning(
                            NULL,
                            "SHARE Teaching/Learning Material",
                            "Do your really want to SHARE the material\nfor the current session with the presenter?",
                            QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

                if(ok_req_share==QMessageBox::Yes)
                {
                    std::string s_syscmd="scp /home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/shared/* ias@10.0.0.101:/home/ias/catkin_ws/src/wai_world/wai_oa/wai_oa_gazebo/resources/shared";
                    int retval=system(s_syscmd.c_str());
                    QMessageBox::information(NULL,"SHARE","The contents were shared with the Presenter!");
                    SetActionTriggerLabel("SHARE");
                }
                else
                {
                    sst_audience_request << " (DIS)";
                    QMessageBox::information(NULL,"SHARE","The sharing will be dismissed!");
                    SetActionTriggerLabel("SHARE (DIS)");
                }

                SetOAStatusLabel("LST TRG: [KEY] SHARE");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_MOOD)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_MOOD=false;

                SetActionTriggerLabel("MOOD");

                QVBoxLayout* vbox=new QVBoxLayout();

                QDialog* d=new QDialog();
                d->setWindowTitle("MOOD Level");
                QLabel* lbl_mood=new QLabel();
                lbl_mood->setText("Select a mood Level (1-5):");

                QSpinBox* spb_mood=new QSpinBox();
                spb_mood->setMinimum(1);
                spb_mood->setMaximum(5);
                spb_mood->setSingleStep(1);
                spb_mood->setValue(3);

                QObject::connect(spb_mood,QOverload<int>::of(&QSpinBox::valueChanged),[=](int i_mood)
                {
                    F_LISTENER_MOOD=i_mood;
                });

                QDialogButtonBox* buttonBox=new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
                QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
                QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
                vbox->addWidget(lbl_mood);
                vbox->addWidget(spb_mood);
                vbox->addWidget(buttonBox);
                d->setLayout(vbox);
                int result=d->exec();
                F_LISTENER_MOOD=spb_mood->value();

                SetOAStatusLabel("LST TRG: [KEY] MOOD");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_SELF_EVALUATE)
            {
                m_tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SELF_EVALUATE=false;

                // Self evaluate learning progress of current activity
                QDialog* d=new QDialog();
                d->setWindowTitle("SELF Evaluate");
                QLabel* lbl_self_eval_rating=new QLabel();
                lbl_self_eval_rating->setText("Please rate your learning progress:");
                QVBoxLayout* vbox=new QVBoxLayout();
                QComboBox* cmb_self_eval_rating=new QComboBox();
                cmb_self_eval_rating->addItem("(1) Very Good");
                cmb_self_eval_rating->addItem("(2) Good");
                cmb_self_eval_rating->addItem("(3) Satisfactory");
                cmb_self_eval_rating->addItem("(4) Suffiencent");
                cmb_self_eval_rating->addItem("(5) Insufficient");
                QDialogButtonBox* buttonBox=new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
                QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
                QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
                vbox->addWidget(lbl_self_eval_rating);
                vbox->addWidget(cmb_self_eval_rating);
                vbox->addWidget(buttonBox);
                d->setLayout(vbox);

                int result=d->exec();
                if(result == QDialog::Accepted)
                {
                    SetActionTriggerLabel("SEVAL-"+cmb_self_eval_rating->currentText().toStdString());
                }
                else
                {
                    SetActionTriggerLabel("SEVAL (DIS)");
                }

                SetOAStatusLabel("LST TRG: [KEY] SEVAL");
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                m_tim_trigger_action=ros::Time::now();
                B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW=!B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW;
                if(B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW)
                {
                    SetOAStatusLabel("LST TRG: [KEY] CAMERA Follow (ON)");
                    SetActionTriggerLabel("CAMERA Follow (ON)");
                }
                else
                {
                    SetOAStatusLabel("LST TRG: [KEY] CAMERA Follow (OFF)");
                    SetActionTriggerLabel("CAMERA Follow (OFF)");
                }
                InteractionVoiceFromFile("bleep");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT
                    && !B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                m_tim_trigger_action=ros::Time::now();
                I_CAMERA_RVIZ_CYCLE_VIEW=0;

                geometry_msgs::Point pnt_cycle_eye;
                geometry_msgs::Point pnt_cycle_focus;
                pnt_cycle_eye.x=0.88793; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z= 1.1733;
                pnt_cycle_focus.x=0.0; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=1.0236;
                TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+m_s_audience_id+"/link_base");

                SetOAStatusLabel("LST TRG: [KEY] CAMERA Default");
                SetActionTriggerLabel("CAMERA Default");
                InteractionVoiceFromFile("tick");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE
                    && !B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                m_tim_trigger_action=ros::Time::now();
                I_CAMERA_RVIZ_CYCLE_VIEW++;
                if(I_CAMERA_RVIZ_CYCLE_VIEW>3)I_CAMERA_RVIZ_CYCLE_VIEW=0;

                geometry_msgs::Point pnt_cycle_eye;
                geometry_msgs::Point pnt_cycle_focus;
                switch(I_CAMERA_RVIZ_CYCLE_VIEW)
                {
                    case 0: // First person view (default)
                        pnt_cycle_eye.x=0.88793; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z= 1.1733;
                        pnt_cycle_focus.x=0.0; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=1.0236;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+m_s_audience_id+"/link_base");
                    break;
                    case 1: // Sketch view
                        pnt_cycle_eye.x=0.51397; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z=1.4861;
                        pnt_cycle_focus.x=0.41221; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=0.56779;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+m_s_audience_id+"/link_base");
                    break;
                    case 2: // Third person view
                        pnt_cycle_eye.x=1.4279; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z=1.3769;
                        pnt_cycle_focus.x=0.16624; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=0.61416;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+m_s_audience_id+"/link_base");
                    break;
                    case 3: // Audience overview
                        pnt_cycle_eye.x=4.0; pnt_cycle_eye.y=2.0; pnt_cycle_eye.z=4.5;
                        pnt_cycle_focus.x=-3.0; pnt_cycle_focus.y=2.0; pnt_cycle_focus.z=0.0;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"world");
                    break;

                    default:
                    break;
                }

                SetOAStatusLabel("LST TRG: [KEY] CAMERA Cycle");
                SetActionTriggerLabel("CAMERA Cycle");
                InteractionVoiceFromFile("tick");
            }
            else if(I_EVENT_INPDEV_REQUEST!=0)
            {
                m_tim_trigger_action=ros::Time::now();

                std::stringstream sst_audience_request;
                QString hea_req_aud_text;
                int ok_req_aud_que=QMessageBox::No;
                int ok_req_aud_brk=QMessageBox::No;
                bool ok_req_aud_msg=false;
                switch(I_EVENT_INPDEV_REQUEST)
                {
                    case 1:
                        sst_audience_request << "ID" << m_s_audience_id << "-QUESTION";
                        ok_req_aud_que=QMessageBox::warning(
                                    NULL,
                                    "QUESTION",
                                    "Do you really want to send the\nrequest to the presenter?",
                                    QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

                        if(ok_req_aud_que==QMessageBox::Yes)
                        {
                            QMessageBox::information(NULL,"QUESTION","The request will be sent to the Presenter!");
                        }
                        else
                        {
                            sst_audience_request << " (DIS)";
                            QMessageBox::information(NULL,"QUESTION","The request will be dismissed!");
                        }
                    break;

                    case 2:
                        sst_audience_request << "ID" << m_s_audience_id << "-BREAK";
                        ok_req_aud_brk=QMessageBox::warning(
                                    NULL,
                                    "BREAK",
                                    "Do you reall want to send the\nrequest to the presenter?",
                                    QMessageBox::Yes|QMessageBox::No, QMessageBox::No);

                        if(ok_req_aud_brk==QMessageBox::Yes)
                        {
                            QMessageBox::information(NULL,"BREAK","The request will be sent to the Presenter!");
                        }
                        else
                        {
                            sst_audience_request << " (DIS)";
                            QMessageBox::information(NULL,"BREAK","The request will be dismissed!");
                        }
                    break;

                    case 3:
                        sst_audience_request << "ID" << m_s_audience_id << "-MESSAGE";

                        // Limit message length
                        do
                        {
                            hea_req_aud_text=QInputDialog::getText(
                                NULL,
                                "MESSAGE",
                                "Input a short message\n(Up to 100 characters!):",
                                QLineEdit::Normal,
                                "Type in your message here...",
                                &ok_req_aud_msg,
                                Qt::WindowFlags());
                        }while(hea_req_aud_text.length()>100);

                        if(ok_req_aud_msg)
                        {
                            sst_audience_request << ":" << hea_req_aud_text.toStdString();
                            QMessageBox::information(NULL,"MESSAGE","The request will be sent to the Presenter!");
                        }
                        else
                        {
                            sst_audience_request << " (DIS)";
                            QMessageBox::information(NULL,"MESSAGE","The request will be dismissed!");
                        }
                    break;

                    default:
                        sst_audience_request << "ID" << m_s_audience_id << "N/A";
                    break;
                }

                if(ok_req_aud_brk==QMessageBox::Yes
                    || ok_req_aud_que==QMessageBox::Yes
                    || ok_req_aud_msg==true)
                {
                    m_msg_hea_audience_request.stamp=ros::Time::now();
                    m_msg_hea_audience_request.frame_id=sst_audience_request.str();
                    m_pub_msg_hea_audience_request.publish(m_msg_hea_audience_request);

                    SetActionTriggerLabel("REQUEST");
                    InteractionVoiceFromFile("ping");
                }
                SetOAStatusLabel("LST TRG: [REQ] "+sst_audience_request.str());

                I_EVENT_INPDEV_REQUEST=0;
            }
            else if(I_EVENT_INFO_FROM_PRESENTER!=0)
            {
                m_tim_trigger_action=ros::Time::now();
                std::stringstream sst_info_from_presenter;
                switch(I_EVENT_INFO_FROM_PRESENTER)
                {
                    case 1:
                        sst_info_from_presenter << m_msg_hea_info_from_presenter.frame_id;
                    break;

                    case 2:
                        B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW=!B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW;
                        sst_info_from_presenter << m_msg_hea_info_from_presenter.frame_id;
                    break;

                    default:
                    break;
                }
                I_EVENT_INFO_FROM_PRESENTER=0;

                SetOAStatusLabel("LST TRG: [INF] "+sst_info_from_presenter.str());
                //SetActionTriggerLabel(sst_info_from_presenter.str());
                SetActionTriggerLabel("PRE-INFO");
                InteractionVoiceFromFile("ping");
            }
            else
            {
                // Do nothing...
            }
        }
        else
        {
            // Do nothing...
        }


        if(F_LISTENER_PRESENCE_VISUAL==1.0f
           && (ros::Time::now()-m_tim_event_received_img_camera_3d).toSec()<1.0
           && !m_mat_img_camera_rgbd_depth_32fc1.empty())
        {
            // Process image and detect hand pose
            m_mat_img_camera_rgbd_depth_32fc1.copyTo(m_mat_img_camera_rgbd_depth_32fc1_processed);
            m_wai_oa_image_processor.SetProcessedImage(m_mat_img_camera_rgbd_depth_32fc1_processed);
            tf::Matrix3x3 m33_vc3_hands_and_head_buf=m_wai_oa_image_processor.Process();
            if(!std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getX())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getY())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getZ()))
            {
                m_vc3_camera_rgbd_wrt_head=m33_vc3_hands_and_head_buf.getRow(2);
            }

            /*
            if(!B_ENABLE_BODY_INTERACTION)
            {
                tf::Vector3 vc3_hand_left_origin(0.0,0.2,0.05); // origin is different with audience/listener workspace!
                tf::Vector3 vc3_hand_left(0,0,0);
                if(m_msg_joy_input_dev.axes.size()!=0)
                {
                    vc3_hand_left.setX(-3.0*m_msg_joy_input_dev.axes[3]);
                    vc3_hand_left.setY(-1.0*m_msg_joy_input_dev.axes[1]);
                    vc3_hand_left.setZ(+3.0*m_msg_joy_input_dev.axes[4]);
                }
                vc3_camera_rgbd_wrt_hand_left=vc3_hand_left_origin+vc3_hand_left;
            }
            */

            if(!B_ENABLE_AVATAR)
            {
                m_vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);
            }
        }
        else
        {
            m_vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);
        }
        ((WAIRepOOI*)m_wai_oaa_ooi_head)->UpdateModel(
                    m_vc3_camera_rgbd_wrt_head,
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.025,0.075),"Head",m_col_oa_shiny,"head",m_s_path_resources_reps,true,1.1);
        m_wai_oaa_ooi_head->UpdateView();
        m_wai_oaa_trg_action->UpdateView();


        // ROS Spin
        ros::spinOnce();
        ros_rate.sleep();
    }
}

sensor_msgs::CompressedImage WAIOAAudienceListener::EncodeImage(cv::Mat mat_input)
{
    cv::imencode(".jpg",mat_input,m_uch_buf_compression,m_par_img_compression);
    sensor_msgs::CompressedImage msg_img_compressed;
    msg_img_compressed.header.stamp=ros::Time::now();
    msg_img_compressed.format="jpeg";
    msg_img_compressed.data.assign(m_uch_buf_compression.begin(),m_uch_buf_compression.end());
    return msg_img_compressed;
}
