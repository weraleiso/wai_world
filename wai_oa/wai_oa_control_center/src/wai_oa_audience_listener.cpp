#include<wai_oa_audience_listener.h>



/////////////////////////////////////////////////
/// Implementation of state machine
/////////////////////////////////////////////////
void WAIOAAudienceListener::StateTransitionTo(WAIAuditState *state)
{
    ROS_WARN("State machine: Changing state to %s.",typeid(*state).name());
    if(this->wai_audit_state != NULL)
    {
        delete this->wai_audit_state;
    }
    this->wai_audit_state = state;
    this->wai_audit_state->SetContext(this);
}

// Common requests of context amongst all(!) states
void WAIOAAudienceListener::RequestSetupModel()
{
    this->wai_audit_state->HandleSetupModel();
}
void WAIOAAudienceListener::RequestSetupView()
{
    this->wai_audit_state->HandleSetupView();
}
void WAIOAAudienceListener::RequestSetupLeaveState()
{
    this->wai_audit_state->HandleSetupLeaveState();
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
WAIOAAudienceListener* WAIOAAudienceListener::instance = 0;
WAIOAAudienceListener* WAIOAAudienceListener::getInstance()
{
    if (instance == 0)
    {
        instance = new WAIOAAudienceListener();
    }
    return instance;
}



//////////////////////////////////////////////////
/// Construct object including all initialization
//////////////////////////////////////////////////
WAIOAAudienceListener::WAIOAAudienceListener():
    it(nh),
    ImageProcessor(new HandsAndHeadDetector(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
{
    // Resolve parameters from launchfile
    s_path_nodename=ros::this_node::getName()+"/";
    nh.getParam(s_path_nodename+"F_NODE_SAMPLE_FREQUENCY", F_NODE_SAMPLE_FREQUENCY);
    nh.getParam(s_path_nodename+"F_SCENE_TRANSITION_TIMEOUT", F_SCENE_TRANSITION_TIMEOUT);
    nh.getParam(s_path_nodename+"F_SCENE_TRIGGER_TIMEOUT", F_SCENE_TRIGGER_TIMEOUT);
    nh.getParam(s_path_nodename+"F_SCENE_TRIGGER_COLL_THRES", F_SCENE_TRIGGER_COLL_THRES);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_X", F_CAMERA_RGBD_RESOLUTION_X);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_Y", F_CAMERA_RGBD_RESOLUTION_Y);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_RESOLUTION_SCALE", F_CAMERA_RGBD_RESOLUTION_SCALE);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_FX", F_CAMERA_RGBD_FX);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_FY", F_CAMERA_RGBD_FY);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_CX", F_CAMERA_RGBD_CX);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_CY", F_CAMERA_RGBD_CY);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_RANGE_MIN", F_CAMERA_RGBD_RANGE_MIN);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_RANGE_MAX", F_CAMERA_RGBD_RANGE_MAX);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_THRESHOLD_DIST", F_CAMERA_RGBD_THRESHOLD_DIST);
    nh.getParam(s_path_nodename+"F_CAMERA_RGBD_THRESHOLD_BOUNDS", F_CAMERA_RGBD_THRESHOLD_BOUNDS);
    nh.getParam(s_path_nodename+"B_ENABLE_KALMAN", B_ENABLE_KALMAN);
    nh.getParam(s_path_nodename+"B_ENABLE_CAMERA_RVIZ_FLY_IN", B_ENABLE_CAMERA_RVIZ_FLY_IN);
    nh.getParam(s_path_nodename+"B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW", B_ENABLE_CAMERA_RVIZ_PRESENTER_FOLLOW);
    nh.getParam(s_path_nodename+"B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW", B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW);
    nh.getParam(s_path_nodename+"B_ENABLE_3D_MODE", B_ENABLE_3D_MODE);
    nh.getParam(s_path_nodename+"B_ENABLE_BODY_INTERACTION", B_ENABLE_BODY_INTERACTION);
    nh.getParam(s_path_nodename+"B_ENABLE_AVATAR", B_ENABLE_AVATAR);

    // Resolve resource paths and configs for this session...
    s_path_resources=ros::package::getPath("wai_oa_gazebo")+"/resources/";
    s_path_resources_representatives=s_path_resources+"reps/";
    s_path_resources_logo=s_path_resources+"icons/open_auditorium_logo.png";

    if(ros::get_environment_variable(s_audience_id,"WAI_OA_AUDIENCE_ID"))
    {
        ROS_WARN_STREAM("OA Audience - Found AUDIENCE ID as \""+s_audience_id+"\"");
    }
    else
    {
        ROS_WARN_STREAM("OA Audience - No AUDIENCE ID found! Exiting...");
        exit(1);
    }

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

    // Init ROS Publishers
    pub_img_camera_2d=it.advertise("camera_rgb_out/usb_cam/image_raw",1);
    pub_img_camera_audit_rgbd_rgb=it.advertise("camera_rgbd_out"+s_audience_id+"/rgb/image_raw",1);
    pub_img_camera_audit_rgbd_depth=it.advertise("camera_rgbd_out"+s_audience_id+"/depth/image_raw",1);
    pub_cai_img_camera_audit_rgbd_rgb=nh.advertise<sensor_msgs::CameraInfo>("camera_rgbd_out"+s_audience_id+"/rgb/camera_info",1);
    pub_cai_img_camera_audit_rgbd_depth=nh.advertise<sensor_msgs::CameraInfo>("camera_rgbd_out"+s_audience_id+"/depth/camera_info",1);
    pub_cpl_rviz_camera=nh.advertise<view_controller_msgs::CameraPlacement>("camera_rviz/placement",1,true);
    pub_msg_hea_audience_request=nh.advertise<std_msgs::Header>("/wai_world/oa/audience_request",1,true); // Modified: 10
    pub_hea_ping_to_presenter=nh.advertise<std_msgs::Header>("/wai_world/oa/ping_from_audience_"+s_audience_id,1);
    pub_s_oa_status=nh.advertise<std_msgs::String>("oa_status",1);

    // Welcome audience
    SetOAStatusLabel("WELCOME TO W.A.I. WORLD! Intel is busy with preparing your workspace...");

    // Init ROS Subscribers
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    sub_img_camera_rgbd_rgb = it.subscribe("camera_rgbd_in"+s_audience_id+"/rgb/image_raw", 1, &WAIOAAudienceListener::cb_sub_img_camera_rgbd_rgb, this);
    sub_img_camera_rgbd_depth = it.subscribe("camera_rgbd_in"+s_audience_id+"/depth/image", 1, &WAIOAAudienceListener::cb_sub_img_camera_rgbd_depth, this);
    sub_cai_img_camera_rgbd_rgb=nh.subscribe("camera_rgbd_in"+s_audience_id+"/rgb/camera_info", 1, &WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_rgb, this);
    sub_cai_img_camera_rgbd_depth=nh.subscribe("camera_rgbd_in"+s_audience_id+"/depth/camera_info", 1, &WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_depth, this);
    sub_img_camera_livecam = it.subscribe("livecam/usb_cam/image_raw", 1, &WAIOAAudienceListener::cb_sub_img_camera_livecam, this,hints);
    sub_img_camera_2d = it.subscribe("camera_rgb_in/usb_cam/image_raw", 1, &WAIOAAudienceListener::cb_sub_img_camera_2d, this,hints);
    sub_cpl_rviz_camera = nh.subscribe("/wai_world/oa/camera_rviz/placement", 1, &WAIOAAudienceListener::cb_sub_cpl_rviz_camera, this);
    sub_joy_controller = nh.subscribe("joy", 1, &WAIOAAudienceListener::cb_sub_joy_controller, this);
    sub_bol_mob_camera_rviz_follow_presenter = nh.subscribe("mob_camera_rviz_follow_presenter", 1, &WAIOAAudienceListener::cb_sub_bol_mob_camera_rviz_follow_presenter, this);
    sub_bol_mob_3d_mode = nh.subscribe("mob_3d_mode", 1, &WAIOAAudienceListener::cb_sub_bol_mob_3d_mode, this);
    sub_bol_mob_body_interaction = nh.subscribe("mob_body_interaction", 1, &WAIOAAudienceListener::cb_sub_bol_mob_body_interaction, this);
    sub_bol_mob_avatar = nh.subscribe("mob_avatar", 1, &WAIOAAudienceListener::cb_sub_bol_mob_avatar, this);
    sub_bol_mob_question = nh.subscribe("mob_question", 1, &WAIOAAudienceListener::cb_sub_bol_mob_question, this);
    sub_bol_mob_break = nh.subscribe("mob_break", 1, &WAIOAAudienceListener::cb_sub_bol_mob_break, this);
    sub_bol_mob_message = nh.subscribe("mob_message", 1, &WAIOAAudienceListener::cb_sub_bol_mob_message, this);
    sub_hea_ping_from_presenter=nh.subscribe("ping_to_audience", 1, &WAIOAAudienceListener::cb_sub_hea_ping_from_presenter, this);
    sub_hea_info_from_presenter=nh.subscribe("info_to_audience",10, &WAIOAAudienceListener::cb_sub_hea_info_from_presenter, this);

    // Send confirmation for connection attempt to presenter
    std::string s_audience_request="ID"+s_audience_id+"-CONNECT";
    msg_hea_audience_request.stamp=ros::Time::now();
    msg_hea_audience_request.frame_id=s_audience_request;
    while(pub_msg_hea_audience_request.getNumSubscribers()==0)
    {
        ros::spinOnce();
        ros::Rate(1.0).sleep();
    }
    pub_msg_hea_audience_request.publish(msg_hea_audience_request);
    SetOAStatusLabel("Intel notified the presenter that you are there...");

    /* For debugging purposes only (waiting for presenter to acknowledge request):
    while(ros::ok())
    {
        if(msg_hea_info_from_presenter.frame_id.find("ACCEPTED")!=std::string::npos)
        {
            break;
        }
        else if(msg_hea_info_from_presenter.frame_id.find("REJECTED")!=std::string::npos)
        {
            exit(0);
        }
        else
        {
            // Do nothing...
        }
        ros::spinOnce();
        ros::Rate(2.0).sleep();
    }*/

    // Init X11 display for user inputs and screen properties
    dsp_x11_display=XOpenDisplay(NULL);

    // Init user input device events
    I_CAMERA_RVIZ_CYCLE_VIEW=0;
    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_PRESENCE_MODE=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    B_EVENT_INPDEV_SKETCH=false;
    I_EVENT_INPDEV_REQUEST=0;

    tim_trigger_action=ros::Time::now();
    i_scene_count=0; // Subscribe from presenter!

    // Init sketch
    wai_oa_sketch.Initialize(&nh,&it,&sc,F_NODE_SAMPLE_FREQUENCY,0.5);

    // Init transforms
    vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);

    // Init intrinsic parameters from CameraInfo topic (however, takes longer!)
    /*
    boost::shared_ptr<sensor_msgs::CameraInfo const> shared_cai_img_camera_rgbd;
    sensor_msgs::CameraInfo msg_cai_img_camera_rgbd;
    shared_cai_img_camera_rgbd = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera/depth/camera_info", nh);
    if(shared_cai_img_camera_rgbd != NULL){
        msg_cai_img_camera_rgbd = *shared_cai_img_camera_rgbd;
    }
    F_CAMERA_RGBD_FX=msg_cai_img_camera_rgbd.K[0];
    F_CAMERA_RGBD_FY=msg_cai_img_camera_rgbd.K[4];
    F_CAMERA_RGBD_CX=msg_cai_img_camera_rgbd.K[2];
    F_CAMERA_RGBD_CY=msg_cai_img_camera_rgbd.K[5];
    */
    s = new HandsAndHeadDetector(
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
    ImageProcessor.ChangeProcessingStrategy(s);


    // Init image for livecam in case of no stream available
    mat_img_camera_livecam=cv::imread(s_path_resources_logo);
    mat_img_camera_2d=cv::imread(s_path_resources_logo);


    wai_oaa_trg_action=WAIReps::create_representative("TRIGGER");
    wai_oaa_trg_action->Initialize(&nh,
                                   s_path_nodename,
                                   GET_OBJECT_NAME(wai_oaa_trg_action),
                                   "workspace_audience_"+s_audience_id+"/link_base");
    SetActionTriggerLabel("ACTION");

    wai_oaa_ooi_head=WAIReps::create_representative("OOI");
    wai_oaa_ooi_head->Initialize(&nh,s_path_nodename,"wai_oa_rep_ooi_head","camera_rgbd_out"+s_audience_id+"_depth_optical_frame");
    ((WAIRepOOI*)wai_oaa_ooi_head)->UpdateModel(
                vc3_camera_rgbd_wrt_head,
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.025,0.075),"Head",col_oa_shiny,"head",s_path_resources_representatives,true,1.1);

    // Init request message
    msg_hea_audience_request.stamp.sec=0;
    msg_hea_audience_request.stamp.nsec=0;
    msg_hea_audience_request.frame_id="Request!";
}



/////////////////////////////////////////////////
/// Delete instance
/////////////////////////////////////////////////
WAIOAAudienceListener::~WAIOAAudienceListener()
{
    delete wai_audit_state;
    delete instance;
}



/////////////////////////////////////////////////
/// Callback to receive inputs from joypad
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_joy_controller(const sensor_msgs::JoyPtr& msg)
{
    msg_joy_input_dev=*msg;

    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_PRESENCE_MODE=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    B_EVENT_INPDEV_SKETCH=false;
    I_EVENT_INPDEV_REQUEST=0;

    // SHARE + OPTIONS
    if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[9]==1)
    {
        // PRESENTER MODE
        B_EVENT_INPDEV_PRESENCE_MODE=true;
    }
    // JSLeftButton + JSRightButton
    else if(msg_joy_input_dev.buttons[11]==1 && msg_joy_input_dev.buttons[12]==1)
    {
        // BODY INTERACTION
        B_EVENT_INPDEV_BODY_INTERACTION=true;
    }

    // OPTIONS + CURSOR DOWN
    else if(msg_joy_input_dev.buttons[9]==1 && msg_joy_input_dev.buttons[14]==1)
    {
        // CAMERA RVIZ DEFAULT
        B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=true;
    }
    // OPTIONS + CURSOR UP
    else if(msg_joy_input_dev.buttons[9]==1 && msg_joy_input_dev.buttons[13]==1)
    {
        // CAMERA RVIZ CYCLE
        B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=true;
    }

    // SHARE + RECTANGLE
    else if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[3]==1)
    {
        // FOLLOW RVIZ CAMERA PRESENTER
        B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=true;
    }
    // SHARE + CROSS
    else if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[0]==1)
    {
        // AVATAR
        B_EVENT_INPDEV_AVATAR=true;
    }
    // SHARE + TRIANGLE
    else if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[2]==1)
    {
        // QUESTION
        I_EVENT_INPDEV_REQUEST=1;
    }
    // SHARE + CIRCLE
    else if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[1]==1)
    {
        // BREAK
        I_EVENT_INPDEV_REQUEST=2;
    }
    // SHARE + PSButton
    else if(msg_joy_input_dev.buttons[8]==1 && msg_joy_input_dev.buttons[10]==1)
    {
        // SEND MESSAGE
        I_EVENT_INPDEV_REQUEST=3;
    }
    // SHARE + PSButton
    else if(msg_joy_input_dev.buttons[9]==1 && msg_joy_input_dev.buttons[10]==1)
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
    B_EVENT_INPDEV_PRESENCE_MODE=msg->data;
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
/// Callback to receive ping request from presenter
////////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_hea_ping_from_presenter(const std_msgs::HeaderPtr& msg)
{
    std_msgs::Header msg_hea_ping_to_presenter;
    msg_hea_ping_to_presenter.stamp=msg->stamp; // Return timestamp of presenter
    std::stringstream sstr_ping_to_presenter;
    sstr_ping_to_presenter << "ping:from_audience_" << s_audience_id;
    msg_hea_ping_to_presenter.frame_id=sstr_ping_to_presenter.str();
    pub_hea_ping_to_presenter.publish(msg_hea_ping_to_presenter);
}



/////////////////////////////////////////////////
/// Callback to receive info msgs from presenter
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_hea_info_from_presenter(const std_msgs::HeaderPtr& msg)
{
    msg_hea_info_from_presenter=*msg;
    I_EVENT_INFO_FROM_PRESENTER=1;
    if(msg_hea_info_from_presenter.frame_id.find("CAMERA Enforce")!=std::string::npos)
    {
        I_EVENT_INFO_FROM_PRESENTER=2;
    }
    else if(msg_hea_info_from_presenter.frame_id.find("PRE-SHUTDOWN")!=std::string::npos)
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
        pub_cpl_rviz_camera.publish(*msg);
    }
}


///////////////////////////////////////////////////
/// Callback for LinkStat messages from Gazebo
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_lns_gazebo(const gazebo_msgs::LinkStatesPtr& msg)
{
    msg_lns_gazebo=(*msg);
}



/////////////////////////////////////////////////
/// Callback to receive camera intrinsics
/////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr& msg)
{
    if(B_ENABLE_3D_MODE==false) return;

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
    msg->header.frame_id="camera_rgbd_out"+s_audience_id+"_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    pub_cai_img_camera_audit_rgbd_rgb.publish(msg);
}
void WAIOAAudienceListener::cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr& msg)
{
    if(B_ENABLE_3D_MODE==false) return;

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
    msg->header.frame_id="camera_rgbd_out"+s_audience_id+"_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    pub_cai_img_camera_audit_rgbd_depth.publish(msg);
}



///////////////////////////////////////////////////
/// Callback for receiving RGB camera images
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    if(B_ENABLE_3D_MODE==false) return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mat_img_camera_rgbd_rgb = cv_ptr->image;
    if(F_CAMERA_RGBD_RESOLUTION_SCALE!=1.0)
        cv::resize(cv_ptr->image, mat_img_camera_rgbd_rgb,cv::Size(),F_CAMERA_RGBD_RESOLUTION_SCALE,F_CAMERA_RGBD_RESOLUTION_SCALE);
    msg_img_camera_rgbd_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_img_camera_rgbd_rgb).toImageMsg();
    msg_img_camera_rgbd_rgb->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_rgb->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg_img_camera_rgbd_rgb->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg_img_camera_rgbd_rgb->step=msg_img_camera_rgbd_rgb->width*3;
    msg_img_camera_rgbd_rgb->header.frame_id="camera_rgbd_out"+s_audience_id+"_rgb_optical_frame";

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    pub_img_camera_audit_rgbd_rgb.publish(msg_img_camera_rgbd_rgb);
}

///////////////////////////////////////////////////
/// Callback for receiving DEPTH camera images
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr& msg)
{
    tim_event_received_img_camera_3d=ros::Time::now();
    if(B_ENABLE_3D_MODE==false) return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.setTo(0, cv_ptr->image != cv_ptr->image);
        cv_ptr->image.setTo(0, cv_ptr->image > F_CAMERA_RGBD_RANGE_MAX+0.5);
        cv_ptr->image.setTo(0, cv_ptr->image < F_CAMERA_RGBD_RANGE_MIN);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mat_img_camera_rgbd_depth_32fc1=cv_ptr->image;
    if(F_CAMERA_RGBD_RESOLUTION_SCALE!=1.0)
        cv::resize(cv_ptr->image, mat_img_camera_rgbd_depth_32fc1,cv::Size(),F_CAMERA_RGBD_RESOLUTION_SCALE,F_CAMERA_RGBD_RESOLUTION_SCALE,cv::INTER_NEAREST);

    msg_img_camera_rgbd_depth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", mat_img_camera_rgbd_depth_32fc1).toImageMsg();
    msg_img_camera_rgbd_depth->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_depth->width=F_CAMERA_RGBD_RESOLUTION_X*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg_img_camera_rgbd_depth->height=F_CAMERA_RGBD_RESOLUTION_Y*F_CAMERA_RGBD_RESOLUTION_SCALE;
    msg_img_camera_rgbd_depth->step=msg_img_camera_rgbd_depth->width*4;
    msg_img_camera_rgbd_depth->header.frame_id="camera_rgbd_out"+s_audience_id+"_rgb_optical_frame"; // Still RGB frame, althouhg for depth images!

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    pub_img_camera_audit_rgbd_depth.publish(msg_img_camera_rgbd_depth);
}



///////////////////////////////////////////////////
/// Callback for receiving camera_2d
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_2d(const sensor_msgs::ImageConstPtr& msg)
{
    tim_event_received_img_camera_2d=ros::Time::now();
    if(B_ENABLE_3D_MODE==true) return;

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert ROS Image to OpenCV Image
    mat_img_camera_2d=cv_ptr->image;
    //msg_img_camera_2d=cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_camera_2d).toImageMsg();
    //msg_img_camera_2d = cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_camera_2d(cv::Rect(640/6,480/6,2*640/3,2*480/3))).toImageMsg();
    msg_img_camera_2d=cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_camera_2d(cv::Rect(160,120,320,240))).toImageMsg();
    pub_img_camera_2d.publish(msg_img_camera_2d);
}



///////////////////////////////////////////////////
/// Callback for receiving livecam
///////////////////////////////////////////////////
void WAIOAAudienceListener::cb_sub_img_camera_livecam(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert ROS Image to OpenCV Image
    mat_img_camera_livecam=cv_ptr->image;
    //cv::resize(mat_img_camera_robot_rgb, mat_img_camera_robot_rgb,cv::Size(),1.5,1.5); // resize to better fit projection wall
}



/////////////////////////////////////////////////
/// Helper methods
/////////////////////////////////////////////////
int WAIOAAudienceListener::GetSceneCountCurrent()
{
    return i_scene_count;
}

void WAIOAAudienceListener::SetOAStatusLabel(std::string s_oa_status)
{
    std_msgs::String msg_s_oa_status;
    msg_s_oa_status.data="ID"+s_audience_id+"-"+s_oa_status;
    pub_s_oa_status.publish(msg_s_oa_status);
    ros::spinOnce();
}

void WAIOAAudienceListener::SetActionTriggerLabel(std::string s_label)
{
    ((WAIRepTrigger*)wai_oaa_trg_action)->UpdateModel(
                tf::Vector3(0.0,0.5,0.85),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.25,0.125),
                "ID"+s_audience_id+"-"+s_label,col_cyan,false,true,F_SCENE_TRIGGER_TIMEOUT);
    wai_oaa_trg_action->UpdateView();
}
void WAIOAAudienceListener::PlaySound(std::string s_name_sound)
{
    sound_play::Sound snd_play=sc.waveSoundFromPkg("wai_oa_gazebo","resources/sounds/audience/"+s_name_sound+".wav");
    snd_play.play();
}

bool WAIOAAudienceListener::KeyIsPressed(KeySym ks)
{
    char keys_return[32];
    XQueryKeymap(dsp_x11_display, keys_return);
    KeyCode kc2=XKeysymToKeycode(dsp_x11_display, ks);
    bool isPressed= !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
    return isPressed;
}
void WAIOAAudienceListener::CheckTriggersFromKeyboard()
{
    // Reset all triggers
    B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE=false;
    B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW=false;
    B_EVENT_INPDEV_PRESENCE_MODE=false;
    B_EVENT_INPDEV_BODY_INTERACTION=false;
    B_EVENT_INPDEV_AVATAR=false;
    I_EVENT_INPDEV_REQUEST=0;

    // MODIFIER
    bool b_keyboard_modifier=KeyIsPressed(XK_Tab);

    // BASIC Interactions
    if(b_keyboard_modifier && KeyIsPressed(XK_Down))
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
        B_EVENT_INPDEV_PRESENCE_MODE=true;
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
    sst_setup_camera_rviz_defaults << s_path_nodename << "setup_scene_defaults/camera/default";

    nh.getParam(sst_setup_camera_rviz_defaults.str(), vec_camera_rviz_temp);
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
    msg_cpl_rviz_camera.interpolation_mode=i_mode;
    msg_cpl_rviz_camera.time_from_start.sec=i_duration;
    msg_cpl_rviz_camera.time_from_start.nsec=0;
    msg_cpl_rviz_camera.target_frame=s_frame;
    msg_cpl_rviz_camera.up.header.stamp=ros::Time::now();
    msg_cpl_rviz_camera.up.header.frame_id=s_frame;
    msg_cpl_rviz_camera.up.vector.x=0.0;
    msg_cpl_rviz_camera.up.vector.y=0.0;
    msg_cpl_rviz_camera.up.vector.z=1.0;
    msg_cpl_rviz_camera.eye.header.stamp=ros::Time::now();
    msg_cpl_rviz_camera.eye.header.frame_id=s_frame;
    msg_cpl_rviz_camera.eye.point=pnt_eye;
    msg_cpl_rviz_camera.focus.header.stamp=ros::Time::now();
    msg_cpl_rviz_camera.focus.header.frame_id=s_frame;
    msg_cpl_rviz_camera.focus.point=pnt_focus;
    pub_cpl_rviz_camera.publish(msg_cpl_rviz_camera);
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
        if((ros::Time::now()-tim_trigger_action).toSec()>F_SCENE_TRIGGER_TIMEOUT)
        {
            if(B_EVENT_INPDEV_PRESENCE_MODE)
            {
                tim_trigger_action=ros::Time::now();
                B_ENABLE_3D_MODE=!B_ENABLE_3D_MODE;
                if(B_ENABLE_3D_MODE==true)
                {
                    cv::Mat mat_img_empty(1,1,CV_8UC3,cv::Scalar(0,0,0));
                    msg_img_camera_2d=cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_empty).toImageMsg();
                    pub_img_camera_2d.publish(msg_img_camera_2d);

                    SetOAStatusLabel("LST TRG: [KEY] PRESENCE (3D)");
                    SetActionTriggerLabel("PRESENCE (3D)");
                }
                else
                {
                    SetOAStatusLabel("LST TRG: [KEY] PRESENCE (2D)");
                    SetActionTriggerLabel("PRESENCE (2D)");
                }
                PlaySound("bleep");
            }
            else if(B_EVENT_INPDEV_AVATAR)
            {
                tim_trigger_action=ros::Time::now();
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
                PlaySound("bleep");
            }
            else if(B_EVENT_INPDEV_SKETCH)
            {
                tim_trigger_action=ros::Time::now();
                B_EVENT_INPDEV_SKETCH=false;

                wai_oa_sketch.ReopenSketch();

                SetOAStatusLabel("LST TRG: [KEY] SKETCH");
                SetActionTriggerLabel("SKETCH");
                PlaySound("bleep");
            }
            else if(B_EVENT_INPDEV_BODY_INTERACTION)
            {
                tim_trigger_action=ros::Time::now();
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
                PlaySound("bleep");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                tim_trigger_action=ros::Time::now();
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
                PlaySound("bleep");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_DEFAULT
                    && !B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                tim_trigger_action=ros::Time::now();
                I_CAMERA_RVIZ_CYCLE_VIEW=0;

                geometry_msgs::Point pnt_cycle_eye;
                geometry_msgs::Point pnt_cycle_focus;
                pnt_cycle_eye.x=0.88793; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z= 1.1733;
                pnt_cycle_focus.x=0.0; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=1.0236;
                TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+s_audience_id+"/link_base");

                SetOAStatusLabel("LST TRG: [KEY] CAMERA Default");
                SetActionTriggerLabel("CAMERA Default");
                PlaySound("tick");
            }
            else if(B_EVENT_INPDEV_CAMERA_RVIZ_CYCLE
                    && !B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW)
            {
                tim_trigger_action=ros::Time::now();
                I_CAMERA_RVIZ_CYCLE_VIEW++;
                if(I_CAMERA_RVIZ_CYCLE_VIEW>3)I_CAMERA_RVIZ_CYCLE_VIEW=0;

                geometry_msgs::Point pnt_cycle_eye;
                geometry_msgs::Point pnt_cycle_focus;
                switch(I_CAMERA_RVIZ_CYCLE_VIEW)
                {
                    case 0: // First person view (default)
                        pnt_cycle_eye.x=0.88793; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z= 1.1733;
                        pnt_cycle_focus.x=0.0; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=1.0236;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+s_audience_id+"/link_base");
                    break;
                    case 1: // Sketch view
                        pnt_cycle_eye.x=0.51397; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z=1.4861;
                        pnt_cycle_focus.x=0.41221; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=0.56779;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+s_audience_id+"/link_base");
                    break;
                    case 2: // Third person view
                        pnt_cycle_eye.x=1.4279; pnt_cycle_eye.y=0.0; pnt_cycle_eye.z=1.3769;
                        pnt_cycle_focus.x=0.16624; pnt_cycle_focus.y=0.0; pnt_cycle_focus.z=0.61416;
                        TransitionCameraRviz(pnt_cycle_eye,pnt_cycle_focus,3,"workspace_audience_"+s_audience_id+"/link_base");
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
                PlaySound("tick");
            }
            else if(I_EVENT_INPDEV_REQUEST!=0)
            {
                tim_trigger_action=ros::Time::now();

                std::stringstream sst_audience_request;
                QString hea_req_aud_text;
                int ok_req_aud_que=QMessageBox::No;
                int ok_req_aud_brk=QMessageBox::No;
                bool ok_req_aud_msg=false;
                switch(I_EVENT_INPDEV_REQUEST)
                {
                    case 1:
                        sst_audience_request << "ID" << s_audience_id << "-QUESTION";
                        ok_req_aud_que = QMessageBox::warning(
                                    NULL,
                                    "QUESTION",
                                    "Do you reall want to send the\nrequest to the presenter?",
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
                        sst_audience_request << "ID" << s_audience_id << "-BREAK";
                        ok_req_aud_brk = QMessageBox::warning(
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
                        sst_audience_request << "ID" << s_audience_id << "-MESSAGE";

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
                        sst_audience_request << "ID" << s_audience_id << "N/A";
                    break;
                }

                if(ok_req_aud_brk==QMessageBox::Yes
                    || ok_req_aud_que==QMessageBox::Yes
                    || ok_req_aud_msg==true)
                {
                    msg_hea_audience_request.stamp=ros::Time::now();
                    msg_hea_audience_request.frame_id=sst_audience_request.str();
                    pub_msg_hea_audience_request.publish(msg_hea_audience_request);

                    SetActionTriggerLabel("REQUEST");
                    PlaySound("ping");
                }
                SetOAStatusLabel("LST TRG: [REQ] "+sst_audience_request.str());

                I_EVENT_INPDEV_REQUEST=0;
            }
            else if(I_EVENT_INFO_FROM_PRESENTER!=0)
            {
                tim_trigger_action=ros::Time::now();
                std::stringstream sst_info_from_presenter;
                switch(I_EVENT_INFO_FROM_PRESENTER)
                {
                    case 1:
                        sst_info_from_presenter << msg_hea_info_from_presenter.frame_id;
                    break;

                    case 2:
                        B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW=!B_ENABLE_ENFORCE_CAMERA_RVIZ_PRESENTER_FOLLOW;
                        sst_info_from_presenter << msg_hea_info_from_presenter.frame_id;
                    break;

                    default:
                    break;
                }
                I_EVENT_INFO_FROM_PRESENTER=0;

                SetOAStatusLabel("LST TRG: [INF] "+sst_info_from_presenter.str());
                //SetActionTriggerLabel(sst_info_from_presenter.str());
                SetActionTriggerLabel("PRE-INFO");
                PlaySound("ping");
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


        if(B_ENABLE_3D_MODE
                && (ros::Time::now()-tim_event_received_img_camera_3d).toSec()<1.0
                && !mat_img_camera_rgbd_depth_32fc1.empty())
        {
            // Process image and detect hand pose
            mat_img_camera_rgbd_depth_32fc1.copyTo(mat_img_camera_rgbd_depth_32fc1_processed);
            ImageProcessor.SetProcessedImage(mat_img_camera_rgbd_depth_32fc1_processed);
            tf::Matrix3x3 m33_vc3_hands_and_head_buf=ImageProcessor.Process();
            if(!std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getX())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getY())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getZ()))
            {
                vc3_camera_rgbd_wrt_head=m33_vc3_hands_and_head_buf.getRow(2);
            }

            /*
            if(!B_ENABLE_BODY_INTERACTION)
            {
                tf::Vector3 vc3_hand_left_origin(0.0,0.2,0.05); // origin is different with audience/listener workspace!
                tf::Vector3 vc3_hand_left(0,0,0);
                if(msg_joy_input_dev.axes.size()!=0)
                {
                    vc3_hand_left.setX(-3.0*msg_joy_input_dev.axes[3]);
                    vc3_hand_left.setY(-1.0*msg_joy_input_dev.axes[1]);
                    vc3_hand_left.setZ(+3.0*msg_joy_input_dev.axes[4]);
                }
                vc3_camera_rgbd_wrt_hand_left=vc3_hand_left_origin+vc3_hand_left;
            }
            */

            if(!B_ENABLE_AVATAR)
            {
                vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);
            }
        }
        else
        {
            vc3_camera_rgbd_wrt_head=tf::Vector3(0.4,-0.15,0.15);
        }
        ((WAIRepOOI*)wai_oaa_ooi_head)->UpdateModel(
                    vc3_camera_rgbd_wrt_head,
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    tf::Vector3(0.025,0.025,0.075),"Head",col_oa_shiny,"head",s_path_resources_representatives,true,1.1);
        wai_oaa_ooi_head->UpdateView();
        wai_oaa_trg_action->UpdateView();


        // ROS Spin
        ros::spinOnce();
        ros_rate.sleep();
    }
}
