#include<wai_oa_presenter.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAPresenter
/////////////////////////////////////////////////

WAIOAPresenter::WAIOAPresenter():
    hints_camera_rgb_color("compressed", ros::TransportHints()), //hints_camera_rgb_color("theora", ros::TransportHints()),
    hints_camera_rgbd_color("compressed", ros::TransportHints()),
    hints_camera_rgbd_depth("compressedDepth", ros::TransportHints()),
    ImageProcessor(new HandsAndHeadDetector(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
{
}

WAIOAPresenter::~WAIOAPresenter()
{
}

void WAIOAPresenter::cb_tmr_presenter(const ros::TimerEvent& event)
{
    ProcessBodyInteraction();

    ((WAIRepOOI*)wai_oa_rep_ooi_hand_left)->UpdateModel(
                m_tf_world_wrt_hand_left.getOrigin(),
                m_tf_world_wrt_hand_left.getRotation(),
                tf::Vector3(0.025,0.25,0.075),"HAL",col_blue_trans,"hand_left",m_s_path_resources);
    wai_oa_rep_ooi_hand_left->UpdateView();
    ((WAIRepOOI*)wai_oa_rep_ooi_hand_right)->UpdateModel(
                m_tf_world_wrt_hand_right.getOrigin(),
                m_tf_world_wrt_hand_right.getRotation(),
                tf::Vector3(0.025,0.25,0.075),m_s_hand_right_text,col_red_trans,"hand_right",m_s_path_resources);
    wai_oa_rep_ooi_hand_right->UpdateView();

    ((WAIRepOOI*)wai_oa_rep_ooi_head)->UpdateModel(
                m_tf_world_wrt_head.getOrigin()+tf::Vector3(-0.05,-0.05,0.0),
                m_tf_world_wrt_head.getRotation(),
                tf::Vector3(0.025,0.025,0.075),"Head",col_green_trans,"head",m_s_path_resources,false,1.1);
    wai_oa_rep_ooi_head->UpdateView();
}

void WAIOAPresenter::cb_tmr_avatar_2d(const ros::TimerEvent& event)
{
    qt_qpo_mouse_cursor_pos=QApplication::desktop()->cursor().pos();
    cv_pnt_mouse_cursor_vel.x=(qt_qpo_mouse_cursor_pos.x()-qt_qpo_mouse_cursor_pos_old.x())*m_f_node_sample_frequency;
    cv_pnt_mouse_cursor_vel.y=(qt_qpo_mouse_cursor_pos.y()-qt_qpo_mouse_cursor_pos_old.y())*m_f_node_sample_frequency;

    int i_vel_min=250;
    int i_vel_max=750;
    int i_vel_delta=250;
    if(abs(cv_pnt_mouse_cursor_vel.x)>=i_vel_min && abs(cv_pnt_mouse_cursor_vel.x)<=i_vel_max) cv_pnt_mouse_cursor_vel.x=cv_pnt_mouse_cursor_vel.x/i_vel_delta;
    else if(cv_pnt_mouse_cursor_vel.x>i_vel_max) cv_pnt_mouse_cursor_vel.x=i_vel_max/i_vel_delta;
    else if(cv_pnt_mouse_cursor_vel.x<-i_vel_max) cv_pnt_mouse_cursor_vel.x=-i_vel_max/i_vel_delta;
    else cv_pnt_mouse_cursor_vel.x=0;
    if(abs(cv_pnt_mouse_cursor_vel.y)>=i_vel_min && abs(cv_pnt_mouse_cursor_vel.y)<=i_vel_max) cv_pnt_mouse_cursor_vel.y=cv_pnt_mouse_cursor_vel.y/i_vel_delta;
    else if(cv_pnt_mouse_cursor_vel.y>i_vel_max) cv_pnt_mouse_cursor_vel.y=i_vel_max/i_vel_delta;
    else if(cv_pnt_mouse_cursor_vel.y<-i_vel_max) cv_pnt_mouse_cursor_vel.y=-i_vel_max/i_vel_delta;
    else cv_pnt_mouse_cursor_vel.y=0;

    /* ROS_WARN_STREAM("Loading: " << m_s_path_resources+
                    "avatar2d/image"+
                    std::to_string(cv_pnt_mouse_cursor_vel.x)+
                    std::to_string(cv_pnt_mouse_cursor_vel.y)+
                    ".png"); */
    mat_img_avatar_2d=cv::imread(m_s_path_resources+
                                 "/avatar2d/image"+
                                 std::to_string(cv_pnt_mouse_cursor_vel.x)+
                                 std::to_string(cv_pnt_mouse_cursor_vel.y)+
                                 ".png");
    if(!mat_img_avatar_2d.empty())
    {
        cv::resize(mat_img_avatar_2d,mat_img_avatar_2d,cv::Size(426,320));
        msg_img_camera_2d = cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_avatar_2d).toImageMsg();
        pub_img_camera_2d.publish(msg_img_camera_2d);
    }

    qt_qpo_mouse_cursor_pos_old=qt_qpo_mouse_cursor_pos;
}

void WAIOAPresenter::Initialize(ros::NodeHandle* hdl_node,
                                        image_transport::ImageTransport* hdl_it,
                                        std::string s_path_nodename,
                                        std::string s_path_resources,
                                        tf::TransformBroadcaster* tfb_transforms,
                                        float f_node_sample_frequency,
                                        float f_camera_rgbd_resolution_x,
                                        float f_camera_rgbd_resolution_y,
                                        float f_camera_rgbd_resolution_scale,
                                        float f_camera_rgbd_fx,
                                        float f_camera_rgbd_fy,
                                        float f_camera_rgbd_cx,
                                        float f_camera_rgbd_cy,
                                        float f_camera_rgbd_range_min,
                                        float f_camera_rgbd_range_max,
                                        float f_camera_rgbd_mask_width,
                                        float f_camera_rgbd_threshold_dist,
                                        float f_camera_rgbd_threshold_bounds
                                        )
{
    m_hdl_node=hdl_node;
    m_hdl_it=hdl_it;
    m_s_path_nodename=s_path_nodename;
    m_s_path_resources=s_path_resources;
    m_s_hand_right_text="Dave";
    m_f_node_sample_frequency=f_node_sample_frequency;

    m_tfb_transforms=tfb_transforms;

    qt_qpo_mouse_cursor_pos.setX(0);
    qt_qpo_mouse_cursor_pos.setY(0);
    qt_qpo_mouse_cursor_pos_old.setX(0);
    qt_qpo_mouse_cursor_pos_old.setY(0);
    cv_pnt_mouse_cursor_vel.x=0;
    cv_pnt_mouse_cursor_vel.y=0;

    col_red_trans.r=1.0; col_red_trans.g=0.0; col_red_trans.b=0.0; col_red_trans.a=0.3;
    col_green_trans.r=0.0; col_green_trans.g=1.0; col_green_trans.b=0.0; col_green_trans.a=0.3;
    col_blue_trans.r=0.0; col_blue_trans.g=0.0; col_blue_trans.b=1.0; col_blue_trans.a=0.3;
    col_oa_trans.r=0.101960784; col_oa_trans.g=0.42745098; col_oa_trans.b=0.588235294; col_oa_trans.a=0.3;
    col_oa_shiny.r=0.101960784; col_oa_shiny.g=0.42745098; col_oa_shiny.b=0.588235294; col_oa_shiny.a=0.1;
    col_oa.r=0.101960784; col_oa.g=0.42745098; col_oa.b=0.588235294; col_oa.a=0.9;
    col_oa_opaque.r=0.101960784; col_oa_opaque.g=0.42745098; col_oa_opaque.b=0.588235294; col_oa_opaque.a=1.0;

    m_tf_presenter=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,1.0));
    m_tf_world_wrt_presenter_camera=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,1.0));
    m_tf_presenter_camera_wrt_camera_optical=tf::Transform(tf::Quaternion(-0.5,0.5,-0.5,0.5),tf::Vector3(0.0,0.0,0.0));

    m_lpf_presenter_pose.InitializeParameters(1.0/m_f_node_sample_frequency,1.0,0.01);
    m_lpf_presenter_pose.InitializeSetpoint(m_tf_world_wrt_presenter_camera);

    // Init other presenter transforms
    m_vc3_camera_rgbd_wrt_hand_left.setValue(0.0,0.0,0.0);
    m_vc3_camera_rgbd_wrt_hand_right.setValue(0.0,0.0,0.0);
    m_vc3_camera_rgbd_wrt_head.setValue(0.0,0.0,0.0);
    m_tf_camera_optical_wrt_hand_left=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    m_tf_camera_optical_wrt_hand_right=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    m_tf_camera_optical_wrt_head=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.8,0.5)); // Put it on floor in front of 3D sensor
    m_tf_world_wrt_hand_left=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_hand_left;
    m_tf_world_wrt_hand_right=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_hand_right;
    m_tf_world_wrt_head=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_head;

    m_f_camera_rgbd_resolution_x=f_camera_rgbd_resolution_x;
    m_f_camera_rgbd_resolution_y=f_camera_rgbd_resolution_y;
    m_f_camera_rgbd_resolution_scale=f_camera_rgbd_resolution_scale;
    m_f_camera_rgbd_fx=f_camera_rgbd_fx;
    m_f_camera_rgbd_fy=f_camera_rgbd_fy;
    m_f_camera_rgbd_cx=f_camera_rgbd_cx;
    m_f_camera_rgbd_cy=f_camera_rgbd_cy;
    m_f_camera_rgbd_range_min=f_camera_rgbd_range_min;
    m_f_camera_rgbd_range_max=f_camera_rgbd_range_max;
    m_f_camera_rgbd_mask_width=f_camera_rgbd_mask_width;
    m_f_camera_rgbd_threshold_dist=f_camera_rgbd_threshold_dist;
    m_f_camera_rgbd_threshold_bounds=f_camera_rgbd_threshold_bounds;

    // Init RGBD camera objects
    //mat_img_camera_rgbd_depth_32fc1=cv::Mat::zeros(cv::Size(F_CAMERA_RGBD_RESOLUTION_X,F_CAMERA_RGBD_RESOLUTION_Y),CV_32F);
    //mat_gc_mask=cv::Mat::zeros(cv::Size(F_CAMERA_RGBD_RESOLUTION_X,F_CAMERA_RGBD_RESOLUTION_Y),CV_8UC1);
    //mat_bgd_model=cv::Mat::zeros(cv::Size(1,65),CV_64FC1);
    //mat_fgd_model=cv::Mat::zeros(cv::Size(1,65),CV_64FC1);
    //p_bg_sub=cv::createBackgroundSubtractorKNN(); //cv::createBackgroundSubtractorMOG2();
    msg_cai_img_camera_virtual_rgbd_depth.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_depth.P.fill(0.0);

    msg_cai_img_camera_virtual_rgbd_rgb.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.D.push_back(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.K.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.R.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);
    msg_cai_img_camera_virtual_rgbd_rgb.P.fill(0.0);

    //sub_img_camera_rgbd_rgb=m_hdl_it->subscribe("camera_rgbd_in/rgb/image_raw", 1, &WAIOAPresenter::cb_sub_img_camera_rgbd_rgb,this);
    //sub_img_camera_rgbd_depth=m_hdl_it->subscribe("camera_rgbd_in/depth/image", 1, &WAIOAPresenter::cb_sub_img_camera_rgbd_depth,this); // Distances in coherent units already in /image topic!
    //sub_cai_img_camera_rgbd_rgb=m_hdl_node->subscribe("camera_rgbd_in/rgb/camera_info", 1, &WAIOAPresenter::cb_sub_cai_img_camera_rgbd_rgb,this);
    //sub_cai_img_camera_rgbd_depth=m_hdl_node->subscribe("camera_rgbd_in/depth/camera_info", 1, &WAIOAPresenter::cb_sub_cai_img_camera_rgbd_depth,this);

    // Virtual RGBD-camera frames are simply recorded in root namespace:
    sub_img_camera_virtual_rgbd_rgb=m_hdl_it->subscribe("/camera_rgbd_in/rgb/image_raw",1,&WAIOAPresenter::cb_sub_img_camera_virtual_rgbd_rgb,this,hints_camera_rgbd_color); // VIRTUAL PRESENTER
    sub_img_camera_virtual_rgbd_depth=m_hdl_it->subscribe("/camera_rgbd_in/depth/image",1,&WAIOAPresenter::cb_sub_img_camera_virtual_rgbd_depth,this,hints_camera_rgbd_depth); // VIRTUAL PRESENTER
    sub_img_camera_2d=m_hdl_it->subscribe("camera_rgb_in/usb_cam/image_raw",1,&WAIOAPresenter::cb_sub_img_camera_2d,this,hints_camera_rgb_color);
    //sub_img_camera_2d=m_hdl_node->subscribe("camera_rgb/usb_cam/image_raw/compressed", 1, &WAIOAPresenter::cb_sub_img_camera_2d,this);
    //sub_img_camera_robot_rgb=m_hdl_it->subscribe("/wai_world/car/camera_rgbd/rgb/image_raw", 1, &WAIOAPresenter::cb_sub_img_camera_robot_rgb,this);
    //sub_img_camera_robot_rgb=m_hdl_it->subscribe("/wai_world/tello/camera_rgb_front/image_raw", 1, &WAIOAPresenter::cb_sub_img_camera_robot_rgb, this);
    //sub_pcl_camera_virtual=m_hdl_node->subscribe("/camera/depth_registered/points", 1, &WAIOAPresenter::cb_sub_pcl_camera_virtual,this);

    pub_img_oa_camera_rgbd_rgb=m_hdl_it->advertise("camera_rgbd_out/rgb/image_raw",1); // Modified 2
    pub_img_oa_camera_rgbd_depth=m_hdl_it->advertise("camera_rgbd_out/depth/image_raw",1); // Modified 2
    pub_cai_oa_camera_rgbd_rgb=m_hdl_node->advertise<sensor_msgs::CameraInfo>("camera_rgbd_out/rgb/camera_info",1);
    pub_cai_oa_camera_rgbd_depth=m_hdl_node->advertise<sensor_msgs::CameraInfo>("camera_rgbd_out/depth/camera_info",1);
    pub_img_camera_virtual_rgbd_rgb=m_hdl_it->advertise("camera_rgbd_virtual_out/rgb/image_raw",1);
    pub_img_camera_virtual_rgbd_depth=m_hdl_it->advertise("camera_rgbd_virtual_out/depth/image_raw",1);
    pub_cai_img_camera_virtual_rgbd_rgb=m_hdl_node->advertise<sensor_msgs::CameraInfo>("camera_rgbd_virtual_out/rgb/camera_info",1);
    pub_cai_img_camera_virtual_rgbd_depth=m_hdl_node->advertise<sensor_msgs::CameraInfo>("camera_rgbd_virtual_out/depth/camera_info",1);
    //pub_pcl_camera_virtual=m_hdl_node->advertise<sensor_msgs::PointCloud2>("camera_rgbd_virtual_out/depth_registered/points",1);
    pub_img_camera_2d=m_hdl_it->advertise("camera_rgb_out/usb_cam/image_raw",1); // Modified
    pub_pos_world_wrt_head=m_hdl_node->advertise<geometry_msgs::Pose>("presenter_head_pose",1);

    // Init processing strategy for hand interaction
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
                m_f_camera_rgbd_range_min,
                m_f_camera_rgbd_range_max,
                m_f_camera_rgbd_threshold_dist,
                m_f_camera_rgbd_threshold_bounds,
                m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale,
                m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale,
                m_f_camera_rgbd_fx*m_f_camera_rgbd_resolution_scale,
                m_f_camera_rgbd_fy*m_f_camera_rgbd_resolution_scale,
                m_f_camera_rgbd_cx*m_f_camera_rgbd_resolution_scale,
                m_f_camera_rgbd_cy*m_f_camera_rgbd_resolution_scale);
    ImageProcessor.ChangeProcessingStrategy(s);

    cv::Mat mat_img_camera_2d_init(m_f_camera_rgbd_resolution_y,m_f_camera_rgbd_resolution_x,CV_8UC3,cv::Scalar(0,0,0));
    mat_img_camera_2d=mat_img_camera_2d_init; //cv::imread(wai_oa_session_manager.GetPathResourcesLogo());

    m_i_presence_mode=0;
    m_b_enable_body_interaction=false;
    m_b_enable_avatar=false;
    m_b_enable_kmf=false;

    // HAND REPs Initialization
    wai_oa_rep_ooi_hand_left=WAIReps::create_representative("OOI");
    wai_oa_rep_ooi_hand_left->Initialize(m_hdl_node,s_path_nodename,"wai_oa_rep_ooi_hand_left","world");
    ((WAIRepOOI*)wai_oa_rep_ooi_hand_left)->UpdateModel(
                m_tf_world_wrt_hand_left.getOrigin(),
                m_tf_world_wrt_hand_left.getRotation(),
                tf::Vector3(0.025,0.25,0.075),"HAL",col_blue_trans,"hand_left",m_s_path_resources,true);

    wai_oa_rep_ooi_hand_right=WAIReps::create_representative("OOI");
    wai_oa_rep_ooi_hand_right->Initialize(m_hdl_node,s_path_nodename,"wai_oa_rep_ooi_hand_right","world");
    ((WAIRepOOI*)wai_oa_rep_ooi_hand_right)->UpdateModel(
                m_tf_world_wrt_hand_right.getOrigin(),
                m_tf_world_wrt_hand_right.getRotation(),
                tf::Vector3(0.025,0.25,0.075),m_s_hand_right_text,col_red_trans,"hand_right",m_s_path_resources,true);

    wai_oa_rep_ooi_head=WAIReps::create_representative("OOI");
    wai_oa_rep_ooi_head->Initialize(m_hdl_node,s_path_nodename,"wai_oa_rep_ooi_head","world");
    ((WAIRepOOI*)wai_oa_rep_ooi_head)->UpdateModel(
                m_tf_world_wrt_head.getOrigin(),
                m_tf_world_wrt_head.getRotation(),
                tf::Vector3(0.025,0.025,0.075),"Head",col_oa_shiny,"head",m_s_path_resources,true,1.1);

    m_tmr_presenter=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAPresenter::cb_tmr_presenter,this,false,true);
    m_tmr_avatar_2d=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAPresenter::cb_tmr_avatar_2d,this,false,false);
}

void WAIOAPresenter::UpdateModel(int i_presence_mode,bool b_enable_kmf)
{
    m_i_presence_mode=i_presence_mode;
    m_b_enable_kmf=b_enable_kmf;

    if(m_i_presence_mode==0) // NO PRESENCE
    {
        // RESET PRESENTER INTERACTIONS
        m_b_enable_body_interaction=false;
        m_b_enable_avatar=false;
        m_tmr_avatar_2d.stop();

        sub_img_camera_rgbd_rgb.shutdown();
        sub_img_camera_rgbd_depth.shutdown();
        sub_cai_img_camera_rgbd_rgb.shutdown();
        sub_cai_img_camera_rgbd_depth.shutdown();
        sub_img_camera_2d.shutdown();

        DisableCameraRGB();
        DisableCameraRGBD();
    }
    else if(m_i_presence_mode==1) // 2D-AVATAR
    {
        // RESET PRESENTER INTERACTIONS
        m_b_enable_body_interaction=true;
        m_b_enable_avatar=false;
        m_tmr_avatar_2d.stop();

        // DISABLE 3D-Modes
        sub_img_camera_rgbd_rgb.shutdown();
        sub_img_camera_rgbd_depth.shutdown();
        sub_cai_img_camera_rgbd_rgb.shutdown();
        sub_cai_img_camera_rgbd_depth.shutdown();
        DisableCameraRGBD();

        // ENABLE 2D-AVATAR
        m_tmr_avatar_2d.start();
    }
    else if(m_i_presence_mode==2) // 2D-CAMERA
    {
        // RESET PRESENTER INTERACTIONS
        m_b_enable_body_interaction=true;
        m_b_enable_avatar=false;
        m_tmr_avatar_2d.stop();

        // DISABLE 3D-Modes
        sub_img_camera_rgbd_rgb.shutdown();
        sub_img_camera_rgbd_depth.shutdown();
        sub_cai_img_camera_rgbd_rgb.shutdown();
        sub_cai_img_camera_rgbd_depth.shutdown();
        DisableCameraRGBD();

        // ENABLE 2D-CAMERA
        sub_img_camera_2d=m_hdl_it->subscribe("camera_rgb_in/usb_cam/image_raw", 1, &WAIOAPresenter::cb_sub_img_camera_2d,this,hints_camera_rgb_color);
    }
    else if(m_i_presence_mode==3) // 3D-AVATAR (With NITE OpenNI library and movable avatar)
    {
        // DISABLE 2D-Mode
        sub_img_camera_2d.shutdown();
        DisableCameraRGB();
        m_tmr_avatar_2d.stop();

        // ENABLE 3D-Avatar Mode with NITE driver (currently unused)
        // ...
    }
    else if(m_i_presence_mode==4) // 3D-POINTCLOUD (from Kinect-like sensor)
    {
        // DISABLE 2D-Mode
        sub_img_camera_2d.shutdown();
        DisableCameraRGB();
        m_tmr_avatar_2d.stop();

        // ENABLE 3D-Mode
        sub_img_camera_rgbd_rgb=m_hdl_it->subscribe("camera_rgbd_in/rgb/image_raw", 1, &WAIOAPresenter::cb_sub_img_camera_rgbd_rgb,this);
        sub_img_camera_rgbd_depth=m_hdl_it->subscribe("camera_rgbd_in/depth/image", 1, &WAIOAPresenter::cb_sub_img_camera_rgbd_depth,this); // Distances in coherent units already in /image topic!
        sub_cai_img_camera_rgbd_rgb=m_hdl_node->subscribe("camera_rgbd_in/rgb/camera_info", 1, &WAIOAPresenter::cb_sub_cai_img_camera_rgbd_rgb,this);
        sub_cai_img_camera_rgbd_depth=m_hdl_node->subscribe("camera_rgbd_in/depth/camera_info", 1, &WAIOAPresenter::cb_sub_cai_img_camera_rgbd_depth,this);
    }
    else
    {
        // Do nothing...
    }
}

void WAIOAPresenter::UpdateView()
{

}



int WAIOAPresenter::GetPresenceMode()
{
    return m_i_presence_mode;
}

void WAIOAPresenter::ToggleAvatar()
{
    m_b_enable_avatar=!m_b_enable_avatar;
}
bool WAIOAPresenter::GetAvatarEnabled()
{
    return m_b_enable_avatar;
}

tf::Vector3 WAIOAPresenter::GetHandLeftPosition()
{
    return ((WAIRepOOI*)wai_oa_rep_ooi_hand_left)->GetPosition();
}
tf::Vector3 WAIOAPresenter::GetHandRightPosition()
{
    return ((WAIRepOOI*)wai_oa_rep_ooi_hand_right)->GetPosition();
}
void WAIOAPresenter::SetJoypadInputAxis(tf::Vector3 vc3_joypad_input_axis)
{
    m_vc3_joypad_input_axis=vc3_joypad_input_axis;
}

void WAIOAPresenter::SetupHandText(std::string s_hand_right_text)
{
    m_s_hand_right_text=s_hand_right_text;

    ((WAIRepOOI*)wai_oa_rep_ooi_hand_right)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,-1.0,0.0),
                tf::Vector3(0.025,0.25,0.075),m_s_hand_right_text,col_red_trans,"hand_right",m_s_path_resources,true);
    wai_oa_rep_ooi_hand_right->UpdateView();
}

void WAIOAPresenter::SetupRespawn2D3D(std::string s_rep_respawn_2d3d)
{
    ((WAIRepOOI*)wai_oa_rep_ooi_hand_right)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,-1.0,0.0),
                tf::Vector3(0.025,0.25,0.075),s_rep_respawn_2d3d,col_red_trans,s_rep_respawn_2d3d,m_s_path_resources,true);
    wai_oa_rep_ooi_hand_right->UpdateView();
}

void WAIOAPresenter::SetPresenterPose(float f_pose_x,float f_pose_y,float f_pose_z,float f_pose_yaw)
{
    tf::Vector3 vc3_world_wrt_presenter;
    tf::Quaternion qua_world_wrt_presenter;
    vc3_world_wrt_presenter.setValue(f_pose_x,f_pose_y,f_pose_z);
    qua_world_wrt_presenter.setRPY(0.0,0.0,f_pose_yaw);
    qua_world_wrt_presenter=qua_world_wrt_presenter.normalize();
    m_tf_world_wrt_presenter_camera.setOrigin(vc3_world_wrt_presenter);
    m_tf_world_wrt_presenter_camera.setRotation(qua_world_wrt_presenter);

    //m_lpf_presenter_pose.UpdateSetpoint(m_tf_world_wrt_presenter_camera);
}

tf::Transform WAIOAPresenter::GetPresenterLPFPose()
{
    m_tf_presenter=m_tf_world_wrt_presenter_camera;
    //m_tf_presenter=m_lpf_presenter_pose.CalculateIteration();
    return m_tf_presenter;
}

void WAIOAPresenter::ToggleBodyInteraction()
{
    m_b_enable_body_interaction=!m_b_enable_body_interaction;
}
bool WAIOAPresenter::GetBodyInteractionEnabled()
{
    return m_b_enable_body_interaction;
}
void WAIOAPresenter::ProcessBodyInteraction()
{
    // Reinit body interaction transforms, update on demand
    m_tf_camera_optical_wrt_hand_left=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.5,0.95,m_f_camera_rgbd_range_max/2.0)); // Put it on floor
    m_tf_camera_optical_wrt_hand_right=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(-0.5,0.95,m_f_camera_rgbd_range_max/2.0)); // Put it on floor
    m_tf_camera_optical_wrt_head=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.8,0.5)); // Put it on floor in front of 3D sensor
    //m_tf_world_wrt_head=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_head;

    if(m_i_presence_mode==4) // Only process in 3D-POINTCLOUD presence mode
    {
        // Process image and detect hand pose
        // Algorithm sees presenters left hand as right (from camera) and vice versa, we switch here at this interface!
        if(!mat_img_camera_rgbd_depth_32fc1.empty())
        {
            mat_img_camera_rgbd_depth_32fc1.copyTo(mat_img_camera_rgbd_depth_32fc1_detector);
            ImageProcessor.SetProcessedImage(mat_img_camera_rgbd_depth_32fc1_detector);

            tf::Matrix3x3 m33_vc3_hands_and_head_buf=ImageProcessor.Process();
            if(!std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getX())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getY())
                && !std::isnan(m33_vc3_hands_and_head_buf.getRow(2).getZ()))
            {
                m_vc3_camera_rgbd_wrt_head=m33_vc3_hands_and_head_buf.getRow(2);
            }

            m_vc3_camera_rgbd_wrt_hand_left=m33_vc3_hands_and_head_buf.getRow(1); // Switch hands for rviz camera perspective!
            m_vc3_camera_rgbd_wrt_hand_right=m33_vc3_hands_and_head_buf.getRow(0);

            //ROS_WARN_STREAM("TF Pres: "<<m_m33_vc3_hands_and_head.getRow(2).getX() << ";" << m_m33_vc3_hands_and_head.getRow(2).getY() << ";" << m_m33_vc3_hands_and_head.getRow(2).getZ());
        }

        // HAND LEFT AND RIGHT
        m_tf_camera_optical_wrt_hand_left=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),m_vc3_camera_rgbd_wrt_hand_left);
        if(m_b_enable_kmf)
        {
            m_kmf_hand_right.UpdateMeasurement3DOF(m_vc3_camera_rgbd_wrt_hand_right);
            m_vc3_camera_rgbd_wrt_hand_right=m_kmf_hand_right.Predict3DOF(1);
        }
        m_tf_camera_optical_wrt_hand_right=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),m_vc3_camera_rgbd_wrt_hand_right);

        // HEAD/AVATAR
        if(m_b_enable_avatar==true)
        {
            //ROS_WARN_STREAM("Data is not NAN: "<<m_m33_vc3_hands_and_head.getRow(2).getX() << ";" << m_m33_vc3_hands_and_head.getRow(2).getY() << ";" << m_m33_vc3_hands_and_head.getRow(2).getZ());

            // Bad idea with distribution of pointers amongst objects(!)
            // Be careful --> *m_tf_world_wrt_head_buffer=m_tf_world_wrt_head;
            // --> Either use boost shared pointer or currently synced via topic!
            m_tf_camera_optical_wrt_head=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),m_vc3_camera_rgbd_wrt_head);
            m_tf_world_wrt_head=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_head;
            m_msg_pos_world_wrt_head.position.x=m_tf_world_wrt_head.getOrigin().getX();
            m_msg_pos_world_wrt_head.position.y=m_tf_world_wrt_head.getOrigin().getY();
            m_msg_pos_world_wrt_head.position.z=m_tf_world_wrt_head.getOrigin().getZ();
            m_msg_pos_world_wrt_head.position.x=m_tf_world_wrt_head.getOrigin().getX();
            m_msg_pos_world_wrt_head.orientation.w=m_tf_world_wrt_head.getRotation().getW();
            m_msg_pos_world_wrt_head.orientation.x=m_tf_world_wrt_head.getRotation().getX();
            m_msg_pos_world_wrt_head.orientation.y=m_tf_world_wrt_head.getRotation().getY();
            m_msg_pos_world_wrt_head.orientation.z=m_tf_world_wrt_head.getRotation().getZ();
            pub_pos_world_wrt_head.publish(m_msg_pos_world_wrt_head);
        }
    }
    if(m_b_enable_body_interaction==true)
    {
        m_tf_camera_optical_wrt_hand_left=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.5,0.95,m_f_camera_rgbd_range_max/2.0)); // Put it on floor
        m_tf_camera_optical_wrt_hand_right=
                tf::Transform(
                    tf::Quaternion(0.0,0.0,0.0,1.0),
                    m_vc3_joypad_input_axis+tf::Vector3(0.0,0.0,2.0));
    }

    m_tf_world_wrt_hand_left=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_hand_left;
    m_tf_world_wrt_hand_right=m_tf_world_wrt_presenter_camera*m_tf_presenter_camera_wrt_camera_optical*m_tf_camera_optical_wrt_hand_right;

    m_tfb_transforms->sendTransform(tf::StampedTransform(GetPresenterLPFPose(),ros::Time::now(),"world","camera_rgbd_out_link"));
}



///////////////////////////////////////////////////
/// Callbacks for processing 2D CAMERA (PRESENTER)
///////////////////////////////////////////////////
void WAIOAPresenter::cb_sub_img_camera_2d(const sensor_msgs::ImageConstPtr& msg)
{
    // Check for proper presence mode
    if(m_i_presence_mode!=2) return; // Only process in 2D-CAMERA presence mode

    std_msgs::Header hea_temp=msg->header;
    hea_temp.stamp=ros::Time::now();

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // Convert ROS Image to OpenCV Image
        mat_img_camera_2d=cv_ptr->image;
        //cv::Mat roi=mat_img_camera_2d(cv::Rect(640/4,480/4,640/2,480/2));
        //cv::resize(mat_img_camera_2d, mat_img_camera_2d,cv::Size(),0.5,0.5);
        //Update GrabCut Model - Too slow for 25fps
        //cv::grabCut(mat_img_camera_2d,mat_gc_mask,cv::Rect(64,48,m_f_camera_rgbd_resolution_x-128,m_f_camera_rgbd_resolution_y-96),mat_bgd_model,mat_fgd_model,1,cv::GC_INIT_WITH_RECT);
        //cv::image.copyTo(mat_gc_mask,~result);
        //cv::imshow("FG Mask",mat_gc_mask*50);
        msg_img_camera_2d = cv_bridge::CvImage(hea_temp,"bgr8",mat_img_camera_2d(cv::Rect(640/6,480/6,2*640/3,2*480/3))).toImageMsg();
        //msg_img_camera_2d = cv_bridge::CvImage(hea_temp,"bgr8",mat_img_camera_2d(cv::Rect(640/4,480/4,1*640/2,1*480/2))).toImageMsg();
        pub_img_camera_2d.publish(msg_img_camera_2d);
        // ROS_WARN("cb_sub_img_camera_2d republished to out...");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
    }
}

void WAIOAPresenter::DisableCameraRGB()
{
    cv::Mat mat_img_empty(1, 1, CV_8UC3, cv::Scalar(0,0,0));
    msg_img_camera_2d = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_img_empty).toImageMsg();
    pub_img_camera_2d.publish(msg_img_camera_2d);
}



///////////////////////////////////////////////////
/// Callbacks for processing 3D CAMERA (PRESENTER)
///////////////////////////////////////////////////
void WAIOAPresenter::cb_sub_img_camera_rgbd_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    // Check for proper presence mode
    if(m_i_presence_mode!=4) return; // Only process in 3D-POINTCLOUD presence mode

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }

    //fastNL works very well, however way too slow for 25fps!
    //cv::fastNlMeansDenoisingColored(cv_ptr->image,mat_img_camera_rgbd_rgb,10,10,7,21);
    // cv::medianBlur(cv_ptr->image,mat_img_camera_rgbd_rgb,3);
    mat_img_camera_rgbd_rgb = cv_ptr->image;
    if(m_f_camera_rgbd_resolution_scale!=1.0)
        cv::resize(cv_ptr->image, mat_img_camera_rgbd_rgb,cv::Size(),m_f_camera_rgbd_resolution_scale,m_f_camera_rgbd_resolution_scale,cv::INTER_NEAREST);

    msg_img_camera_rgbd_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_img_camera_rgbd_rgb).toImageMsg();
    msg_img_camera_rgbd_rgb->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_rgb->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_rgb->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_rgb->step=msg_img_camera_rgbd_rgb->width*3;
    msg_img_camera_rgbd_rgb->header.frame_id="camera_rgbd_out_rgb_optical_frame";

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    pub_img_oa_camera_rgbd_rgb.publish(msg_img_camera_rgbd_rgb);
}

void WAIOAPresenter::cb_sub_cai_img_camera_rgbd_rgb(const sensor_msgs::CameraInfoPtr& msg)
{
    // Check for proper presence mode
    if(m_i_presence_mode!=4) return; // Only process in 3D-POINTCLOUD presence mode

    // Receive intrinsics via camera_info topic
    //F_CAMERA_RGBD_FX=msg->K[0];
    //F_CAMERA_RGBD_FY=msg->K[4];
    //F_CAMERA_RGBD_CX=msg->K[2];
    //F_CAMERA_RGBD_CY=msg->K[5];
    m_f_camera_rgbd_p_fx=msg->P[0];
    m_f_camera_rgbd_p_fy=msg->P[5];
    m_f_camera_rgbd_p_cx=msg->P[2];
    m_f_camera_rgbd_p_cy=msg->P[6];
    msg->K[0]=msg->K[0]*m_f_camera_rgbd_resolution_scale;
    msg->K[4]=msg->K[4]*m_f_camera_rgbd_resolution_scale;
    msg->K[2]=msg->K[2]*m_f_camera_rgbd_resolution_scale;
    msg->K[5]=msg->K[5]*m_f_camera_rgbd_resolution_scale;
    msg->P[0]=msg->P[0]*m_f_camera_rgbd_resolution_scale;
    msg->P[5]=msg->P[5]*m_f_camera_rgbd_resolution_scale;
    msg->P[2]=msg->P[2]*m_f_camera_rgbd_resolution_scale;
    msg->P[6]=msg->P[6]*m_f_camera_rgbd_resolution_scale;
    msg->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg->header.frame_id="camera_rgbd_out_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    pub_cai_oa_camera_rgbd_rgb.publish(msg);
}

void WAIOAPresenter::cb_sub_img_camera_rgbd_depth(const sensor_msgs::ImageConstPtr& msg)
{
    // Check for proper presence mode
    if(m_i_presence_mode!=4) return; // Only process in 3D-POINTCLOUD presence mode

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.setTo(0, cv_ptr->image != cv_ptr->image);
        cv_ptr->image.setTo(0, cv_ptr->image > m_f_camera_rgbd_range_max+0.5);
        cv_ptr->image.setTo(0, cv_ptr->image < m_f_camera_rgbd_range_min);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }

    // Convert ROS Image to OpenCV Image, PREPROCESS and REPUBLISH
    //Noteworthy conversions:
    //mono16: CV_16UC1, 16-bit grayscale image
    //bgr8: CV_8UC3, color image with blue-green-red color order
    //mat_depth_uint16.convertTo(mat_img_camera_rgbd_depth_32fc1, CV_32FC1);
    //mat_img_camera_rgbd_depth_32fc1=mat_img_camera_rgbd_depth_32fc1/1000.0;

    mat_img_camera_rgbd_depth_32fc1=cv_ptr->image;
    if(m_f_camera_rgbd_resolution_scale!=1.0)
        cv::resize(cv_ptr->image, mat_img_camera_rgbd_depth_32fc1,cv::Size(),m_f_camera_rgbd_resolution_scale,m_f_camera_rgbd_resolution_scale,cv::INTER_NEAREST);

    cv::Mat roi_top=mat_img_camera_rgbd_depth_32fc1(cv::Rect(0,0,m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale,m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale*m_f_camera_rgbd_mask_width*4.0/3.0));
    roi_top.setTo(0.0);
    cv::Mat roi_left=mat_img_camera_rgbd_depth_32fc1(cv::Rect(0,0,m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale*m_f_camera_rgbd_mask_width,m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale));
    roi_left.setTo(0.0);
    cv::Mat roi_right=mat_img_camera_rgbd_depth_32fc1(cv::Rect(m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale*(1.0-m_f_camera_rgbd_mask_width),0,m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale*m_f_camera_rgbd_mask_width,m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale));
    roi_right.setTo(0.0);

    //cv::medianBlur(mat_img_camera_rgbd_depth_32fc1,mat_img_camera_rgbd_depth_32fc1,3);
    //cv::GaussianBlur(mat_img_camera_rgbd_depth_32fc1,mat_img_camera_rgbd_depth_32fc1,cv::Size(13,13),0.0,0.0,cv::BORDER_DEFAULT);
    //cv::bilateralFilter(mat_img_camera_rgbd_depth_32fc1,mat_img_camera_rgbd_depth_32fc1,3,50,50,cv::BORDER_DEFAULT);

    msg_img_camera_rgbd_depth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", mat_img_camera_rgbd_depth_32fc1).toImageMsg();
    msg_img_camera_rgbd_depth->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_depth->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_depth->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_depth->step=msg_img_camera_rgbd_depth->width*4;
    msg_img_camera_rgbd_depth->header.frame_id="camera_rgbd_out_rgb_optical_frame";

    // REPUBLISH ORIGINAL IMAGE FROM SENSOR HERE:
    pub_img_oa_camera_rgbd_depth.publish(msg_img_camera_rgbd_depth);
}

void WAIOAPresenter::cb_sub_cai_img_camera_rgbd_depth(const sensor_msgs::CameraInfoPtr& msg)
{
    // Check for proper presence mode
    if(m_i_presence_mode!=4) return; // Only process in 3D-POINTCLOUD presence mode

    // Receive intrinsics via camera_info topic
    //F_CAMERA_RGBD_FX=msg->K[0];
    //F_CAMERA_RGBD_FY=msg->K[4];
    //F_CAMERA_RGBD_CX=msg->K[2];
    //F_CAMERA_RGBD_CY=msg->K[5];
    m_f_camera_rgbd_p_fx=msg->P[0];
    m_f_camera_rgbd_p_fy=msg->P[5];
    m_f_camera_rgbd_p_cx=msg->P[2];
    m_f_camera_rgbd_p_cy=msg->P[6];
    msg->K[0]=msg->K[0]*m_f_camera_rgbd_resolution_scale;
    msg->K[4]=msg->K[4]*m_f_camera_rgbd_resolution_scale;
    msg->K[2]=msg->K[2]*m_f_camera_rgbd_resolution_scale;
    msg->K[5]=msg->K[5]*m_f_camera_rgbd_resolution_scale;
    msg->P[0]=msg->P[0]*m_f_camera_rgbd_resolution_scale;
    msg->P[5]=msg->P[5]*m_f_camera_rgbd_resolution_scale;
    msg->P[2]=msg->P[2]*m_f_camera_rgbd_resolution_scale;
    msg->P[6]=msg->P[6]*m_f_camera_rgbd_resolution_scale;
    msg->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg->header.frame_id="camera_rgbd_out_rgb_optical_frame";
    msg->header.stamp=ros::Time::now();
    pub_cai_oa_camera_rgbd_depth.publish(msg);
}

void WAIOAPresenter::DisableCameraRGBD()
{
    cv::Mat mat_img_empty_rgbd_rgb(m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale,m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale,CV_8UC3);
    mat_img_empty_rgbd_rgb.setTo(cv::Scalar(0,0,0));
    msg_img_camera_rgbd_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_img_empty_rgbd_rgb).toImageMsg();
    msg_img_camera_rgbd_rgb->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_rgb->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_rgb->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_rgb->step=msg_img_camera_rgbd_rgb->width*3; // BGR8-->1Byte*3-->*3
    msg_img_camera_rgbd_rgb->header.frame_id="camera_rgbd_out_rgb_optical_frame";

    cv::Mat mat_img_empty_rgbd_depth(m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale,m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale,CV_32FC1);
    mat_img_empty_rgbd_depth.setTo(0.0);
    msg_img_camera_rgbd_depth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", mat_img_empty_rgbd_depth).toImageMsg();
    msg_img_camera_rgbd_depth->header.stamp=ros::Time::now();
    msg_img_camera_rgbd_depth->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_depth->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_rgbd_depth->step=msg_img_camera_rgbd_depth->width*4; // 32FC1-->4Byte*1-->*4
    msg_img_camera_rgbd_depth->header.frame_id="camera_rgbd_out_rgb_optical_frame";

    for(int i=0;i<3;i++)
    {
        pub_img_oa_camera_rgbd_rgb.publish(msg_img_camera_rgbd_rgb);
        pub_img_oa_camera_rgbd_depth.publish(msg_img_camera_rgbd_depth);
        ros::spinOnce();
        ros::Rate(m_f_node_sample_frequency).sleep();
    }
}



/////////////////////////////////////////////////////
/// Callbacks for processing VIRTUAL PRESENTER (PCL)
/////////////////////////////////////////////////////
void WAIOAPresenter::cb_sub_img_camera_virtual_rgbd_rgb(const sensor_msgs::ImageConstPtr& msg)
{
    // Intrinsics are multiplied by 0.5 due to switch to 320x240 resolution!
    msg_cai_img_camera_virtual_rgbd_rgb.header.frame_id="camera_virtual_rgb_optical_frame";
    msg_cai_img_camera_virtual_rgbd_rgb.header.stamp=ros::Time::now();
    msg_cai_img_camera_virtual_rgbd_rgb.width=m_f_camera_rgbd_resolution_x;
    msg_cai_img_camera_virtual_rgbd_rgb.height=m_f_camera_rgbd_resolution_y;
    msg_cai_img_camera_virtual_rgbd_rgb.K[0]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.K[4]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.K[2]=0.5*319.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.K[5]=0.5*239.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.K[8]=1.0;
    msg_cai_img_camera_virtual_rgbd_rgb.P[0]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.P[5]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.P[2]=0.5*319.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.P[6]=0.5*239.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_rgb.P[10]=1.0;
    msg_cai_img_camera_virtual_rgbd_rgb.distortion_model="plumb_bob";
    msg_cai_img_camera_virtual_rgbd_rgb.D[0]=0.0;
    msg_cai_img_camera_virtual_rgbd_rgb.D[1]=0.0;
    msg_cai_img_camera_virtual_rgbd_rgb.D[2]=0.0;
    msg_cai_img_camera_virtual_rgbd_rgb.D[3]=0.0;
    msg_cai_img_camera_virtual_rgbd_rgb.binning_x=0;
    msg_cai_img_camera_virtual_rgbd_rgb.binning_y=0;
    pub_cai_img_camera_virtual_rgbd_rgb.publish(msg_cai_img_camera_virtual_rgbd_rgb);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }

    msg_img_camera_virtual_rgbd_rgb = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    msg_img_camera_virtual_rgbd_rgb->header.stamp=ros::Time::now();
    msg_img_camera_virtual_rgbd_rgb->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_virtual_rgbd_rgb->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_virtual_rgbd_rgb->step=msg_img_camera_virtual_rgbd_rgb->width*3;
    msg_img_camera_virtual_rgbd_rgb->header.frame_id="camera_virtual_rgb_optical_frame";
    pub_img_camera_virtual_rgbd_rgb.publish(msg_img_camera_virtual_rgbd_rgb);
}

void WAIOAPresenter::cb_sub_img_camera_virtual_rgbd_depth(const sensor_msgs::ImageConstPtr& msg)
{
    // Intrinsics are multiplied by 0.5 due to switch to 320x240 resolution!
    msg_cai_img_camera_virtual_rgbd_depth.header.frame_id="camera_virtual_rgb_optical_frame";
    msg_cai_img_camera_virtual_rgbd_depth.header.stamp=ros::Time::now();
    msg_cai_img_camera_virtual_rgbd_depth.width=m_f_camera_rgbd_resolution_x;
    msg_cai_img_camera_virtual_rgbd_depth.height=m_f_camera_rgbd_resolution_y;
    msg_cai_img_camera_virtual_rgbd_depth.K[0]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.K[4]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.K[2]=0.5*319.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.K[5]=0.5*239.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.K[8]=1.0;
    msg_cai_img_camera_virtual_rgbd_depth.P[0]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.P[5]=0.5*570.3422241210938*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.P[2]=0.5*319.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.P[6]=0.5*239.5*m_f_camera_rgbd_resolution_scale;
    msg_cai_img_camera_virtual_rgbd_depth.P[10]=1.0;
    msg_cai_img_camera_virtual_rgbd_depth.distortion_model="plumb_bob";
    msg_cai_img_camera_virtual_rgbd_depth.D[0]=0.0;
    msg_cai_img_camera_virtual_rgbd_depth.D[1]=0.0;
    msg_cai_img_camera_virtual_rgbd_depth.D[2]=0.0;
    msg_cai_img_camera_virtual_rgbd_depth.D[3]=0.0;
    msg_cai_img_camera_virtual_rgbd_depth.binning_x=0;
    msg_cai_img_camera_virtual_rgbd_depth.binning_y=0;
    pub_cai_img_camera_virtual_rgbd_depth.publish(msg_cai_img_camera_virtual_rgbd_depth);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.setTo(0, cv_ptr->image != cv_ptr->image);
        cv_ptr->image.setTo(0, cv_ptr->image > m_f_camera_rgbd_range_max+0.5);
        cv_ptr->image.setTo(0, cv_ptr->image < m_f_camera_rgbd_range_min);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }

    msg_img_camera_virtual_rgbd_depth = cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_ptr->image).toImageMsg();
    msg_img_camera_virtual_rgbd_depth->header.stamp=ros::Time::now();
    msg_img_camera_virtual_rgbd_depth->width=m_f_camera_rgbd_resolution_x*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_virtual_rgbd_depth->height=m_f_camera_rgbd_resolution_y*m_f_camera_rgbd_resolution_scale;
    msg_img_camera_virtual_rgbd_depth->step=msg_img_camera_virtual_rgbd_depth->width*4;
    msg_img_camera_virtual_rgbd_depth->header.frame_id="camera_virtual_rgb_optical_frame";
    pub_img_camera_virtual_rgbd_depth.publish(msg_img_camera_virtual_rgbd_depth);
}

void WAIOAPresenter::cb_sub_pcl_camera_virtual(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    sensor_msgs::PointCloud2 msg_pcl_camera_virtual_processed;
    msg_pcl_camera_virtual_processed=*msg;
    msg_pcl_camera_virtual_processed.header.stamp=ros::Time::now();
    msg_pcl_camera_virtual_processed.header.frame_id="camera_virtual_rgb_optical_frame";
    //msg_pcl_camera_virtual_processed.header

    // Here goes some processing
    //msg_pcl_camera_virtual_processed.

    pub_pcl_camera_virtual.publish(msg_pcl_camera_virtual_processed);
}
