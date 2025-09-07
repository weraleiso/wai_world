#include<wai_oa_projector.h>



/////////////////////////////////////////////////
/// Implementation of WAIOAProjector
/////////////////////////////////////////////////

WAIOAProjector::WAIOAProjector():
    hints("compressed", ros::TransportHints())
{
}

WAIOAProjector::~WAIOAProjector()
{
}

void WAIOAProjector::cb_tmr_projector_play_video(const ros::TimerEvent& event)
{
    if(m_vcp_play_video.isOpened())
    {
        m_vcp_play_video >> m_mat_play_video_frame;
        if(m_mat_play_video_frame.empty())
        {
            m_vcp_play_video.release();
            m_tmr_projector_play_video.stop();
        }
        else
        {
            m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(),"bgr8",m_mat_play_video_frame).toImageMsg();
            m_pub_img_projector.publish(m_msg_img_projector);
        }
    }
}

void WAIOAProjector::cb_tmr_projector_ooi(const ros::TimerEvent& event)
{
    // Publish TF for projector
    m_tfb_transforms->sendTransform(tf::StampedTransform(GetProjectionPose(),ros::Time::now(),"world","projector"));
}

void WAIOAProjector::cb_tmr_projector(const ros::TimerEvent& event)
{
    float f_tim_elapsed=(ros::Time::now()-m_tim_transition_start).toSec();

    if(f_tim_elapsed<m_f_transition_time)
    {
        if(m_s_transition_mode.compare("dark_to_light")==0)
        {
            m_mat_img_projector_transition=m_mat_img_projector*f_tim_elapsed/m_f_transition_time;
            m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_projector_transition).toImageMsg();

        }
        else if(m_s_transition_mode.compare("zoom_rectangle_middle")==0)
        {
            m_mat_img_projector_transition=m_mat_img_projector*0.0;
            int i_img_width=f_tim_elapsed/m_f_transition_time*m_mat_img_projector.cols;
            int i_img_height=f_tim_elapsed/m_f_transition_time*m_mat_img_projector.rows;
            if(i_img_width==0)i_img_width=1;
            if(i_img_height==0)i_img_height=1;

            cv::Rect img_roi((m_mat_img_projector_transition.cols-i_img_width)/2,
                             (m_mat_img_projector_transition.rows-i_img_height)/2,
                             i_img_width,
                             i_img_height);
            m_mat_img_projector(img_roi).copyTo(m_mat_img_projector_transition(img_roi));
            m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_projector_transition).toImageMsg();
        }
        else if(m_s_transition_mode.compare("zoom_rectangle_top_left")==0)
        {
            m_mat_img_projector_transition=m_mat_img_projector*0.0;
            int i_img_width=f_tim_elapsed/m_f_transition_time*m_mat_img_projector.cols;
            int i_img_height=f_tim_elapsed/m_f_transition_time*m_mat_img_projector.rows;
            if(i_img_width==0)i_img_width=1;
            if(i_img_height==0)i_img_height=1;

            cv::Rect img_roi(0,0,i_img_width,i_img_height);
            m_mat_img_projector(img_roi).copyTo(m_mat_img_projector_transition(img_roi));
            m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_projector_transition).toImageMsg();
        }
        else
        {
            // No valid transition mode selected, just publish original image...
            m_mat_img_projector_transition=m_mat_img_projector;
            m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_projector_transition).toImageMsg();
        }
        m_pub_img_projector.publish(m_msg_img_projector);
        // ros::spinOnce(); Check if necessary...
    }
    else
    {
        m_tmr_projector.stop();
    }
}

void WAIOAProjector::Initialize(ros::NodeHandle* hdl_node,
                                        image_transport::ImageTransport* hdl_it,
                                        tf::TransformBroadcaster* tfb_transforms,
                                        float f_node_sample_frequency,
                                        std::string s_path_image_default)
{
    m_hdl_node=hdl_node;
    m_hdl_it=hdl_it;
    m_tfb_transforms=tfb_transforms;
    m_f_node_sample_frequency=f_node_sample_frequency;

    m_tf_world_wrt_projection.setOrigin(tf::Vector3(0.0,0.0,1.0));
    m_tf_world_wrt_projection.setRotation(tf::Quaternion(-0.5,0.5,-0.5,0.5)*tf::Quaternion(0.0,0.0,0.0,1.0));

    m_s_path_image_default=s_path_image_default;
    m_s_transition_mode="";
    m_f_transition_time=3.0;

    // Initialize image with logo
    m_mat_img_projector=cv::imread(m_s_path_image_default);
    m_pub_img_projector=m_hdl_it->advertise("projector/image_raw",1);

    m_tmr_projector=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAProjector::cb_tmr_projector,this,false,false);
    m_tmr_projector_ooi=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAProjector::cb_tmr_projector_ooi,this,false,true);
    m_tmr_projector_play_video=m_hdl_node->createTimer(ros::Duration(1.0/29.97),&WAIOAProjector::cb_tmr_projector_play_video,this,false,false);
}

void WAIOAProjector::UpdateModel(cv::Mat mat_img_projector,std::string s_path_image)
{
    m_mat_img_projector=mat_img_projector;

    if(m_mat_img_projector.empty())
    {
        m_mat_img_projector=cv::imread(s_path_image);
        if(m_mat_img_projector.empty())
        {
            m_mat_img_projector=cv::imread(m_s_path_image_default);
        }
    }

    // Shrink image improving performance
    /*
    cv::Size siz_shrink;
    siz_shrink.width=960;
    siz_shrink.height=540;
    cv::resize(m_mat_img_projector,m_mat_img_projector,siz_shrink);
    */
}

void WAIOAProjector::UpdateView(std::string s_transition_mode,float f_transition_time)
{
    m_s_transition_mode=s_transition_mode;
    m_f_transition_time=f_transition_time;

    if(m_s_transition_mode.compare("")==0)
    {
        m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(),"bgr8",m_mat_img_projector).toImageMsg();
        m_pub_img_projector.publish(m_msg_img_projector);
    }
    else
    {
        m_tim_transition_start=ros::Time::now();
        m_mat_img_projector.copyTo(m_mat_img_projector_transition);
        m_tmr_projector.start();
    }
}

void WAIOAProjector::SetProjectionPose(float f_pose_x,float f_pose_y,float f_pose_z,float f_pose_yaw)
{
    tf::Vector3 vc3_world_wrt_projection;
    tf::Quaternion qua_world_wrt_projection;
    vc3_world_wrt_projection.setValue(f_pose_x,f_pose_y,f_pose_z);
    qua_world_wrt_projection.setRPY(0.0,0.0,f_pose_yaw);
    qua_world_wrt_projection=qua_world_wrt_projection.normalize();
    m_tf_world_wrt_projection.setOrigin(vc3_world_wrt_projection);
    m_tf_world_wrt_projection.setRotation(tf::Quaternion(0.0,0.0,-0.5,0.5)*qua_world_wrt_projection);
}

tf::Transform WAIOAProjector::GetProjectionPose()
{
    return m_tf_world_wrt_projection;
}

void WAIOAProjector::EnablePlayVideo(std::string s_path_video,std::string s_logo_image)
{
    m_s_path_video=s_path_video;

    if(m_vcp_play_video.isOpened())
    {
        m_vcp_play_video.release();
    }

    if(m_vcp_play_video.open(s_path_video))
    {
        m_tmr_projector_play_video.start();
    }
    else
    {
        ROS_WARN("Video: Could not open video file!");
        m_mat_img_projector=cv::imread(s_logo_image);
        if(m_mat_img_projector.empty())
        {
            m_mat_img_projector=cv::imread(m_s_path_image_default);
        }
        m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_projector).toImageMsg();
        m_pub_img_projector.publish(m_msg_img_projector);
    }
}
void WAIOAProjector::DisablePlayVideo()
{
    m_tmr_projector_play_video.stop();
    if(m_vcp_play_video.isOpened())
    {
        m_vcp_play_video.release();
    }
}

void WAIOAProjector::EnableRobotcam(std::string s_robot_name)
{
    // Update ROBOTCAM and LIVECAM (data flow is controlled by subscribers)
    m_sub_img_camera_robot_rgb=m_hdl_it->subscribe("/wai_world/"+s_robot_name+"/camera_front/image_raw", 1, &WAIOAProjector::cb_m_sub_img_camera_robot_rgb,this,hints);
}
void WAIOAProjector::DisableRobotcam()
{
    m_sub_img_camera_robot_rgb.shutdown();
    UpdateView();
}

void WAIOAProjector::EnableLivecam()
{
    m_sub_img_camera_livecam=m_hdl_it->subscribe("camera_rgb_livecam/usb_cam/image_raw", 1, &WAIOAProjector::cb_m_sub_img_camera_livecam,this,hints);
}
void WAIOAProjector::DisableLivecam()
{
    m_sub_img_camera_livecam.shutdown();
    UpdateView();
}

///////////////////////////////////////////////////
/// Callback for ROBOTCAM (rgb images)
///////////////////////////////////////////////////
void WAIOAProjector::cb_m_sub_img_camera_robot_rgb(const sensor_msgs::ImageConstPtr& msg)
{
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

    // Convert ROS Image to OpenCV Image
    m_mat_img_camera_robotcam=cv_ptr->image;
    //cv::Size s; s.width=960; s.height=720;
    //cv::resize(m_mat_img_camera_robotcam, m_mat_img_camera_robotcam,s); // resize to better fit projection wall

    // Publish image (data flow of image controlled directly via subscriber!)
    try
    {
        m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_camera_robotcam).toImageMsg();
        m_pub_img_projector.publish(m_msg_img_projector);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }
}

///////////////////////////////////////////////////
/// Callback for receiving LIVECAM
///////////////////////////////////////////////////
void WAIOAProjector::cb_m_sub_img_camera_livecam(const sensor_msgs::ImageConstPtr& msg)
{
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

    // Convert ROS Image to OpenCV Image
    m_mat_img_camera_livecam=cv_ptr->image;
    //cv::resize(m_mat_img_camera_livecam, m_mat_img_camera_livecam,cv::Size(),1.5,1.5); // resize to better fit projection wall

    // Publish image (data flow of image controlled directly via subscriber!)
    try
    {
        m_msg_img_projector=cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_mat_img_camera_livecam).toImageMsg();
        m_pub_img_projector.publish(m_msg_img_projector);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_DEBUG("cv_bridge exception: %s", e.what());
        return;
    }
}
