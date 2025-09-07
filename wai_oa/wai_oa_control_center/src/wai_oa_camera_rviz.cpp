#include<wai_oa_camera_rviz.h>



/////////////////////////////////////////////////
/// Implementation of WAIOACameraRviz
/////////////////////////////////////////////////

WAIOACameraRviz::WAIOACameraRviz()
{
}

WAIOACameraRviz::~WAIOACameraRviz()
{
}

void WAIOACameraRviz::cb_tmr_camera_rviz_idle(const ros::TimerEvent& event)
{
    float m_f_camera_rviz_time=(ros::Time::now()-m_tim_camera_rviz_idle_start).toSec();

    if(m_i_camera_rviz_idle_mode==0)
    {
        m_pnt_camera_rviz_idle_orbit_eye.x=m_pnt_camera_rviz_idle_frame_origin.x -m_f_camera_rviz_idle_amplitude*cos(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time);
        m_pnt_camera_rviz_idle_orbit_eye.y=m_pnt_camera_rviz_idle_frame_origin.y -m_f_camera_rviz_idle_amplitude*sin(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time);
        m_pnt_camera_rviz_idle_orbit_eye.z=m_pnt_camera_rviz_idle_frame_origin.z +m_f_camera_rviz_idle_amplitude;
        TransitionCameraRviz(m_pnt_camera_rviz_idle_orbit_eye,m_pnt_camera_rviz_idle_frame_origin,0.0,"world");
    }
    else if(m_i_camera_rviz_idle_mode==1)
    {
        m_pnt_camera_rviz_idle_cylindric_focus.x=m_pnt_camera_rviz_focus_current.x+2.0*m_f_camera_rviz_idle_amplitude*sin(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time)*cos(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time);
        m_pnt_camera_rviz_idle_cylindric_focus.y=m_pnt_camera_rviz_focus_current.y+2.0*m_f_camera_rviz_idle_amplitude*sin(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time)*sin(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time);
        m_pnt_camera_rviz_idle_cylindric_focus.z=m_pnt_camera_rviz_focus_current.z+m_f_camera_rviz_idle_amplitude*cos(m_f_camera_rviz_idle_omega*m_f_camera_rviz_time);
        TransitionCameraRviz(m_pnt_camera_rviz_eye_current,m_pnt_camera_rviz_idle_cylindric_focus,0.0,m_s_frame);
    }
    else
    {
        // Invalid mode: Do nothing...
    }
}
void WAIOACameraRviz::cb_sub_pos_camera_rviz_follow_presenter_head(const geometry_msgs::PosePtr& msg)
{
    geometry_msgs::Pose pos_pres_head=*msg;
    m_tf_camera_rviz_follow_presenter_head.setOrigin(tf::Vector3(pos_pres_head.position.x,pos_pres_head.position.y,pos_pres_head.position.z));
    m_tf_camera_rviz_follow_presenter_head.setRotation(tf::Quaternion(pos_pres_head.orientation.x,pos_pres_head.orientation.y,pos_pres_head.orientation.z,pos_pres_head.orientation.w));
    /* ROS_WARN_STREAM("TF Camera: "
                   << m_tf_camera_rviz_follow_presenter_head.getOrigin().getX() << ";"
                   << m_tf_camera_rviz_follow_presenter_head.getOrigin().getY() << ";"
                   << m_tf_camera_rviz_follow_presenter_head.getOrigin().getZ()); */
}

void WAIOACameraRviz::cb_tmr_camera_rviz_follow_rep(const ros::TimerEvent& event)
{
    GetRepPositionViaService(m_s_rep_name_follow);
    geometry_msgs::Point pnt_focus_follow;
    pnt_focus_follow.x=m_vc3_rep_follow.getX();
    pnt_focus_follow.y=m_vc3_rep_follow.getY();
    pnt_focus_follow.z=m_vc3_rep_follow.getZ();
    TransitionCameraRviz(m_pnt_camera_rviz_eye_current,
                         pnt_focus_follow,
                         9.9*1.0/m_f_node_sample_frequency,
                         m_s_frame);
}

void WAIOACameraRviz::cb_tmr_camera_rviz_follow_presenter(const ros::TimerEvent& event)
{
    tf::Vector3 vc3_camera_follow_buffer=m_tf_camera_rviz_follow_presenter_head.getOrigin();
    geometry_msgs::Point pnt_focus_follow;
    pnt_focus_follow.x=vc3_camera_follow_buffer.getX();
    pnt_focus_follow.y=vc3_camera_follow_buffer.getY();
    pnt_focus_follow.z=vc3_camera_follow_buffer.getZ();
    double d_dur=0.5*1.0/m_f_node_sample_frequency;
    TransitionCameraRviz(m_pnt_camera_rviz_eye_current,pnt_focus_follow,d_dur,m_s_frame);
}

void WAIOACameraRviz::cb_tmr_camera_rviz_ooi(const ros::TimerEvent& event)
{
    // Publish TF for detail
    m_tfb_transforms->sendTransform(
            tf::StampedTransform( tf::Transform(tf::Quaternion(m_msg_pos_camera_rviz.pose.orientation.x,m_msg_pos_camera_rviz.pose.orientation.y,m_msg_pos_camera_rviz.pose.orientation.z,m_msg_pos_camera_rviz.pose.orientation.w),tf::Vector3(m_msg_pos_camera_rviz.pose.position.x,m_msg_pos_camera_rviz.pose.position.y,m_msg_pos_camera_rviz.pose.position.z)),
                                  ros::Time::now(),
                                  m_msg_pos_camera_rviz.header.frame_id,
                                  "presenter/camera_rviz") );
    m_tfb_transforms->sendTransform(
            tf::StampedTransform( m_tf_camera_wrt_detail,
                                  ros::Time::now(),
                                  "presenter/camera_rviz",
                                  "detail") );

    ((WAIRepOOI*)m_wai_oa_rep_ooi_camera_rviz)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,1.35,0.125),
                "Camera \n Presenter",
                col_oa_shiny,
                "camera_rviz",
                m_s_path_resources,
                false,
                1.0,
                1.0,
                true); // Set is camera to true to align label properly
    m_wai_oa_rep_ooi_camera_rviz->UpdateView();
}

void WAIOACameraRviz::cb_sub_pos_camera_rviz(const geometry_msgs::PoseStampedPtr& msg)
{
    m_msg_pos_camera_rviz=*msg;
}

void WAIOACameraRviz::Initialize(ros::NodeHandle* hdl_node,
                                 tf::TransformBroadcaster* tfb_transforms,
                                 std::string s_path_nodename,
                                 std::string s_path_resources,
                                 float f_node_sample_frequency,
                                 float f_duration,
                                 std::string s_frame,
                                 int i_mode)
{
    m_hdl_node=hdl_node;
    m_f_node_sample_frequency=f_node_sample_frequency;

    m_msg_pos_camera_rviz.header.frame_id="world";
    m_msg_pos_camera_rviz.header.stamp=ros::Time::now();
    m_msg_pos_camera_rviz.pose.position.x=1.0;
    m_msg_pos_camera_rviz.pose.position.y=1.0;
    m_msg_pos_camera_rviz.pose.position.y=1.0;
    m_msg_pos_camera_rviz.pose.orientation.w=1.0;
    m_msg_pos_camera_rviz.pose.orientation.x=0.0;
    m_msg_pos_camera_rviz.pose.orientation.y=0.0;
    m_msg_pos_camera_rviz.pose.orientation.z=0.0;

    m_tfb_transforms=tfb_transforms;
    m_tf_camera_rviz_follow_presenter_head=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    m_tf_camera_wrt_detail=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));

    m_d_duration=f_duration;
    m_s_path_nodename=s_path_nodename;
    m_s_path_resources=s_path_resources;
    m_s_frame=s_frame;
    m_i_mode=i_mode;
    m_i_camera_rviz_view_count=0;
    m_b_camera_follow_presenter_enabled=false;
    m_b_camera_follow_rep_enabled=false;

    // Init colors
    col_oa_shiny.r=0.101960784; col_oa_shiny.g=0.42745098; col_oa_shiny.b=0.588235294; col_oa_shiny.a=0.1;
    col_oa.r=0.101960784; col_oa.g=0.42745098; col_oa.b=0.588235294; col_oa.a=0.9;
    col_oa_opaque.r=0.101960784; col_oa_opaque.g=0.42745098; col_oa_opaque.b=0.588235294; col_oa_opaque.a=1.0;

    // Init camera idle
    m_tim_camera_rviz_idle_start=ros::Time::now();
    m_f_camera_rviz_idle_amplitude=10.0;
    m_f_camera_rviz_idle_omega=0.1;
    m_i_camera_rviz_idle_mode=0;

    // Init reference frame offset for idle cameras
    m_pnt_camera_rviz_idle_frame_origin.x=0.0;
    m_pnt_camera_rviz_idle_frame_origin.y=0.0;
    m_pnt_camera_rviz_idle_frame_origin.z=0.0;

    // Init current and old camera view
    m_pnt_camera_rviz_eye_current.x=1.0;
    m_pnt_camera_rviz_eye_current.y=1.0;
    m_pnt_camera_rviz_eye_current.z=1.0;
    m_pnt_camera_rviz_focus_current.x=0.0;
    m_pnt_camera_rviz_focus_current.y=0.0;
    m_pnt_camera_rviz_focus_current.z=0.0;
    m_pnt_camera_rviz_eye_old=m_pnt_camera_rviz_eye_current;
    m_pnt_camera_rviz_focus_old=m_pnt_camera_rviz_focus_current;
    m_vc3_rep_follow.setValue(0.0,0.0,0.0);

    // Init vectors to store eye focus for predefined views
    for(int i=0;i<6;i++)
    {
        m_vec_camera_rviz_target.push_back(0.0);

        m_vec_camera_rviz_break.push_back(0.0);
        m_vec_camera_rviz_default.push_back(0.0);
        m_vec_camera_rviz_graph_3d.push_back(0.0);
        m_vec_camera_rviz_graph_eval.push_back(0.0);
        m_vec_camera_rviz_overview.push_back(0.0);
        m_vec_camera_rviz_presenter.push_back(0.0);
        m_vec_camera_rviz_presenter_fpv.push_back(0.0);
        m_vec_camera_rviz_projection.push_back(0.0);
        m_vec_camera_rviz_startup.push_back(0.0);
        m_vec_camera_rviz_wim.push_back(0.0);
    }

    m_wai_oa_rep_ooi_camera_rviz=WAIReps::create_representative("OOI");
    m_wai_oa_rep_ooi_camera_rviz->Initialize(m_hdl_node,m_s_path_nodename,"wai_oa_rep_ooi_camera_rviz","presenter/camera_rviz");
    ((WAIRepOOI*)m_wai_oa_rep_ooi_camera_rviz)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,1.35,0.125),
                "Camera \n Presenter",
                col_oa_shiny,
                "camera_rviz",
                m_s_path_resources,
                true,
                1.0,
                1.0,
                true);

    ser_cli_gazebo_get_model_state=m_hdl_node->serviceClient<gazebo_msgs::GetModelState>("/wai_world/gazebo/get_model_state");

    m_pub_cpl_camera_rviz=m_hdl_node->advertise<view_controller_msgs::CameraPlacement>("/wai_world/oa/camera_rviz/placement",1,true);
    m_sub_pos_camera_rviz=m_hdl_node->subscribe("/wai_world/oa/camera_rviz/pose",1,&WAIOACameraRviz::cb_sub_pos_camera_rviz,this);
    m_sub_pos_camera_rviz_follow_presenter_head=m_hdl_node->subscribe("presenter_head_pose",1,&WAIOACameraRviz::cb_sub_pos_camera_rviz_follow_presenter_head,this);
    m_tmr_camera_rviz_idle=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOACameraRviz::cb_tmr_camera_rviz_idle,this,false,false);
    m_tmr_camera_rviz_follow_rep=m_hdl_node->createTimer(ros::Duration(10.0*1.0/m_f_node_sample_frequency),&WAIOACameraRviz::cb_tmr_camera_rviz_follow_rep,this,false,false);
    m_tmr_camera_rviz_follow_presenter=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOACameraRviz::cb_tmr_camera_rviz_follow_presenter,this,false,false);
    m_tmr_camera_rviz_ooi=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOACameraRviz::cb_tmr_camera_rviz_ooi,this,false,true);
}

void WAIOACameraRviz::UpdateModel(std::vector<double> vec_camera_rviz_target,bool b_cycle, bool b_back_forth, int i_view_index,float f_duration,std::string s_frame,int i_mode,bool b_camera_rviz_store_as_prev)
{
    m_d_duration=f_duration;
    m_s_frame=s_frame;
    m_i_mode=i_mode;

    if(b_cycle==false)
    {
        m_i_camera_rviz_view_count=0; // Modified
        m_vec_camera_rviz_target=vec_camera_rviz_target;
    }
    else
    {
        if(b_back_forth==false)
        {
            m_i_camera_rviz_view_count=0;
            m_vec_camera_rviz_target=m_vec_camera_rviz_views_predefined[m_i_camera_rviz_view_count];
        }
        else
        {
            if(i_view_index==-1)
            {
                m_i_camera_rviz_view_count++;
                if(m_i_camera_rviz_view_count==int(m_vec_camera_rviz_views_predefined.size()))
                {
                    m_i_camera_rviz_view_count=0;
                }
                m_vec_camera_rviz_target=m_vec_camera_rviz_views_predefined[m_i_camera_rviz_view_count];
            }
            else
            {
                m_vec_camera_rviz_target=m_vec_camera_rviz_views_predefined[i_view_index];
            }
        }
    }
    m_pnt_camera_rviz_eye_current.x=m_vec_camera_rviz_target[0];
    m_pnt_camera_rviz_eye_current.y=m_vec_camera_rviz_target[1];
    m_pnt_camera_rviz_eye_current.z=m_vec_camera_rviz_target[2];
    m_pnt_camera_rviz_focus_current.x=m_vec_camera_rviz_target[3];
    m_pnt_camera_rviz_focus_current.y=m_vec_camera_rviz_target[4];
    m_pnt_camera_rviz_focus_current.z=m_vec_camera_rviz_target[5];

    // Store current setting for reverting back to "previous" view in the next cycle
    if(b_camera_rviz_store_as_prev==true)
    {
        SaveCurrentViewAsPrevious();
    }
}

void WAIOACameraRviz::SaveCurrentViewAsPrevious()
{
    m_s_frame_old=m_s_frame;
    m_pnt_camera_rviz_eye_old=m_pnt_camera_rviz_eye_current;
    m_pnt_camera_rviz_focus_old=m_pnt_camera_rviz_focus_current;
    m_d_duration_old=m_d_duration;
}

std::string WAIOACameraRviz::GetCurrentFrame()
{
    return m_s_frame;
}
std::string WAIOACameraRviz::GetOldFrame()
{
    return m_s_frame_old;
}

void WAIOACameraRviz::RestorePreviousView()
{
    TransitionCameraRviz(m_pnt_camera_rviz_eye_old,
                         m_pnt_camera_rviz_focus_old,
                         m_d_duration_old,
                         m_s_frame_old,
                         m_i_mode); // Leave mode unchanged
    m_s_frame=m_s_frame_old;
}

void WAIOACameraRviz::UpdateView()
{
    TransitionCameraRviz(m_pnt_camera_rviz_eye_current,
                         m_pnt_camera_rviz_focus_current,
                         m_d_duration,
                         m_s_frame,
                         m_i_mode);
}

void WAIOACameraRviz::UpdateCameraRvizViews(std::vector< std::vector<double>> vec_camera_rviz_views_predefined)
{
    m_vec_camera_rviz_views_predefined=vec_camera_rviz_views_predefined;
}

void WAIOACameraRviz::TransitionCameraRviz(geometry_msgs::Point pnt_eye,geometry_msgs::Point pnt_focus,double d_duration,std::string s_frame,int i_mode)
{
    // Also update members here
    m_d_duration=d_duration;
    m_s_frame=s_frame;
    m_i_mode=i_mode;

    msg_cpl_camera_rviz.interpolation_mode=i_mode;
    msg_cpl_camera_rviz.time_from_start.sec=int(m_d_duration);
    msg_cpl_camera_rviz.time_from_start.nsec=(m_d_duration-int(m_d_duration))*1000000000.0;
    msg_cpl_camera_rviz.target_frame=s_frame;
    msg_cpl_camera_rviz.up.header.stamp=ros::Time::now();
    msg_cpl_camera_rviz.up.header.frame_id=s_frame;
    msg_cpl_camera_rviz.up.vector.x=0.0;
    msg_cpl_camera_rviz.up.vector.y=0.0;
    msg_cpl_camera_rviz.up.vector.z=1.0;
    msg_cpl_camera_rviz.eye.header.stamp=ros::Time::now();
    msg_cpl_camera_rviz.eye.header.frame_id=s_frame;
    msg_cpl_camera_rviz.eye.point=pnt_eye;
    msg_cpl_camera_rviz.focus.header.stamp=ros::Time::now();
    msg_cpl_camera_rviz.focus.header.frame_id=s_frame;
    msg_cpl_camera_rviz.focus.point=pnt_focus;
    m_pub_cpl_camera_rviz.publish(msg_cpl_camera_rviz);
}

void WAIOACameraRviz::SetTransformDetail(tf::Transform tf_camera_wrt_detail)
{
    m_tf_camera_wrt_detail=tf_camera_wrt_detail;
}

void WAIOACameraRviz::CameraIdleStart(float f_camera_rviz_idle_amplitude,
                                              float f_camera_rviz_idle_fomega,
                                              int i_camera_rviz_idle_mode,
                                              tf::Vector3 vc3_camera_rviz_idle_frame_origin)
{
    m_tmr_camera_rviz_idle.stop();
    m_tmr_camera_rviz_follow_rep.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_pnt_camera_rviz_idle_frame_origin.x=vc3_camera_rviz_idle_frame_origin.getX();
    m_pnt_camera_rviz_idle_frame_origin.y=vc3_camera_rviz_idle_frame_origin.getY();
    m_pnt_camera_rviz_idle_frame_origin.z=vc3_camera_rviz_idle_frame_origin.getZ();
    m_f_camera_rviz_idle_amplitude=f_camera_rviz_idle_amplitude;
    m_f_camera_rviz_idle_omega=f_camera_rviz_idle_fomega;
    m_i_camera_rviz_idle_mode=i_camera_rviz_idle_mode;
    m_tim_camera_rviz_idle_start=ros::Time::now();
    m_tmr_camera_rviz_idle.start();
}
void WAIOACameraRviz::CameraIdleStop()
{
    m_tmr_camera_rviz_follow_rep.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_tmr_camera_rviz_idle.stop();
}

bool WAIOACameraRviz::GetCameraFollowPresenterEnabled()
{
    return m_b_camera_follow_presenter_enabled;
}
void WAIOACameraRviz::ToggleCameraFollowPresenter()
{
    m_b_camera_follow_presenter_enabled=!m_b_camera_follow_presenter_enabled;
    if(m_b_camera_follow_presenter_enabled==true)
    {
        CameraFollowPresenterStart();
    }
    else
    {
        CameraFollowPresenterStop();
    }
}
void WAIOACameraRviz::CameraFollowPresenterStart()
{
    /*
    lpf_camera_rviz_follow->InitializeSetpoint(
                tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),
                              tf::Vector3(0.0,0.0,1.0)));
    */

    //SaveCurrentViewAsPrevious();

    m_s_frame="world";
    m_tmr_camera_rviz_idle.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_tmr_camera_rviz_follow_rep.stop();

    m_tmr_camera_rviz_follow_presenter.start();
}
void WAIOACameraRviz::CameraFollowPresenterStop(bool b_rest_prev_view)
{
    m_b_camera_follow_presenter_enabled=false;
    m_s_frame="world";
    m_tmr_camera_rviz_idle.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_tmr_camera_rviz_follow_rep.stop();

    //if(b_rest_prev_view==true) RestorePreviousView();
}

/*
bool WAIOACameraRviz::GetCameraFollowRepEnabled()
{
    return m_b_camera_follow_rep_enabled;
}
*/
void WAIOACameraRviz::CameraFollowRepStart(std::string s_rep_name_follow)
{
    //SaveCurrentViewAsPrevious();

    //m_b_camera_follow_rep_enabled=true;
    m_s_frame="world";
    m_tmr_camera_rviz_idle.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_tmr_camera_rviz_follow_rep.stop();

    m_s_rep_name_follow=s_rep_name_follow;
    m_tmr_camera_rviz_follow_rep.start();
}
void WAIOACameraRviz::CameraFollowRepStop()//bool b_rest_prev_view)
{
    //m_b_camera_follow_rep_enabled=false;
    m_s_frame="world";
    m_tmr_camera_rviz_idle.stop();
    m_tmr_camera_rviz_follow_presenter.stop();
    m_tmr_camera_rviz_follow_rep.stop();

    //if(b_rest_prev_view==true) RestorePreviousView();
}



void WAIOACameraRviz::GetRepPositionViaService(std::string s_rep_name)
{
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name=s_rep_name;
    get_model_state.request.relative_entity_name="world"; //Always given in world frame, s_rep_detail_name unused currently!
    if(ser_cli_gazebo_get_model_state.isValid())
    {
        ser_cli_gazebo_get_model_state.waitForExistence();
        if(ser_cli_gazebo_get_model_state.call(get_model_state))
        {
            m_vc3_rep_follow.setValue(get_model_state.response.pose.position.x,
                                      get_model_state.response.pose.position.y,
                                      get_model_state.response.pose.position.z);
        }
        else
        {
            ROS_WARN("Camera Follow: Service getting model state failed!");
        }
    }
}
