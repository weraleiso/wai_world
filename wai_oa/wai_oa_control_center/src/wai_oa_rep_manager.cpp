#include<wai_oa_rep_manager.h>



/////////////////////////////////////////////////
/// Implementation of WAIOARepManager
/////////////////////////////////////////////////

WAIOARepManager::WAIOARepManager()
{
}

WAIOARepManager::~WAIOARepManager()
{
}
void WAIOARepManager::cb_tmr_rep_manager_eval_metaphor(const ros::TimerEvent& event)
{
    mrk_eval_metaphor.points.clear();
    mrk_eval_metaphor.colors.clear();

    for(int i=0;i<msg_lns_gazebo.pose.size();i++) // Crawl all links
    {
        if(msg_lns_gazebo.name[i].find("eval_sphere_green")!=std::string::npos)
        {
            geometry_msgs::Point pnt_eval_metaphor;
            pnt_eval_metaphor.x=msg_lns_gazebo.pose[i].position.x;
            pnt_eval_metaphor.y=msg_lns_gazebo.pose[i].position.y;
            pnt_eval_metaphor.z=msg_lns_gazebo.pose[i].position.z;
            std_msgs::ColorRGBA col_eval_metaphor;
            col_eval_metaphor.a = 1.0;
            col_eval_metaphor.r = 0.0;
            col_eval_metaphor.g = 1.0;
            col_eval_metaphor.b = 0.0;
            mrk_eval_metaphor.points.push_back(pnt_eval_metaphor);
            mrk_eval_metaphor.colors.push_back(col_eval_metaphor);
        }
        else if(msg_lns_gazebo.name[i].find("eval_sphere_orange")!=std::string::npos)
        {
            geometry_msgs::Point pnt_eval_metaphor;
            pnt_eval_metaphor.x=msg_lns_gazebo.pose[i].position.x;
            pnt_eval_metaphor.y=msg_lns_gazebo.pose[i].position.y;
            pnt_eval_metaphor.z=msg_lns_gazebo.pose[i].position.z;
            std_msgs::ColorRGBA col_eval_metaphor;
            col_eval_metaphor.a = 1.0;
            col_eval_metaphor.r = 1.0;
            col_eval_metaphor.g = 0.65;
            col_eval_metaphor.b = 0.0;
            mrk_eval_metaphor.points.push_back(pnt_eval_metaphor);
            mrk_eval_metaphor.colors.push_back(col_eval_metaphor);
        }
        else if(msg_lns_gazebo.name[i].find("eval_sphere_red")!=std::string::npos)
        {
            geometry_msgs::Point pnt_eval_metaphor;
            pnt_eval_metaphor.x=msg_lns_gazebo.pose[i].position.x;
            pnt_eval_metaphor.y=msg_lns_gazebo.pose[i].position.y;
            pnt_eval_metaphor.z=msg_lns_gazebo.pose[i].position.z;
            std_msgs::ColorRGBA col_eval_metaphor;
            col_eval_metaphor.a = 1.0;
            col_eval_metaphor.r = 1.0;
            col_eval_metaphor.g = 0.0;
            col_eval_metaphor.b = 0.0;
            mrk_eval_metaphor.points.push_back(pnt_eval_metaphor);
            mrk_eval_metaphor.colors.push_back(col_eval_metaphor);
        }
        else
        {
            // Do nothing...
        }

        if(mrk_eval_metaphor.points.size()!=0
            && mrk_eval_metaphor.colors.size()!=0)
        {
            pub_mrk_eval_metaphor.publish(mrk_eval_metaphor);
        }
    }
}

void WAIOARepManager::cb_tmr_rep_manager(const ros::TimerEvent& event)
{
    if(m_b_rep_detail_rot)
    {
        m_f_rep_detail_yaw+=0.0;
        m_f_rep_detail_pitch+=0.015;
        m_f_rep_detail_roll+=0.01;
    }

    if(m_i_counter_downsample==int(m_f_node_sample_frequency/5.0))
    {
        GetRepStateViaService(m_s_rep_selected,m_s_rep_detail_selected,&m_vc3_rep_state_pos,&m_qua_rep_state_ori,&m_vc3_rep_state_eul,&m_vc3_rep_state_vel_lin,&m_vc3_rep_state_vel_ang);
        m_sst_oa_detail_state_label.str("");
        m_sst_oa_detail_state_label.setf(std::ios::fixed);
        m_sst_oa_detail_state_label << std::setprecision(2) << m_sst_oa_detail_label.str();

        // Exclude STATE info for "PRESENTER Introduction" mode
        if(m_s_rep_selected.compare("presenter")!=0)
        {
            m_sst_oa_detail_state_label    << "\nSTATE:\n"
                                                << "[X,Y,Z]: " << m_vc3_rep_state_pos.getX() << "|"<< m_vc3_rep_state_pos.getY() << "|"<< m_vc3_rep_state_pos.getZ() << "\n"
                                                << "[R,P,Y]: " << m_vc3_rep_state_eul.getX()*180.0/M_PI << "|"<< m_vc3_rep_state_eul.getY()*180.0/M_PI << "|"<< m_vc3_rep_state_eul.getZ()*180.0/M_PI << "\n"
                                                << "[Vx,Vy,Vz]: " << m_vc3_rep_state_vel_lin.getX() << "|"<< m_vc3_rep_state_vel_lin.getY() << "|"<< m_vc3_rep_state_vel_lin.getZ() << "\n"
                                                << "[Wx,Wy,Wz]: " << m_vc3_rep_state_vel_ang.getX()*180.0/M_PI << "|"<< m_vc3_rep_state_vel_ang.getY()*180.0/M_PI << "|"<< m_vc3_rep_state_vel_ang.getZ()*180.0/M_PI;
        }
        ((TextMarker*)mrk_oa_detail_label)->UpdateText(m_sst_oa_detail_state_label.str());

        m_i_counter_downsample=0;
    }
    m_i_counter_downsample++;

    m_qua_oa_detail.setRPY(m_f_rep_detail_roll,m_f_rep_detail_pitch,m_f_rep_detail_yaw);
    ((WAIRepOOI*)wai_oa_rep_detail)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_detail,
                tf::Vector3(0.025,0.05,0.05),
                CleanupStringDetail(m_s_rep_detail_selected),
                col_oa_trans,
                m_s_rep_detail_selected,
                m_s_path_reps+m_s_rep_selected+"/"+m_s_rep_detail_selected+".dae",
                true,
                1.0,
                1.0);

    // Update views of detail mesh and label
    wai_oa_rep_detail->UpdateView();
    pub_mrk_oa_detail_label.publish(mrk_oa_detail_label->GetMarker());
}

void WAIOARepManager::Initialize(ros::NodeHandle* hdl_node,
                                         std::string s_path_nodename,
                                         std::string s_path_reps,
                                         float f_node_sample_frequency)
{
    m_hdl_node=hdl_node;
    m_s_path_nodename=s_path_nodename;
    m_s_path_reps=s_path_reps;
    m_f_node_sample_frequency=f_node_sample_frequency;
    m_i_counter_downsample=0; // 1 sec interval to update state details

    col_invisible.r=0.0; col_invisible.g=0.0; col_invisible.b=0.0; col_invisible.a=0.0;
    col_black.r=0.0; col_black.g=0.0; col_black.b=0.0; col_black.a=1.0;
    col_white.r=1.0; col_white.g=1.0; col_white.b=1.0; col_white.a=1.0;
    col_oa_trans.r=0.101960784; col_oa_trans.g=0.42745098; col_oa_trans.b=0.588235294; col_oa_trans.a=0.3;
    col_oa_shiny.r=0.101960784; col_oa_shiny.g=0.42745098; col_oa_shiny.b=0.588235294; col_oa_shiny.a=0.1;
    col_oa.r=0.101960784; col_oa.g=0.42745098; col_oa.b=0.588235294; col_oa.a=0.9;
    col_oa_opaque.r=0.101960784; col_oa_opaque.g=0.42745098; col_oa_opaque.b=0.588235294; col_oa_opaque.a=1.0;
    col_cyan_opaque.r=0.0; col_cyan_opaque.g=1.0; col_cyan_opaque.b=1.0; col_cyan_opaque.a=1.0;

    m_vec_s_reps.clear();
    m_vec_s_reps_details.clear();

    m_i_rep_selected=0; // reps and details selection belongs to WIM
    m_i_rep_detail_selected=0;
    m_s_rep_selected="workspace_presenter";
    m_s_rep_detail_selected="link_base";
    m_b_rep_detail_rot=true;
    m_f_rep_detail_yaw=0.0;
    m_f_rep_detail_pitch=0.0;
    m_f_rep_detail_roll=0.0;

    ser_cli_gazebo_get_model_state=m_hdl_node->serviceClient<gazebo_msgs::GetModelState>("/wai_world/gazebo/get_model_state");
    ser_cli_gazebo_set_model_state=m_hdl_node->serviceClient<gazebo_msgs::SetModelState>("/wai_world/gazebo/set_model_state");
    ser_cli_gazebo_apply_body_wrench=m_hdl_node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/wai_world/gazebo/apply_body_wrench");
    ser_cli_gazebo_delete_model=m_hdl_node->serviceClient<gazebo_msgs::DeleteModel>("/wai_world/gazebo/delete_model");
    ser_cli_gazebo_spawn_model=m_hdl_node->serviceClient<gazebo_msgs::SpawnModel>("/wai_world/gazebo/spawn_urdf_model");

    sub_lns_gazebo=m_hdl_node->subscribe("/wai_world/gazebo/link_states",1,&WAIOARepManager::cb_sub_lns_gazebo,this);
    //sub_mds_gazebo=m_hdl_node->subscribe("/wai_world/gazebo/model_states",1,&WAIOARepManager::cb_sub_mds_gazebo,this);
    //sub_tf2_transforms=m_hdl_node->subscribe("/tf",1,&WAIOARepManager::cb_sub_tf2_transforms,this);

    pub_mrk_oa_detail_label=m_hdl_node->advertise<visualization_msgs::Marker>("mrk_oa_detail_label",1);
    pub_mrk_eval_metaphor=m_hdl_node->advertise<visualization_msgs::Marker>("mrk_oa_eval_metaphor",1);

    m_tmr_rep_manager=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOARepManager::cb_tmr_rep_manager,this,false,false);
    m_tmr_rep_manager_eval_metaphor=m_hdl_node->createTimer(ros::Duration(2.0*1.0/m_f_node_sample_frequency),&WAIOARepManager::cb_tmr_rep_manager_eval_metaphor,this,false,false);

    m_sst_oa_detail_label.str("DETAILS");
    mrk_oa_detail_label=WAIRvizMarkers::create_rviz_marker("TEXT");
    mrk_oa_detail_label->Initialize(s_path_nodename,"presenter/camera_rviz",m_sst_oa_detail_label.str());
    mrk_oa_detail_label->UpdatePose(tf::Vector3(-0.21,0.0,0.5),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.015)); // 0.0175
    mrk_oa_detail_label->UpdateColor(col_invisible);

    // Init detail marker and disable
    wai_oa_rep_detail=WAIReps::create_representative("OOI");
    wai_oa_rep_detail->Initialize(hdl_node,s_path_nodename,"wai_oa_rep_detail","detail");
    ((WAIRepOOI*)wai_oa_rep_detail)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.025,0.05,0.05),
                CleanupStringDetail(m_s_rep_detail_selected),
                col_oa_trans,
                m_s_rep_detail_selected,
                m_s_path_reps+m_s_rep_selected+"/"+m_s_rep_detail_selected+".dae",
                true,
                1.0,
                0.0);
    wai_oa_rep_detail->UpdateView();

    // Init spheres marker for eval models
    InitEvalMetaphor();
}

void WAIOARepManager::InitEvalMetaphor()
{
    mrk_eval_metaphor.header.frame_id = "world";
    mrk_eval_metaphor.header.stamp = ros::Time::now();
    mrk_eval_metaphor.ns = m_s_path_nodename+"/eval_metaphor";
    mrk_eval_metaphor.id = 0; // Modified 5000
    mrk_eval_metaphor.type = visualization_msgs::Marker::SPHERE_LIST;
    mrk_eval_metaphor.action = visualization_msgs::Marker::ADD;
    mrk_eval_metaphor.pose.position.x = 0.0;
    mrk_eval_metaphor.pose.position.y = 0.0;
    mrk_eval_metaphor.pose.position.z = 0.0;
    mrk_eval_metaphor.pose.orientation.x = 0.0;
    mrk_eval_metaphor.pose.orientation.y = 0.0;
    mrk_eval_metaphor.pose.orientation.z = 0.0;
    mrk_eval_metaphor.pose.orientation.w = 1.0;
    mrk_eval_metaphor.scale.x = 0.1;
    mrk_eval_metaphor.scale.y = 0.1;
    mrk_eval_metaphor.scale.z = 0.1;
    mrk_eval_metaphor.lifetime=ros::Duration(0.0); // Modified
    std_msgs::ColorRGBA col_rgbd;
    col_rgbd.a=1.0;
    col_rgbd.r=1.0;
    col_rgbd.g=1.0;
    col_rgbd.b=1.0;
    mrk_eval_metaphor.color = col_rgbd;

    // Init points and colors properties
    mrk_eval_metaphor.points.clear();
    mrk_eval_metaphor.colors.clear();
}

void WAIOARepManager::UpdateModel(std::string s_rep,std::string s_detail)
{
    m_s_rep_selected=s_rep;
    m_s_rep_detail_selected=s_detail;

    UpdateRepsDetailsLabel();
}

void WAIOARepManager::UpdateView()
{

}

void WAIOARepManager::UpdateRepsDetailsLabel()
{
    std::stringstream sst_details;
    std::string s_details_name="";
    std::string s_details_year="";
    std::string s_details_specs="";
    std::string s_details_description="";

    sst_details.str("");
    sst_details << "/wai_world/" << m_s_rep_selected << "/details/" << m_s_rep_detail_selected << "/name";
    if(ros::param::has(sst_details.str()))
    {
        m_hdl_node->getParam(sst_details.str(),s_details_name);
    }
    else
    {
        s_details_name="No name available!";
    }

    sst_details.str("");
    sst_details << "/wai_world/" << m_s_rep_selected << "/details/" << m_s_rep_detail_selected << "/year";
    if(ros::param::has(sst_details.str()))
    {
        m_hdl_node->getParam(sst_details.str(), s_details_year);
    }
    else
    {
        s_details_year="No year available!";
    }

    sst_details.str("");
    sst_details << "/wai_world/" << m_s_rep_selected << "/details/" << m_s_rep_detail_selected << "/specs";
    if(ros::param::has(sst_details.str()))
    {
        m_hdl_node->getParam(sst_details.str(), s_details_specs);
        s_details_specs=GetRepDetailsFormatted(s_details_specs);
    }
    else s_details_specs="No specs available!";

    sst_details.str("");
    sst_details << "/wai_world/" << m_s_rep_selected << "/details/" << m_s_rep_detail_selected << "/description";
    if(ros::param::has(sst_details.str()))
    {
        m_hdl_node->getParam(sst_details.str(), s_details_description);
        s_details_description=GetRepDetailsFormatted(s_details_description);
    }
    else s_details_description="No description available!";

    m_sst_oa_detail_label.str("");
    m_sst_oa_detail_label  << "========== REPS AND DETAILS ==========\n"
                                << "REPS: \"" << m_s_rep_selected << "\"/\"" << m_s_rep_detail_selected << "\"\n"
                                << "NAME: " << s_details_name << "\n"
                                << "YEAR: " << s_details_year << "\n"
                                << "SPEC: " << s_details_specs << "\n"
                                << "DSCR: " << "\n" << s_details_description;

    ((TextMarker*)mrk_oa_detail_label)->UpdateText(m_sst_oa_detail_label.str());
    mrk_oa_detail_label->UpdateColor(col_cyan_opaque);
}

std::vector<std::string> WAIOARepManager::GetCurrentRepsIndex()
{
    sub_mds_gazebo=m_hdl_node->subscribe("/wai_world/gazebo/model_states",1,&WAIOARepManager::cb_sub_mds_gazebo,this);
    ros::Rate r_sleep(m_f_node_sample_frequency);
    while(msg_mds_gazebo.name.size()==0)
    {
        msg_mds_gazebo=msg_mds_gazebo_sub;
        ros::spinOnce();
        r_sleep.sleep();
    }
    m_s_rep_selected=msg_mds_gazebo.name[0]; // Initialize selected rep

    m_vec_s_reps.clear();
    for(int i=0;i<msg_mds_gazebo.name.size();i++)
    {
        m_vec_s_reps.push_back(msg_mds_gazebo.name[i]);
    }
    sub_mds_gazebo.shutdown();
    msg_mds_gazebo.name.clear();
    return m_vec_s_reps;
}
std::vector<std::string> WAIOARepManager::GetCurrentRepsDetailsIndex()
{
    sub_tf2_transforms=m_hdl_node->subscribe("/tf",200,&WAIOARepManager::cb_sub_tf2_transforms,this);
    ros::Rate r_sleep(m_f_node_sample_frequency);
    for(int i=0;i<2*int(m_f_node_sample_frequency);i++)
    {
        ros::spinOnce();
        r_sleep.sleep();
    }
    sub_tf2_transforms.shutdown();
    return m_vec_s_reps_details;
}

bool WAIOARepManager::GetRepExists(std::string s_rep_name)
{
    bool b_success_call=false;
    bool b_success_msg=false;
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name=s_rep_name;
    get_model_state.request.relative_entity_name="world"; //Always given in world frame, s_rep_detail_name unused currently!
    if(ser_cli_gazebo_get_model_state.isValid())
    {
        ser_cli_gazebo_get_model_state.waitForExistence();
        b_success_call=ser_cli_gazebo_get_model_state.call(get_model_state);
        b_success_msg=get_model_state.response.success;
        if(b_success_call && b_success_msg)
        {
            ROS_DEBUG_STREAM("REPMANAGER: Rep " << s_rep_name << " exists!");
        }
        else
        {
            ROS_DEBUG_STREAM("REPMANAGER: Rep " << s_rep_name << " does not exist!");
        }
    }
    return (b_success_call & b_success_msg);
}

void WAIOARepManager::GetRepStateViaService(std::string s_rep_name,
                                                std::string s_rep_detail_name,
                                                tf::Vector3* vc3_position,
                                                tf::Quaternion* qua_orientation,
                                                tf::Vector3* vc3_euler_angles,
                                                tf::Vector3* vc3_vel_linear,
                                                tf::Vector3* vc3_vel_angular)
{
    gazebo_msgs::GetModelState get_model_state;
    get_model_state.request.model_name=s_rep_name;
    get_model_state.request.relative_entity_name="world"; //Always given in world frame, s_rep_detail_name unused currently!
    if(ser_cli_gazebo_get_model_state.isValid())
    {
        ser_cli_gazebo_get_model_state.waitForExistence();
        ser_cli_gazebo_get_model_state.call(get_model_state);
    }

    vc3_position->setValue(get_model_state.response.pose.position.x,
                           get_model_state.response.pose.position.y,
                           get_model_state.response.pose.position.z);
    qua_orientation->setValue(get_model_state.response.pose.orientation.x,
                              get_model_state.response.pose.orientation.y,
                              get_model_state.response.pose.orientation.z,
                              get_model_state.response.pose.orientation.w);
    double d_yaw=0.0,d_pitch=0.0,d_roll=0.0;
    tf::Matrix3x3 mat_rot(*qua_orientation);
    mat_rot.getEulerYPR(d_yaw,d_pitch,d_roll);
    vc3_euler_angles->setValue(d_roll,d_pitch,d_yaw);

    vc3_vel_linear->setValue(get_model_state.response.twist.linear.x,
                             get_model_state.response.twist.linear.y,
                             get_model_state.response.twist.linear.z);
    vc3_vel_angular->setValue(get_model_state.response.twist.angular.x,
                             get_model_state.response.twist.angular.y,
                             get_model_state.response.twist.angular.z);
}

void WAIOARepManager::SetRepStateViaService(std::string s_rep_name,
                                            tf::Vector3 vc3_rep_position,
                                            tf::Vector3 vc3_rep_orientation,
                                            tf::Vector3 vc3_rep_vel_lin,
                                            tf::Vector3 vc3_rep_vel_ang)
{
    tf::Quaternion qua_rep_orientation;
    qua_rep_orientation.setRPY(vc3_rep_orientation.getX(),
                           vc3_rep_orientation.getY(),
                           vc3_rep_orientation.getZ());
    qua_rep_orientation=qua_rep_orientation.normalize();

    gazebo_msgs::SetModelState set_model_state;
    gazebo_msgs::ModelState model_state;
    model_state.model_name = s_rep_name;
    geometry_msgs::Pose pos_setup_state;
    pos_setup_state.position.x=vc3_rep_position.getX();
    pos_setup_state.position.y=vc3_rep_position.getY();
    pos_setup_state.position.z=vc3_rep_position.getZ();
    pos_setup_state.orientation.w=qua_rep_orientation.getW();
    pos_setup_state.orientation.x=qua_rep_orientation.getX();
    pos_setup_state.orientation.y=qua_rep_orientation.getY();
    pos_setup_state.orientation.z=qua_rep_orientation.getZ();
    model_state.pose=pos_setup_state;
    geometry_msgs::Twist twi_setup_state;
    twi_setup_state.linear.x=vc3_rep_vel_lin.getX();
    twi_setup_state.linear.y=vc3_rep_vel_lin.getY();
    twi_setup_state.linear.z=vc3_rep_vel_lin.getZ();
    twi_setup_state.angular.x=vc3_rep_vel_ang.getX();
    twi_setup_state.angular.y=vc3_rep_vel_ang.getY();
    twi_setup_state.angular.z=vc3_rep_vel_ang.getZ();
    model_state.twist=twi_setup_state;
    set_model_state.request.model_state=model_state;
    if(ser_cli_gazebo_set_model_state.isValid())
    {
        ser_cli_gazebo_set_model_state.waitForExistence();
        ser_cli_gazebo_set_model_state.call(set_model_state);
    }
}
void WAIOARepManager::SetRepStateViaServiceQuaternion(std::string s_rep_name,
                                            tf::Vector3 vc3_rep_position,
                                            tf::Quaternion qua_rep_orientation,
                                            tf::Vector3 vc3_rep_vel_lin,
                                            tf::Vector3 vc3_rep_vel_ang)
{
    gazebo_msgs::SetModelState set_model_state;
    gazebo_msgs::ModelState model_state;
    model_state.model_name = s_rep_name;
    geometry_msgs::Pose pos_setup_state;
    pos_setup_state.position.x=vc3_rep_position.getX();
    pos_setup_state.position.y=vc3_rep_position.getY();
    pos_setup_state.position.z=vc3_rep_position.getZ();
    pos_setup_state.orientation.w=qua_rep_orientation.getW();
    pos_setup_state.orientation.x=qua_rep_orientation.getX();
    pos_setup_state.orientation.y=qua_rep_orientation.getY();
    pos_setup_state.orientation.z=qua_rep_orientation.getZ();
    model_state.pose=pos_setup_state;
    geometry_msgs::Twist twi_setup_state;
    twi_setup_state.linear.x=vc3_rep_vel_lin.getX();
    twi_setup_state.linear.y=vc3_rep_vel_lin.getY();
    twi_setup_state.linear.z=vc3_rep_vel_lin.getZ();
    twi_setup_state.angular.x=vc3_rep_vel_ang.getX();
    twi_setup_state.angular.y=vc3_rep_vel_ang.getY();
    twi_setup_state.angular.z=vc3_rep_vel_ang.getZ();
    model_state.twist=twi_setup_state;
    set_model_state.request.model_state=model_state;
    if(ser_cli_gazebo_set_model_state.isValid())
    {
        ser_cli_gazebo_set_model_state.waitForExistence();
        ser_cli_gazebo_set_model_state.call(set_model_state);
    }
}

void WAIOARepManager::SetRepForceViaService(std::string s_rep_name,
                                            std::string s_link_name,
                                            tf::Vector3 vc3_rep_force,
                                            tf::Vector3 vc3_rep_torque,
                                            double d_rep_force_duration)
{
    gazebo_msgs::ApplyBodyWrench gaz_app_bod_wre;
    gaz_app_bod_wre.request.body_name=s_rep_name+"::"+s_link_name; // =s_rep_name+"::link_base";
    gaz_app_bod_wre.request.reference_frame="world";
    geometry_msgs::Point pnt_model;
    pnt_model.x=0.0;
    pnt_model.y=0.0;
    pnt_model.z=0.0;
    gaz_app_bod_wre.request.reference_point=pnt_model;
    geometry_msgs::Wrench wre_model;
    wre_model.force.x=vc3_rep_force.getX();
    wre_model.force.y=vc3_rep_force.getY();
    wre_model.force.z=vc3_rep_force.getZ();
    wre_model.torque.x=vc3_rep_torque.getX();
    wre_model.torque.y=vc3_rep_torque.getY();
    wre_model.torque.z=vc3_rep_torque.getZ();
    gaz_app_bod_wre.request.wrench=wre_model;
    gaz_app_bod_wre.request.start_time=ros::Time(0,0);
    if(d_rep_force_duration==-1.0) gaz_app_bod_wre.request.duration=ros::Duration(-1,0);
    else gaz_app_bod_wre.request.duration=ros::Duration(d_rep_force_duration);
    if(ser_cli_gazebo_apply_body_wrench.isValid())
    {
        ser_cli_gazebo_apply_body_wrench.waitForExistence();
        ser_cli_gazebo_apply_body_wrench.call(gaz_app_bod_wre);
    }
}

void WAIOARepManager::EnableEvalMetaphors()
{
    m_tmr_rep_manager_eval_metaphor.start();
}
void WAIOARepManager::DisableEvalMetaphors()
{
    m_tmr_rep_manager_eval_metaphor.stop();
    mrk_eval_metaphor.points.clear();
    mrk_eval_metaphor.colors.clear();
    pub_mrk_eval_metaphor.publish(mrk_eval_metaphor);
}

void WAIOARepManager::EnableRepAndDetail()
{
    mrk_oa_detail_label->UpdateColor(col_cyan_opaque);
    m_tmr_rep_manager.start();
}
void WAIOARepManager::DisableRepAndDetail()
{
    m_tmr_rep_manager.stop();

    m_f_rep_detail_yaw=0.0;
    m_f_rep_detail_pitch=0.0;
    m_f_rep_detail_roll=0.0;
    mrk_oa_detail_label->UpdateColor(col_invisible);
    pub_mrk_oa_detail_label.publish(mrk_oa_detail_label->GetMarker());

    ((WAIRepOOI*)wai_oa_rep_detail)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(0.0,0.0,0.0),
                "",
                col_oa_trans,
                CleanupStringDetail(m_s_rep_detail_selected),
                m_s_path_reps+m_s_rep_selected+"/"+m_s_rep_detail_selected+".dae",
                false,
                0.001,
                0.001);
    wai_oa_rep_detail->UpdateView();

    ros::spinOnce();
}

void WAIOARepManager::ToggleRepDetailRotation()
{
    m_b_rep_detail_rot=!m_b_rep_detail_rot;
}



///////////////////////////////////////////////////
/// Callback for LinkStates messages from Gazebo
///////////////////////////////////////////////////
void WAIOARepManager::cb_sub_lns_gazebo(const gazebo_msgs::LinkStatesConstPtr& msg)
{
    msg_lns_gazebo_sub=(*msg);
    msg_lns_gazebo=msg_lns_gazebo_sub;
}



///////////////////////////////////////////////////
/// Callback for ModelStates messages from Gazebo
///////////////////////////////////////////////////
void WAIOARepManager::cb_sub_mds_gazebo(gazebo_msgs::ModelStates msg)
{
    msg_mds_gazebo_sub=msg;
}



///////////////////////////////////////////////////
/// Callback for Transforms
///////////////////////////////////////////////////
void WAIOARepManager::cb_sub_tf2_transforms(const tf2_msgs::TFMessageConstPtr& msg)
{
    msg_tf2_transforms_sub=(*msg);

    // Periodically update all links during startup
    msg_tf2_transforms=msg_tf2_transforms_sub;
    for(int i=0;i<msg_tf2_transforms.transforms.size();i++)
    {
        if(std::find(m_vec_s_reps_details.begin(),m_vec_s_reps_details.end(),
                    msg_tf2_transforms.transforms[i].child_frame_id) == m_vec_s_reps_details.end() )
        {
            std::size_t pos = msg_tf2_transforms.transforms[i].child_frame_id.find("/");
            if(pos!=std::string::npos
                && pos!=msg_tf2_transforms.transforms[i].child_frame_id.length()-1
                && pos!=0)
            {
                //ROS_WARN_STREAM("Added: " << msg_tf2_transforms.transforms[i].child_frame_id);
                m_vec_s_reps_details.push_back(msg_tf2_transforms.transforms[i].child_frame_id);
            }
        }
    }
}



std::string WAIOARepManager::CleanupStringRep(std::string s_string)
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



std::string WAIOARepManager::CleanupStringDetail(std::string s_string)
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
        s_string_cleanedup="NONE";
    }
    return s_string_cleanedup;
}

gazebo_msgs::LinkStates* WAIOARepManager::GetRepLinkStates()
{
    return &msg_lns_gazebo;
}
/*
std::vector<ros::Publisher>* WAIOARepManager::GetRepStatePublishers()
{
    return &vec_pub_pos_state;
}*/


std::string WAIOARepManager::GetRepSelectedClean()
{
    return CleanupStringRep(m_s_rep_selected);
}
std::string WAIOARepManager::GetRepDetailSelectedClean()
{
    return CleanupStringDetail(m_s_rep_detail_selected);
}

std::string WAIOARepManager::GetRepSelected()
{
    return m_s_rep_selected;
}
std::string WAIOARepManager::GetRepDetailSelected()
{
    return m_s_rep_detail_selected;
}
std::string WAIOARepManager::GetRepDetailsFormatted(std::string s_detail,int i_char_count)
{
    // Insert line break every Nth characters:
    std::stringstream sst_det_desc_formatted;
    for(int i = 0; i < s_detail.size(); i++)
    {
        if(i%i_char_count==0 && i!=0 && i!=s_detail.size()-1 && i!=s_detail.size()-2) // Skip first and last characters
        {
            if(s_detail[i]==' ') sst_det_desc_formatted << " \n";
            else if(s_detail[i]=='.') sst_det_desc_formatted << ".\n";
            else sst_det_desc_formatted << s_detail[i] << "-\n";
        }
        else sst_det_desc_formatted << s_detail[i];
    }
    return sst_det_desc_formatted.str();
}






