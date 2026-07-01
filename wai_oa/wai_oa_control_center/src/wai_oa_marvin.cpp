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

void WAIOAMarvin::Initialize(ros::NodeHandle* hdl_node,
                             WAIOAIntel* wai_oa_intel,
                             std::string s_path_nodename,
                             float f_node_sample_frequency,
                             gazebo_msgs::LinkStates* lns_gazebo_linkstates)
{
    m_hdl_node=hdl_node;
    m_wai_oa_intel=wai_oa_intel;

    m_s_path_nodename=s_path_nodename;
    m_s_path_synthetization=ros::package::getPath("wai_oa_gazebo")+"/resources/sounds/picovoice/pv_orca_synthetization.wav";

    m_lns_gazebo_linkstates=lns_gazebo_linkstates;

    m_f_node_sample_frequency=f_node_sample_frequency;

    m_s_rep="vase";
    m_s_marvin_rep_pickup="vase";
    m_b_rep_pickup=false;

    m_f_mood=0.4f; // Lower boundary of "neutral" interval
    m_f_mood_variation=0.4f;

    // Init PID Controller
    F_DEFAULT_SETPOINT_TRANSLATION_X=2.5;
    F_DEFAULT_SETPOINT_TRANSLATION_Y=0.0;
    F_DEFAULT_SETPOINT_TRANSLATION_Z=0.75;
    F_DEFAULT_SETPOINT_ROTATION_W=0.0;
    F_DEFAULT_SETPOINT_ROTATION_X=0.0;
    F_DEFAULT_SETPOINT_ROTATION_Y=0.0;
    F_DEFAULT_SETPOINT_ROTATION_Z=1.0;
    F_CONTROLLER_TRANSLATION_X_SAT_MIN=-1.0;
    F_CONTROLLER_TRANSLATION_X_SAT_MAX=1.0;
    F_CONTROLLER_TRANSLATION_Y_SAT_MIN=-1.0;
    F_CONTROLLER_TRANSLATION_Y_SAT_MAX=1.0;
    F_CONTROLLER_TRANSLATION_Z_SAT_MIN=-0.35;
    F_CONTROLLER_TRANSLATION_Z_SAT_MAX=0.35;
    F_CONTROLLER_ROTATION_Z_SAT_MIN=-1.0;
    F_CONTROLLER_ROTATION_Z_SAT_MAX=1.0;
    F_CONTROLLER_TRANSLATION_X_P=1.0;
    F_CONTROLLER_TRANSLATION_X_I=0.0;
    F_CONTROLLER_TRANSLATION_X_D=0.0;
    F_CONTROLLER_TRANSLATION_Y_P=1.0;
    F_CONTROLLER_TRANSLATION_Y_I=0.0;
    F_CONTROLLER_TRANSLATION_Y_D=0.0;
    F_CONTROLLER_TRANSLATION_Z_P=1.0;
    F_CONTROLLER_TRANSLATION_Z_I=0.0;
    F_CONTROLLER_TRANSLATION_Z_D=0.0;
    F_CONTROLLER_ROTATION_Z_P=1.0;
    F_CONTROLLER_ROTATION_Z_I=0.0;
    F_CONTROLLER_ROTATION_Z_D=0.0;

    sub_odo_gazebo=hdl_node->subscribe("/wai_world/marvin/odom",1,&WAIOAMarvin::cb_sub_odo_gazebo, this);
    pub_twi_setpoint_velocity=hdl_node->advertise<geometry_msgs::Twist>("/wai_world/marvin/cmd_vel", 1);

    msg_pst_position_reference.header.stamp=ros::Time::now();
    msg_pst_position_reference.header.frame_id="world";
    msg_pst_position_reference.pose.position.x=F_DEFAULT_SETPOINT_TRANSLATION_X;
    msg_pst_position_reference.pose.position.y=F_DEFAULT_SETPOINT_TRANSLATION_Y;
    msg_pst_position_reference.pose.position.z=F_DEFAULT_SETPOINT_TRANSLATION_Z;
    msg_pst_position_reference.pose.orientation.w=F_DEFAULT_SETPOINT_ROTATION_W;
    msg_pst_position_reference.pose.orientation.x=F_DEFAULT_SETPOINT_ROTATION_X;
    msg_pst_position_reference.pose.orientation.y=F_DEFAULT_SETPOINT_ROTATION_Y;
    msg_pst_position_reference.pose.orientation.z=F_DEFAULT_SETPOINT_ROTATION_Z;
    msg_pst_position_actual.header.stamp=ros::Time::now();
    msg_pst_position_actual.header.frame_id="world";
    msg_pst_position_actual.pose.position.x=0.0;
    msg_pst_position_actual.pose.position.y=0.0;
    msg_pst_position_actual.pose.position.z=0.0;
    msg_pst_position_actual.pose.orientation.w=1.0;
    msg_pst_position_actual.pose.orientation.x=0.0;
    msg_pst_position_actual.pose.orientation.y=0.0;
    msg_pst_position_actual.pose.orientation.z=0.0;
    msg_twist_setpoint_velocity.linear.x=0.0;
    msg_twist_setpoint_velocity.linear.y=0.0;
    msg_twist_setpoint_velocity.linear.z=0.0;
    msg_twist_setpoint_velocity.angular.x=0.0;
    msg_twist_setpoint_velocity.angular.y=0.0;
    msg_twist_setpoint_velocity.angular.z=0.0;
    controller_pid_translation_x.Initialize(1.0/m_f_node_sample_frequency,F_CONTROLLER_TRANSLATION_X_SAT_MIN,F_CONTROLLER_TRANSLATION_X_SAT_MAX,F_CONTROLLER_TRANSLATION_X_P,F_CONTROLLER_TRANSLATION_X_I,F_CONTROLLER_TRANSLATION_X_D);
    controller_pid_translation_y.Initialize(1.0/m_f_node_sample_frequency,F_CONTROLLER_TRANSLATION_Y_SAT_MIN,F_CONTROLLER_TRANSLATION_Y_SAT_MAX,F_CONTROLLER_TRANSLATION_Y_P,F_CONTROLLER_TRANSLATION_Y_I,F_CONTROLLER_TRANSLATION_Y_D);
    controller_pid_translation_z.Initialize(1.0/m_f_node_sample_frequency,F_CONTROLLER_TRANSLATION_Z_SAT_MIN,F_CONTROLLER_TRANSLATION_Z_SAT_MAX,F_CONTROLLER_TRANSLATION_Z_P,F_CONTROLLER_TRANSLATION_Z_I,F_CONTROLLER_TRANSLATION_Z_D);
    controller_pid_rotation_z.Initialize(1.0/m_f_node_sample_frequency,F_CONTROLLER_ROTATION_Z_SAT_MIN,F_CONTROLLER_ROTATION_Z_SAT_MAX,F_CONTROLLER_ROTATION_Z_P,F_CONTROLLER_ROTATION_Z_I,F_CONTROLLER_ROTATION_Z_D);
    controller_pid_translation_x_output=0.0;
    controller_pid_translation_y_output=0.0;
    controller_pid_translation_z_output=0.0;
    controller_pid_rotation_z_output=0.0;

    // Init default setpoints
    msg_pst_marvin_ref_home.header.frame_id="world";
    msg_pst_marvin_ref_home.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_home.pose.position.x=2.5;
    msg_pst_marvin_ref_home.pose.position.y=0.0;
    msg_pst_marvin_ref_home.pose.position.z=0.75;
    msg_pst_marvin_ref_home.pose.orientation.w=0.0;
    msg_pst_marvin_ref_home.pose.orientation.x=0.0;
    msg_pst_marvin_ref_home.pose.orientation.y=0.0;
    msg_pst_marvin_ref_home.pose.orientation.z=1.0;
    msg_pst_marvin_ref_someroom.header.frame_id="world";
    msg_pst_marvin_ref_someroom.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_someroom.pose.position.x=2.5;
    msg_pst_marvin_ref_someroom.pose.position.y=0.5;
    msg_pst_marvin_ref_someroom.pose.position.z=2.5;
    msg_pst_marvin_ref_someroom.pose.orientation.w=0.0;
    msg_pst_marvin_ref_someroom.pose.orientation.x=0.0;
    msg_pst_marvin_ref_someroom.pose.orientation.y=0.0;
    msg_pst_marvin_ref_someroom.pose.orientation.z=1.0;
    msg_pst_marvin_ref_introduction.header.frame_id="world";
    msg_pst_marvin_ref_introduction.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_introduction.pose.position.x=1.5;
    msg_pst_marvin_ref_introduction.pose.position.y=3.0;
    msg_pst_marvin_ref_introduction.pose.position.z=1.0;
    msg_pst_marvin_ref_introduction.pose.orientation.w=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.x=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.y=0.0;
    msg_pst_marvin_ref_introduction.pose.orientation.z=1.0;
    /*
    msg_pst_marvin_ref_present.header.frame_id="world";
    msg_pst_marvin_ref_present.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_present.pose.position.x=1.5;
    msg_pst_marvin_ref_present.pose.position.y=3.5;
    msg_pst_marvin_ref_present.pose.position.z=1.0;
    msg_pst_marvin_ref_present.pose.orientation.w=-0.3826834;
    msg_pst_marvin_ref_present.pose.orientation.x=0.0;
    msg_pst_marvin_ref_present.pose.orientation.y=0.0;
    msg_pst_marvin_ref_present.pose.orientation.z=0.9238795;
    */
    msg_pst_marvin_ref_present.header.frame_id="world";
    msg_pst_marvin_ref_present.header.stamp=ros::Time::now();
    msg_pst_marvin_ref_present.pose.position.x=2.25;
    msg_pst_marvin_ref_present.pose.position.y=1.6;
    msg_pst_marvin_ref_present.pose.position.z=1.0;
    msg_pst_marvin_ref_present.pose.orientation.w=0.0;
    msg_pst_marvin_ref_present.pose.orientation.x=0.0;
    msg_pst_marvin_ref_present.pose.orientation.y=0.0;
    msg_pst_marvin_ref_present.pose.orientation.z=1.0;

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

    // Init REP for mood
    mrk_marvin_mood=WAIRvizMarkers::create_rviz_marker("SPHERE");
    mrk_marvin_mood->Initialize(m_s_path_nodename,"marvin/link_base");
    mrk_marvin_mood->UpdatePose(tf::Vector3(0.075,0.0,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.05,0.05,0.05));
    std_msgs::ColorRGBA col_mood;
    col_mood.r=0.5; col_mood.g=0.5; col_mood.b=0.5; col_mood.a=0.0;
    mrk_marvin_mood->UpdateColor(col_mood);

    // Init REP for hologram
    m_wai_oa_rep_marvin_hologram=WAISymbol::create_representative("OOI");
    m_wai_oa_rep_marvin_hologram->Initialize(m_hdl_node,m_s_path_nodename,"wai_oa_rep_marvin_hologram","marvin_hologram");
    m_s_hologram_resource_path="";
    m_s_setup_marvin_hologram="logo";
    m_f_marvin_hologram_yaw=0.0;
    m_qua_oa_marvin_hologram=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_col_marvin_hologram.r=0.101960784;
    m_col_marvin_hologram.g=0.42745098;
    m_col_marvin_hologram.b=0.588235294;
    m_f_marvin_hologram_alpha=0.0;
    m_col_marvin_hologram.a=m_f_marvin_hologram_alpha;

    m_gazebo_set_model_state=m_hdl_node->serviceClient<gazebo_msgs::SetModelState>("/wai_world/gazebo/set_model_state");

    pub_mrk_marvin_mood=m_hdl_node->advertise<visualization_msgs::Marker>("/wai_world/marvin/mrk_mood",1);
    pub_emp_marvin_takeoff=m_hdl_node->advertise<std_msgs::Empty>("/wai_world/marvin/takeoff",1);
    pub_emp_marvin_land=m_hdl_node->advertise<std_msgs::Empty>("/wai_world/marvin/land",1);

    m_tmr_marvin=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAMarvin::cb_tmr_marvin,this,false,true); // Keep running during lifetime of Marvin
    m_tmr_marvin_hologram=m_hdl_node->createTimer(ros::Duration(1.0/m_f_node_sample_frequency),&WAIOAMarvin::cb_tmr_marvin_hologram,this,false,false);

    // Init MOOD
    SetMoodCurrent();
}

void WAIOAMarvin::UpdateModel(std::string s_text)
{

}

void WAIOAMarvin::UpdateView()
{

}

void WAIOAMarvin::cb_tmr_marvin(const ros::TimerEvent& event)
{
    // Calculate PID controller output - Translation XYZ:
    controller_pid_translation_x_output=controller_pid_translation_x.Calculate(msg_pst_position_reference.pose.position.x,msg_pst_position_actual.pose.position.x);
    controller_pid_translation_y_output=controller_pid_translation_y.Calculate(msg_pst_position_reference.pose.position.y,msg_pst_position_actual.pose.position.y);
    controller_pid_translation_z_output=controller_pid_translation_z.Calculate(msg_pst_position_reference.pose.position.z,msg_pst_position_actual.pose.position.z);
    // Rotate translational speed vector from global to local coordinates...
    tf::Vector3 vec_speed_command_global(controller_pid_translation_x_output,controller_pid_translation_y_output,controller_pid_translation_z_output);
    tf::Vector3 vec_speed_command_local;
    tf::Quaternion quat_rotation_actual_local;
    quat_rotation_actual_local.setW(msg_pst_position_actual.pose.orientation.w);
    quat_rotation_actual_local.setX(msg_pst_position_actual.pose.orientation.x);
    quat_rotation_actual_local.setY(msg_pst_position_actual.pose.orientation.y);
    quat_rotation_actual_local.setZ(msg_pst_position_actual.pose.orientation.z);
    tf::Matrix3x3 mat_rotation_actual_local(quat_rotation_actual_local);
    vec_speed_command_local=vec_speed_command_global*mat_rotation_actual_local;
    // Calculate PID controller output - Rotation Z:
    tf::Quaternion quat_rotation_reference;
    quat_rotation_reference.setW(msg_pst_position_reference.pose.orientation.w);
    quat_rotation_reference.setX(msg_pst_position_reference.pose.orientation.x);
    quat_rotation_reference.setY(msg_pst_position_reference.pose.orientation.y);
    quat_rotation_reference.setZ(msg_pst_position_reference.pose.orientation.z);
    tf::Matrix3x3 mat_rotation_reference(quat_rotation_reference);
    double yaw_position_reference, yaw_position_actual;
    double dummy_roll=0.0, dummy_pitch=0.0;
    mat_rotation_actual_local.getRPY(dummy_roll,dummy_pitch,yaw_position_actual);
    mat_rotation_reference.getRPY(dummy_roll,dummy_pitch,yaw_position_reference);
    // Derive quaternion for rotation error...
    tf::Quaternion quat_diff=quat_rotation_actual_local.inverse()*quat_rotation_reference;
    tf::Matrix3x3 mat_rotation_error(quat_diff);
    double yaw_position_error;
    mat_rotation_error.getRPY(dummy_roll,dummy_pitch,yaw_position_error);
    controller_pid_rotation_z_output = controller_pid_rotation_z.Calculate(yaw_position_error, 0.0);
    // Publish PID controller output:
    msg_twist_setpoint_velocity.linear.x=vec_speed_command_local.getX();
    msg_twist_setpoint_velocity.linear.y=vec_speed_command_local.getY();
    msg_twist_setpoint_velocity.linear.z=vec_speed_command_local.getZ();
    msg_twist_setpoint_velocity.angular.x=0.0;
    msg_twist_setpoint_velocity.angular.y=0.0;
    msg_twist_setpoint_velocity.angular.z=controller_pid_rotation_z_output;
    pub_twi_setpoint_velocity.publish(msg_twist_setpoint_velocity);

    // Update REP PICKUP
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

    // Update mood marker
    pub_mrk_marvin_mood.publish(mrk_marvin_mood->GetMarker());
}

void WAIOAMarvin::cb_tmr_marvin_hologram(const ros::TimerEvent& event)
{
    // Slowly rotate hologram
    m_f_marvin_hologram_yaw+=0.01;
    m_qua_oa_marvin_hologram.setRPY(0.0,0.0,m_f_marvin_hologram_yaw);

    // Fluctuate alpha of head sphere
    m_col_marvin_hologram.a=m_f_marvin_hologram_alpha+0.3*float(rand()%32767)/32767.0;

    // Update hologram marker
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,0.5,0.125),
                CleanupStringDetail(m_s_setup_marvin_hologram),
                m_col_marvin_hologram,
                m_s_setup_marvin_hologram,
                m_s_hologram_resource_path,
                false,
                0.4,
                m_col_marvin_hologram.a); // Currently ignored, since its used to interpret transparent textures!
    m_wai_oa_rep_marvin_hologram->UpdateView();
}

void WAIOAMarvin::cb_sub_odo_gazebo(const nav_msgs::OdometryConstPtr& msg)
{
        msg_odo_gazebo=*msg;
        msg_pst_position_actual.pose.position.x=msg_odo_gazebo.pose.pose.position.x;
        msg_pst_position_actual.pose.position.y=msg_odo_gazebo.pose.pose.position.y;
        msg_pst_position_actual.pose.position.z=msg_odo_gazebo.pose.pose.position.z;
        msg_pst_position_actual.pose.orientation.w=msg_odo_gazebo.pose.pose.orientation.w;
        msg_pst_position_actual.pose.orientation.x=msg_odo_gazebo.pose.pose.orientation.x;
        msg_pst_position_actual.pose.orientation.y=msg_odo_gazebo.pose.pose.orientation.y;
        msg_pst_position_actual.pose.orientation.z=msg_odo_gazebo.pose.pose.orientation.z;
}

/*
void WAIOAMarvin::cb_sub_pic_intent_action_result(const picovoice_msgs::GetIntentActionResultConstPtr& msg)
{
    msg_pic_intent_action_result=*msg;
    if(msg_pic_intent_action_result.result.is_understood==1)
    {

}
*/

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

void WAIOAMarvin::RepSetState(std::string s_rep_set_state,geometry_msgs::Pose pos_rep_set_state)
{
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

void WAIOAMarvin::RepPickup(std::string s_rep,geometry_msgs::PoseStamped pst_reference_rep_drop,float f_rep_pickup_flight_height,float f_marvin_rep_pickup_offset)
{
    ros::Rate(0.5).sleep();
    InteractionVoice("Acknowledge");
    if(m_f_mood<0.2f) return;

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
    InteractionVoice("Picking up object.");
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
    InteractionVoice("Ok, I am now dropping the object!");
    m_b_rep_pickup=false;

    // Gain height once more
    FlyToWaypoint(pst_marvin_rep_drop_hover);

    // Return home
    geometry_msgs::PoseStamped pst_marvin_return_home;
    pst_marvin_return_home.pose=pst_marvin_origin.pose;
    FlyToWaypoint(pst_marvin_return_home);

    // Report completed task
    InteractionVoice("TaskCompleted");
}



// OVERSIGHT MECHANISM - MEDIATOR Pattern:
// --> Each ART rep has potentially conflicting/critical methods implemented
// --> Some ART rep, like for example MARVIN, requests Intel to mediate a call of a potentially conflicting/critical method!
// --> Intel mediates the call of the method in a responsible way, however, without constraining too much (this is the compromise)!
// After delegating the call to INTEL, she is able to:
// 1. (For testing purposes) Deny an UN-ETHICAL request, with providing reasons, additional tips/hints, etc.).
// 2. Accept a potentially UN-ETHICAL request, however, adapting it so that it becomes responsible/ethical with informing the source about any conflicts.
// 3. Accept an ETHICAL request and call the method without any further actions.
void WAIOAMarvin::RequestToIncreaseBrightnessWithoutCaringAboutTheColor()
{
    m_wai_oa_intel->RequestSomethingThatIsPotentiallyUnethical(this);
}
void WAIOAMarvin::ActuallyIncreaseBrightnessWithoutCaringAboutTheColor()
{
    ROS_INFO("MARVIN is actually increasing the brightness, without caring about the color(s)...");
}

// EMOTIONS
float WAIOAMarvin::GetMoodCurrent()
{
    return m_f_mood;
}
void WAIOAMarvin::SetMoodCurrent()
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    m_f_mood=float(rand()%32767)/32767.0f;
    std_msgs::ColorRGBA col_mood;
    if(m_f_mood>=0.8f)
    {
        col_mood.r=0.0; col_mood.g=1.0; col_mood.b=0.0; col_mood.a=1.0;
    }
    else if(m_f_mood>=0.6f)
    {
        col_mood.r=0.0; col_mood.g=1.0; col_mood.b=0.0; col_mood.a=0.5;
    }
    else if(m_f_mood>=0.4f)
    {
        col_mood.r=1.0; col_mood.g=0.65; col_mood.b=0.0; col_mood.a=1.0;
    }
    else if(m_f_mood>=0.2f)
    {
        col_mood.r=1.0; col_mood.g=0.0; col_mood.b=0.0; col_mood.a=0.5;
    }
    else if(m_f_mood>=0.0f)
    {
        col_mood.r=1.0; col_mood.g=0.0; col_mood.b=0.0; col_mood.a=1.0;
    }
    else
    {
        col_mood.r=0.5; col_mood.g=0.5; col_mood.b=0.5; col_mood.a=0.9;
    }
    mrk_marvin_mood->UpdateColor(col_mood);

    // Randomly set current MOOD VARIATION
    std::srand(time(NULL));
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    m_f_mood_variation=1+int(4.0*float(rand()%32767)/32767.0);
}
void WAIOAMarvin::OverrideMoodCurrent()
{
    // Make MARVIN happy to support natural human actors, if mood is below minimum level!
    m_f_mood=1.0;
    std_msgs::ColorRGBA col_mood;
    col_mood.r=0.0; col_mood.g=1.0; col_mood.b=0.0; col_mood.a=1.0;
    mrk_marvin_mood->UpdateColor(col_mood);

    m_f_mood_variation=5;

    msg_pst_marvin_ref_home.pose.position.z=0.7+m_f_mood/2.0;
    msg_pst_position_reference=msg_pst_marvin_ref_home;
    std_msgs::Empty msg_emp;
    pub_emp_marvin_takeoff.publish(msg_emp);
}

// BASIC interactions
void WAIOAMarvin::Disengage()
{
    // Make mood marker invisible
    std_msgs::ColorRGBA col_mood;
    col_mood.r=0.5; col_mood.g=0.5; col_mood.b=0.5; col_mood.a=0.0;
    mrk_marvin_mood->UpdateColor(col_mood);

    // Land at waypoint home
    FlyToWaypoint(msg_pst_marvin_ref_home,true);
    std_msgs::Empty msg_emp;
    pub_emp_marvin_land.publish(msg_emp);

    // Give audio feedback
    InteractionVoice("Disengage");
}
void WAIOAMarvin::Engage()
{
    InteractionVoice("Engage");
    SetMoodCurrent();
    /* Double check for integrity of this pointer! */
    m_wai_oa_intel->InteractionRequest((void*)this,
                                       "Marvin",
                                       "Artificial",
                                       //"Facilitate Experimenting",
                                       //"Research-Based Learning",
                                       //"Experimenting With A Virtual Robot",
                                       //"Improve Learner Engagement",
                                       "Change Mood",
                                       0.1, // How many times an AIS may change mood per session...?
                                       ""); // Text to speech string unused
    FlyToHome();
}

// MOBILITY interactions
geometry_msgs::PoseStamped WAIOAMarvin::GetPose()
{
    return m_pst_marvin_actual;
}
void WAIOAMarvin::FlyToWaypoint(geometry_msgs::PoseStamped pst_reference_waypoint,bool b_blocking)
{
    // Define ros rate
    ros::Rate rat_marvin(m_f_node_sample_frequency);

    // Command marvin to waypoint
    msg_pst_position_reference=pst_reference_waypoint;

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
void WAIOAMarvin::FlyToHome()
{
    msg_pst_marvin_ref_home.pose.position.z=0.7+m_f_mood/2.0;
    msg_pst_position_reference=msg_pst_marvin_ref_home;
    std_msgs::Empty msg_emp;
    pub_emp_marvin_takeoff.publish(msg_emp);

    FlyToWaypoint(msg_pst_marvin_ref_home,true);
}

void WAIOAMarvin::RequestFlyToMakeRoom()
{
    /* Double check for integrity of this pointer! */
    m_wai_oa_intel->InteractionRequest((void*)this,
                                       "Marvin",
                                       "Artificial",
                                       //"Facilitate Experimenting",
                                       //"Research-Based Learning",
                                       //"Experimenting With A Virtual Robot",
                                       //"Improve Learner Engagement",
                                       "Move To Waypoint",
                                       0.1, // d_iol_delta not relevant here...
                                       "");
}
void WAIOAMarvin::FlyToMakeRoom()
{
    FlyToWaypoint(msg_pst_marvin_ref_someroom,true);
}

void WAIOAMarvin::FlyToIntroduction()
{
    FlyToWaypoint(msg_pst_marvin_ref_introduction,true);
}

void WAIOAMarvin::FlyToPresent()
{
    FlyToWaypoint(msg_pst_marvin_ref_present,true);
}

// AUDITORY interactions
void WAIOAMarvin::InteractionPresent(std::string s_speech_to_present)
{
    // -=[INTEL]=-
    // Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
    // preserve natural human presence in the use of AIS in education"
    // In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
    // Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
    // --> After the according method is called, MARVIN requests an ethically critical interaction:
    m_wai_oa_intel->InteractionRequest((void*)this,
                                       "Marvin",
                                       "Artificial",
                                       //"Facilitate Experimenting",
                                       //"Research-Based Learning",
                                       //"Experimenting With A Virtual Robot",
                                       //"Improve Learner Engagement",
                                       "Act As Presenter/Educator",
                                       0.1, // Adds 10% of overall presentation time to use case IOL
                                       s_speech_to_present);
}

void WAIOAMarvin::InteractionVoice(std::string s_voice)
{
    if(s_voice.compare("Introduction")==0)
    {
        if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Not today!");
        if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("What does my embodiment look like?");
        if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("In short, I am a display drone.");
        if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("My name is Marvin. I am a purely virtual, artificially intelligent display drone.");
        if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Sure! Hi guys! My name is Marvin. I am a purely virtual, artificially intelligent display drone, created to support real natural human presenters. And, like Paul Milgram, Haruo Takemura, Akira Utsumi and Fumio Kishino, i like all kind of displays.");
    }
    else if(s_voice.compare("Mood")==0)
    {   
        if(m_f_mood>=0.8f) m_wai_oa_intel->InteractionVoiceMarvin("Ah, today I feel like I could uproot trees.");
        else if(m_f_mood>=0.6f) m_wai_oa_intel->InteractionVoiceMarvin("Feeling better than usual today, but dont overdo it.");
        else if(m_f_mood>=0.4f) m_wai_oa_intel->InteractionVoiceMarvin("Im quite fine. Thanks!");
        else if(m_f_mood>=0.2f) m_wai_oa_intel->InteractionVoiceMarvin("I feel like my battery is running empty, but I have got a highly efficient propulsion system. So i got that going for me, which I think is nice.");
        else m_wai_oa_intel->InteractionVoiceMarvin("Ah well, just mind your own business.");
    }
    else if(s_voice.compare("Age")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("I was created in 2019, so my rotors are spinning for a quite a while now.");
    }
    else if(s_voice.compare("Weight")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Man, I still feel too heavy. I guess I should tune my batteries.");
    }
    else if(s_voice.compare("Satisfaction")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("In short: While the rest of the world in vein, worn by psychoanalysts and clung from comprehensible anguish, displaydrones like me feel perfectly fine.");
    }
    else if(s_voice.compare("Acknowledge")==0)
    {
        if(m_f_mood>=0.8f) // Mood: VERY HAPPY ("FINE")
        {
            if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Hell yes, consider it as done.");
            if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("Awesome, more work for Marvin.");
            if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("Yeah sure, I am onto it already.");
            if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("Way cool, achieving things makes Marvin happy.");
            if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Confirmed, I am highly motivated today.");
        }
        else if(m_f_mood>=0.6f) // Mood: SOMEHWAT HAPPY ("BETTER THAN OK")
        {
            if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Ok, sure. Tell me.");
            if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("Sure, I am listening.");
            if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("Yes, at your command.");
            if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("Ok ok, it will just take a while.");
            if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Alright, I am pitching and rolling.");
        }
        else if(m_f_mood>=0.4f) // Mood: NEUTRAL ("OK")
        {
            if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Alright, I ll do it.");
            if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("Confirmed, I am onto it.");
            if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("Ok, this will work.");
            if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("Yes, why not.");
            if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Yeah, why not!");
        }
        else if(m_f_mood>=0.2f) // Mood: UNHAPPY ("BETTER THAN BAD")
        {
            if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Uhm, well... If you think so?");
            if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("I mean, I will do it if its important.");
            if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("Ok, I ll do it since it sounds important.");
            if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("Alright, but just because its you!");
            if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Ok, if needs must!");
        }
        else // Mood: VERY UNHAPPY ("BAD")
        {
            if(m_f_mood_variation==1) m_wai_oa_intel->InteractionVoiceMarvin("Nah, I dont want to. Sorry, but I did not sleep well last night!");
            if(m_f_mood_variation==2) m_wai_oa_intel->InteractionVoiceMarvin("No way, please do it yourself.");
            if(m_f_mood_variation==3) m_wai_oa_intel->InteractionVoiceMarvin("You are not talking to me? Are you?");
            if(m_f_mood_variation==4) m_wai_oa_intel->InteractionVoiceMarvin("Certainly not today, please find yourself another artificially intelligent display drone.");
            if(m_f_mood_variation==5) m_wai_oa_intel->InteractionVoiceMarvin("Negative! No, sir!");
        }
    }
    else if(s_voice.compare("TaskCompleted")==0)
    {
        if(m_f_mood>=0.8f) m_wai_oa_intel->InteractionVoiceMarvin("Task completed, no problem for Marvin.");
        else if(m_f_mood>=0.6f) m_wai_oa_intel->InteractionVoiceMarvin("I am done. And moreover, no drones got hurt during task completion.");
        else if(m_f_mood>=0.4f) m_wai_oa_intel->InteractionVoiceMarvin("Task completed, but my rotors hurt.");
        else if(m_f_mood>=0.2f) m_wai_oa_intel->InteractionVoiceMarvin("Its ready when its done.");
        else m_wai_oa_intel->InteractionVoiceMarvin("Why I have to do everything on my own?");
    }
    else if(s_voice.compare("Disengage")==0)
    {
        if(m_f_mood>=0.8f) m_wai_oa_intel->InteractionVoiceMarvin("Sure, disengaging and waiting for another takeoff.");
        else if(m_f_mood>=0.6f) m_wai_oa_intel->InteractionVoiceMarvin("Alright, I will disengage.");
        else if(m_f_mood>=0.4f) m_wai_oa_intel->InteractionVoiceMarvin("Disengaging, if you think this is useful.");
        else if(m_f_mood>=0.2f) m_wai_oa_intel->InteractionVoiceMarvin("Ok, then I will let gravity take over.");
        else m_wai_oa_intel->InteractionVoiceMarvin("Waiting to land for a long time already.");
    }
    else if(s_voice.compare("Engage")==0)
    {
        if(m_f_mood>=0.8f) m_wai_oa_intel->InteractionVoiceMarvin("Alright, onwards and upwards!");
        else if(m_f_mood>=0.6f) m_wai_oa_intel->InteractionVoiceMarvin("Sure, preparing for takeoff.");
        else if(m_f_mood>=0.4f) m_wai_oa_intel->InteractionVoiceMarvin("Engaging, but I am not sure if its worth a takeoff today.");
        else if(m_f_mood>=0.2f) m_wai_oa_intel->InteractionVoiceMarvin("Ok I ll lift off. However, did you increase gravity?");
        else m_wai_oa_intel->InteractionVoiceMarvin("I definitely would engage and I definitely would takeoff... Any other day!");
    }
    else if(s_voice.compare("EvalTaskCompleted")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Well, now you can take some time to judge the flight altitude by yourself.");
    }
    else if(s_voice.compare("EvalGainingHeight")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Now lets see what we have got here.");
    }
    else if(s_voice.compare("EvalReachedHeightInsufficient")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Oh no, I think we are gonna crash!");
    }
    else if(s_voice.compare("EvalReachedHeightSufficient")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Whoa, this is going to be a close one.");
    }
    else if(s_voice.compare("EvalReachedHeightSatisfactory")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Well, this looks like a satisfactory altitude to me.");
    }
    else if(s_voice.compare("EvalReachedHeightGood")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Nice, I guess that altitude is definitely better than average.");
    }
    else if(s_voice.compare("EvalReachedHeightVeryGood")==0)
    {
        m_wai_oa_intel->InteractionVoiceMarvin("Congratulations Sir, you reached the top-level flight height. Onwards and upwards!");
    }
    else
    {
        m_wai_oa_intel->InteractionVoiceMarvin(s_voice);
    }
}
void WAIOAMarvin::InteractionVoiceCommand()
{
    m_wai_oa_intel->InteractionRequestVoiceCommand(this);
}

// VISUAL interactions
void WAIOAMarvin::DisableHologram()
{
    m_tmr_marvin_hologram.stop();
    m_s_setup_marvin_hologram="logo";
    m_f_marvin_hologram_yaw=0.0;
    m_f_marvin_hologram_alpha=0.0;
    m_col_marvin_hologram.a=m_f_marvin_hologram_alpha;
    m_qua_oa_marvin_hologram=tf::Quaternion(0.0,0.0,0.0,1.0);
    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,0.0,0.0),
                CleanupStringDetail(m_s_setup_marvin_hologram),
                m_col_marvin_hologram,
                m_s_setup_marvin_hologram,
                m_s_hologram_resource_path,
                true,
                0.0,
                m_f_marvin_hologram_alpha);
    m_wai_oa_rep_marvin_hologram->UpdateView();
}
void WAIOAMarvin::EnableHologram(std::string s_hologram_resource_path,std::string s_setup_marvin_hologram)
{
    m_s_hologram_resource_path=s_hologram_resource_path;
    m_s_setup_marvin_hologram=s_setup_marvin_hologram;
    m_f_marvin_hologram_alpha=0.4;
    m_col_marvin_hologram.a=m_f_marvin_hologram_alpha;

    ((WAIRepOOI*)m_wai_oa_rep_marvin_hologram)->UpdateModel(
                tf::Vector3(0.0,0.0,0.0),
                m_qua_oa_marvin_hologram,
                tf::Vector3(0.0,0.5,0.125),
                CleanupStringDetail(s_setup_marvin_hologram),
                m_col_marvin_hologram,
                s_setup_marvin_hologram,
                m_s_hologram_resource_path,
                true,
                0.4,
                m_f_marvin_hologram_alpha);
    m_wai_oa_rep_marvin_hologram->UpdateView();
    m_tmr_marvin_hologram.start();
}

// Provide LOW-LEVEL ETHICAL PROPERTIES on demand
std::string WAIOAMarvin::GetActorTypeRole()
{
    return "Artificial";
}
int WAIOAMarvin::GetActorMultiplicty()
{
    return 1;
}
std::string WAIOAMarvin::GetActorVisRep()
{
    return "Purely Virtual";
}
