#include<wai_oa_rep_sequencer.h>



/////////////////////////////////////////////////
/// Implementation of WAIOARepSequencer
/////////////////////////////////////////////////

WAIOARepSequencer::WAIOARepSequencer()
{
}

WAIOARepSequencer::~WAIOARepSequencer()
{
}

void WAIOARepSequencer::cb_sub_clk_gazebo(const rosgraph_msgs::ClockPtr& msg)
{
    m_msg_clk_gazebo=*msg;
    //ROS_WARN("m_msg_clk_gazebo: %3.3f",m_msg_clk_gazebo.clock.toSec());

    if(m_b_rep_seq_enabled==true)
    {
        if(m_b_rep_seq_save_start_time==true)
        {
            m_tim_setup_rep_start=m_msg_clk_gazebo.clock;
            ResetSequencerTimestamp(true);
            m_b_rep_seq_save_start_time=false;
        }
    }
    else
    {
        return;
    }

    // Reset changed variable
    m_b_rep_seq_changed=false;

    // Include 5secs of dead time before start
    m_f_rep_seq_elapsed=roundf((m_msg_clk_gazebo.clock-m_tim_setup_rep_start).toSec()*100)/100; //-5.0;
    //ROS_WARN("elapsed: %3.3f",m_f_rep_seq_elapsed);

    // SET FORCE for N times [t0,Fx,Fy,Fz,Tx,Ty,Tz, t1,...]
    for(int i=0;i<m_vec_d_rep_seq_force.size()/7;i++)
    {
        if(m_f_rep_seq_elapsed>=m_vec_d_rep_seq_force[i*7]
           && m_vec_b_rep_seq_force[i*7]!=true)
        {
            // Get current state up to velocities before setting FORCE
            m_oa_rep_manager->GetRepStateViaService(m_s_rep_seq_name,"link_base",&m_vc3_rep_seq_position,&m_qua_rep_seq_orientation,&m_vc3_rep_seq_orientation,&m_vc3_rep_seq_vel_linear,&m_vc3_rep_seq_vel_angular);
            m_vc3_rep_seq_force.setValue(m_vec_d_rep_seq_force[i*7+1],m_vec_d_rep_seq_force[i*7+2],m_vec_d_rep_seq_force[i*7+3]);
            m_vc3_rep_seq_torque.setValue(m_vec_d_rep_seq_force[i*7+4],m_vec_d_rep_seq_force[i*7+5],m_vec_d_rep_seq_force[i*7+6]);
            m_oa_rep_manager->SetRepForceViaService(m_s_rep_seq_name,"link_base",m_vc3_rep_seq_force,m_vc3_rep_seq_torque);
            m_vec_b_rep_seq_force[i*7]=true;
            m_b_rep_seq_changed=true;
            //ROS_WARN("Force changed!");
        }
    }

    // SET TWIST for N times [t0,lin_x,lin_y,lin_z,ang_x,ang_y,ang_z, t1,...]
    for(int i=0;i<m_vec_d_rep_seq_twist.size()/7;i++)
    {
        if(m_f_rep_seq_elapsed>=m_vec_d_rep_seq_twist[i*7]
           && m_vec_b_rep_seq_twist[i*7]!=true)
        {
            // Get current state up to velocities before setting TWIST
            m_oa_rep_manager->GetRepStateViaService(m_s_rep_seq_name,"link_base",&m_vc3_rep_seq_position,&m_qua_rep_seq_orientation,&m_vc3_rep_seq_orientation,&m_vc3_rep_seq_vel_linear,&m_vc3_rep_seq_vel_angular);
            m_vc3_rep_seq_vel_linear.setValue(m_vec_d_rep_seq_twist[i*7+1],m_vec_d_rep_seq_twist[i*7+2],m_vec_d_rep_seq_twist[i*7+3]);
            m_vc3_rep_seq_vel_angular.setValue(m_vec_d_rep_seq_twist[i*7+4],m_vec_d_rep_seq_twist[i*7+5],m_vec_d_rep_seq_twist[i*7+6]);
            m_oa_rep_manager->SetRepStateViaServiceQuaternion(m_s_rep_seq_name,m_vc3_rep_seq_position,m_qua_rep_seq_orientation,m_vc3_rep_seq_vel_linear,m_vc3_rep_seq_vel_angular);
            m_vec_b_rep_seq_twist[i*7]=true;
            m_b_rep_seq_changed=true;
            //ROS_WARN("Twist changed!");
        }
    }

    // SET STATE for N times [t0,x,y,z,R,P,Y, t1,...]
    for(int i=0;i<m_vec_d_rep_seq_state.size()/7;i++)
    {
        if(m_f_rep_seq_elapsed>=m_vec_d_rep_seq_state[i*7]
           && m_vec_b_rep_seq_state[i*7]!=true)
        {
            // Get current state up to velocities before setting STATE
            m_oa_rep_manager->GetRepStateViaService(m_s_rep_seq_name,"link_base",&m_vc3_rep_seq_position,&m_qua_rep_seq_orientation,&m_vc3_rep_seq_orientation,&m_vc3_rep_seq_vel_linear,&m_vc3_rep_seq_vel_angular);
            m_vc3_rep_seq_position.setValue(m_vec_d_rep_seq_state[i*7+1],m_vec_d_rep_seq_state[i*7+2],m_vec_d_rep_seq_state[i*7+3]);
            m_vc3_rep_seq_orientation.setValue(m_vec_d_rep_seq_state[i*7+4],m_vec_d_rep_seq_state[i*7+5],m_vec_d_rep_seq_state[i*7+6]);
            m_oa_rep_manager->SetRepStateViaService(m_s_rep_seq_name,m_vc3_rep_seq_position,m_vc3_rep_seq_orientation,m_vc3_rep_seq_vel_linear,m_vc3_rep_seq_vel_angular);
            m_vec_b_rep_seq_state[i*7]=true;
            m_b_rep_seq_changed=true;
            //ROS_WARN("State changed!");
        }
    }

    if(m_b_rep_seq_changed==true &&
       m_f_rep_seq_elapsed!=m_f_rep_seq_duration)
    {
        //ROS_WARN("Adding status info");
        sprintf(m_c_state_info,"<table border=\"1\">"
                               "<tr bgcolor=\"darkgray\"> <th bgcolor=#04597f>\"%s\"@(t=%3.3fs)</th><th>TRA-X</th><th>TRA-Y</th><th>TRA-Z</th> <th>ROT-X</th><th>ROT-Y</th><th>ROT-Z</th> </tr>"
                               "<tr> <td bgcolor=\"lightgrey\">Pos[m,rad]</td><td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> <td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> </tr>"
                               "<tr> <td bgcolor=\"lightgrey\">Vel[m/s,rad/s]</td><td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> <td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> </tr>"
                               "<tr> <td bgcolor=\"lightgrey\">For[N,Nm]</td><td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> <td>%3.3f</td><td>%3.3f</td><td>%3.3f</td> </tr>"
                               "<tr> <td bgcolor=\"lightgrey\">Ene[J]</td><td>Pot</td><td>%3.3f</td><td bgcolor=\"lightgrey\"></td> <td>Kin</td><td>%3.3f</td><td bgcolor=\"lightgrey\"></td> </tr>"
                               "</table><br><br>",
                m_s_rep_seq_name.c_str(),
                m_f_rep_seq_elapsed,
                m_vc3_rep_seq_position.getX(),
                m_vc3_rep_seq_position.getY(),
                m_vc3_rep_seq_position.getZ(),
                m_vc3_rep_seq_orientation.getX(),
                m_vc3_rep_seq_orientation.getY(),
                m_vc3_rep_seq_orientation.getZ(),
                m_vc3_rep_seq_vel_linear.getX(),
                m_vc3_rep_seq_vel_linear.getY(),
                m_vc3_rep_seq_vel_linear.getZ(),
                m_vc3_rep_seq_vel_angular.getX(),
                m_vc3_rep_seq_vel_angular.getY(),
                m_vc3_rep_seq_vel_angular.getZ(),
                m_vc3_rep_seq_force.getX(),
                m_vc3_rep_seq_force.getY(),
                m_vc3_rep_seq_force.getZ(),
                m_vc3_rep_seq_torque.getX(),
                m_vc3_rep_seq_torque.getY(),
                m_vc3_rep_seq_torque.getZ(),
                1.0*9.81*m_vc3_rep_seq_position.getZ(),
                0.5*1.0*pow(m_vc3_rep_seq_vel_linear.length(),2.0)+0.5*1.0*pow(m_vc3_rep_seq_vel_angular.length(),2.0)
                );
        m_s_state_info+=std::string(m_c_state_info);
    }
}

void WAIOARepSequencer::Initialize(ros::NodeHandle* hdl_node,
                                   WAIOARepManager* oa_rep_manager,
                                   float f_node_sample_frequency)
{
    m_hdl_node=hdl_node;
    m_oa_rep_manager=oa_rep_manager;
    m_f_node_sample_frequency=f_node_sample_frequency;

    ResetSequencerStates();

    // REP SETUP Initialization
    //m_sub_clk_gazebo=m_hdl_node->subscribe("/clock",1,&WAIOARepSequencer::cb_sub_clk_gazebo,this);
    m_pub_str_rep_sequence=m_hdl_node->advertise<std_msgs::String>("rep_sequence",1);
    m_pub_hea_reset=m_hdl_node->advertise<std_msgs::Header>("/wai_world/sequencer_demo/state_reset",1);
}

void WAIOARepSequencer::UpdateModel(int i_rep_seq_scene,
                                    std::string s_rep_seq_name,
                                    float f_rep_seq_duration,
                                    std::vector<double> vec_d_rep_seq_state,
                                    std::vector<double> vec_d_rep_seq_twist,
                                    std::vector<double> vec_d_rep_seq_force)
{
    // Init CLOCK subscriber
    m_sub_clk_gazebo=m_hdl_node->subscribe("/clock",1,&WAIOARepSequencer::cb_sub_clk_gazebo,this);

    // Reset and prepare rep sequencer states
    ResetSequencerStates();

    m_i_rep_seq_scene=i_rep_seq_scene;
    m_s_rep_seq_name=s_rep_seq_name;
    m_f_rep_seq_duration=f_rep_seq_duration;
    m_vec_d_rep_seq_state=vec_d_rep_seq_state;
    m_vec_d_rep_seq_twist=vec_d_rep_seq_twist;
    m_vec_d_rep_seq_force=vec_d_rep_seq_force;
    for(int i=0;i<m_vec_d_rep_seq_state.size();i++) m_vec_b_rep_seq_state.push_back(false);
    for(int i=0;i<m_vec_d_rep_seq_twist.size();i++) m_vec_b_rep_seq_twist.push_back(false);
    for(int i=0;i<m_vec_d_rep_seq_force.size();i++) m_vec_b_rep_seq_force.push_back(false);

    // Reset sequencer_demo REP to starting pose (first position in sequence with zero forces and twists)
    m_oa_rep_manager->SetRepForceViaService(m_s_rep_seq_name,"link_base",tf::Vector3(0.0,0.0,0.0),tf::Vector3(0.0,0.0,0.0));
    m_oa_rep_manager->SetRepStateViaService(m_s_rep_seq_name,tf::Vector3(m_vec_d_rep_seq_state[1],m_vec_d_rep_seq_state[2],m_vec_d_rep_seq_state[3]),tf::Vector3(m_vec_d_rep_seq_state[4],m_vec_d_rep_seq_state[5],m_vec_d_rep_seq_state[6]),tf::Vector3(0.0,0.0,0.0),tf::Vector3(0.0,0.0,0.0));

    // Reset plots
    ResetSequencerPlots();

    // Update state info
    sprintf(m_c_state_info,"<b><u>REP \"%s\"-Sequence INITIALIZED (Scene %d):</u></b><br>",m_s_rep_seq_name.c_str(),m_i_rep_seq_scene);
    m_s_state_info=std::string(m_c_state_info);

    m_b_rep_seq_save_start_time=true;
    m_b_rep_seq_enabled=true;
    while(ros::ok() && m_f_rep_seq_elapsed<=m_f_rep_seq_duration)
    {
        ros::spinOnce();
        ros::Rate(200).sleep();
    }
    m_b_rep_seq_enabled=false;

    ResetSequencerTimestamp(false); // Disable publishing state messages for rqt_multiplot on sides of gazebo state plugin
    UpdateSequencerStatesInfo();

    m_sub_clk_gazebo.shutdown();
}

void WAIOARepSequencer::UpdateView()
{
}

void WAIOARepSequencer::ResetSequencerStates()
{
    m_s_rep_seq_name="sequencer_demo";
    m_vec_d_rep_seq_state.clear();
    m_vec_d_rep_seq_twist.clear();
    m_vec_d_rep_seq_force.clear();
    m_vec_b_rep_seq_state.clear();
    m_vec_b_rep_seq_twist.clear();
    m_vec_b_rep_seq_force.clear();
    m_msg_hea_reset.frame_id="disable";
    m_tim_setup_rep_start=m_msg_clk_gazebo.clock;
    m_i_rep_seq_scene=0;
    m_f_rep_seq_duration=0.0;
    m_f_rep_seq_elapsed=0.0;
    m_b_rep_seq_enabled=false;
    m_b_rep_seq_save_start_time=false;
    m_b_rep_seq_changed=false;
    m_s_state_info="";
    m_c_state_info[0]='\0';
}

void WAIOARepSequencer::ResetSequencerPlots()
{
    // Close all potential rqt_plot instances
    std::string s_syscmd="";
    //s_syscmd="killall rqt_plot";
    //int retval=system(s_syscmd.c_str());
    //s_syscmd="rqt_plot topics /wai_world/"+m_s_rep_seq_name+"/state_pose/pose/position /wai_world/"+m_s_rep_seq_name+"/state_pose/pose/orientation &";
    //retval=system(s_syscmd.c_str());
    //s_syscmd="rqt_plot topics /wai_world/"+m_s_rep_seq_name+"/state_twist/twist/linear /wai_world/"+m_s_rep_seq_name+"/state_twist/twist/angular &";
    //retval=system(s_syscmd.c_str());
    //s_syscmd="rqt_plot topics /wai_world/"+m_s_rep_seq_name+"/state_acceleration/accel/linear /wai_world/"+m_s_rep_seq_name+"/state_acceleration/accel/angular &";
    //retval=system(s_syscmd.c_str());

    // Use more flexible rqt_multiplot!
    /*
    s_syscmd="killall rqt_multiplot";
    int retval=system(s_syscmd.c_str());
    s_syscmd="rqt_multiplot --multiplot-run-all &";
    retval=system(s_syscmd.c_str());
    */
    int retval=system(s_syscmd.c_str());
    s_syscmd="rqt_multiplot --multiplot-config "+ros::package::getPath("wai_oa_gazebo")+"/resources/multiplots/"+m_s_rep_seq_name+".xml --multiplot-run-all &";
    retval=system(s_syscmd.c_str());
    SpinAndWaitForSeconds(5.0);
}

void WAIOARepSequencer::ResetSequencerTimestamp(bool b_enable)
{
    // Send signal to gazebo state pluging to enable/disable publishing rqt_multiplot_messages!
    if(b_enable)
    {
        m_msg_hea_reset.stamp=m_tim_setup_rep_start;
        m_msg_hea_reset.frame_id="enable";
        m_pub_hea_reset.publish(m_msg_hea_reset);
    }
    else
    {
        m_msg_hea_reset.frame_id="disable";
        m_pub_hea_reset.publish(m_msg_hea_reset);
    }
}

void WAIOARepSequencer::UpdateSequencerStatesInfo()
{
    sprintf(m_c_state_info,"<i>REP \"%s\"-Sequence FINISHED (t=%3.3fs)!</i><br><br><br>",
            m_s_rep_seq_name.c_str(),
            m_f_rep_seq_duration
            );
    m_s_state_info+=std::string(m_c_state_info);
    m_msg_str_rep_sequence.data=m_s_state_info;
    m_pub_str_rep_sequence.publish(m_msg_str_rep_sequence);
}

void WAIOARepSequencer::SpinAndWaitForSeconds(float f_seconds)
{
    ros::Rate r_sleep(m_f_node_sample_frequency);
    ros::Time tim_start=ros::Time::now();
    while((ros::Time::now()-tim_start).toSec()<f_seconds)
    {
        ros::spinOnce();
        r_sleep.sleep();
    }
}
