#include<wai_lowpass_filter.h>



/////////////////////////////////////////////////
/// Implementations of lowpass filter
/////////////////////////////////////////////////
WAILowpassFilter::WAILowpassFilter()
{
    // Constrctor
    m_d_sample_time=1.0;
    m_d_rise_time=1.0;
    m_d_epsilon=0.01;
    m_d_iteration_res=0.0;
    m_tf_setpoint=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    m_tf_current=tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.0));
    m_vc3_xyz_curr=tf::Vector3(0.0,0.0,0.0);
    m_vc3_xyz_sp=tf::Vector3(0.0,0.0,0.0);
    m_vc3_xyz_temp=tf::Vector3(0.0,0.0,0.0);
    m_d_x=0.0;
    m_d_y=0.0;
    m_d_z=0.0;
    m_d_yaw_curr=0.0;
    m_d_pitch_curr=0.0;
    m_d_roll_curr=0.0;
    m_d_yaw_sp=0.0;
    m_d_pitch_sp=0.0;
    m_d_roll_sp=0.0;
    m_d_yaw_temp=0.0;
    m_d_pitch_temp=0.0;
    m_d_roll_temp=0.0;
}

WAILowpassFilter::~WAILowpassFilter()
{
    // Destructur
}

void WAILowpassFilter::InitializeParameters(double d_sample_time,double d_rise_time,double d_epsilon)
{
    m_d_sample_time=d_sample_time;
    m_d_rise_time=d_rise_time;
    m_d_epsilon=d_epsilon;
}


double WAILowpassFilter::LowPassFilter(double x,double y0,double dt,double T)          // Taken from http://en.wikipedia.org/wiki/Low-pass_filter
{
   m_d_iteration_res=y0+(x-y0)*(dt/(dt+T));
   return m_d_iteration_res;
}

void WAILowpassFilter::InitializeSetpoint(tf::Transform tf_setpoint)
{
    m_tf_setpoint=tf_setpoint;
    m_tf_current=m_tf_setpoint;
    m_vc3_xyz_sp=m_tf_setpoint.getOrigin();
    m_vc3_xyz_curr=m_tf_setpoint.getOrigin();
    m_vc3_xyz_temp=tf::Vector3(0.0,0.0,0.0);
    tf::Matrix3x3 m_setp_rot(m_tf_setpoint.getRotation());
    m_setp_rot.getRPY(m_d_roll_sp,m_d_pitch_sp,m_d_yaw_sp);
    m_d_roll_curr=0.0;
    m_d_pitch_curr=0.0;
    m_d_yaw_curr=0.0;
    m_d_roll_sp=0.0;
    m_d_pitch_sp=0.0;
    m_d_yaw_sp=0.0;
    m_d_roll_temp=0.0;
    m_d_pitch_temp=0.0;
    m_d_yaw_temp=0.0;
}

void WAILowpassFilter::UpdateSetpoint(tf::Transform tf_setpoint)
{
    m_tf_setpoint=tf_setpoint;
    m_vc3_xyz_sp=m_tf_setpoint.getOrigin();
    tf::Matrix3x3 m_setp_rot(m_tf_setpoint.getRotation());
    m_setp_rot.getRPY(m_d_roll_sp,m_d_pitch_sp,m_d_yaw_sp);
}


tf::Transform WAILowpassFilter::CalculateIteration()
{
    if((m_vc3_xyz_sp-m_vc3_xyz_curr).length()>=m_d_epsilon)
    {
        m_vc3_xyz_temp=tf::Vector3(
                    LowPassFilter(m_vc3_xyz_sp.getX(),m_vc3_xyz_curr.getX(),m_d_sample_time,m_d_rise_time),
                    LowPassFilter(m_vc3_xyz_sp.getY(),m_vc3_xyz_curr.getY(),m_d_sample_time,m_d_rise_time),
                    LowPassFilter(m_vc3_xyz_sp.getZ(),m_vc3_xyz_curr.getZ(),m_d_sample_time,m_d_rise_time));
        m_vc3_xyz_curr=m_vc3_xyz_temp;

        m_d_yaw_temp=LowPassFilter(m_d_yaw_sp,m_d_yaw_curr,m_d_sample_time,m_d_rise_time);
        m_d_pitch_temp=0.0;//LowPassFilter(m_d_pitch_sp,m_d_pitch_curr,m_d_sample_time,m_d_rise_time);
        m_d_roll_temp=0.0;//LowPassFilter(m_d_roll_sp,m_d_roll_curr,m_d_sample_time,m_d_rise_time);
        m_d_yaw_curr=m_d_yaw_temp;
        m_d_pitch_curr=m_d_pitch_temp;
        m_d_roll_curr=m_d_roll_temp;

        m_tf_current.setOrigin(m_vc3_xyz_curr);
        m_tf_current.setRotation(tf::Quaternion(m_d_yaw_curr,m_d_pitch_curr,m_d_roll_curr));
    }

    return m_tf_current;
}


