#ifndef WAI_LOWPASS_FILTER_H
#define WAI_LOWPASS_FILTER_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<string>

#include<tf/transform_datatypes.h>



/////////////////////////////////////////////////
/// Lowpass filter
/////////////////////////////////////////////////
class WAILowpassFilter
{
    double m_d_sample_time;
    double m_d_rise_time;
    double m_d_epsilon;
    double m_d_iteration_res;
    tf::Transform m_tf_setpoint;
    tf::Transform m_tf_current;
    double m_d_x;
    double m_d_y;
    double m_d_z;
    tf::Vector3 m_vc3_xyz_curr;
    double m_d_yaw_curr;
    double m_d_pitch_curr;
    double m_d_roll_curr;
    tf::Vector3 m_vc3_xyz_sp;
    double m_d_yaw_sp;
    double m_d_pitch_sp;
    double m_d_roll_sp;
    tf::Vector3 m_vc3_xyz_temp;
    double m_d_yaw_temp;
    double m_d_pitch_temp;
    double m_d_roll_temp;

public:
    WAILowpassFilter();
    ~WAILowpassFilter();
    double LowPassFilter(double x,double y0,double dt,double T);
    void InitializeParameters(double d_sample_time=1.0,double d_rise_time=1.0,double d_epsilon=0.01);
    void InitializeSetpoint(tf::Transform tf_setpoint);
    void UpdateSetpoint(tf::Transform tf_setpoint);
    tf::Transform CalculateIteration();
};


#endif //WAI_LOWPASS_FILTER_H
