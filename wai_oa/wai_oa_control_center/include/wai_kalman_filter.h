#ifndef WAI_KALMAN_FILTER_H
#define WAI_KALMAN_FILTER_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<string>

#include<tf/transform_datatypes.h>

#include<opencv2/opencv.hpp>



/////////////////////////////////////////////////
/// Kalman filter
/////////////////////////////////////////////////
class WAIKalmanFilter
{
    cv::KalmanFilter KF;
    cv::Mat mat_state; // X,Y,Z,ROLL,PITCH,YAW
    cv::Mat mat_processNoise;
    cv::Mat mat_measurement;
    cv::Mat mat_prediction;
    cv::Mat mat_post;

    tf::Vector3 m_vc3_measurement;
    tf::Transform m_tf_measurement;
    tf::Vector3 m_vc3_prediction;
    tf::Transform m_tf_prediction;
    tf::Quaternion m_qua_prediction;

    double m_d_measure_roll,m_d_measure_pitch,m_d_measure_yaw;
    float m_f_prediction_timeout;

    float m_f_sample_time;

    ros::Time tim_measurement_last;

public:
    WAIKalmanFilter();
    ~WAIKalmanFilter();
    void Initialize3DOF(float f_sample_time,
                    float f_state_init_x=0.0,
                    float f_state_init_y=0.0,
                    float f_state_init_z=0.0,
                    float f_processNoiseCov=1.0,
                    float f_measurementNoiseCov=0.002,
                    float f_errorCovPost=0.0002,
                    float f_prediction_timeout=1.0);
    void Initialize6DOF(float f_sample_time,
                    float f_state_init_x=0.0,
                    float f_state_init_y=0.0,
                    float f_state_init_z=0.0,
                    float f_state_init_roll=0.0,
                    float f_state_init_pitch=0.0,
                    float f_state_init_yaw=0.0,
                    float f_processNoiseCov=1.0,
                    float f_measurementNoiseCov=0.002,
                    float f_errorCovPost=0.0002,
                    float f_prediction_timeout=1.0);
    void UpdateMeasurement3DOF(tf::Vector3 vc3_measurement);
    void UpdateMeasurement6DOF(tf::Transform tf_measurement);
    tf::Vector3 Predict3DOF(int i_iterations,bool b_prediction_fallback=false);
    tf::Transform Predict6DOF(int i_iterations,bool b_prediction_fallback=false);
};


#endif //WAI_KALMAN_FILTER_H
