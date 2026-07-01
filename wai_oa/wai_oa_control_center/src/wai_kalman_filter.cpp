#include<wai_kalman_filter.h>



/////////////////////////////////////////////////
/// Implementations of lowpass filter
/////////////////////////////////////////////////
WAIKalmanFilter::WAIKalmanFilter()
{
    // Constrctor
}

WAIKalmanFilter::~WAIKalmanFilter()
{
    // Destructur
}

void WAIKalmanFilter::Initialize3DOF(float f_sample_time,
                                     float f_state_init_x,
                                     float f_state_init_y,
                                     float f_state_init_z,
                                     float f_processNoiseCov,
                                     float f_measurementNoiseCov,
                                     float f_errorCovPost,
                                     float f_prediction_timeout)
{
    m_f_sample_time=f_sample_time;
    m_f_prediction_timeout=f_prediction_timeout;

    float dt=m_f_sample_time; // 1.0/15.0
    float v=dt;
    float a=0.5*dt*dt;

    // Init Kalmanfilter with 3D-Translation only
    KF.init(9,3,0);
    KF.transitionMatrix = (cv::Mat_<float>(9, 9) <<
                            1, 0, 0, v, 0, 0, a, 0, 0,
                            0, 1, 0, 0, v, 0, 0, a, 0,
                            0, 0, 1, 0, 0, v, 0, 0, a,
                            0, 0, 0, 1, 0, 0, v, 0, 0,
                            0, 0, 0, 0, 1, 0, 0, v, 0,
                            0, 0, 0, 0, 0, 1, 0, 0, v,
                            0, 0, 0, 0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 1);

    KF.statePre.at<float>(0)=f_state_init_x;
    KF.statePre.at<float>(1)=f_state_init_y;
    KF.statePre.at<float>(2)=f_state_init_z;
    KF.statePre.at<float>(3)=0.0;
    KF.statePre.at<float>(4)=0.0;
    KF.statePre.at<float>(5)=0.0;
    KF.statePre.at<float>(6)=0.0;
    KF.statePre.at<float>(7)=0.0;
    KF.statePre.at<float>(8)=0.0;

    mat_state=cv::Mat(3, 1, CV_32F);
    mat_state.at<float>(0) = 0.0f;
    mat_state.at<float>(1) = 0.0f;
    mat_state.at<float>(2) = 0.0f;

    mat_measurement=cv::Mat::zeros(3, 1, CV_32F);
    mat_processNoise=cv::Mat(3, 1, CV_32F);

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(f_processNoiseCov));
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(f_measurementNoiseCov));
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(f_errorCovPost));
}

void WAIKalmanFilter::Initialize6DOF(float f_sample_time,
                                    float f_state_init_x,
                                    float f_state_init_y,
                                    float f_state_init_z,
                                    float f_state_init_roll,
                                    float f_state_init_pitch,
                                    float f_state_init_yaw,
                                    float f_processNoiseCov,
                                    float f_measurementNoiseCov,
                                    float f_errorCovPost,
                                    float f_prediction_timeout)
{
    m_f_sample_time=f_sample_time;
    m_f_prediction_timeout=f_prediction_timeout;

    m_d_measure_roll=0.0;
    m_d_measure_pitch=0.0;
    m_d_measure_yaw=0.0;

    float dt=m_f_sample_time;
    float v=dt;
    float a=0.5*dt*dt;

    // Init Kalmanfilter with full POSE (however up to now with Euler angles!)
    KF.init(18,6,0);
    KF.transitionMatrix=(cv::Mat_<float>(18, 18) <<
                            1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a, 0, 0, 0, 0, 0,
                            0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a, 0, 0, 0, 0,
                            0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a, 0, 0, 0,
                            0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a, 0, 0,
                            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a, 0,
                            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0, a,
                            0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, v,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

    KF.statePre.at<float>(0)=f_state_init_x; // X
    KF.statePre.at<float>(1)=f_state_init_y; // Y
    KF.statePre.at<float>(2)=f_state_init_z; // Z
    KF.statePre.at<float>(3)=f_state_init_roll; // ROLL
    KF.statePre.at<float>(4)=f_state_init_pitch; // PITCH
    KF.statePre.at<float>(5)=f_state_init_yaw; // YAW
    KF.statePre.at<float>(6)=0.0; // Init state for vels and accel. not mandatory
    KF.statePre.at<float>(7)=0.0;
    KF.statePre.at<float>(8)=0.0;
    KF.statePre.at<float>(9)=0.0;
    KF.statePre.at<float>(10)=0.0;
    KF.statePre.at<float>(11)=0.0;
    KF.statePre.at<float>(12)=0.0;
    KF.statePre.at<float>(13)=0.0;
    KF.statePre.at<float>(14)=0.0;
    KF.statePre.at<float>(15)=0.0;
    KF.statePre.at<float>(16)=0.0;
    KF.statePre.at<float>(17)=0.0;

    mat_state=cv::Mat(6,1,CV_32F);
    mat_state.at<float>(0)=0.0f; // X,Y,Z,ROLL,PITCH,YAW
    mat_state.at<float>(1)=0.0f;
    mat_state.at<float>(2)=0.0f;
    mat_state.at<float>(3)=0.0f;
    mat_state.at<float>(4)=0.0f;
    mat_state.at<float>(5)=0.0f;

    mat_measurement=cv::Mat::zeros(6,1,CV_32F);
    mat_processNoise=cv::Mat(6,1,CV_32F);

    cv::setIdentity(KF.measurementMatrix);
    cv::setIdentity(KF.processNoiseCov,cv::Scalar::all(f_processNoiseCov));
    cv::setIdentity(KF.measurementNoiseCov,cv::Scalar::all(f_measurementNoiseCov));
    cv::setIdentity(KF.errorCovPost,cv::Scalar::all(f_errorCovPost));
}

void WAIKalmanFilter::UpdateMeasurement3DOF(tf::Vector3 vc3_measurement)
{
    tim_measurement_last=ros::Time::now();
    m_vc3_measurement=vc3_measurement;
}

void WAIKalmanFilter::UpdateMeasurement6DOF(tf::Transform tf_measurement)
{
    tim_measurement_last=ros::Time::now();
    m_tf_measurement=tf_measurement;
    tf::Matrix3x3 m_setp_rot(m_tf_measurement.getRotation());
    m_setp_rot.getRPY(m_d_measure_roll,m_d_measure_pitch,m_d_measure_yaw);
}

tf::Vector3 WAIKalmanFilter::Predict3DOF(int i_iterations,bool b_prediction_fallback)
{
    // First iteration
    KF.predict();
    if(b_prediction_fallback==true
        && (ros::Time::now()-tim_measurement_last).toSec()<m_f_prediction_timeout)
    {
        mat_measurement.at<float>(0)=mat_post.at<float>(0);
        mat_measurement.at<float>(1)=mat_post.at<float>(1);
        mat_measurement.at<float>(2)=mat_post.at<float>(2);
    }
    else
    {
        mat_measurement.at<float>(0)=m_vc3_measurement.getX();
        mat_measurement.at<float>(1)=m_vc3_measurement.getY();
        mat_measurement.at<float>(2)=m_vc3_measurement.getZ();
    }
    KF.correct(mat_measurement);
    mat_post=KF.transitionMatrix*KF.statePost;

    if(i_iterations>1)
    {
        for(int i=0;i<i_iterations-1;i++)
        {
            KF.predict();
            mat_measurement.at<float>(0)=mat_post.at<float>(0);
            mat_measurement.at<float>(1)=mat_post.at<float>(1);
            mat_measurement.at<float>(2)=mat_post.at<float>(2);
            KF.correct(mat_measurement);
            mat_post=KF.transitionMatrix*KF.statePost;
        }
    }

    m_vc3_prediction.setValue(mat_post.at<float>(0),mat_post.at<float>(1),mat_post.at<float>(2));
    return m_vc3_prediction;
}

tf::Transform WAIKalmanFilter::Predict6DOF(int i_iterations,bool b_prediction_fallback)
{
    // First iteration
    KF.predict();
    if(b_prediction_fallback==true
        && (ros::Time::now()-tim_measurement_last).toSec()<m_f_prediction_timeout)
    {
        mat_measurement.at<float>(0)=mat_post.at<float>(0);
        mat_measurement.at<float>(1)=mat_post.at<float>(1);
        mat_measurement.at<float>(2)=mat_post.at<float>(2);
        mat_measurement.at<float>(3)=mat_post.at<float>(3);
        mat_measurement.at<float>(4)=mat_post.at<float>(4);
        mat_measurement.at<float>(5)=mat_post.at<float>(5);
    }
    else
    {
        mat_measurement.at<float>(0)=m_tf_measurement.getOrigin().getX();
        mat_measurement.at<float>(1)=m_tf_measurement.getOrigin().getY();
        mat_measurement.at<float>(2)=m_tf_measurement.getOrigin().getZ();
        mat_measurement.at<float>(3)=m_d_measure_roll;
        mat_measurement.at<float>(4)=m_d_measure_pitch;
        mat_measurement.at<float>(5)=m_d_measure_yaw;
    }
    KF.correct(mat_measurement);
    mat_post=KF.transitionMatrix*KF.statePost;

    if(i_iterations>1)
    {
        for(int i=0;i<i_iterations-1;i++)
        {
            KF.predict();
            mat_measurement.at<float>(0)=mat_post.at<float>(0);
            mat_measurement.at<float>(1)=mat_post.at<float>(1);
            mat_measurement.at<float>(2)=mat_post.at<float>(2);
            mat_measurement.at<float>(3)=mat_post.at<float>(3);
            mat_measurement.at<float>(4)=mat_post.at<float>(4);
            mat_measurement.at<float>(5)=mat_post.at<float>(5);
            KF.correct(mat_measurement);
            mat_post=KF.transitionMatrix*KF.statePost;
        }
    }

    m_qua_prediction.setEulerZYX(mat_post.at<float>(5),mat_post.at<float>(4),mat_post.at<float>(3));
    m_tf_prediction.setOrigin(tf::Vector3(mat_post.at<float>(0),mat_post.at<float>(1),mat_post.at<float>(2)));
    m_tf_prediction.setRotation(m_qua_prediction);
    return m_tf_prediction;
}
