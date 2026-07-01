#ifndef WAI_OA_IMAGE_PROCESSING_STRATEGY_H
#define WAI_OA_IMAGE_PROCESSING_STRATEGY_H



/////////////////////////////////////////////////
/// Selective inclusion of OpenCV libraries
/////////////////////////////////////////////////
#include<tf/transform_datatypes.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>



/////////////////////////////////////////////////////
/// Abstract base class of all processing strategies
/////////////////////////////////////////////////////
class WAIOAImageProcessingStrategy
{
public:
    virtual tf::Matrix3x3 process(cv::Mat) = 0;

protected:
    WAIOAImageProcessingStrategy()
    {
    }
};



/////////////////////////////////////////////////
/// Class defintion of advanced detection
/////////////////////////////////////////////////
class HandsAndHeadDetector : public WAIOAImageProcessingStrategy
{
    float m_f_range_min;
    float m_f_range_max;
    float m_f_threshold_dist;
    float m_f_threshold_bounds;
    float m_f_res_x;
    float m_f_res_y;
    float m_f_fx;
    float m_f_fy;
    float m_f_cx;
    float m_f_cy;
    cv::Size m_siz_gaussian_blur;

    tf::Matrix3x3 m_m33_hah;
    tf::Vector3 m_vc3_hand_scaled_origin;
    tf::Vector3 m_vc3_camera_rgbd_wrt_head_old;

public:
    HandsAndHeadDetector(float,float,float,float,float,float,float,float,float,float);
    tf::Matrix3x3 process(cv::Mat);
};



#endif //WAI_OA_IMAGE_PROCESSING_STRATEGY_H
