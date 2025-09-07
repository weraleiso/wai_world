#ifndef WAI_OA_TELEPROMPTER_H
#define WAI_OA_TELEPROMPTER_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<iostream>
#include<math.h>
#include<vector>
#include<queue>
#include<string>
#include<string.h>
#include<sstream>
#include<fstream>

#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<view_controller_msgs/CameraPlacement.h>

#include<X11/Xlib.h>
#include<X11/keysym.h>



/////////////////////////////////////////////////
/// Class definition of WAIOATeleprompter
/////////////////////////////////////////////////


class WAIOATeleprompter
{
    ros::NodeHandle* m_hdl_node;
    float m_f_node_sample_frequency;

    // DISPLAY settings helper members
    Display* m_os_display;
    Screen*  m_os_screen;

    cv::Mat mat_img_teleprompter;

    int i_offset_x;
    int i_offset_y;
    int i_size_x;
    int i_size_y;
    int i_text_offset_y;
    double d_text_size;
    int i_text_delta;
    int i_counter;

    std::string s_teleprompter_text;

    ros::Timer tmr_teleprompter;

    void cb_tmr_teleprompter(const ros::TimerEvent& event);

public:
    WAIOATeleprompter();
    ~WAIOATeleprompter();

    void Initialize(ros::NodeHandle* hdl_node,float f_node_sample_frequency,int i_txt_del);
    void UpdateModel(std::string s_text);
    void UpdateView();
};

#endif //WAI_OA_TELEPROMPTER_H
