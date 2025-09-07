#ifndef WAI_OA_SKETCH_H
#define WAI_OA_SKETCH_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<ros/ros.h>
#include<ros/package.h>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>

#include<sound_play/sound_play.h>

#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<stdio.h>
#include<iostream>
#include<iomanip>
#include<ctime>
#include<sstream>

#include<QMessageBox>



/////////////////////////////////////////////////
/// OpenCV Sketch Tool
/////////////////////////////////////////////////
class WAIOASketch
{
    ros::NodeHandle* m_hdl_node;
    image_transport::ImageTransport* m_hdl_it;
    sound_play::SoundClient* m_hdl_snd_client;
    std::string m_s_package_path;
    std::string m_s_projector_topic;
    std::string m_s_rep_id;

    image_transport::Publisher m_pub_img_sketch;
    sensor_msgs::ImagePtr m_msg_img_sketch;

    int m_i_scroll_height;
    int m_i_scroll_width;
    cv::Size m_siz_sketch_size;
    cv::Size m_siz_sketch_size_scroll;
    cv::Mat mat_img_sketch;
    cv::Mat mat_img_sketch_scroll;
    cv::Mat mat_img_grid;
    cv::Mat mat_img_grid_scroll;
    cv::Mat mat_img_gui;
    cv::Mat mat_img_overlay;
    double m_alpha;
    double m_beta;
    std::vector<cv::Mat> vec_mat_img_sketches;
    cv::Point m_pnt_cursor_pos;
    cv::Point m_pnt_cursor_pos_old;
    cv::Point m_pnt_cursor_pos_1;
    cv::Point m_pnt_cursor_pos_2;
    cv::Point m_pnt_cursor_offset;
    cv::Point m_pnt_gui_element_dims;
    cv::Scalar m_scl_brush_color;
    cv::Scalar m_scl_color_grid;
    char m_c_key_pressed;
    float m_f_node_sample_frequency;
    float m_f_sketch_resize_factor;
    float m_f_line_angle_deg;
    int m_i_image_history_counter;
    int m_i_brush_thickness;
    int m_i_brush_shape;
    int m_i_brush_mode;
    int m_i_scroll_start_x;
    int m_i_scroll_start_y;
    int m_i_scroll_delta_x;
    int m_i_scroll_delta_y;
    int m_i_scroll_stop_x;
    int m_i_scroll_stop_y;
    bool m_b_scroll_enabled;
    bool m_b_brush_enabled;
    bool m_b_line_enabled;
    bool m_b_circle_enabled;
    bool m_b_text_enabled;
    cv::Point m_pnt_line_length;
    int m_i_line_length;

    ros::Timer tmr_sketch;

    void cb_tmr_sketch(const ros::TimerEvent& event);
    static void cb_trackbar_on_change(int i,void* ptr);

public:
    WAIOASketch();
    ~WAIOASketch();

    void Initialize(ros::NodeHandle* nh_,
                    image_transport::ImageTransport* it_,
                    sound_play::SoundClient* hdl_snd_client,
                    float f_node_sample_frequency,
                    float f_sketch_resize_factor);
    void ReopenSketch();
    // A4 Size with 300DPI equals 2480 x 3508 pixels
    void ResetSketch(cv::Size siz_sketch_size=cv::Size(1240,1754), // A4 Size with 150DPI
                    cv::Scalar sca_col_bg=cv::Scalar(0,0,0),
                    bool b_grid_enabled=true,
                    cv::Size siz_grid=cv::Size(30,30), // Approx. 5mm grid @ 150DPI
                    cv::Scalar scl_col_grid=cv::Scalar(128,128,128));
    void RedrawSketchOnScroll();
    void DrawGUIIcon(int i_icon_id);
    void DrawGUI();
    void AddCurrentSketchToHistoryAndDraw();
    void DrawSketch();
    void ShowAboutSketch();
    void LoadLastSketchInHistoryAndDraw();
    void SaveCurrentSketchToFile();
    void SendSketchToProjection();
    void PlaySound(std::string s_sound);
    int GetMMFromPixels(int i_pixels);
    static void onWAIOASketchMouseMove(int event,int x,int y,int flags,void* userdata)
    {
        WAIOASketch* tolskt = reinterpret_cast<WAIOASketch*>(userdata);
        tolskt->onWAIOASketchMove(event,x,y,flags);
    }
    void onWAIOASketchMove(int event,int x,int y,int flags);
};

#endif //WAI_OA_SKETCH_H
