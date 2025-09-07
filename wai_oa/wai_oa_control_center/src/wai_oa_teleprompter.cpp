#include<wai_oa_teleprompter.h>



/////////////////////////////////////////////////
/// Implementation of WAIOATeleprompter
/////////////////////////////////////////////////

WAIOATeleprompter::WAIOATeleprompter()
{
}

WAIOATeleprompter::~WAIOATeleprompter()
{
}

void WAIOATeleprompter::Initialize(ros::NodeHandle* hdl_node,float f_node_sample_frequency,int i_txt_delta)
{
    m_hdl_node=hdl_node;
    m_f_node_sample_frequency=f_node_sample_frequency;

    m_os_display = XOpenDisplay(NULL);
    m_os_screen = DefaultScreenOfDisplay(m_os_display);

    i_offset_x=int(double(m_os_screen->width)*0.05);
    i_offset_y=int(double(m_os_screen->height)*0.8);
    i_size_x=int(double(m_os_screen->width)*0.9);
    i_size_y=int(double(m_os_screen->height)*0.083);
    i_text_offset_y=int(double(i_size_y)*0.72);
    d_text_size=double(i_size_y)/30.0;
    i_text_delta=i_txt_delta;
    i_counter=i_size_x;

    cv::namedWindow("Teleprompter",cv::WINDOW_AUTOSIZE);
    cv::waitKey(1);
    cv::resizeWindow("Teleprompter",i_size_x,i_size_y);
    cv::waitKey(1);
    cv::moveWindow("Teleprompter",i_offset_x,i_offset_y);
    cv::waitKey(1);
    ROS_DEBUG("i_offset_x, i_offset_y, i_size_x, i_size_y, i_text_offset_y, d_text_size: %d , %d , %d , %d , %d , %lf , ",
             i_offset_x,
             i_offset_y,
             i_size_x,
             i_size_y,
             i_text_offset_y,
             d_text_size);

    cv::Mat mat_img_init(i_size_y,i_size_x,CV_8UC3,cv::Scalar(0, 0, 0));
    mat_img_init.copyTo(mat_img_teleprompter);

    s_teleprompter_text="Welcome to Open Auditorium!";
    cv::putText(mat_img_teleprompter,
                s_teleprompter_text,
                cv::Point(i_counter, i_text_offset_y),
                cv::FONT_HERSHEY_DUPLEX,
                d_text_size, //d_teleprompter_text_size,
                CV_RGB(255, 255, 255), //font colors
                2);
    cv::imshow("Teleprompter",mat_img_teleprompter);
    cv::waitKey(1);

    tmr_teleprompter=m_hdl_node->createTimer(ros::Duration(1.0/f_node_sample_frequency),&WAIOATeleprompter::cb_tmr_teleprompter,this,false,true);
}

void WAIOATeleprompter::UpdateModel(std::string s_text)
{
    i_counter=i_size_x; // Reset text offset counter
    s_teleprompter_text=s_text;
}

void WAIOATeleprompter::UpdateView()
{
    mat_img_teleprompter.setTo(cv::Scalar(0,0,0));
    cv::putText(mat_img_teleprompter,
                s_teleprompter_text,
                cv::Point(i_counter, i_text_offset_y),
                cv::FONT_HERSHEY_DUPLEX,
                d_text_size,
                CV_RGB(255, 255, 255), //font color
                2);
    cv::imshow("Teleprompter",mat_img_teleprompter);
    i_counter-=20; //20 Shift text to the left
    cv::waitKey(1);
}

void WAIOATeleprompter::cb_tmr_teleprompter(const ros::TimerEvent& event)
{
    UpdateView();
}
