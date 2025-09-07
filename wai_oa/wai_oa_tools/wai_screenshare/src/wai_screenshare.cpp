#include<wai_screenshare.h>



WAIScreenshare::WAIScreenshare():m_hdl_it(hdl_node)
{
    s_path_nodename=ros::this_node::getName();
    hdl_node.getParam(s_path_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY",F_NODE_SAMPLE_FREQUENCY);
    hdl_node.getParam(s_path_nodename+"/"+"I_SCREEN_OFFSET_X",I_SCREEN_OFFSET_X);
    hdl_node.getParam(s_path_nodename+"/"+"I_SCREEN_OFFSET_Y",I_SCREEN_OFFSET_Y);
    hdl_node.getParam(s_path_nodename+"/"+"I_SCREEN_RESOLUTION_X",I_SCREEN_RESOLUTION_X);
    hdl_node.getParam(s_path_nodename+"/"+"I_SCREEN_RESOLUTION_Y",I_SCREEN_RESOLUTION_Y);
    hdl_node.getParam(s_path_nodename+"/"+"F_SCREEN_RESOLUTION_SCALE",F_SCREEN_RESOLUTION_SCALE);

    m_pub_img_screenshot=m_hdl_it.advertise("screenshare/image_raw",1);
}

WAIScreenshare::~WAIScreenshare()
{

}

void WAIScreenshare::run()
{
    ros::Rate loop_rate(F_NODE_SAMPLE_FREQUENCY);
    ScreenShot scr_screenshot(I_SCREEN_OFFSET_X,I_SCREEN_OFFSET_Y,I_SCREEN_RESOLUTION_X,I_SCREEN_RESOLUTION_Y);

    while(ros::ok())
    {
        scr_screenshot(m_mat_img_screenshot);
        cv::resize(m_mat_img_screenshot,m_mat_img_screenshot,cv::Size(),F_SCREEN_RESOLUTION_SCALE,F_SCREEN_RESOLUTION_SCALE);
        m_msg_img_screenshot=cv_bridge::CvImage(std_msgs::Header(),"bgra8",m_mat_img_screenshot).toImageMsg();
        m_pub_img_screenshot.publish(m_msg_img_screenshot);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
