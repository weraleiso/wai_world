#ifndef WAI_OA_INTEL_H
#define WAI_OA_INTEL_H

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
#include<cstdlib>
#include<ctime>

#include<QMessageBox>

#include<ros/ros.h>
#include<ros/package.h>
#include<std_msgs/Empty.h>
#include<std_msgs/Bool.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Point.h>
#include<gazebo_msgs/LinkStates.h>
#include<gazebo_msgs/SetModelState.h>
#include<std_srvs/Empty.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
#include<tf/transform_datatypes.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<view_controller_msgs/CameraPlacement.h>

#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>

#include<wai_reps.h>
#include<wai_ethical_model.h>
#include<wai_picovoice.h>
#include<wai_oa_triggers.h>

// -=[INTEL]=-
// Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
// preserve natural human presence in the use of AIS in education"
// In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
// Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
// --> Define Marvin as another assistive AIS:
#include<wai_oa_marvin.h>
class WAIOAMarvin;



////////////////////////////////////////////////////
/// Class definition of WAIOAIntel
////////////////////////////////////////////////////
class WAIOAIntel:WAIArtificial
{
    ros::NodeHandle* m_hdl_node;
    image_transport::ImageTransport* m_hdl_it;
    image_transport::TransportHints hints;

    WAIPicovoice m_wai_picovoice;

    WAIOATriggers* m_oa_triggers;

    ros::Publisher pub_s_oa_ethics;
    ros::Publisher pub_s_oa_ethical_status;

    std::string m_s_path_nodename;
    std::string m_s_path_package;
    float m_f_node_sample_frequency;

    ros::Publisher m_pub_mrk_intel_state;
    visualization_msgs::Marker m_msg_mrk_intel_state;

    ros::Publisher m_pub_img_intel_display;
    cv::VideoCapture m_vcp_display_video;
    cv::Mat m_mat_display_video_frame;
    cv::Mat m_mat_img_display;
    std::string m_s_path_display_video;

    ros::Timer m_tmr_intel;

public:
    WAIEthicalModel* m_oa_ethical_model;
    int m_i_count_prompts;

    WAIOAIntel();
    ~WAIOAIntel();

    void Initialize(ros::NodeHandle* hdl_node,
                    image_transport::ImageTransport* hdl_it,
                    WAIEthicalModel* oa_ethical_model,
                    WAIOATriggers* oa_triggers,
                    std::string s_path_nodename,
                    float f_node_sample_frequency);
    void UpdateModel();
    void UpdateView();

    void cb_tmr_intel(const ros::TimerEvent& event);

    void UpdateEthicalStatusLabel(std::string s_ethical_status);

    // -=[INTEL]=-
    // Reference: W. A. Isop. "INTEL – An oversight mechanism to sustainably
    // preserve natural human presence in the use of AIS in education"
    // In Proceedings: Proceedings of European Seminar on AI for Sustainability and Climate Change,
    // Hamburg, 15.Jan 2026, edited by ..., Springer, 2026, pp. 1-17.
    // --> Exemplary method to request an ethically critical use case:
    void RequestSomethingThatIsPotentiallyUnethical(WAIOAMarvin* m_oa_marvin);

    //INTEL (internal interactions)
    //=============================
    void DisplayStateCancel();
    void DisplayState(std::string s_state_name);
    void DisplayStateImage(std::string s_state_name);
    void ProcessTextToSpeech(std::string s_text);
    void ProcessIntent();
    void ProcessTextPrompt(std::string s_prompt_text);
    void ProcessVoicePrompt();
    void InteractionVoiceMarvin(std::string s_voice);
    void InteractionVoice(std::string s_voice);
    void InteractionVoiceFromFile(std::string s_filename,bool b_blocking=false);


    // Interaction Requests
    //======================
    int InteractionRequest( void* v_actor,
                            std::string s_actor_name,
                            std::string s_actor_type_role,
                            //std::string s_principle,
                            //std::string s_method,
                            //std::string s_activity,
                            //std::string s_sub_goal,
                            std::string s_use_case,
                            double d_iol_delta,
                            std::string s_string="");
    void InteractionRequestVoiceCommand(WAIOAMarvin* m_marvin);

    std::vector<int> m_par_img_compression={cv::IMWRITE_JPEG_QUALITY,75};
    std::vector<uchar> m_uch_buf_compression;
    sensor_msgs::CompressedImage EncodeImage(cv::Mat mat_input);
};

#endif //WAI_OA_INTEL_H
