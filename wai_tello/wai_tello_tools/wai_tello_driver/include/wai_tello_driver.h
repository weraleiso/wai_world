#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<geometry_msgs/Twist.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/imgcodecs/imgcodecs.hpp>
#include<opencv4/opencv2/imgproc.hpp>

#include<arpa/inet.h>
#include<ifaddrs.h>
#include<errno.h>
#include<memory.h>
#include<net/if.h>
#include<netdb.h>
#include<netinet/in.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<unistd.h>
#include<sstream>
#include<optional>
#include<thread>

#include<X11/Xlib.h>
#include<X11/keysym.h>

#define TELLO_SERVER_IP "192.168.10.1" // This is the IP adress of the Tello
#define TELLO_SERVER_COMMAND_PORT "8889" // This is the command port to send commands to Tello

//#define TELLO_STREAM_URL "udp://0.0.0.0:11111" // This is the video stream port to receive the H.264-encoded video stream
#define TELLO_STREAM_URL "udp://0.0.0.0:11111?fifo_size=500000&overrun_nonfatal=1&fflags=nobuffer&flags=low_delay"

#define LOCAL_CLIENT_COMMAND_PORT 9000 // This is the local port where we bind our local UDP client to.
#define LOCAL_SERVER_STATE_PORT 8890 // We need to start a local UPD server to receive state updates.


static sensor_msgs::ImagePtr msg_mat_img_tello_camera_rgb_front;



class WAITello
{
    WAITello(const WAITello&) = delete;
    WAITello(const WAITello&&) = delete;
    WAITello& operator=(const WAITello&) = delete;
    WAITello& operator=(const WAITello&&) = delete;

    int m_command_sockfd{0};
    int m_state_sockfd{0};
    int m_local_client_command_port{LOCAL_CLIENT_COMMAND_PORT};
    sockaddr_storage m_tello_server_command_addr{};

    std::string s_nodename;
    ros::NodeHandle m_hdl_node;
    image_transport::ImageTransport m_hdl_it;

    float F_NODE_SAMPLE_FREQUENCY;
    float F_COMMAND_SCALE;
    float F_COMMAND_KEYBOARD_SPEED;

    ros::Subscriber sub_twi_command_velocity;
    ros::Subscriber sub_emp_command_takeoff;
    ros::Subscriber sub_emp_command_land;
    image_transport::Publisher pub_img_camera_tello_mono_rgb;

    geometry_msgs::Twist msg_twi_command_velocity;

    int i_tello_command_rc_x;
    int i_tello_command_rc_y;
    int i_tello_command_rc_z;
    int i_tello_command_rc_yaw;

    Display* dsp_x11_display;

public:
    WAITello();
    ~WAITello();

    bool KeyIsPressed(KeySym ks);
    void TelloCheckTriggers();

    static void cb_thread_tello_videostream();
    void cb_sub_twi_command_velocity(const geometry_msgs::Twist::ConstPtr& msg);
    void cb_sub_emp_command_emergency(const std_msgs::Empty::ConstPtr& msg);
    void cb_sub_emp_command_land(const std_msgs::Empty::ConstPtr& msg);
    void cb_sub_emp_command_takeoff(const std_msgs::Empty::ConstPtr& msg);

    bool Bind(int local_client_command_port = LOCAL_CLIENT_COMMAND_PORT);
    bool SendCommand(const std::string& command);
    std::optional<std::string> ReceiveResponse();
    std::optional<std::string> GetState();

    void FindTello();
    void ShowTelloInfo();

    void Emergency();
    void Land();
    void Takeoff();
    void FlipLeft();
    void FlipRight();
    void FlipBack();

    void TelloCommandSpeed();
    void TelloCommandFlyDelta(int i_delta);
    void TelloPublishCameraRGBStream();

    void run();
};

