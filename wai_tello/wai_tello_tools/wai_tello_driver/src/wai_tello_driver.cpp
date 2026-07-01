#include<wai_tello_driver.h>



void WAITello::cb_thread_tello_videostream()
{
    cv::Mat mat_img_tello_camera_rgb_front;
    cv::VideoCapture cap_tello_livetream{TELLO_STREAM_URL}; //,cv::CAP_FFMPEG};
    cap_tello_livetream.set(cv::CAP_PROP_BUFFERSIZE,1);
    cap_tello_livetream.set(cv::CAP_PROP_FRAME_WIDTH,648); // HIGH RES: 960
    cap_tello_livetream.set(cv::CAP_PROP_FRAME_HEIGHT,478); // HIGH RES: 720
    cap_tello_livetream.set(cv::CAP_PROP_FPS,3000.0);
    cap_tello_livetream.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('H','2','6','4'));
    //ROS_WARN("Supported: %f",cap_tello_livetream.get(cv::CAP_PROP_BUFFERSIZE));
    for(int i=0;i<100;i++) cap_tello_livetream.grab();

    ROS_WARN("STARTING VIDEOSTREAM!");
    while(ros::ok())
    {
        //cap_tello_livetream.grab();
        //cap_tello_livetream.retrieve(mat_img_tello_camera_rgb_front);
        cap_tello_livetream.read(mat_img_tello_camera_rgb_front);
        cv::imshow("Stream",mat_img_tello_camera_rgb_front);
        cv::waitKey(1);
        /*
        msg_mat_img_tello_camera_rgb_front=cv_bridge::CvImage(std_msgs::Header(),"bgr8",mat_img_tello_camera_rgb_front).toImageMsg();
        msg_mat_img_tello_camera_rgb_front->header.stamp=ros::Time::now();
        msg_mat_img_tello_camera_rgb_front->header.frame_id="tello/camera_rgb_front";
        */
    }
}



// Binds the given socket file descriptor ot the given port.
// Returns whether it succeeds or not and the error message.
std::pair<bool, std::string> BindSocketToPort(const int sockfd, const int port)
{
    sockaddr_in listen_addr{};
    // htons converts from host byte order to network byte order.
    listen_addr.sin_port=htons(port);
    listen_addr.sin_addr.s_addr=INADDR_ANY;
    listen_addr.sin_family=AF_INET;
    int result=bind(sockfd,reinterpret_cast<sockaddr*>(&listen_addr),sizeof(listen_addr));

    if (result == -1)
    {
        std::stringstream ss;
        ss << "bind to " << port << ": " << errno;
        ss << " (" << strerror(errno) << ")";
        return {false, ss.str()};
    }

    return {true, ""};
}

// Finds the socket address given an ip and a port.
// Returns whether it succeeds or not and the error message.
std::pair<bool, std::string> FindSocketAddr(const char* const ip,
                                            const char* const port,
                                            sockaddr_storage* const addr)
{
    addrinfo* result_list{nullptr};
    addrinfo hints{};
    hints.ai_family=AF_INET;
    hints.ai_socktype=SOCK_DGRAM;
    int result=getaddrinfo(ip, port, &hints, &result_list);

    if (result)
    {
        std::stringstream ss;
        ss << "getaddrinfo: " << result;
        ss << " (" << gai_strerror(result) << ") ";
        return {false, ss.str()};
    }

    memcpy(addr, result_list->ai_addr, result_list->ai_addrlen);
    freeaddrinfo(result_list);

    return {true, ""};
}

// Sends a string of bytes to the given destination address.
// Returns the number of sent bytes and, if -1, the error message.
std::pair<int, std::string> SendTo(const int sockfd,
                                   sockaddr_storage& dest_addr,
                                   const std::vector<unsigned char>& message)
{
    const socklen_t addr_len{sizeof(dest_addr)};
    int result=sendto(sockfd, message.data(), message.size(), 0,
                        reinterpret_cast<sockaddr*>(&dest_addr), addr_len);

    if (result == -1)
    {
        std::stringstream ss;
        ss << "sendto: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }

    return {result, ""};
}

// Receives a text response from the given destination address.
// Returns the number of received bytes and, if -1, the error message.
std::pair<int, std::string> ReceiveFrom(const int sockfd,
                                        sockaddr_storage& addr,
                                        std::vector<unsigned char>& buffer,
                                        const int buffer_size=1024,
                                        const int flags=MSG_DONTWAIT)
{
    socklen_t addr_len{sizeof(addr)};
    buffer.resize(buffer_size, '\0');
    // MSG_DONTWAIT -> Non-blocking
    // recvfrom is storing (re-populating) the sender address in addr.
    int result=recvfrom(sockfd, buffer.data(), buffer_size, flags,
                          reinterpret_cast<sockaddr*>(&addr), &addr_len);
    if (result==-1)
    {
        std::stringstream ss;
        ss << "recvfrom: " << errno;
        ss << " (" << strerror(errno) << ")";
        return {-1, ss.str()};
    }

    return {result, ""};
}


WAITello::WAITello():m_hdl_it(m_hdl_node)
{
    m_command_sockfd=socket(AF_INET, SOCK_DGRAM, 0);
    m_state_sockfd=socket(AF_INET, SOCK_DGRAM, 0);

    // Init paramters
    s_nodename=ros::this_node::getName();
    m_hdl_node.getParam(s_nodename+"/"+"F_NODE_SAMPLE_FREQUENCY",F_NODE_SAMPLE_FREQUENCY);
    m_hdl_node.getParam(s_nodename+"/"+"F_COMMAND_SCALE",F_COMMAND_SCALE);
    m_hdl_node.getParam(s_nodename+"/"+"F_COMMAND_KEYBOARD_SPEED",F_COMMAND_KEYBOARD_SPEED);

    // Init publishers and subscribers
    sub_twi_command_velocity=m_hdl_node.subscribe("cmd_vel",1,&WAITello::cb_sub_twi_command_velocity,this);
    sub_emp_command_land=m_hdl_node.subscribe("emergency",1,&WAITello::cb_sub_emp_command_emergency,this);
    sub_emp_command_land=m_hdl_node.subscribe("land",1,&WAITello::cb_sub_emp_command_land,this);
    sub_emp_command_takeoff=m_hdl_node.subscribe("takeoff",1,&WAITello::cb_sub_emp_command_takeoff,this);
    pub_img_camera_tello_mono_rgb=m_hdl_it.advertise("camera_rgb_front/image_raw",1);

    // Init helper members
    i_tello_command_rc_x=0;
    i_tello_command_rc_y=0;
    i_tello_command_rc_z=0;
    i_tello_command_rc_yaw=0;

    // Initialize
    if (!Bind())
    {
        ROS_ERROR("Tello: Bind failed!");
        return;
    }
    SendCommand("setspeed 100");
    while(!(ReceiveResponse()));
    SendCommand("setfps high");
    while(!(ReceiveResponse()));
    SendCommand("setbitrate 1");
    while(!(ReceiveResponse()));
    SendCommand("setresolution low");
    while(!(ReceiveResponse()));
    //SendCommand("downvision 1");
    //while(!(ReceiveResponse()));
    SendCommand("streamon");
    while(!(ReceiveResponse()));

    // Initialize Keyboard inputs via X11 library
    dsp_x11_display=XOpenDisplay(NULL); // NULL is autoselect valid display
}

WAITello::~WAITello()
{
    close(m_command_sockfd);
    close(m_state_sockfd);
    XCloseDisplay(dsp_x11_display);
}

// Receive remote ROS commands
void WAITello::cb_sub_twi_command_velocity(const geometry_msgs::Twist::ConstPtr& msg)
{
    msg_twi_command_velocity=(*msg);
}
void WAITello::cb_sub_emp_command_emergency(const std_msgs::Empty::ConstPtr& msg)
{
    Emergency();
}
void WAITello::cb_sub_emp_command_land(const std_msgs::Empty::ConstPtr& msg)
{
    Land();
}
void WAITello::cb_sub_emp_command_takeoff(const std_msgs::Empty::ConstPtr& msg)
{
    Takeoff();
}

bool WAITello::Bind(const int local_client_command_port)
{
    // UDP Client to send commands and receive responses
    auto result =
        ::BindSocketToPort(m_command_sockfd, local_client_command_port);
    if (!result.first)
    {
        ROS_ERROR_STREAM(result.second);
        return false;
    }
    m_local_client_command_port=local_client_command_port;
    result=::FindSocketAddr(TELLO_SERVER_IP,TELLO_SERVER_COMMAND_PORT,&m_tello_server_command_addr);
    if(!result.first)
    {
        ROS_ERROR_STREAM(result.second);
        return false;
    }

    // Local UDP Server to listen for the Tello Status
    result=::BindSocketToPort(m_state_sockfd,LOCAL_SERVER_STATE_PORT);
    if(!result.first)
    {
        ROS_ERROR_STREAM(result.second);
        return false;
    }

    // Finding Tello
    ROS_INFO_STREAM("Finding Tello ...");
    FindTello();
    ROS_INFO_STREAM("Entered SDK mode");
    ShowTelloInfo();

    return true;
}

bool WAITello::SendCommand(const std::string& command)
{
    const std::vector<unsigned char> message{std::cbegin(command),
                                             std::cend(command)};
    const auto result =
        ::SendTo(m_command_sockfd, m_tello_server_command_addr, message);
    const int bytes{result.first};
    if (bytes == -1)
    {
        ROS_ERROR_STREAM(result.second);
        return false;
    }
    ROS_DEBUG_STREAM("127.0.0.1:" << m_local_client_command_port
                           << " >>>> " << bytes << " bytes >>>> "
                           << TELLO_SERVER_IP << ":" << TELLO_SERVER_COMMAND_PORT << ": " << command);
    return true;
}

std::optional<std::string> WAITello::ReceiveResponse()
{
    const int size{32};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result=::ReceiveFrom(m_command_sockfd, m_tello_server_command_addr, buffer, size);
    const int bytes{result.first};
    if (bytes < 1)
    {
        return {};
    }
    std::string response{buffer.cbegin(), buffer.cbegin() + bytes};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);
    ROS_DEBUG_STREAM("127.0.0.1:"<< m_local_client_command_port <<" <<<< " << bytes << " bytes <<<< " << TELLO_SERVER_IP << ":" << TELLO_SERVER_COMMAND_PORT << ":" << response);
    return response;
}

std::optional<std::string> WAITello::GetState()
{
    sockaddr_storage addr;
    const int size{1024};
    std::vector<unsigned char> buffer(size, '\0');
    const auto result=::ReceiveFrom(m_state_sockfd, addr, buffer, size);
    const int bytes{result.first};
    if(bytes<1)
    {
        return {};
    }
    std::string response{std::cbegin(buffer), std::cbegin(buffer) + bytes};
    // Some responses contain trailing white spaces.
    response.erase(response.find_last_not_of(" \n\r\t") + 1);
    ROS_DEBUG_STREAM("127.0.0.1:" << m_local_client_command_port << " <<<< " << bytes << " bytes <<<< " << TELLO_SERVER_IP << ":" << TELLO_SERVER_COMMAND_PORT << ": <state>");
    return response;
}

void WAITello::FindTello()
{
    do
    {
        SendCommand("command");
        sleep(1);
    }while(!(ReceiveResponse()));
}

void WAITello::ShowTelloInfo()
{
    std::optional<std::string> response;

    SendCommand("sn?");
    response=ReceiveResponse();
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Serial Number: " << *response);

    SendCommand("sdk?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Tello SDK: " << *response);

    SendCommand("time?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Motor Running time: " << *response);

    SendCommand("hardware?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Hardware: " << *response);

    SendCommand("battery?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Battery: " << *response);

    SendCommand("wifi?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("WIFI SNR: " << *response);

    SendCommand("speed?");
    while(!(response=ReceiveResponse()));
    ROS_INFO_STREAM("Speed: " << *response);
}

// KEYBOARD inputs
bool WAITello::KeyIsPressed(KeySym ks)
{
    char keys_return[32];
    XQueryKeymap(dsp_x11_display, keys_return);
    KeyCode kc2=XKeysymToKeycode(dsp_x11_display, ks);
    bool isPressed= !!(keys_return[kc2 >> 3] & (1 << (kc2 & 7)));
    return isPressed;
}
void WAITello::TelloCheckTriggers()
{
    bool b_keyboard_modifier=KeyIsPressed(XK_Tab);

    if(KeyIsPressed(XK_Escape))
    {
        Emergency();
    }
    else if(KeyIsPressed(XK_l))
    {
        Land();
    }
    else if(KeyIsPressed(XK_t))
    {
        Takeoff();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Shift_L))
    {
        FlipLeft();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_Shift_R))
    {
        FlipRight();
    }
    else if(b_keyboard_modifier && KeyIsPressed(XK_b))
    {
        FlipBack();
    }
    else if(KeyIsPressed(XK_w))
    {
        msg_twi_command_velocity.linear.x=F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_s))
    {
        msg_twi_command_velocity.linear.x=-F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_a))
    {
        msg_twi_command_velocity.linear.y=-F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_d))
    {
        msg_twi_command_velocity.linear.y=F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_Down))
    {
        msg_twi_command_velocity.linear.z=-F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_Up))
    {
        msg_twi_command_velocity.linear.z=F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_Left))
    {
        msg_twi_command_velocity.angular.z=-F_COMMAND_KEYBOARD_SPEED;
    }
    else if(KeyIsPressed(XK_Right))
    {
        msg_twi_command_velocity.angular.z=F_COMMAND_KEYBOARD_SPEED;
    }
    else
    {
        msg_twi_command_velocity.linear.x=0.0;
        msg_twi_command_velocity.linear.y=0.0;
        msg_twi_command_velocity.linear.z=0.0;
        msg_twi_command_velocity.angular.z=0.0;
    }
}

// MOTION commands
void WAITello::Emergency()
{
    SendCommand("emergency");
    while(!(ReceiveResponse()));
    ROS_ERROR_STREAM("EMERGENCY!");
}

void WAITello::Land()
{
    SendCommand("land");
    while(!(ReceiveResponse()));
    ROS_WARN_STREAM("LAND!");
}

void WAITello::Takeoff()
{
    // Aufgabe [1]:
    // Ergänze hier den richtigen Befehl für TAKEOFF:
    // ...? SendCommand("takeoff");
    while(!(ReceiveResponse()));
    ROS_WARN_STREAM("TAKEOFF!");
}

void WAITello::TelloCommandSpeed()
{
    // Aufgabe [2]:
    // Ergänze hier die richtigen STEUER-Befehle:
    i_tello_command_rc_x=// ...? int(msg_twi_command_velocity.linear.x*F_COMMAND_SCALE);
    i_tello_command_rc_y=// ...? int(msg_twi_command_velocity.linear.y*F_COMMAND_SCALE);
    i_tello_command_rc_z=int(msg_twi_command_velocity.linear.z*F_COMMAND_SCALE);
    i_tello_command_rc_yaw=int(msg_twi_command_velocity.angular.z*F_COMMAND_SCALE);

    std::stringstream sst_tello_command; // x and y axis are translated into roll and pitch!
    sst_tello_command << "rc "
                         << i_tello_command_rc_y << " "
                         << i_tello_command_rc_x << " "
                         << i_tello_command_rc_z << " "
                         << i_tello_command_rc_yaw;
    SendCommand(sst_tello_command.str());
}

void WAITello::FlipLeft()
{
    SendCommand("flip b");
    while(!(ReceiveResponse()));
    ROS_WARN("SIDEFLIP LEFT!");
}

void WAITello::FlipRight()
{
    SendCommand("flip r");
    while(!(ReceiveResponse()));
    ROS_WARN("SIDEFLIP LEFT!");
}

void WAITello::FlipBack()
{
    // Aufgabe [3]:
    // Ergänze hier den richtigen Befehl für BACKFLIP:
    // ...? SendCommand("flip b");
    while(!(ReceiveResponse()));
    ROS_WARN("BACKFLIP!");
}

void WAITello::TelloCommandFlyDelta(int i_delta)
{
    // Aufgabe [4]:
    // Ergänze hier den richtigen Befehl für FLY LEFT:
    // ...? SendCommand("left "+std::to_string(i_delta));
    // SendCommand("right "+std::to_string(i_delta));


    /*
    // Command: Up, Down, Left, Right, Forward, Back, CW, CCW
    i_tello_command_rc_x=int(msg_twi_command_velocity.linear.x*F_COMMAND_SCALE);
    i_tello_command_rc_y=int(msg_twi_command_velocity.linear.y*F_COMMAND_SCALE);
    i_tello_command_rc_z=int(msg_twi_command_velocity.linear.z*F_COMMAND_SCALE);
    i_tello_command_rc_yaw=int(msg_twi_command_velocity.angular.z*F_COMMAND_SCALE);
    //std::stringstream sst_debug;
    //sst_debug << "commands: " << i_tello_command_rc_x << " " << i_tello_command_rc_y << " " << i_tello_command_rc_z << " " << i_tello_command_rc_yaw;
    //ROS_WARN_STREAM(sst_debug.str());

    if(i_tello_command_rc_x<0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "back " << i_tello_command_rc_x;
        SendCommand(sst_tello_command.str());
    }
    else if(i_tello_command_rc_x>=0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "forward " << i_tello_command_rc_x;
        SendCommand(sst_tello_command.str());
    }
    else
    {
    }

    if(i_tello_command_rc_y<0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "left " << i_tello_command_rc_y;
        SendCommand(sst_tello_command.str());
    }
    else if(i_tello_command_rc_y>=0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "right " << i_tello_command_rc_y;
        SendCommand(sst_tello_command.str());
    }
    else
    {
    }

    if(i_tello_command_rc_z<0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "down " << i_tello_command_rc_z;
        SendCommand(sst_tello_command.str());
    }
    else if(i_tello_command_rc_z>=0)
    {
        std::stringstream sst_tello_command;
        sst_tello_command << "up " << i_tello_command_rc_z;
        SendCommand(sst_tello_command.str());
    }
    else
    {
    }
    */
}

void WAITello::TelloPublishCameraRGBStream()
{
    /*
    cv::resize(frame, frame, cv::Size(), 0.5, 0.5);
    cv::imshow("Stream", mat_img_tello_camera_rgb_front);
    cv::waitKey(1);
    */
    pub_img_camera_tello_mono_rgb.publish(msg_mat_img_tello_camera_rgb_front);
}

void WAITello::run()
{
    ros::Rate loop_rate(F_NODE_SAMPLE_FREQUENCY);

    // Initialize multithreaded videostream
    // --> Initialize after object was instatiated!
    std::thread thread_tello_videostream(cb_thread_tello_videostream);

    while(ros::ok())
    {
        TelloCheckTriggers();

        //TelloCommandFlyDelta();
        TelloCommandSpeed();

        // Publish camera images
        TelloPublishCameraRGBStream();

        // Do the good old spinning stuff
        ros::spinOnce();
        loop_rate.sleep();
    }
}
