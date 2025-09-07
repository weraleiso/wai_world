#include<wai_screenshare.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    WAIScreenshare wai_screenshare;
    ROS_INFO("Initialized. About to run...");
    wai_screenshare.run();
    ROS_INFO("Exiting...");
    return 0;
}
