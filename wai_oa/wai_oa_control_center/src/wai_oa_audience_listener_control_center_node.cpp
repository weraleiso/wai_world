/////////////////////////////////////////////////
/// Literature and other sources
/////////////////////////////////////////////////
// ...



/////////////////////////////////////////////////
/// Include Header of Node Implementation
/////////////////////////////////////////////////
#include<wai_oa_audience_listener.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    WAIOAAudienceListener* wai_oa_audience_listener=WAIOAAudienceListener::getInstance();

    QApplication qap_qapplication_audience(argc, argv);

    ROS_INFO("Node with name \"%s\" Initialized. About to call run()...",ros::this_node::getName().c_str());
    wai_oa_audience_listener->run();

    qap_qapplication_audience.quit();
    ROS_INFO("Exiting...");

    return 0;
}
