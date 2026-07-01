/////////////////////////////////////////////////
/// Include Header of Node Implementation
/////////////////////////////////////////////////
#include<wai_oa.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    WAIOpenAuditorium* wai_oa=WAIOpenAuditorium::getInstance();
    QApplication qap_qapplication(argc,argv);
    ROS_INFO("Node with name \"%s\" Initialized. About to call run()...",ros::this_node::getName().c_str());
    wai_oa->run();
    qap_qapplication.quit();
    ROS_INFO("Exiting...");

    return 0;
}
