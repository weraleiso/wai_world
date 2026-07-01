#include<QApplication>

#include<wai_sketch.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    WAISketch wai_sketch;
    QApplication qap_qapplication(argc,argv);
    ROS_INFO("Initialized. About to run...");
    wai_sketch.run();
    qap_qapplication.quit();
    ROS_INFO("Exiting...");
    return 0;
}
