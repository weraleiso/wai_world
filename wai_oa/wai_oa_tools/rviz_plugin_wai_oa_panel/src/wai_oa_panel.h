#ifndef WAI_OA_PANEL_H
#define WAI_OA_PANEL_H

#ifndef Q_MOC_RUN
    #include<ros/ros.h>
    #include<ros/package.h>
    #include<std_msgs/String.h>
    #include<rviz/panel.h>
#endif

#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
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
#include<chrono>

#include<QPainter>
#include<QLineEdit>
#include<QVBoxLayout>
#include<QHBoxLayout>
#include<QLabel>
#include<QTimer>
#include<QPushButton>
#include<QInputDialog>
#include<QMessageBox>
#include<QTabWidget>
#include<QTextBrowser>



namespace rviz_plugin_wai_oa_panel
{

    class RVizPluginWAIOAPanel:public rviz::Panel
    {
        // This class uses Qt slots and is a subclass of QObject, so it needs
        // the Q_OBJECT macro.
        Q_OBJECT

    public:
        // QWidget subclass constructors usually take a parent widget
        // parameter (which usually defaults to 0).  At the same time,
        // pluginlib::ClassLoader creates instances by calling the default
        // constructor (with no arguments).  Taking the parameter and giving
        // a default of 0 lets the default constructor work and also lets
        // someone using the class for something else to pass in a parent
        // widget as they normally would with Qt.
        RVizPluginWAIOAPanel(QWidget* parent=0);

        // Now we declare overrides of rviz::Panel functions for saving and
        // loading data from the config file.  Here the data is the
        // topic name.
        virtual void load( const rviz::Config& config );
        virtual void save( rviz::Config config ) const;

        // Next come a couple of public Qt slots.
    public Q_SLOTS:
        // The control area, DriveWidget, sends its output to a Qt signal
        // for ease of re-use, so here we declare a Qt slot to receive it.
        //void setVel( float linear_velocity_, float angular_velocity_ );

        // In this example setTopic() does not get connected to any signal
        // (it is called directly), but it is easy to define it as a public
        // slot instead of a private function in case it would be useful to
        // some other user.
        //void setTopic( const QString& topic );

        // Here we declare some internal slots.
    protected Q_SLOTS:
        // sendvel() publishes the current velocity values to a ROS
        // topic.  Internally this is connected to a timer which calls it 10
        // times per second.
        //void sendVel();

        // ConfirmEval() reads the topic name from the QLineEdit and calls
        // setTopic() with the result.
        void ResetSessionLog();
        void ResetRepSequenceLog();
        void ShowAbout();

        // Then we finish up with protected member variables.
    protected:

        // QT Widgets
        QTabWidget *m_tab_oa_panel;

        QWidget* m_wid_session_log;
        QLabel* m_lbl_session_log;
        QTextBrowser* m_txb_session_log;
        QPushButton* m_cmd_reset_session_log;

        QWidget* m_wid_rep_sequence_log;
        QLabel* m_lbl_rep_sequence_log;
        QTextBrowser* m_txb_rep_sequence_log;
        QPushButton* m_cmd_reset_rep_sequence_log;

        QPushButton* m_cmd_show_about;

        // Other helper members
        std::string m_s_session_log;
        std::string m_s_rep_sequence;

        // ROS helper members
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;

        // Subscribers and Publishers
        ros::Subscriber m_sub_s_session_log;
        ros::Subscriber m_sub_s_rep_sequence_log;

        // Callbacks
        void cb_sub_s_session_log(const std_msgs::HeaderPtr&);
        void cb_sub_s_rep_sequence_log(const std_msgs::StringPtr&);
    };

} // end namespace rviz_plugin_wai_auditorium

#endif // WAI_OA_PANEL_H
