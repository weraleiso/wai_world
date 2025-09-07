#include "wai_oa_panel.h"



namespace rviz_plugin_wai_oa_panel
{
    RVizPluginWAIOAPanel::RVizPluginWAIOAPanel(QWidget* parent):rviz::Panel(parent),it_(nh_)
    {
        QVBoxLayout* lay_tab_oa_panel=new QVBoxLayout;
        m_tab_oa_panel=new QTabWidget(parent);
        m_cmd_show_about=new QPushButton("About!");
        lay_tab_oa_panel->addWidget(m_tab_oa_panel);
        lay_tab_oa_panel->addWidget(m_cmd_show_about);
        this->setLayout(lay_tab_oa_panel);

        m_wid_session_log=new QWidget(m_tab_oa_panel);
        m_wid_rep_sequence_log=new QWidget(m_tab_oa_panel);

        QVBoxLayout* lay_session_log=new QVBoxLayout;
        m_lbl_session_log=new QLabel("Session");
        m_txb_session_log=new QTextBrowser();
        m_txb_session_log->setReadOnly(true);
        m_txb_session_log->setOpenExternalLinks(true);
        m_txb_session_log->setTextInteractionFlags(Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
        m_cmd_reset_session_log=new QPushButton("Reset Log!");
        lay_session_log->addWidget(m_lbl_session_log);
        lay_session_log->addWidget(m_txb_session_log);
        lay_session_log->addWidget(m_cmd_reset_session_log);
        m_wid_session_log->setLayout(lay_session_log);

        // Rep Sequencer Log is text-editable!
        QVBoxLayout* lay_rep_sequence_log=new QVBoxLayout;
        m_lbl_rep_sequence_log=new QLabel("Rep Sequence");
        m_txb_rep_sequence_log=new QTextBrowser();
        m_txb_rep_sequence_log->setReadOnly(false);
        m_txb_rep_sequence_log->setOpenExternalLinks(true);
        m_txb_rep_sequence_log->setTextInteractionFlags(Qt::TextEditable | Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
        m_cmd_reset_rep_sequence_log=new QPushButton("Reset Log!");
        lay_rep_sequence_log->addWidget(m_lbl_rep_sequence_log);
        lay_rep_sequence_log->addWidget(m_txb_rep_sequence_log);
        lay_rep_sequence_log->addWidget(m_cmd_reset_rep_sequence_log);
        m_wid_rep_sequence_log->setLayout(lay_rep_sequence_log);

        m_tab_oa_panel->addTab(m_wid_session_log,"SESSION Log");
        m_tab_oa_panel->addTab(m_wid_rep_sequence_log, "REP SEQUENCE Log");

        connect(m_cmd_reset_session_log,SIGNAL(clicked()),this,SLOT(ResetSessionLog()));
        connect(m_cmd_reset_rep_sequence_log,SIGNAL(clicked()),this,SLOT(ResetRepSequenceLog()));
        connect(m_cmd_show_about,SIGNAL(clicked()),this,SLOT(ShowAbout()));

        m_sub_s_session_log=nh_.subscribe("/wai_world/oa/audience_request_to_panel",1,&RVizPluginWAIOAPanel::cb_sub_s_session_log,this);
        m_sub_s_rep_sequence_log=nh_.subscribe("/wai_world/oa/rep_sequence",1,&RVizPluginWAIOAPanel::cb_sub_s_rep_sequence_log,this);
        m_s_rep_sequence="";

        ResetSessionLog();
        ResetRepSequenceLog();
    }

    void RVizPluginWAIOAPanel::ResetSessionLog()
    {
        m_s_session_log="";
        m_txb_session_log->clear();
        m_txb_session_log->setText("<a></a>Log messages from session...");
    }
    void RVizPluginWAIOAPanel::ResetRepSequenceLog()
    {
        m_s_rep_sequence="";
        m_txb_rep_sequence_log->clear();
        m_txb_rep_sequence_log->setText("<a></a>Log messages from sequencer...");
    }

    void RVizPluginWAIOAPanel::ShowAbout()
    {
        QMessageBox::about(this,"About","OA Rviz panel plugin.\n(©2019, W. A. Isop)");
    }

    // Read the topic name from the QLineEdit and call setTopic() with the
    // results.  This is connected t QLineEdit::editingFinished() which
    // fires when the user presses Enter or Tab or otherwise moves focus
    // away.
    void RVizPluginWAIOAPanel::cb_sub_s_session_log(const std_msgs::HeaderPtr& msg)
    {
        std::stringstream sst_time;
        std::time_t t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm ltime;
        localtime_r(&t,&ltime);
        sst_time << std::put_time(&ltime,"%H:%M:%S");

        std::string s_current_msg="<b>["+sst_time.str()+"]</b> "+(*msg).frame_id+"<br>";
        m_s_session_log+=s_current_msg;
        m_txb_session_log->setText(QString::fromStdString(m_s_session_log));
    }
    void RVizPluginWAIOAPanel::cb_sub_s_rep_sequence_log(const std_msgs::StringPtr& msg)
    {
        m_s_rep_sequence+=(*msg).data;
        m_txb_rep_sequence_log->setText(QString::fromStdString(m_s_rep_sequence));
    }

    // Save all configuration data from this panel to the given
    // Config object.  It is important here that you call save()
    // on the parent class so the class id and panel name get saved.
    void RVizPluginWAIOAPanel::save( rviz::Config config ) const
    {
        rviz::Panel::save( config );
        //config.mapSetValue( "Topic", output_topic_ );
    }

    // Load all configuration data for this panel from the given Config object.
    void RVizPluginWAIOAPanel::load( const rviz::Config& config )
    {
        rviz::Panel::load( config );
    }

} // end namespace rviz_plugin_wai_oa_panel

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_oa_panel::RVizPluginWAIOAPanel,rviz::Panel)
// END_TUTORIAL
