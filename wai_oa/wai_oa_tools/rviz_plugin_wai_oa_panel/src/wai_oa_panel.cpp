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
        m_wid_ethics_log=new QWidget(m_tab_oa_panel);

        // SESSION LOG Tab
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

        // LEARNING (REP)-SEQUENCER Tab
        QVBoxLayout* lay_rep_sequence_log=new QVBoxLayout;
        m_lbl_rep_sequence_log=new QLabel("Rep Sequence");
        m_txb_rep_sequence_log=new QTextBrowser();
        m_txb_rep_sequence_log->setReadOnly(false); // Rep Sequencer Log is text-editable!
        m_txb_rep_sequence_log->setOpenExternalLinks(true);
        m_txb_rep_sequence_log->setTextInteractionFlags(Qt::TextEditable | Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);
        m_cmd_reset_rep_sequence_log=new QPushButton("Reset Log!");
        lay_rep_sequence_log->addWidget(m_lbl_rep_sequence_log);
        lay_rep_sequence_log->addWidget(m_txb_rep_sequence_log);
        lay_rep_sequence_log->addWidget(m_cmd_reset_rep_sequence_log);
        m_wid_rep_sequence_log->setLayout(lay_rep_sequence_log);



        // ETHICS Tab
        QVBoxLayout* lay_ethics_log=new QVBoxLayout;
        m_lbl_ethics_log=new QLabel("<b>ETHICS OVERVIEW</b>");

        m_lbl_ethics_indicator_log=new QLabel("Ethical <b>Interactivity</b> (Overall Indicator)");
        m_stb_ethics_log=new QProgressBar();
        m_stb_ethics_log->setValue(33.0);

        /* Ethical Properties Summary
        TCoh ... Time coherence
        DLis ... Distance from listener
        VDev ... View deviation from listener
        Mood ... Mood of actor
        PVis ... Visual Presence
        PAud ... Audio Presence
        */
        m_lbl_ethics_properties_log=new QLabel("Ethical <b>Properties</b>");
        m_qtw_ethics=new QTreeWidget();
        m_qtw_ethics->setColumnCount(2);
        QHeaderView* header=m_qtw_ethics->header();
        header->setSectionResizeMode(0,QHeaderView::Stretch);
        header->setSectionResizeMode(1,QHeaderView::Stretch);
        m_qtw_ethics->setHeaderLabels(QStringList() << "Ethical Property" << "Value (normed)");
        int i_num_pres=1;
        int i_num_aud=10;
        int i_num_ais=1;
        QTreeWidgetItem* twi_pres_top=new QTreeWidgetItem(m_qtw_ethics);
        twi_pres_top->setText(0,QString("PRESENTERS"));
        QFont font=twi_pres_top->font(0); font.setBold(true); twi_pres_top->setFont(0,font);
        for(int i=0;i<i_num_pres;i++)
        {
            QTreeWidgetItem* twi_pres=new QTreeWidgetItem(twi_pres_top);
            twi_pres->setText(0,QString("Presenter %1").arg(i));

            QTreeWidgetItem* twi_prp_tcoh=new QTreeWidgetItem(twi_pres);
            twi_prp_tcoh->setText(0,QString("TCoh"));
            twi_prp_tcoh->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_dlis=new QTreeWidgetItem(twi_pres);
            twi_prp_dlis->setText(0,QString("DLis"));
            twi_prp_dlis->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_vdev=new QTreeWidgetItem(twi_pres);
            twi_prp_vdev->setText(0,QString("VDev"));
            twi_prp_vdev->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_mood=new QTreeWidgetItem(twi_pres);
            twi_prp_mood->setText(0,QString("Mood"));
            twi_prp_mood->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_pvis=new QTreeWidgetItem(twi_pres);
            twi_prp_pvis->setText(0,QString("PVis"));
            twi_prp_pvis->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_paud=new QTreeWidgetItem(twi_pres);
            twi_prp_paud->setText(0,QString("PAud"));
            twi_prp_paud->setText(1,QString("0.0"));
        }
        QTreeWidgetItem* twi_aud_top=new QTreeWidgetItem(m_qtw_ethics);
        twi_aud_top->setText(0,QString("AUDIENCE"));
        font=twi_aud_top->font(0); font.setBold(true); twi_aud_top->setFont(0,font);
        for(int j=0;j<i_num_aud;j++)
        {
            QTreeWidgetItem* twi_aud=new QTreeWidgetItem(twi_aud_top);
            twi_aud->setText(0,QString("Listener %1").arg(j));

            QTreeWidgetItem* twi_prp_tcoh=new QTreeWidgetItem(twi_aud);
            twi_prp_tcoh->setText(0,QString("TCoh"));
            twi_prp_tcoh->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_dpre=new QTreeWidgetItem(twi_aud);
            twi_prp_dpre->setText(0,QString("DPre"));
            twi_prp_dpre->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_vdev=new QTreeWidgetItem(twi_aud);
            twi_prp_vdev->setText(0,QString("VDev"));
            twi_prp_vdev->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_mood=new QTreeWidgetItem(twi_aud);
            twi_prp_mood->setText(0,QString("Mood"));
            twi_prp_mood->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_prev=new QTreeWidgetItem(twi_aud);
            twi_prp_prev->setText(0,QString("PreV"));
            twi_prp_prev->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_paud=new QTreeWidgetItem(twi_aud);
            twi_prp_paud->setText(0,QString("PAud"));
            twi_prp_paud->setText(1,QString("0.0"));
        }
        QTreeWidgetItem* twi_ais_top=new QTreeWidgetItem(m_qtw_ethics);
        twi_ais_top->setText(0,QString("AIS"));
        font=twi_ais_top->font(0); font.setBold(true); twi_ais_top->setFont(0,font);
        for(int k=0;k<i_num_ais;k++)
        {
            QTreeWidgetItem* twi_ais=new QTreeWidgetItem(twi_ais_top);
            twi_ais->setText(0,QString("AIS %1").arg(k));

            QTreeWidgetItem* twi_prp_dlis=new QTreeWidgetItem(twi_ais);
            twi_prp_dlis->setText(0,QString("DPre"));
            twi_prp_dlis->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_vdev=new QTreeWidgetItem(twi_ais);
            twi_prp_vdev->setText(0,QString("VDev"));
            twi_prp_vdev->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_mood=new QTreeWidgetItem(twi_ais);
            twi_prp_mood->setText(0,QString("Mood"));
            twi_prp_mood->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_pvis=new QTreeWidgetItem(twi_ais);
            twi_prp_pvis->setText(0,QString("PVis"));
            twi_prp_pvis->setText(1,QString("0.0"));
            QTreeWidgetItem* twi_prp_paud=new QTreeWidgetItem(twi_ais);
            twi_prp_paud->setText(0,QString("PAud"));
            twi_prp_paud->setText(1,QString("0.0"));
        }
        m_qtw_ethics->expandAll();

        m_lbl_ethics_decisions_log=new QLabel("Ethical <b>Decisions</b>");
        m_txb_ethics_decisions_log=new QTextBrowser();
        m_txb_ethics_decisions_log->setReadOnly(false); // ETHICS Log is text-editable!
        m_txb_ethics_decisions_log->setOpenExternalLinks(true);
        m_txb_ethics_decisions_log->setTextInteractionFlags(Qt::TextEditable | Qt::TextSelectableByMouse | Qt::LinksAccessibleByMouse);

        m_cmd_reset_ethics_log=new QPushButton("Reset Log!");
        lay_ethics_log->addWidget(m_lbl_ethics_log);
        lay_ethics_log->addWidget(m_lbl_ethics_indicator_log);
        lay_ethics_log->addWidget(m_stb_ethics_log);
        lay_ethics_log->addWidget(m_lbl_ethics_properties_log);
        //lay_ethics_log->addWidget(m_txb_ethics_properties_log);
        lay_ethics_log->addWidget(m_qtw_ethics);
        lay_ethics_log->addWidget(m_lbl_ethics_decisions_log);
        lay_ethics_log->addWidget(m_txb_ethics_decisions_log);
        lay_ethics_log->addWidget(m_cmd_reset_ethics_log);
        m_wid_ethics_log->setLayout(lay_ethics_log);



        m_tab_oa_panel->addTab(m_wid_session_log,"SESSION Log");
        m_tab_oa_panel->addTab(m_wid_rep_sequence_log, "REP SEQUENCE Log");
        m_tab_oa_panel->addTab(m_wid_ethics_log, "ETHICS Log");

        connect(m_cmd_reset_session_log,SIGNAL(clicked()),this,SLOT(ResetSessionLog()));
        connect(m_cmd_reset_rep_sequence_log,SIGNAL(clicked()),this,SLOT(ResetRepSequenceLog()));
        connect(m_cmd_reset_ethics_log,SIGNAL(clicked()),this,SLOT(ResetEthicsLog()));
        connect(m_cmd_show_about,SIGNAL(clicked()),this,SLOT(ShowAbout()));

        m_sub_s_session_log=nh_.subscribe("/wai_world/oa/audience_request_to_panel",1,&RVizPluginWAIOAPanel::cb_sub_s_session_log,this);
        m_sub_s_rep_sequence_log=nh_.subscribe("/wai_world/oa/rep_sequence",1,&RVizPluginWAIOAPanel::cb_sub_s_rep_sequence_log,this);
        m_sub_s_ethics_log=nh_.subscribe("/wai_world/oa/ethics",1,&RVizPluginWAIOAPanel::cb_sub_s_ethics_log,this);
        m_sub_s_ethics_properties=nh_.subscribe("/wai_world/oa/ethical_properties_summary",1,&RVizPluginWAIOAPanel::cb_sub_f32_ethics_properties,this);

        m_s_session_log="";
        m_s_rep_sequence="";
        m_s_ethics="";

        ResetSessionLog();
        ResetRepSequenceLog();
        ResetEthicsLog();
    }

    void RVizPluginWAIOAPanel::ResetSessionLog()
    {
        m_s_session_log="";
        m_txb_session_log->clear();
        m_txb_session_log->setText("<a></a>SESSION Log Messages...");
    }
    void RVizPluginWAIOAPanel::ResetRepSequenceLog()
    {
        m_s_rep_sequence="";
        m_txb_rep_sequence_log->clear();
        m_txb_rep_sequence_log->setText("<a></a>LEARNING SEQUENCER Log Messages...");
    }
    void RVizPluginWAIOAPanel::ResetEthicsLog()
    {
        m_s_ethics="";
        m_txb_ethics_decisions_log->clear();
        m_txb_ethics_decisions_log->setText("<a></a>ETHICS Log Messages...");
    }
    void RVizPluginWAIOAPanel::ShowAbout()
    {
        QMessageBox::about(this,"About","OPEN AUDITORIUM Panel (RViz Plugin).\n(©2019, W. A. Isop)");
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
    void RVizPluginWAIOAPanel::cb_sub_s_ethics_log(const std_msgs::StringPtr& msg)
    {
        m_s_ethics+=(*msg).data;
        m_txb_ethics_decisions_log->setText(QString::fromStdString(m_s_ethics));
    }
    void RVizPluginWAIOAPanel::cb_sub_f32_ethics_properties(std_msgs::Float32MultiArray msg_properties)
    {
        int i_num_elements_header=6;
        int i_num_presenters=msg_properties.data[0];
        int i_num_prp_presenters=msg_properties.data[1];
        int i_num_audience=msg_properties.data[2];
        int i_num_prp_audience=msg_properties.data[3];
        int i_num_ais=msg_properties.data[4];
        int i_num_prp_ais=msg_properties.data[5];

        // Update PRESENTERS
        int i_offset_actors=0;
        QTreeWidgetItem* twi_presenters_top=m_qtw_ethics->topLevelItem(0);
        for(int i=0;i<i_num_presenters;i++)
        {
            QTreeWidgetItem* twi_presenter=twi_presenters_top->child(i);
            for(int j=0;j<i_num_prp_presenters;j++)
            {
                QTreeWidgetItem* twi_property=twi_presenter->child(j);
                twi_property->setText(1,QString::number(msg_properties.data[i_num_elements_header+i_num_prp_presenters*i+j]));
            }
        }
        // Update AUDIENCE
        i_offset_actors+=i_num_prp_presenters*i_num_presenters;
        QTreeWidgetItem* twi_audience_top=m_qtw_ethics->topLevelItem(1);
        for(int i=0;i<i_num_audience;i++)
        {
            QTreeWidgetItem* twi_audience=twi_audience_top->child(i);
            for(int j=0;j<i_num_prp_audience;j++)
            {
                QTreeWidgetItem* twi_property=twi_audience->child(j);
                twi_property->setText(1,QString::number(msg_properties.data[i_offset_actors+i_num_elements_header+i_num_prp_audience*i+j]));
            }
        }
        // Update AIS
        i_offset_actors+=i_num_prp_audience*i_num_audience;
        QTreeWidgetItem* twi_ais_top=m_qtw_ethics->topLevelItem(2);
        for(int i=0;i<i_num_ais;i++)
        {
            QTreeWidgetItem* twi_ais=twi_ais_top->child(i);
            for(int j=0;j<i_num_prp_ais;j++)
            {
                QTreeWidgetItem* twi_property=twi_ais->child(j);
                twi_property->setText(1,QString::number(msg_properties.data[i_offset_actors+i_num_elements_header+i_num_prp_ais*i+j]));
            }
        }
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
