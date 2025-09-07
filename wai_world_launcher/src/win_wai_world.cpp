#include "win_wai_world.h"
#include "./ui_win_wai_world.h"

win_wai_world::win_wai_world(QWidget *parent)
    : QMainWindow(parent)
    ,ui(new Ui::win_wai_world)
    ,m_hdl_node()
{
    ui->setupUi(this); // Needs to be called BEFORE accessing ui elements!

    b_profile_load_at_startup=false;

    qst_path_icons=QString::fromStdString(ros::package::getPath("wai_world_launcher")+"/resources/icons/");

    img_wai_world_settings_logo.load(qst_path_icons+QString::fromStdString("world_launcher_logo.png"));
    img_wai_world_reps_logo.load(qst_path_icons+QString::fromStdString("qtcreator.png"));
    QIcon ico_win_wai_world(QPixmap::fromImage(img_wai_world_settings_logo));
    QGuiApplication::setWindowIcon(ico_win_wai_world);
    ui->lbl_wai_world_settings_logo->setPixmap(QPixmap::fromImage(img_wai_world_settings_logo.scaled(416,234)));
    ui->lbl_wai_world_reps_logo->setPixmap(QPixmap::fromImage(img_wai_world_reps_logo.scaled(416,234)));

    //m_tmr_launcher=m_hdl_node.createTimer(ros::Duration(1.0),&win_wai_world::cb_tmr_launcher,this,false,true);
    m_tmr_launcher = new QTimer();

    connect(m_tmr_launcher,&QTimer::timeout, this, &win_wai_world::on_tmr_launcher_timeout);
    connect(ui->btn_wai_world_quit, SIGNAL(clicked()), this, SLOT(on_btn_wai_world_quit_clicked()));
    //connect(ui->btn_wai_world_settings_load, SIGNAL(clicked()), this, SLOT(on_btn_wai_world_settings_load_clicked()));
    //connect(ui->btn_wai_world_settings_save, SIGNAL(clicked()), this, SLOT(on_btn_wai_world_settings_save_clicked()));
    //connect(ui->btn_wai_world_reps_edit, SIGNAL(clicked()), this, SLOT(on_btn_wai_world_reps_edit_clicked()));
    connect(ui->lst_wai_world_reps, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(on_lst_wai_world_reps_itemClicked(QListWidgetItem *)));

    m_tmr_launcher->start(1000);

    // Initialize other members
    s_path_wai_world_launch=ros::package::getPath("wai_world_bringup")+"/launch/wai_world_bringup.launch";
    s_path_wai_oa_launch=ros::package::getPath("wai_oa_gazebo")+"/launch/wai_oa_gazebo_spawn.launch";

    // Init other UI windows
    win_oa=new win_wai_oa();
    win_oa->hide();

    //LoadWAIWorldSettingsFromFile();
    //LoadWAIWorldRepsFromFile();
    // Load "default" profile
    int i_retval = QMessageBox::question(this,"LOAD Profile",
                                       "Would you like to load the <b>DEFAULT<b> profile?",
                                        QMessageBox::Yes|QMessageBox::No, QMessageBox::Yes);
    if(i_retval==QMessageBox::Yes)
    {
        b_profile_load_at_startup=true;
    }
    on_action_Load_Profile_triggered();

    // Load current network settings
    const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
    for (const QHostAddress &address: QNetworkInterface::allAddresses())
    {
        if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost)
        {
             ui->lne_wai_world_infra_ip_local->setText(address.toString());
             ui->lne_wai_world_infra_ip_master->setText(ui->lne_wai_world_infra_ip_local->text());
        }
    }
    if(ui->lne_wai_world_infra_ip_local->text()=="")
    {
        ui->lne_wai_world_infra_ip_local->setText("127.0.0.1");
        ui->lne_wai_world_infra_ip_master->setText("127.0.0.1");
    }
}

win_wai_world::~win_wai_world()
{
    delete ui;
}

//void win_wai_world::cb_tmr_launcher(const ros::TimerEvent& event)
void win_wai_world::on_tmr_launcher_timeout()
{
    if(ros::ok())
    {
        // Do ROS related stuff...
        // ROS_WARN("Tick...");
        ros::spinOnce();
    }
    else
    {
        int i_retval=system("rosnode kill -a");
        QCoreApplication::exit(-1);
    }
}

void win_wai_world::on_btn_wai_world_quit_clicked()
{
    int i_retval=system("rosnode kill -a");
    QCoreApplication::exit(-1);
}

void win_wai_world::on_btn_wai_world_settings_load_clicked()
{
    LoadWAIWorldSettingsFromFile();

    QMessageBox mgs_wai_world_confirmation;
    mgs_wai_world_confirmation.setText("<b>Reloaded</b> original WORLD settings from YAML file!");
    mgs_wai_world_confirmation.setWindowTitle("Load WORLD Settings");
    mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_world_confirmation.exec();
}

void win_wai_world::on_btn_wai_world_settings_save_clicked()
{
    SaveWAIWorldSettingsToFile();

    QMessageBox mgs_wai_world_confirmation;
    mgs_wai_world_confirmation.setText("<b>Saved</b> current WORLD settings to YAML file!");
    mgs_wai_world_confirmation.setWindowTitle("Save WORLD Settings");
    mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_world_confirmation.exec();
}

void win_wai_world::LoadWAIWorldSettingsFromFile()
{
    std::string s_arg_val="";
    double d_arg_val_dummy=0.0;
    GetSetArgTextfile(s_path_wai_world_launch,"node_sample_frequency",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_world_rate_node->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_world_launch,"sensor_sample_frequency",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_world_rate_sensor->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_world_launch,"controller_sample_frequency",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_world_rate_controller->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_world_launch,"tf_pub_interval",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_world_rate_tf->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_world_launch,"rviz_frame_rate",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_world_rate_rviz->setText(QString::fromStdString(s_arg_val));
}

void win_wai_world::SaveWAIWorldSettingsToFile()
{
    double d_val_curr=0.0;
    std::string s_arg_val_curr="",s_arg_val_new="";
    s_arg_val_new=ui->lne_wai_world_rate_node->text().toStdString();
    GetSetArgTextfile(s_path_wai_world_launch,"node_sample_frequency",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_world_rate_sensor->text().toStdString();
    GetSetArgTextfile(s_path_wai_world_launch,"sensor_sample_frequency",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_world_rate_controller->text().toStdString();
    GetSetArgTextfile(s_path_wai_world_launch,"controller_sample_frequency",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_world_rate_tf->text().toStdString();
    GetSetArgTextfile(s_path_wai_world_launch,"tf_pub_interval",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_world_rate_rviz->text().toStdString();
    GetSetArgTextfile(s_path_wai_world_launch,"rviz_frame_rate",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
}

void win_wai_world::LoadWAIWorldRepsFromFile()
{
    // Load current REPs settings from file
    ui->lst_wai_world_reps->clear();
    std::string s_line_textfile;
    std::ifstream ifs_wai_world_reps(s_path_wai_world_launch);
    std::string s_rep_found="N/A";
    while(std::getline(ifs_wai_world_reps,s_line_textfile))
    {
        // Find all available reps via namespaces
        std::string s_ns_rep="<arg name=\"namespace_";
        std::string s_ns_def="default=\"";
        size_t pos_s_rep_start=s_line_textfile.find(s_ns_rep);
        size_t pos_s_rep_end=s_line_textfile.find("\" ");
        size_t pos_s_rep_def=s_line_textfile.find(s_ns_def);
        size_t len_s_ns_rep = s_ns_rep.length();
        size_t len_s_ns_def = s_ns_def.length();
        if(pos_s_rep_start!=std::string::npos
            && pos_s_rep_end!=std::string::npos
            && pos_s_rep_def!=std::string::npos)
        {
            s_rep_found=s_line_textfile.substr(pos_s_rep_start+len_s_ns_rep,pos_s_rep_end-(pos_s_rep_start+len_s_ns_rep));
            if(s_rep_found.compare("global")==0)
            {
                // Do nothing...
            }
            else
            {
                QListWidgetItem *lst_itm_rep = new QListWidgetItem(QString::fromStdString(s_rep_found),ui->lst_wai_world_reps);
                if(s_line_textfile.substr(pos_s_rep_def+len_s_ns_def,1).compare("\"")==0)
                {
                    lst_itm_rep->setCheckState(Qt::Unchecked);
                }
                else
                {
                    lst_itm_rep->setCheckState(Qt::Checked);
                }
                ui->lst_wai_world_reps->addItem(lst_itm_rep);
            }
        }
    }
    ifs_wai_world_reps.close();

    ui->lst_wai_world_reps->setCurrentRow(0);
    ui->lst_wai_world_reps->item(0)->setSelected(true);
    WAIWorldRepsLoadPreviewImage();
}
void win_wai_world::WAIWorldRepsLoadPreviewImage()
{
    // Immediately update imgaes
    std::string s_rep_selected=ui->lst_wai_world_reps->currentItem()->text().toStdString();
    img_wai_world_reps_logo.load(qst_path_icons+QString::fromStdString(s_rep_selected+".png"));
    ui->lbl_wai_world_reps_logo->setPixmap(QPixmap::fromImage(img_wai_world_reps_logo.scaled(416,234)));
}

void win_wai_world::SaveWAIWorldRepsToFile()
{
    // Save new REPs settings to file
    for(int i=0;i<ui->lst_wai_world_reps->count();i++)
    {
        QListWidgetItem *lst_itm_rep=ui->lst_wai_world_reps->item(i);
        std::string s_rep_selected=lst_itm_rep->text().toStdString();

        std::ifstream ifs_wai_world_reps_selected(s_path_wai_world_launch);
        std::vector<std::string > vec_ifs_lines;
        std::string s_line_textfile;

        std::string s_ns_rep="<arg name=\"namespace_"+s_rep_selected+"\" ";
        std::string s_ns_def_start="default=\"";
        std::string s_ns_def_end="\"/>";

        while(std::getline(ifs_wai_world_reps_selected,s_line_textfile))
        {
            size_t pos_s_rep_start=s_line_textfile.find(s_ns_rep);
            size_t pos_s_rep_def_start=s_line_textfile.find(s_ns_def_start);
            size_t pos_s_rep_def_end=s_line_textfile.find(s_ns_def_end);
            size_t len_s_ns_rep = s_ns_rep.length();
            size_t len_s_ns_def_start = s_ns_def_start.length();
            if(pos_s_rep_start!=std::string::npos
                && pos_s_rep_def_start!=std::string::npos
                && pos_s_rep_def_end!=std::string::npos)
            {
                if(lst_itm_rep->checkState())
                {
                    if(s_line_textfile.substr(pos_s_rep_def_start+len_s_ns_def_start,1).compare("\"")==0)
                    {
                        s_line_textfile.replace(pos_s_rep_def_start,len_s_ns_def_start,s_ns_def_start+s_rep_selected);
                        ROS_DEBUG_STREAM("Item " <<s_rep_selected<< " found,"<< " now set to CHECKED!");
                    }
                    else
                    {
                        ROS_DEBUG_STREAM("Item " <<s_rep_selected<< " found,"<< " was CHECKED already!");
                    }
                }
                else
                {
                    if(s_line_textfile.substr(pos_s_rep_def_start+len_s_ns_def_start,1).compare("\"")==0)
                    {
                        ROS_DEBUG_STREAM("Item " <<s_rep_selected<< " found,"<< " was UNCHECKED already!");
                    }
                    else
                    {
                        s_line_textfile.replace(pos_s_rep_def_start+len_s_ns_def_start,pos_s_rep_def_end-(pos_s_rep_def_start+len_s_ns_def_start),"");
                        ROS_DEBUG_STREAM("Item " <<s_rep_selected<< " found,"<< " now set to UNCHECKED!");
                    }
                }
            }
            vec_ifs_lines.push_back(s_line_textfile);
        }
        ifs_wai_world_reps_selected.close();

        std::ofstream ofs_wai_world_reps_selected(s_path_wai_world_launch);
        for(int i=0;i<vec_ifs_lines.size();i++)
        {
            ofs_wai_world_reps_selected << vec_ifs_lines[i] << std::endl;
        }
        ofs_wai_world_reps_selected.close();
    }
}

void win_wai_world::on_btn_wai_world_reps_load_clicked()
{
    LoadWAIWorldRepsFromFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Reloaded</b> original WAI World REPs from YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Load WAI World REPs");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_world::on_btn_wai_world_reps_save_clicked()
{
    SaveWAIWorldRepsToFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Saved</b> current WAI World REPs to YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Save WAI World REPs");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_world::on_lst_wai_world_reps_itemClicked(QListWidgetItem *item)
{
    //SaveWAIWorldRepsToFile();
    WAIWorldRepsLoadPreviewImage();
}

void win_wai_world::on_btn_wai_world_reps_edit_clicked()
{
    std::string s_rep_selected=ui->lst_wai_world_reps->currentItem()->text().toStdString();
    if(s_rep_selected.compare("oa")==0)
    {
        //win_oa=new win_wai_oa();
        win_oa->show();
    }
    else if(s_rep_selected.compare("marvin")==0)
    {
        // More rep settings...
    }
    else
    {
        // Do nothing...
    }

}

void win_wai_world::GetSetArgTextfile(std::string s_filepath,
                       std::string s_arg_name,
                       double* d_arg_value,
                       std::string* s_arg_value_text,
                       std::string s_arg_new_value)
{
    // Prepare text file
    std::ifstream ifs_textfile(s_filepath);
    std::vector<std::string > vec_textfile_lines;
    std::string s_textfile_line="";
    if(!ifs_textfile)
    {
        ROS_WARN_STREAM("Launcher - Could not open WORLD launch file for editing!");
        return;
    }

    std::string s_line_arg="<arg name=\""+s_arg_name+"\""; // Include following quotation mark
    std::string s_line_arg_default="default=\"";
    std::string s_line_arg_default_end="\"/>"; // Todo: Make more robust, if written 'default="..." />' aso.

    while(std::getline(ifs_textfile,s_textfile_line))
    {
        // Find argument name in line of textfile
        size_t pos_s_par = s_textfile_line.find(s_line_arg);
        if(pos_s_par!=std::string::npos)
        {
            size_t siz_pos_arg_default_start = s_textfile_line.find(s_line_arg_default);
            size_t len_arg_default = s_line_arg_default.length();
            size_t siz_pos_arg_default_end = s_textfile_line.find(s_line_arg_default_end);
            size_t len_arg_default_end = s_line_arg_default_end.length();
            if(siz_pos_arg_default_start!=std::string::npos &&
                siz_pos_arg_default_end!=std::string::npos)
            {
                *s_arg_value_text=s_textfile_line.substr(
                            siz_pos_arg_default_start+len_arg_default,
                            siz_pos_arg_default_end-(siz_pos_arg_default_start+len_arg_default));
                try
                {
                    *d_arg_value=std::stod(*s_arg_value_text);
                }
                catch(std::exception e)
                {
                    *d_arg_value=0.0;
                }

                if(s_arg_new_value.compare("")!=0)
                {
                        s_textfile_line.replace(
                                    siz_pos_arg_default_start,
                                    siz_pos_arg_default_end-siz_pos_arg_default_start+len_arg_default_end,
                                    "default=\""+s_arg_new_value+"\"/>");
                }
            }
        }
        else
        {
            // Do nothing...
        }
        vec_textfile_lines.push_back(s_textfile_line);
    }
    ifs_textfile.close();

    // Write updated data to same textfile, if required
    if(s_arg_new_value.compare("")!=0)
    {
        std::ofstream ofs_textfile(s_filepath);
        for(int i=0;i<vec_textfile_lines.size();i++)
        {
            ofs_textfile << vec_textfile_lines[i] << std::endl;
        }
        ofs_textfile.close();
    }
}

void win_wai_world::on_btn_wai_world_about_clicked()
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("About");
    msgBox.setText("<b>WAI World</b><br>An open-source workspace<br>for research and education.<br>(©2019, W. A. Isop)");
    msgBox.exec();
}

void win_wai_world::on_action_About_triggered()
{
    on_btn_wai_world_about_clicked();
}

void win_wai_world::on_action_Save_As_Profile_triggered()
{
    QString qst_path_profile_save_as_filter_selected;
    QFileDialog::Options qst_path_profile_save_as_options;
    qst_path_profile_save_as_options |= QFileDialog::DontUseNativeDialog;
    qst_path_profile_save_as_options |= QFileDialog::ShowDirsOnly;

    QString qst_path_profile_save_as =
            QFileDialog::getExistingDirectory(this,
                    "SAVE AS Profile (Select Folder)",
                    QDir::homePath()+"/catkin_ws/src/wai_world/wai_world_launcher/profiles",
                    qst_path_profile_save_as_options);
    /*
    QString qst_path_profile_save_as =
            QFileDialog::getSaveFileName(
                    this,
                    "Profile (Save As)",
                    QDir::homePath()+"/catkin_ws/src/wai_world/wai_world_launcher/profiles",
                    "Profile (Folder)",
                    &qst_path_profile_save_as_filter_selected,
                    qst_path_profile_save_as_options);
    */

    QDir qdi_path_profile_save_as(qst_path_profile_save_as);

    int ret=QMessageBox::Yes;
    if(qdi_path_profile_save_as.exists())
    {
         ret=QMessageBox::warning(this,"SAVE AS Profile (Select Folder)",
                                       tr("Do your really want to save the new profile?"),
                                        QMessageBox::Yes|QMessageBox::No, QMessageBox::No);
    }
    if(ret==QMessageBox::Yes)
    {
        if(qdi_path_profile_save_as.mkpath(qst_path_profile_save_as))
        {
            // WORLD Launch file
            if (QFile::exists(qst_path_profile_save_as+"/wai_world_bringup.launch"));
            {
                QFile::remove(qst_path_profile_save_as+"/wai_world_bringup.launch");
            }
            QFile::copy(
                QString::fromStdString(s_path_wai_world_launch),
                qst_path_profile_save_as+"/wai_world_bringup.launch");

            // OA Launch file
            if (QFile::exists(qst_path_profile_save_as+"/wai_oa_gazebo_spawn.launch"));
            {
                QFile::remove(qst_path_profile_save_as+"/wai_oa_gazebo_spawn.launch");
            }
            QFile::copy(
                QString::fromStdString(s_path_wai_oa_launch),
                qst_path_profile_save_as+"/wai_oa_gazebo_spawn.launch");

            QMessageBox mgs_wai_world_confirmation;
            mgs_wai_world_confirmation.setText("<b>Saved</b> current Profile \""+qst_path_profile_save_as.section("/",-1)+"\" to folder!");
            mgs_wai_world_confirmation.setWindowTitle("Save Profile As");
            mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
            mgs_wai_world_confirmation.exec();

            this->setWindowTitle("WAI WORLD Launcher (©2019, W. A. Isop) - Selected Profile: \""+qst_path_profile_save_as.section("/",-1)+"\"");
        }
    }
    else
    {
        QMessageBox mgs_wai_world_confirmation;
        mgs_wai_world_confirmation.setText("<b>Skipped saving</b> profile to folder!");
        mgs_wai_world_confirmation.setWindowTitle("SAVE AS Profile");
        mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
        mgs_wai_world_confirmation.exec();
    }
}

void win_wai_world::on_action_Load_Profile_triggered()
{
    QString qst_path_profile_load;
    if(b_profile_load_at_startup==true)
    {
        qst_path_profile_load=QDir::homePath()+"/catkin_ws/src/wai_world/wai_world_launcher/profiles/default";
        b_profile_load_at_startup=false;
    }
    else
    {
        QFileDialog::Options qst_path_profile_load_options;
        qst_path_profile_load_options |= QFileDialog::DontUseNativeDialog;
        qst_path_profile_load =
                QFileDialog::getExistingDirectory(this,
                        "LOAD Profile (Folder)",
                        QDir::homePath()+"/catkin_ws/src/wai_world/wai_world_launcher/profiles",
                        qst_path_profile_load_options);
    }


    // Copy back WORLD Launch file
    if (QFile::exists(QString::fromStdString(s_path_wai_world_launch)));
    {
        QFile::remove(QString::fromStdString(s_path_wai_world_launch));
    }
    QFile::copy(qst_path_profile_load+"/wai_world_bringup.launch",QString::fromStdString(s_path_wai_world_launch));

    // Copy back OA Launch file
    if (QFile::exists(QString::fromStdString(s_path_wai_oa_launch)));
    {
        QFile::remove(QString::fromStdString(s_path_wai_oa_launch));
    }
    QFile::copy(qst_path_profile_load+"/wai_oa_gazebo_spawn.launch",QString::fromStdString(s_path_wai_oa_launch));

    QMessageBox mgs_wai_world_confirmation;
    mgs_wai_world_confirmation.setText("<b>Loaded</b> Profile \""+qst_path_profile_load.section("/",-1)+"\" from folder!");
    mgs_wai_world_confirmation.setWindowTitle("LOAD Profile");
    mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_world_confirmation.exec();

    // Load WORLD settings
    LoadWAIWorldSettingsFromFile();
    LoadWAIWorldRepsFromFile();

    // Load OA settings
    win_oa->LoadWAIOASettingsFromFile();
    win_oa->LoadWAIOARepsFromFile();
    win_oa->LoadWAIOASessionsFromFile();
    //win_oa->SelectWAIOASession("break");
    win_oa->GetAvailableInteractions();
    win_oa->GetAvailableTriggers();

    win_oa->LoadWAIOASchedulerSessionsFromFile();
    win_oa->LoadWAIOASchedulerTimeSlots();
    win_oa->LoadWAIOASchedulerSessions();

    // Fixed bug: profile/settings-update won't work,
    // when reverting to old selected default session, after reloading default config profile:

    this->setWindowTitle("WAI WORLD Launcher (©2019, W. A. Isop) - Selected Profile: \""+qst_path_profile_load.section("/",-1)+"\"");
}

void win_wai_world::on_btn_wai_world_launcher_clicked()
{
    std::string s_syscmd="";

    if(ui->chk_wai_world_infra_pre_or_aud->text()=="Presenter")
    {
        s_syscmd="export WAI_OA_PRESENTER_ID=\""+
                std::to_string(ui->spb_wai_world_infra_pre_or_aud_id->value())+
                "\" && export ROS_IP="+ui->lne_wai_world_infra_ip_local->text().toStdString()+
                " && export ROS_HOSTNAME="+ui->lne_wai_world_infra_ip_local->text().toStdString()+
                " && export ROS_MASTER_URI=http://"+
                ui->lne_wai_world_infra_ip_master->text().toStdString()+
                ":12412 && roslaunch wai_world_bringup wai_world_bringup.launch";
        int i_retval=system(s_syscmd.c_str());
        QMessageBox mgs_wai_world_confirmation;
        mgs_wai_world_confirmation.setText("WAI World was shutdown cleanly!");
        mgs_wai_world_confirmation.setWindowTitle("Shutdown WAI World");
        mgs_wai_world_confirmation.setIcon(QMessageBox::Information);
        mgs_wai_world_confirmation.exec();
    }
    else if(ui->chk_wai_world_infra_pre_or_aud->text()=="Audience (Listener)")
    {
        int i_retval_1=QMessageBox::question(this,"Launch WAI World",
                                           "Would you like to launch WAI World as <b>Listener</b>?\nPlease be patient and wait until the presenter is ready to start the session!",
                                            QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
        if(i_retval_1==QMessageBox::No)
        {
            return;
        }

        s_syscmd="export WAI_OA_AUDIENCE_ID=\""+
                std::to_string(ui->spb_wai_world_infra_pre_or_aud_id->value())+
                "\" && export ROS_IP="+ui->lne_wai_world_infra_ip_local->text().toStdString()+
                " && export ROS_HOSTNAME="+ui->lne_wai_world_infra_ip_local->text().toStdString()+
                " && export ROS_MASTER_URI=http://"+
                ui->lne_wai_world_infra_ip_master->text().toStdString()+
                ":12412 && roslaunch wai_oa_gazebo wai_oa_audience_gazebo_spawn.launch --wait";
        int i_retval_2=system(s_syscmd.c_str());
    }
    else
    {
        // Do nothing
        return;
    }
}

void win_wai_world::on_chk_wai_world_infra_pre_or_aud_stateChanged(int arg1)
{
    if(arg1==Qt::Checked)
    {
        ui->chk_wai_world_infra_pre_or_aud->setText("Presenter");
        ui->lne_wai_world_infra_ip_master->setText(ui->lne_wai_world_infra_ip_local->text());
    }
    else if(arg1==Qt::Unchecked)
    {
        ui->chk_wai_world_infra_pre_or_aud->setText("Audience (Listener)");
    }
    else
    {
        // Do nothing...
    }
}
