#include "win_wai_oa.h"
#include "ui_win_wai_oa.h"

win_wai_oa::win_wai_oa(QWidget *parent) : QMainWindow(parent),ui(new Ui::win_wai_oa)
{
    ui->setupUi(this);

    // Load preview images and icons
    qst_path_icons=QString::fromStdString(ros::package::getPath("wai_world_launcher")+"/resources/icons/");

    //QIcon ico_win_wai_oa(QPixmap::fromImage(img_wai_oa_settings_logo));
    //setWindowIcon(ico_win_wai_oa);

    img_wai_oa_settings_logo.load(qst_path_icons+"open_auditorium_logo.png");
    img_wai_oa_reps_logo.load(qst_path_icons+"open_auditorium_logo.png");
    img_wai_oa_scheduler_preview.load(qst_path_icons+"open_auditorium_logo.png");
    ui->lbl_wai_oa_scheduler_preview->setPixmap(QPixmap::fromImage(img_wai_oa_scheduler_preview.scaled(300,169)));

    img_wai_oa_camera_graph_3d.load(qst_path_icons+"interactions/camera_graph_3d.png");
    img_wai_oa_camera_graph_eval.load(qst_path_icons+"interactions/camera_graph_eval.png");
    img_wai_oa_camera_overview.load(qst_path_icons+"interactions/camera_overview.png");
    img_wai_oa_camera_presenter.load(qst_path_icons+"interactions/camera_presenter.png");

    ui->lbl_wai_oa_settings_logo->setPixmap(QPixmap::fromImage(img_wai_oa_settings_logo.scaled(416,234)));
    ui->lbl_wai_oa_reps_logo->setPixmap(QPixmap::fromImage(img_wai_oa_reps_logo.scaled(416,234)));
    ui->btn_wai_oa_camera_graph_3d->setIcon(QPixmap::fromImage(img_wai_oa_camera_graph_3d.scaled(40,40)));
    ui->btn_wai_oa_camera_graph_eval->setIcon(QPixmap::fromImage(img_wai_oa_camera_graph_eval.scaled(40,40)));
    ui->btn_wai_oa_camera_overview->setIcon(QPixmap::fromImage(img_wai_oa_camera_overview.scaled(40,40)));
    ui->btn_wai_oa_camera_presenter->setIcon(QPixmap::fromImage(img_wai_oa_camera_presenter.scaled(40,40)));

    ui->lne_wai_oa_sessions->setReadOnly(true);

    connect(ui->lst_wai_oa_reps, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(on_lst_wai_oa_reps_itemClicked(QListWidgetItem *)));

    i_scenes_count=0;
    s_path_wai_world_launch=ros::package::getPath("wai_world_bringup")+"/launch/wai_world_bringup.launch";
    s_path_wai_oa_launch=ros::package::getPath("wai_oa_gazebo")+"/launch/wai_oa_gazebo_spawn.launch";
    s_path_wai_oa_session_scene_previews=ros::package::getPath("wai_oa_gazebo")+"/resources/sessions/preview/";
    s_path_wai_oa_sessions_folder=ros::package::getPath("wai_oa_gazebo")+"/resources/sessions/";

    // Load OA settings
    LoadWAIOASettingsFromFile();
    LoadWAIOARepsFromFile();
    LoadWAIOASessionsFromFile();
    GetAvailableInteractions();
    GetAvailableTriggers();

    LoadWAIOASchedulerSessionsFromFile();
    LoadWAIOASchedulerTimeSlots();
    LoadWAIOASchedulerSessions();
}

win_wai_oa::~win_wai_oa()
{
    delete ui;
}



// HELPER methods
void win_wai_oa::GetSetArgTextfile(std::string s_filepath,
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
        ROS_WARN_STREAM("Launcher - Could not open OA launch file for editing!");
        return;
    }

    std::string s_line_arg="<arg name=\""+s_arg_name+"\""; // Include following quotation mark
    std::string s_line_arg_default="default=\"";
    std::string s_line_arg_default_end="\"/>"; // TODO: Make more robust, if written 'default="..." />' aso.

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

void win_wai_oa::LoadWAIOASettingsFromFile()
{
    std::string s_arg_val="";
    double d_arg_val_dummy=0.0;
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_session_length",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_session_length->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_session_break_length",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_session_break_length->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_audience_count_max",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_audience_count_max->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_audience_listeners_per_row",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_audience_listeners_per_row->setText(QString::fromStdString(s_arg_val));

    GetSetArgTextfile(s_path_wai_oa_launch,"oa_scene_transition_timeout",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_scene_transition_timout->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_scene_trigger_timeout",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_scene_trigger_timout->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_workspace_presenter_model",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_workspace_presenter_model->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_enable_session_scheduler",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_enable_session_scheduler->setText(QString::fromStdString(s_arg_val));
    if(s_arg_val.compare("true")==0)
    {
        ui->chk_wai_oa_settings_enable_session_scheduler->setChecked(true);
    }
    else if(s_arg_val.compare("false")==0)
    {
        ui->chk_wai_oa_settings_enable_session_scheduler->setChecked(false);
    }
    else
    {
        // Do nothing...
    }

    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_x",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_presenter_pose_x->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_y",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_presenter_pose_y->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_z",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_presenter_pose_z->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_yaw",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_presenter_pose_yaw->setText(QString::fromStdString(s_arg_val));

    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_x",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_projection_pose_x->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_y",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_projection_pose_y->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_z",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_projection_pose_z->setText(QString::fromStdString(s_arg_val));
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_yaw",&d_arg_val_dummy,&s_arg_val);
    ui->lne_wai_oa_settings_projection_pose_yaw->setText(QString::fromStdString(s_arg_val));
}

void win_wai_oa::SaveWAIOASettingsToFile()
{
    double d_val_curr=0.0;
    std::string s_arg_val_curr="",s_arg_val_new="";
    s_arg_val_new=ui->lne_wai_oa_settings_session_length->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_session_length",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_session_break_length->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_session_break_length",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_audience_count_max->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_audience_count_max",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_audience_listeners_per_row->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_audience_listeners_per_row",&d_val_curr,&s_arg_val_curr,s_arg_val_new);

    s_arg_val_new=ui->lne_wai_oa_settings_scene_transition_timout->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_scene_transition_timout",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_scene_trigger_timout->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_scene_trigger_timout",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_workspace_presenter_model->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_workspace_presenter_model",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_enable_session_scheduler->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_enable_session_scheduler",&d_val_curr,&s_arg_val_curr,s_arg_val_new);

    s_arg_val_new=ui->lne_wai_oa_settings_presenter_pose_x->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_x",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_presenter_pose_y->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_y",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_presenter_pose_z->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_z",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_presenter_pose_yaw->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_presenter_pose_yaw",&d_val_curr,&s_arg_val_curr,s_arg_val_new);

    s_arg_val_new=ui->lne_wai_oa_settings_projection_pose_x->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_x",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_projection_pose_y->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_y",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_projection_pose_z->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_z",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
    s_arg_val_new=ui->lne_wai_oa_settings_projection_pose_yaw->text().toStdString();
    GetSetArgTextfile(s_path_wai_oa_launch,"oa_projection_pose_yaw",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
}

void win_wai_oa::LoadWAIOARepsFromFile()
{
    // Load current REPs settings from file
    ui->lst_wai_oa_reps->clear();
    std::string s_line_textfile;
    std::ifstream ifs_wai_oa_reps(s_path_wai_oa_launch);
    std::string s_rep_found="N/A";
    while(std::getline(ifs_wai_oa_reps,s_line_textfile))
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
            if(    s_rep_found.compare("global")==0
                || s_rep_found.compare("oa")==0
                || s_rep_found.compare("presenter")==0
                || s_rep_found.compare("workspace_presenter")==0
                || s_rep_found.compare("workspace_audience")==0)
            {
                // Do nothing...
            }
            else
            {
                QListWidgetItem *lst_itm_rep = new QListWidgetItem(QString::fromStdString(s_rep_found),ui->lst_wai_oa_reps);
                if(s_line_textfile.substr(pos_s_rep_def+len_s_ns_def,1).compare("\"")==0)
                {
                    lst_itm_rep->setCheckState(Qt::Unchecked);
                }
                else
                {
                    lst_itm_rep->setCheckState(Qt::Checked);
                }
                ui->lst_wai_oa_reps->addItem(lst_itm_rep);
            }
        }
    }
    ifs_wai_oa_reps.close();

    ui->lst_wai_oa_reps->setCurrentRow(0);
    ui->lst_wai_oa_reps->item(0)->setSelected(true);
    WAIOARepsLaodPreviewImage();
}

void win_wai_oa::LoadWAIOASchedulerSessionsFromFile()
{
    // Load current SESSION SCHEDULE from file
    std::string s_wai_oa_scheduler_weekday=
            (ui->cmb_wai_oa_scheduler_weekday->currentText()).toStdString();
    ui->lst_wai_oa_scheduler_sessions->clear();

    std::string s_line_textfile;
    std::ifstream ifs_wai_oa_scheduler_sessions(s_path_wai_oa_launch);
    std::string s_scheduler_session_found="sessions_"+s_wai_oa_scheduler_weekday+":";

    while(std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile))
    {
        // Find all available sessions via parameter name
        if(s_line_textfile.find(s_scheduler_session_found)!=std::string::npos)
        {
            // Read in actual weekday schedule
            std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            while(s_line_textfile.find("sessions_")==std::string::npos
                  && s_line_textfile.find("</rosparam>")==std::string::npos)
            {
                ui->lst_wai_oa_scheduler_sessions->addItem(QString::fromStdString(s_line_textfile));
                std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            }
        }
    }
    ifs_wai_oa_scheduler_sessions.close();

    ui->lst_wai_oa_scheduler_sessions->setCurrentRow(0);
    ui->lst_wai_oa_scheduler_sessions->item(0)->setSelected(true);
}

void win_wai_oa::SaveWAIOARepsToFile()
{
    // Save new REPs settings to file
    for(int i=0;i<ui->lst_wai_oa_reps->count();i++)
    {
        QListWidgetItem *lst_itm_rep=ui->lst_wai_oa_reps->item(i);
        std::string s_rep_selected=lst_itm_rep->text().toStdString();

        std::ifstream ifs_wai_oa_reps_selected(s_path_wai_oa_launch);
        std::vector<std::string > vec_ifs_lines;
        std::string s_line_textfile;

        std::string s_ns_rep="<arg name=\"namespace_"+s_rep_selected+"\" ";
        std::string s_ns_def_start="default=\"";
        std::string s_ns_def_end="\"/>";

        while(std::getline(ifs_wai_oa_reps_selected,s_line_textfile))
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
        ifs_wai_oa_reps_selected.close();

        std::ofstream ofs_wai_oa_reps_selected(s_path_wai_oa_launch);
        for(int i=0;i<vec_ifs_lines.size();i++)
        {
            ofs_wai_oa_reps_selected << vec_ifs_lines[i] << std::endl;
        }
        ofs_wai_oa_reps_selected.close();
    }
}

void win_wai_oa::WAIOARepsLaodPreviewImage()
{
    std::string s_rep_selected=ui->lst_wai_oa_reps->currentItem()->text().toStdString();
    img_wai_oa_reps_logo.load(qst_path_icons+QString::fromStdString(s_rep_selected+".png"));
    ui->lbl_wai_oa_reps_logo->setPixmap(QPixmap::fromImage(img_wai_oa_reps_logo.scaled(416,234)));
}

void win_wai_oa::LoadWAIOASessionsFromFile()
{
    // LOAD available sessions from folder and select default one
    GetAvailableSessions(s_path_wai_oa_sessions_folder);

    /* Deprecated, break session is selected by default:
    std::string s_arg_val;
    double d_dummy=0.0;
    GetSetArgTextfile(s_path_wai_world_launch,"oa_session_filename",&d_dummy,&s_arg_val);
    QListWidgetItem* itm_arg_val = new QListWidgetItem(QString::fromStdString(s_arg_val));
    for(int row=0;row<ui->lst_wai_oa_sessions->count();row++)
    {
             QListWidgetItem *itm_current = ui->lst_wai_oa_sessions->item(row);
             if(itm_current->text()==itm_arg_val->text())
             {
                 ui->lst_wai_oa_sessions->setCurrentItem(itm_current);
                 ui->lst_wai_oa_sessions->item(row)->setSelected(true);
                 ui->lne_wai_oa_sessions->setText(QString::fromStdString(s_arg_val));
                 s_session_selected=s_arg_val;
                 return;
             }
    }
    */

    //QListWidgetItem* itm_break = new QListWidgetItem(QString::fromStdString(s_path_wai_oa_sessions_folder)+"break.odp");
    QListWidgetItem* itm_break = new QListWidgetItem("[PRES] break.odp");
    ui->lst_wai_oa_sessions->setCurrentItem(itm_break);
    ui->lne_wai_oa_sessions->setText(itm_break->text());
    s_session_selected=s_path_wai_oa_sessions_folder+"break.odp";
    ui->lst_wai_oa_sessions->setCurrentRow(0);
}

std::string win_wai_oa::GetSessionSelectedFilename()
{
    return s_session_selected;
}

int win_wai_oa::GetWAIOAPresenterListenerID()
{
    return ui->spb_wai_oa_infra_pre_or_aud_id->value();
}

void win_wai_oa::SetWAIOAPresenterListenerID(int i_id)
{
    ui->spb_wai_oa_infra_pre_or_aud_id->setValue(i_id);
}

void win_wai_oa::SaveWAIOASessionsToFile()
{
    // Update selected session
    QString qst_session_selected=ui->lst_wai_oa_sessions->currentItem()->text();
    ui->lne_wai_oa_sessions->setText(qst_session_selected); // Indicate with shortened path in LineEdit element
    qst_session_selected.replace("[PRES] ",QString::fromStdString(s_path_wai_oa_sessions_folder));
    qst_session_selected.replace("[HTML] ",QString::fromStdString(s_path_wai_oa_sessions_folder));
    s_session_selected=qst_session_selected.toStdString(); // s_session_selected contains full filepath

    // SAVE selected session to *.yaml file
    double d_val_curr=0.0;
    std::string s_arg_val_curr="",s_arg_val_new="";
    s_arg_val_new=s_session_selected; // Save to *.yaml file with full filepath
    GetSetArgTextfile(s_path_wai_world_launch,"oa_session_filename",&d_val_curr,&s_arg_val_curr,s_arg_val_new);
}

void win_wai_oa::GetAvailableSessions(std::string s_filepath)
{
    ui->lst_wai_oa_sessions->clear();

    // Get all *.odp and *.pptx files from SESSIONS directory
    QDir qdi_sessions(QString::fromStdString(s_filepath));
    QStringList qst_list=qdi_sessions.entryList(QStringList() << "*.odp" << "*.pptx",QDir::Files);
    foreach(QString qst_filename,qst_list)
    {
        //ui->lst_wai_oa_sessions->addItem(QString::fromStdString(s_path_wai_oa_sessions_folder)+qst_filename);
        ui->lst_wai_oa_sessions->addItem("[PRES] "+qst_filename);
    }
    QDir qdi_sessions_html(QString::fromStdString(s_filepath));
    QStringList qst_list_html=qdi_sessions_html.entryList(QStringList(),QDir::Dirs);
    foreach(QString qst_dirname,qst_list_html)
    {
        if(qst_dirname=="." || qst_dirname==".." || qst_dirname=="preview")
        {
            // Do nothing...
        }
        else ui->lst_wai_oa_sessions->addItem("[HTML] "+qst_dirname);
    }
}

void win_wai_oa::GetAvailableScenes(std::string s_filepath)
{
    ui->lst_wai_oa_scenes->clear();

    // Get SCENE PREVIEWs as images
    // In a CLEAN AND PROPERLY SORTED ASCENDING ORDER!
    QDir qdi_sessions(QString::fromStdString(s_filepath));
    qdi_sessions.setSorting(QDir::NoSort);
    QStringList qst_list=qdi_sessions.entryList(QStringList() << "*.png",QDir::Files);

    QCollator qco_collator;
    qco_collator.setNumericMode(true); // this is the essential property!
    std::sort(qst_list.begin(), qst_list.end(), qco_collator);

    foreach(QString qst_filename, qst_list)
    {
        ui->lst_wai_oa_scenes->addItem(qst_filename);
    }
}

void win_wai_oa::AddSceneInteractionTextfile(std::string s_filepath)
{
    std::string s_line_textfile;
    std::vector<std::string > vec_ifs_lines;
    std::vector<std::string > vec_ofs_lines;
    std::ifstream ifs_wai_oa_session_yaml(s_filepath);
    bool b_scene_cfg_found=false;
    while(std::getline(ifs_wai_oa_session_yaml,s_line_textfile)) vec_ifs_lines.push_back(s_line_textfile);

    for(int i=0;i<vec_ifs_lines.size();i++)
    {
        vec_ofs_lines.push_back(vec_ifs_lines[i]);

        if(vec_ifs_lines[i].find(s_scene_selected)!=std::string::npos)
        {
            // Insert all current interactions
            for(int row=0;row<ui->lst_wai_oa_interactions->count();row++)
            {
                QListWidgetItem *itm_current = ui->lst_wai_oa_interactions->item(row);
                vec_ofs_lines.push_back(itm_current->text().toStdString());
            }
            vec_ofs_lines.push_back("");
            b_scene_cfg_found=true;

            // Sync with infile stream to continue reading in from the next scene setup
            while(vec_ifs_lines[i+1].find("setup_scene_")==std::string::npos)
            {
                i++;
            }
        }
    }
    if(b_scene_cfg_found==false)
    {
        // If not defined yet add new scene config to the end of file
        vec_ofs_lines.push_back(s_scene_selected);
        for(int row=0;row<ui->lst_wai_oa_interactions->count();row++)
        {
            std::getline(ifs_wai_oa_session_yaml,s_line_textfile);
            QListWidgetItem *itm_current = ui->lst_wai_oa_interactions->item(row);
            vec_ofs_lines.push_back(itm_current->text().toStdString());
        }
        vec_ofs_lines.push_back(""); // Add blank line to separate from next scene config!
    }
    ifs_wai_oa_session_yaml.close();

    std::ofstream ofs_wai_oa_session_yaml(s_filepath);
    for(int i=0;i<vec_ofs_lines.size();i++)
    {
        ofs_wai_oa_session_yaml << vec_ofs_lines[i] << std::endl;
    }
    ofs_wai_oa_session_yaml.close();
}

void win_wai_oa::RemoveSelectedInteraction()
{
    QListWidgetItem *itm_selected = ui->lst_wai_oa_interactions->takeItem(ui->lst_wai_oa_interactions->currentRow());
    delete itm_selected;

    // Open and write new scene config to textfile
    // DEPRECATED: AddSceneInteractionTextfile(s_path_wai_oa_session_yaml);
}
void win_wai_oa::EditSelectedInteraction()
{
    bool ok_interaction_edit=false;

    std::string s_interaction_full=ui->lst_wai_oa_interactions->currentItem()->text().toStdString();
    unsigned u_first=s_interaction_full.find(":");
    std::string s_interaction_extracted=s_interaction_full.substr(u_first+2);
    // s_interaction_extracted.pop_back();

    QInputDialog* inp_interaction_edited=new QInputDialog();
    QString qst_interaction_edited=inp_interaction_edited->getText(
                this,
                "Edit Selected Interaction",
                "Edit Interaction:",
                QLineEdit::Normal,
                QString::fromStdString(s_interaction_extracted),
                &ok_interaction_edit,
                Qt::WindowFlags());

    if(ok_interaction_edit)
    {
        std::string s_interaction_edited=s_interaction_full.erase(u_first);
        s_interaction_edited=s_interaction_edited+": "+qst_interaction_edited.toStdString();
        // Update text in listview and save to file
        ui->lst_wai_oa_interactions->currentItem()->setText(QString::fromStdString(s_interaction_edited));
        // DEPRECATED: AddSceneInteractionTextfile(s_path_wai_oa_session_yaml);
    }
}
void win_wai_oa::AddAvailableInteraction()
{
    if(ui->lst_wai_oa_interactions->findItems(ui->lst_wai_oa_interactions_available->currentItem()->text(),Qt::MatchContains).size()==0)
    {
        QString qst_interaction_new=
                ui->lst_wai_oa_interactions_available->currentItem()->text()+
                ui->lst_wai_oa_triggers_available->currentItem()->text();
        ui->lst_wai_oa_interactions->addItem(qst_interaction_new);

        // Open and write new config to textfile
        // DEPRECATED: AddSceneInteractionTextfile(s_path_wai_oa_session_yaml);
    }
    else
    {
        QMessageBox msg_wai_oa_warning;
        msg_wai_oa_warning.setText("Interaction is already used in the scene!\nInteraction is not added.");
        msg_wai_oa_warning.setIcon(QMessageBox::Warning);
        msg_wai_oa_warning.setWindowTitle("Interaction already used!");
        msg_wai_oa_warning.exec();
    }
}

void win_wai_oa::GetAvailableInteractions()
{
    // Consider advantages and disadvanteges of simplified/unified rep interaction interface:
    // Advantages: Interface is more simple and more clear
    // Disadvantage: Unifying to "rep_interaction" interface makes it harder to trigger several interactions in one slide
    // TODO: Implement both interfaces, so that one can either use the rep_... trigger or the explicit one!
    ui->lst_wai_oa_interactions_available->clear();
    ui->lst_wai_oa_interactions_available->addItem("    projector: ");
    ui->lst_wai_oa_interactions_available->addItem("    projector_transition: ");
    ui->lst_wai_oa_interactions_available->addItem("    light: ");
    ui->lst_wai_oa_interactions_available->addItem("    camera: ");
    ui->lst_wai_oa_interactions_available->addItem("    camera_link: ");
    ui->lst_wai_oa_interactions_available->addItem("    camera_follow: ");
    ui->lst_wai_oa_interactions_available->addItem("    teleprompter: ");
    ui->lst_wai_oa_interactions_available->addItem("    browser_link: ");
    ui->lst_wai_oa_interactions_available->addItem("    sound: ");
    ui->lst_wai_oa_interactions_available->addItem("    lectern: ");
    ui->lst_wai_oa_interactions_available->addItem("    table: ");
    ui->lst_wai_oa_interactions_available->addItem("    virtual_presenter: ");
    ui->lst_wai_oa_interactions_available->addItem("    graph_3d: ");
    ui->lst_wai_oa_interactions_available->addItem("    graph_stats: ");
    ui->lst_wai_oa_interactions_available->addItem("    respawn_2d3d: ");
    ui->lst_wai_oa_interactions_available->addItem("    hand_text: ");
    ui->lst_wai_oa_interactions_available->addItem("    rep_name: ");
    ui->lst_wai_oa_interactions_available->addItem("    rep_interaction: ");
    ui->lst_wai_oa_interactions_available->addItem("    rep_interaction_switch: ");
    ui->lst_wai_oa_interactions_available->addItem("    learning_mode: ");
    ui->lst_wai_oa_interactions_available->addItem("    marvin_presenter: ");
    ui->lst_wai_oa_interactions_available->addItem("    marvin_hologram: ");
    ui->lst_wai_oa_interactions_available->addItem("    link_mass: ");
    ui->lst_wai_oa_interactions_available->addItem("    joint_state: ");
    ui->lst_wai_oa_interactions_available->setCurrentRow(0);
    ui->lst_wai_oa_interactions_available->item(0)->setSelected(true);
}
void win_wai_oa::GetAvailableTriggers()
{
    ui->lst_wai_oa_triggers_available->clear();
    QString qst_int_sel=ui->lst_wai_oa_interactions_available->currentItem()->text();
    if(qst_int_sel=="    projector: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("slide");
        ui->lst_wai_oa_triggers_available->addItem("livecam");
        ui->lst_wai_oa_triggers_available->addItem("repcam");
        ui->lst_wai_oa_triggers_available->addItem("oa_demo.png");
    }
    else if(qst_int_sel=="    projector_transition: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("dark_to_light");
        ui->lst_wai_oa_triggers_available->addItem("zoom_rectangle_middle");
        ui->lst_wai_oa_triggers_available->addItem("zoom_rectangle_top_left");
    }
    else if(qst_int_sel=="    light: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("off");
        ui->lst_wai_oa_triggers_available->addItem("dark");
        ui->lst_wai_oa_triggers_available->addItem("default");
        ui->lst_wai_oa_triggers_available->addItem("bright");
        ui->lst_wai_oa_triggers_available->addItem("cold");
        ui->lst_wai_oa_triggers_available->addItem("warm");
        ui->lst_wai_oa_triggers_available->addItem("[128,128,128, 128,128,128]");
    }
    else if(qst_int_sel=="    camera: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("default");
        ui->lst_wai_oa_triggers_available->addItem("expand_projection");
        ui->lst_wai_oa_triggers_available->addItem("fill");
        ui->lst_wai_oa_triggers_available->addItem("graph_3d");
        ui->lst_wai_oa_triggers_available->addItem("graph_eval");
        ui->lst_wai_oa_triggers_available->addItem("graph_stats");
        ui->lst_wai_oa_triggers_available->addItem("overview");
        ui->lst_wai_oa_triggers_available->addItem("presenter");
        ui->lst_wai_oa_triggers_available->addItem("presenter_fpv");
        ui->lst_wai_oa_triggers_available->addItem("wim");
        ui->lst_wai_oa_triggers_available->addItem("[-2.0,-2.0,2.0, 0.0,0.0,0.0]");
    }
    else if(qst_int_sel=="    camera_link: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("vase/link_base");
    }
    else if(qst_int_sel=="    camera_follow: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("marvin");
    }
    else if(qst_int_sel=="    teleprompter: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("Welcome to OA!");
    }
    else if(qst_int_sel=="    browser_link: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("https://www.google.at");
    }
    else if(qst_int_sel=="    sound: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
        ui->lst_wai_oa_triggers_available->addItem("PlenumSettingUpSessionStartup");
        ui->lst_wai_oa_triggers_available->addItem("PlenumSettingUpSessionClose");
    }
    else if(qst_int_sel=="    lectern: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("false");
        ui->lst_wai_oa_triggers_available->addItem("true");
    }
    else if(qst_int_sel=="    table: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("false");
        ui->lst_wai_oa_triggers_available->addItem("true");
    }
    else if(qst_int_sel=="    virtual_presenter: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
    }
    else if(qst_int_sel=="    graph_3d: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
    }
    else if(qst_int_sel=="    graph_stats: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
    }
    else if(qst_int_sel=="    respawn_2d3d: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
    }
    else if(qst_int_sel=="    hand_text: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("Welcome to OA!");
    }
    else if(qst_int_sel=="    rep_name: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("presenter");
        ui->lst_wai_oa_triggers_available->addItem("projection");
        ui->lst_wai_oa_triggers_available->addItem("marvin");
        ui->lst_wai_oa_triggers_available->addItem("ingenuity");
    }
    else if(qst_int_sel=="    rep_interaction: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("state");
        ui->lst_wai_oa_triggers_available->addItem("force");
        ui->lst_wai_oa_triggers_available->addItem("reference");
        ui->lst_wai_oa_triggers_available->addItem("startup");
        ui->lst_wai_oa_triggers_available->addItem("takeoff");
        ui->lst_wai_oa_triggers_available->addItem("land");
        ui->lst_wai_oa_triggers_available->addItem("shutdown");
        ui->lst_wai_oa_triggers_available->addItem("respawn");
        ui->lst_wai_oa_triggers_available->addItem("multiplot");
        ui->lst_wai_oa_triggers_available->addItem("sequence");
        ui->lst_wai_oa_triggers_available->addItem("gravity");
    }
    else if(qst_int_sel=="    rep_interaction_switch: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("[1.0,1.0,1.0,0.0]");
    }
    else if(qst_int_sel=="    learning_mode: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("plenum");
        ui->lst_wai_oa_triggers_available->addItem("cooperative");
    }
    else if(qst_int_sel=="    marvin_presenter: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("This is a test! Welcome to Open Auditorium!");
    }
    else if(qst_int_sel=="    marvin_hologram: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("oa_demo");
    }
    else if(qst_int_sel=="    link_mass: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("1.0");
    }
    else if(qst_int_sel=="    joint_state: ")
    {
        ui->lst_wai_oa_triggers_available->addItem("1.0");
    }
    else
    {
        // Do nothing...
    }
    ui->lst_wai_oa_triggers_available->setCurrentRow(0);
    ui->lst_wai_oa_triggers_available->item(0)->setSelected(true);
}

void win_wai_oa::SelectWAIOAScene()
{
    // Preview selected scene as preview image
    std::string s_entry=ui->lst_wai_oa_scenes->currentItem()->text().toStdString();
    s_path_scene_selected=s_path_wai_oa_session_scene_previews+s_entry;
    img_wai_oa_scene_preview.load(QString::fromStdString(s_path_scene_selected));
    ui->lbl_wai_oa_scene_preview->setPixmap(QPixmap::fromImage(img_wai_oa_scene_preview).scaled(320,180)); // .scaled(320,180) Keep 16:9 ratio!

    // Preview selected scene config from yaml file
    ui->lst_wai_oa_interactions->clear();
    size_t lastindex=s_entry.find_last_of(".");
    std::string s_entry_name_only=s_entry.substr(0, lastindex);
    std::string s_path_scene_selected_yaml=s_path_wai_oa_session_scene_previews+s_entry_name_only+".yaml";
    std::string s_line_textfile;
    std::ifstream ifs_wai_oa_scene_yaml(s_path_scene_selected_yaml);
    while(std::getline(ifs_wai_oa_scene_yaml,s_line_textfile))
    {
        QListWidgetItem *lst_itm_int = new QListWidgetItem(QString::fromStdString(s_line_textfile),ui->lst_wai_oa_interactions);
        ui->lst_wai_oa_interactions->addItem(lst_itm_int);
    }
}

void win_wai_oa::LoadWAIOASchedulerTimeSlots()
{
    // Load current TIME SLOTS from launch file
    ui->lst_wai_oa_scheduler_time_slots->clear();
    std::string s_line_textfile;
    std::ifstream ifs_wai_oa_scheduler_sessions(s_path_wai_oa_launch);
    std::string s_scheduler_session_found="sessions_schedule:";

    while(std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile))
    {
        // Find all available sessions via parameter name
        if(s_line_textfile.find(s_scheduler_session_found)!=std::string::npos)
        {
            // Read in actual weekday schedule
            std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            while(s_line_textfile.find("sessions_")==std::string::npos
                  && s_line_textfile.find("</rosparam>")==std::string::npos)
            {
                ui->lst_wai_oa_scheduler_time_slots->addItem(QString::fromStdString(s_line_textfile));
                std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            }
        }
    }
    ifs_wai_oa_scheduler_sessions.close();

    ui->lst_wai_oa_scheduler_time_slots->setCurrentRow(0);
    ui->lst_wai_oa_scheduler_time_slots->item(0)->setSelected(true);
}

void win_wai_oa::LoadWAIOASchedulerSessions()
{
    // Exctract [...,...,...] substring and update line edits
    std::string s_scheduler_session_selected=ui->lst_wai_oa_scheduler_sessions->currentItem()->text().toStdString();
    size_t siz_start=s_scheduler_session_selected.find("[")+1;
    std::string s_input = s_scheduler_session_selected.substr(siz_start,s_scheduler_session_selected.length()-siz_start-1);
    std::vector<std::string> vec_s_sched_sel=GetSubstringsBetweenDelimiters(s_input,",");
    ui->lne_wai_oa_scheduler_session_group->setText(QString::fromStdString(RemoveChar(vec_s_sched_sel[0],'\'')));
    ui->lne_wai_oa_scheduler_session_topic->setText(QString::fromStdString(RemoveChar(vec_s_sched_sel[1],'\'')));
    ui->lne_wai_oa_scheduler_session_name->setText(QString::fromStdString(RemoveChar(vec_s_sched_sel[2],'\'')));

    // Preview imge of first slide, according to selected scheduler session
    // Show current slide as image
    std::string s_sched_sess_sel_clean=ui->lne_wai_oa_scheduler_session_name->text().toStdString();
    //s_sched_sess_sel_clean=s_path_wai_oa_sessions_folder+s_sched_sess_sel_clean+"/"+s_sched_sess_sel_clean+"/"+"Slide1.PNG";
    s_sched_sess_sel_clean=s_path_wai_oa_sessions_folder+"/preview/scene_1.png";
    if(QFile::exists(QString::fromStdString(s_sched_sess_sel_clean)))
    {
        img_wai_oa_scheduler_preview.load(QString::fromStdString(s_sched_sess_sel_clean));
    }
    else
    {
        img_wai_oa_scheduler_preview.load(qst_path_icons+"open_auditorium_logo.png");
    }
    ui->lbl_wai_oa_scheduler_preview->setPixmap(QPixmap::fromImage(img_wai_oa_scheduler_preview.scaled(300,169)));

    // Extract scheduled time from selected list index, stays the same for all weekdays
    int i=ui->lst_wai_oa_scheduler_sessions->currentRow()+1;
    std::string s_session_string="session_"+std::to_string(i)+":";

    std::string s_line_textfile;
    std::ifstream ifs_wai_oa_scheduler_sessions(s_path_wai_oa_launch);
    std::string s_scheduler_session_found="sessions_schedule:";

    while(std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile))
    {
        // Find sessions SCHEDULE (times), same for all weekdays
        if(s_line_textfile.find(s_scheduler_session_found)!=std::string::npos)
        {
            // Read in actual weekday schedule
            std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            while(s_line_textfile.find("sessions_default:")==std::string::npos)
            {
                if(s_line_textfile.find(s_session_string)!=std::string::npos)
                {
                    size_t first = s_line_textfile.find("[");
                    size_t last = s_line_textfile.find("]");
                    std::string s_substr_session_times=s_line_textfile.substr(first+1,last-first-1);
                    std::vector<std::string> vec_s_substr_session_times;
                    boost::split(vec_s_substr_session_times,s_substr_session_times,boost::is_any_of(","));
                    ui->lne_wai_oa_scheduler_session_time_start_hour->setText(QString::fromStdString(vec_s_substr_session_times[0]));
                    ui->lne_wai_oa_scheduler_session_time_start_minute->setText(QString::fromStdString(vec_s_substr_session_times[1]));
                    ui->lne_wai_oa_scheduler_session_time_end_hour->setText(QString::fromStdString(vec_s_substr_session_times[2]));
                    ui->lne_wai_oa_scheduler_session_time_end_minute->setText(QString::fromStdString(vec_s_substr_session_times[3]));
                    //ROS_WARN_STREAM(s_line_textfile.substr(first+1,last-first-1));
                }
                std::getline(ifs_wai_oa_scheduler_sessions,s_line_textfile);
            }
        }
    }
    ifs_wai_oa_scheduler_sessions.close();
}



// INTERACTION methods (input events)
void win_wai_oa::on_tab_wai_oa_tabBarClicked(int index)
{

}

void win_wai_oa::on_btn_wai_oa_about_clicked()
{
    QMessageBox msgBox;
    msgBox.setWindowTitle("About");
    msgBox.setText("<b>WAI Open Auditorium (OA)</b><br>A human-centric eXtended Reality (XR)<br>blended learning (BL) tool.<br>(©2019, W. A. Isop)");
    msgBox.exec();
}

void win_wai_oa::on_btn_wai_oa_close_clicked()
{
    this->close();
}

void win_wai_oa::on_lst_wai_oa_sessions_itemClicked(QListWidgetItem *item)
{
    // Select session for launch
    SaveWAIOASessionsToFile();
}

void win_wai_oa::on_lst_wai_oa_scenes_itemClicked(QListWidgetItem *item)
{
    SelectWAIOAScene();
}

void win_wai_oa::on_btn_wai_oa_interaction_remove_clicked()
{
    RemoveSelectedInteraction();
}

void win_wai_oa::on_btn_wai_oa_interaction_edit_clicked()
{
    EditSelectedInteraction();
}

void win_wai_oa::on_btn_wai_oa_interaction_add_clicked()
{
    AddAvailableInteraction();
}

void win_wai_oa::on_lst_wai_oa_interactions_available_itemClicked(QListWidgetItem *item)
{
    GetAvailableTriggers();
}

void win_wai_oa::on_btn_wai_oa_reps_load_clicked()
{
    LoadWAIOARepsFromFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Reloaded</b> original OA REPs from YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Load OA REPs");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_oa::on_btn_wai_oa_reps_save_clicked()
{
    SaveWAIOARepsToFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Saved</b> current OA REPs to YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Save OA REPs");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_oa::on_btn_wai_oa_settings_load_clicked()
{
    LoadWAIOASettingsFromFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Reloaded</b> original OA settings from YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Load OA Settings");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_oa::on_btn_wai_oa_settings_save_clicked()
{
    SaveWAIOASettingsToFile();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Saved</b> current OA settings to YAML file!");
    mgs_wai_oa_confirmation.setWindowTitle("Save OA Settings");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_oa::on_lst_wai_oa_reps_itemClicked(QListWidgetItem *item)
{
    WAIOARepsLaodPreviewImage();
}

void win_wai_oa::on_chk_wai_oa_settings_enable_session_scheduler_stateChanged(int arg1)
{
    if(arg1==Qt::Checked)
    {
        ui->lne_wai_oa_settings_enable_session_scheduler->setText("true");
    }
    else if(arg1==Qt::Unchecked)
    {
        ui->lne_wai_oa_settings_enable_session_scheduler->setText("false");
    }
    else
    {
        // Do nothing...
    }
}

void win_wai_oa::on_lne_wai_oa_settings_enable_session_scheduler_textChanged(const QString &arg1)
{

}


// Shortcuts for common interactions
void win_wai_oa::on_btn_wai_oa_camera_graph_3d_clicked()
{
    ui->lst_wai_oa_interactions_available->setCurrentRow(3);
    GetAvailableTriggers();
    ui->lst_wai_oa_triggers_available->setCurrentRow(3);
    AddAvailableInteraction();
}
void win_wai_oa::on_btn_wai_oa_camera_graph_eval_clicked()
{
    ui->lst_wai_oa_interactions_available->setCurrentRow(3);
    GetAvailableTriggers();
    ui->lst_wai_oa_triggers_available->setCurrentRow(4);
    AddAvailableInteraction();
}
void win_wai_oa::on_btn_wai_oa_camera_overview_clicked()
{
    ui->lst_wai_oa_interactions_available->setCurrentRow(3);
    GetAvailableTriggers();
    ui->lst_wai_oa_triggers_available->setCurrentRow(6);
    AddAvailableInteraction();
}

void win_wai_oa::on_btn_wai_oa_camera_presenter_clicked()
{
    ui->lst_wai_oa_interactions_available->setCurrentRow(3);
    GetAvailableTriggers();
    ui->lst_wai_oa_triggers_available->setCurrentRow(7);
    AddAvailableInteraction();
}

void win_wai_oa::on_btn_wai_oa_scheduler_load_clicked()
{
    LoadWAIOASchedulerSessionsFromFile();
}

void win_wai_oa::on_lst_wai_oa_scheduler_sessions_itemClicked(QListWidgetItem *item)
{
    LoadWAIOASchedulerSessions();
}

std::vector<std::string> win_wai_oa::GetSubstringsBetweenDelimiters(std::string s, std::string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos)
    {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}
std::string win_wai_oa::RemoveChar(std::string str, char c)
{
   std::string result;
   for (size_t i = 0; i < str.size(); i++)
   {
          char currentChar = str[i];
          if (currentChar != c)
              result += currentChar;
   }
       return result;
}
bool win_wai_oa::CheckIfFilExists(const std::string& name)
{
    if (FILE *file = fopen(name.c_str(), "r"))
    {
        fclose(file);
        return true;
    }
    else
    {
        return false;
    }
}

void win_wai_oa::on_cmb_wai_oa_scheduler_weekday_currentTextChanged(const QString &arg1)
{
    LoadWAIOASchedulerSessionsFromFile();
}

void win_wai_oa::on_lst_wai_oa_scheduler_time_slots_itemClicked(QListWidgetItem *item)
{

}
void win_wai_oa::SaveWAIOASchedulerSessions()
{
    // Save current SESSIONS to launch file
    std::string s_line_textfile;
    std::vector<std::string > vec_ofs_lines;
    std::ifstream ifs_wai_oa_gazebo_launch(s_path_wai_oa_launch);
    std::string s_scheduler_sessions_found="sessions_"+(ui->cmb_wai_oa_scheduler_weekday->currentText()).toStdString()+":";

    while(std::getline(ifs_wai_oa_gazebo_launch,s_line_textfile))
    {
        vec_ofs_lines.push_back(s_line_textfile);

        if(s_line_textfile.find(s_scheduler_sessions_found)!=std::string::npos)
        {
            // Insert all scheduled sessions from current list widget
            for(int row=0;row<ui->lst_wai_oa_scheduler_sessions->count();row++)
            {
                std::getline(ifs_wai_oa_gazebo_launch,s_line_textfile); // For sync. --> continue reading original textfile stream
                QListWidgetItem *itm_current = ui->lst_wai_oa_scheduler_sessions->item(row);
                vec_ofs_lines.push_back(itm_current->text().toStdString());
            }
        }
    }
    ifs_wai_oa_gazebo_launch.close();

    std::ofstream ofs_wai_oa_gazebo_launch(s_path_wai_oa_launch);
    for(int i=0;i<vec_ofs_lines.size();i++)
    {
        ofs_wai_oa_gazebo_launch << vec_ofs_lines[i] << std::endl;
    }
    ofs_wai_oa_gazebo_launch.close();
}

void win_wai_oa::SaveWAIOASchedulerTimeSlots()
{
    // Save current TIME SLOTS to launch file
    std::string s_line_textfile;
    std::vector<std::string > vec_ofs_lines;
    std::ifstream ifs_wai_oa_gazebo_launch(s_path_wai_oa_launch);
    std::string s_scheduler_slots_found="sessions_schedule:";

    while(std::getline(ifs_wai_oa_gazebo_launch,s_line_textfile))
    {
        vec_ofs_lines.push_back(s_line_textfile);

        if(s_line_textfile.find(s_scheduler_slots_found)!=std::string::npos)
        {
            // Insert all time slots from current list widget
            for(int row=0;row<ui->lst_wai_oa_scheduler_time_slots->count();row++)
            {
                std::getline(ifs_wai_oa_gazebo_launch,s_line_textfile); // For sync. --> continue reading original textfile stream
                QListWidgetItem *itm_current = ui->lst_wai_oa_scheduler_time_slots->item(row);
                vec_ofs_lines.push_back(itm_current->text().toStdString());
            }
        }
    }
    ifs_wai_oa_gazebo_launch.close();

    std::ofstream ofs_wai_oa_gazebo_launch(s_path_wai_oa_launch);
    for(int i=0;i<vec_ofs_lines.size();i++)
    {
        ofs_wai_oa_gazebo_launch << vec_ofs_lines[i] << std::endl;
    }
    ofs_wai_oa_gazebo_launch.close();
}

void win_wai_oa::on_btn_wai_oa_scheduler_save_clicked()
{
    SaveWAIOASchedulerTimeSlots();
    SaveWAIOASchedulerSessions();

    QMessageBox mgs_wai_oa_confirmation;
    mgs_wai_oa_confirmation.setText("<b>Saved</b> current OA Scheduler settings to LAUNCH file!");
    mgs_wai_oa_confirmation.setWindowTitle("Save OA Settings");
    mgs_wai_oa_confirmation.setIcon(QMessageBox::Information);
    mgs_wai_oa_confirmation.exec();
}

void win_wai_oa::EditWAIOASchedulerTimeSlot()
{
    std::string s_slot_current_item=ui->lst_wai_oa_scheduler_time_slots->currentItem()->text().toStdString();
    size_t siz_start=s_slot_current_item.find("[")+1;
    std::string s_slot_edited_item=s_slot_current_item.substr(siz_start,s_slot_current_item.length()-siz_start-1);
    std::vector<std::string> vec_s_substr_session_times;
    boost::erase_all(s_slot_edited_item," ");
    boost::erase_all(s_slot_edited_item,"'");
    boost::split(vec_s_substr_session_times,s_slot_edited_item,boost::is_any_of(","));

    QHBoxLayout* lay_hor_box = new QHBoxLayout();
    QDialog* d = new QDialog(); d->setWindowTitle("Edit TIME Slot");
    QLabel* lbl_slot_select=new QLabel(); lbl_slot_select->setText("Edit Slot:");
    QLabel* lbl_slot_select_dash=new QLabel(); lbl_slot_select_dash->setText("-");
    QSpinBox* spb_slot_start_hour=new QSpinBox(); spb_slot_start_hour->setMinimum(0); spb_slot_start_hour->setMaximum(23); spb_slot_start_hour->setSingleStep(1); spb_slot_start_hour->setValue(std::stoi(vec_s_substr_session_times[0]));
    QSpinBox* spb_slot_start_minute=new QSpinBox(); spb_slot_start_minute->setMinimum(0); spb_slot_start_minute->setMaximum(59); spb_slot_start_minute->setSingleStep(1); spb_slot_start_minute->setValue(std::stoi(vec_s_substr_session_times[1]));
    QSpinBox* spb_slot_end_hour=new QSpinBox(); spb_slot_end_hour->setMinimum(0); spb_slot_end_hour->setMaximum(23); spb_slot_end_hour->setSingleStep(1); spb_slot_end_hour->setValue(std::stoi(vec_s_substr_session_times[2]));
    QSpinBox* spb_slot_end_minute=new QSpinBox(); spb_slot_end_minute->setMinimum(0); spb_slot_end_minute->setMaximum(59); spb_slot_end_minute->setSingleStep(1); spb_slot_end_minute->setValue(std::stoi(vec_s_substr_session_times[3]));

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
        lay_hor_box->addWidget(lbl_slot_select);
        lay_hor_box->addWidget(spb_slot_start_hour);
        lay_hor_box->addWidget(spb_slot_start_minute);
        lay_hor_box->addWidget(lbl_slot_select_dash);
        lay_hor_box->addWidget(spb_slot_end_hour);
        lay_hor_box->addWidget(spb_slot_end_minute);
        lay_hor_box->addWidget(buttonBox);
        d->setLayout(lay_hor_box);
    int result = d->exec();
    std::string s_result_slot="["+std::to_string(spb_slot_start_hour->value())+","
                                +std::to_string(spb_slot_start_minute->value())+","
                                +std::to_string(spb_slot_end_hour->value())+","
                                +std::to_string(spb_slot_end_minute->value())+"]";

    if(result == QDialog::Accepted)
    {
        ui->lst_wai_oa_scheduler_time_slots->currentItem()->setText(QString::fromStdString(s_slot_current_item.replace(s_slot_current_item.find("["),s_slot_current_item.find("]"),s_result_slot)));
    }
    else
    {
        // Do nothing...
    }
}

void win_wai_oa::on_btn_wai_oa_scheduler_time_slots_clicked()
{
    EditWAIOASchedulerTimeSlot();
}

void win_wai_oa::on_btn_wai_oa_scheduler_session_settings_clicked()
{
    EditWAIOASchedulerSession();
}

void win_wai_oa::EditWAIOASchedulerSession()
{
    /*
    bool ok_scheduler_session_edit=false;
    QInputDialog* inp_scheduler_session_edited=new QInputDialog();
    QString qst_scheduler_session_edited=inp_scheduler_session_edited->getText(
                this,
                "Edit Session Slot",
                "Edit Selected Session Slot:",
                QLineEdit::Normal,
                ui->lst_wai_oa_scheduler_sessions->currentItem()->text(),
                &ok_scheduler_session_edit,
                Qt::WindowFlags());
    if(ok_scheduler_session_edit)
    {
        // Update text in listview and save to file
        ui->lst_wai_oa_scheduler_sessions->currentItem()->setText(qst_scheduler_session_edited);
    }
    */

    std::string s_session_current_item=ui->lst_wai_oa_scheduler_sessions->currentItem()->text().toStdString();
    size_t siz_start=s_session_current_item.find("[")+1;
    // NOTE: Be careful with "substr()" function, since type of parameters should better be given explicitely
    // --> E.g. in this case .substr(Start Ch, Length); siz_start given as size_t (!)
    std::string s_session_edited_item=s_session_current_item.substr(siz_start,s_session_current_item.length()-siz_start-1);
    std::vector<std::string> vec_s_substr_session_times;
    boost::erase_all(s_session_edited_item,"\'");
    boost::split(vec_s_substr_session_times,s_session_edited_item,boost::is_any_of(","));

    QHBoxLayout* lay_hor_box = new QHBoxLayout();
    QDialog* d = new QDialog(); d->setWindowTitle("Edit SESSION Slot");
    QLabel* lbl_session_select=new QLabel(); lbl_session_select->setText("Edit Slot - ");
    QLabel* lbl_session_select_group=new QLabel(); lbl_session_select_group->setText("Session Group:");
    QLabel* lbl_session_select_topic=new QLabel(); lbl_session_select_topic->setText("Session Topic:");
    QLabel* lbl_session_select_name=new QLabel(); lbl_session_select_name->setText("Session Name:");
    QLineEdit* lne_session_select_group=new QLineEdit(); lne_session_select_group->setText(QString::fromStdString(vec_s_substr_session_times[0]));
    QLineEdit* lne_session_select_topic=new QLineEdit(); lne_session_select_topic->setText(QString::fromStdString(vec_s_substr_session_times[1]));
    QLineEdit* lne_session_select_name=new QLineEdit(); lne_session_select_name->setText(QString::fromStdString(vec_s_substr_session_times[2]));

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        QObject::connect(buttonBox, SIGNAL(accepted()), d, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), d, SLOT(reject()));
        lay_hor_box->addWidget(lbl_session_select);
        lay_hor_box->addWidget(lbl_session_select_group);
        lay_hor_box->addWidget(lne_session_select_group);
        lay_hor_box->addWidget(lbl_session_select_topic);
        lay_hor_box->addWidget(lne_session_select_topic);
        lay_hor_box->addWidget(lbl_session_select_name);
        lay_hor_box->addWidget(lne_session_select_name);
        lay_hor_box->addWidget(buttonBox);
        d->setLayout(lay_hor_box);
    int result = d->exec();
    std::string s_result_session="['"+lne_session_select_group->text().toStdString()+"','"
                                +lne_session_select_topic->text().toStdString()+"','"
                                +lne_session_select_name->text().toStdString()+"']";

    if(result == QDialog::Accepted)
    {
        ui->lst_wai_oa_scheduler_sessions->currentItem()->setText(QString::fromStdString(s_session_current_item.replace(s_session_current_item.find("["),s_session_current_item.find("]"),s_result_session)));
    }
    else
    {
        // Do nothing...
    }
}

void win_wai_oa::on_lst_wai_oa_scenes_itemSelectionChanged()
{
    SelectWAIOAScene();
}

void win_wai_oa::on_btn_wai_oa_session_import_clicked()
{   
    // Export scene *.png images and *.yaml config files for PREVIEW
    // Preview files are stored in ../resources/sessions/preview folder!
    ui->lst_wai_oa_scenes->clear();
    std::string s_sys_cmd_cfg="rm "+s_path_wai_oa_session_scene_previews+"*";
    int i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

    // Check if *.PPTX or *.ODP based session,
    // OR HTML-based session:
    if(s_session_selected.find(".odp")==std::string::npos
       && s_session_selected.find(".pptx")==std::string::npos)
    {
        QDir dir(QString::fromStdString(s_session_selected));
        if(!dir.exists())
        {
            ROS_ERROR("LAUNCHER: No valid session file or path, can not create preview!");
            return;
        }
        QStringList filters;
        filters << "*.html";
        QFileInfoList fileList = dir.entryInfoList(filters,QDir::Files|QDir::NoSymLinks);
        int i_scene_files=fileList.count()-1; // Consider the 0.html session config file

        for(int i=1;i<=i_scene_files;i++)
        {
            s_sys_cmd_cfg="cutycapt --url=file://"+s_session_selected+"/"+std::to_string(i)+".html --out="+s_path_wai_oa_sessions_folder+"preview/scene_"+std::to_string(i)+".png";
            i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());

            s_sys_cmd_cfg="xmlstarlet sel -t -v \"//notes\" "+s_session_selected+"/"+std::to_string(i)+".html > "+s_path_wai_oa_sessions_folder+"preview/scene_"+std::to_string(i)+".yaml";
            i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
        }
    }
    else
    {
        s_sys_cmd_cfg="python3 "+s_path_wai_oa_sessions_folder+"sessionexport.py -i "+s_session_selected+" -x "+std::to_string(-1)+" -o "+s_path_wai_oa_sessions_folder;
        i_sys_retval_cfg=std::system(s_sys_cmd_cfg.c_str());
    }

    // Update ListView with session scenes
    GetAvailableScenes(s_path_wai_oa_session_scene_previews);
    ui->lst_wai_oa_scenes->setCurrentRow(0);

    // Select default scene
    SelectWAIOAScene();
    ui->lst_wai_oa_interactions->setCurrentRow(0);
}

void win_wai_oa::on_lst_wai_oa_sessions_itemSelectionChanged()
{

}

Qt::CheckState win_wai_oa::GetWAIOAPresenterMode()
{
    return ui->chk_wai_oa_infra_pre_or_aud->checkState();
}

void win_wai_oa::on_chk_wai_oa_infra_pre_or_aud_stateChanged(int arg1)
{
    if(arg1==Qt::Checked)
    {
        ui->chk_wai_oa_infra_pre_or_aud->setText("Presenter");
        //ui->lne_wai_world_infra_ip_master->setText(ui->lne_wai_world_infra_ip_local->text());
    }
    else if(arg1==Qt::Unchecked)
    {
        ui->chk_wai_oa_infra_pre_or_aud->setText("Listener");
    }
    else
    {
        // Do nothing for now...
    }
}
