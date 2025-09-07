#ifndef WIN_WAI_WORLD_H
#define WIN_WAI_WORLD_H

#include<ros/ros.h>
#include<ros/package.h>
#include<cstdlib>
#include<dirent.h>
#include<stdio.h>
#include<string>
#include<string.h>
#include<fstream>

#include<QProcess>
#include<QMainWindow>
#include<QCoreApplication>
#include<QGuiApplication>
#include<QPushButton>
#include<QTimer>
#include<QImage>
#include<QListWidget>
#include<QListWidgetItem>
#include<QFileDialog>
#include<QtNetwork/QNetworkInterface>
#include<QtNetwork/QHostAddress>

#include<win_wai_oa.h>


QT_BEGIN_NAMESPACE
namespace Ui { class win_wai_world;}
QT_END_NAMESPACE

class win_wai_world : public QMainWindow
{
    Q_OBJECT
    
    ros::NodeHandle m_hdl_node;
    //ros::Timer m_tmr_launcher;
    //void cb_tmr_launcher(const ros::TimerEvent& event);

    QTimer* m_tmr_launcher;

    std::string s_path_wai_world_launch;
    std::string s_path_wai_oa_launch;
    QString qst_path_icons;

    QImage img_wai_world_settings_logo;
    QImage img_wai_world_reps_logo;

    bool b_profile_load_at_startup;

    void on_tmr_launcher_timeout();

public:
    win_wai_world(QWidget *parent = nullptr);
    ~win_wai_world();

    win_wai_oa* win_oa; // Create reference here and show/hide during runtime!

    void GetSetArgTextfile(std::string s_filepath,
                           std::string s_arg_name,
                           double* d_arg_value,
                           std::string* s_arg_value_text,
                           std::string s_arg_new_value="");

    void LoadWAIWorldSettingsFromFile();
    void SaveWAIWorldSettingsToFile();
    void LoadWAIWorldRepsFromFile();
    void SaveWAIWorldRepsToFile();
    void WAIWorldRepsLoadPreviewImage();

private slots:

    void on_btn_wai_world_launcher_clicked();
    void on_btn_wai_world_quit_clicked();
    void on_btn_wai_world_settings_load_clicked();
    void on_btn_wai_world_settings_save_clicked();
    void on_btn_wai_world_reps_load_clicked();
    void on_btn_wai_world_reps_save_clicked();
    void on_lst_wai_world_reps_itemClicked(QListWidgetItem *item);
    void on_btn_wai_world_reps_edit_clicked();
    void on_btn_wai_world_about_clicked();
    void on_action_About_triggered();
    void on_action_Save_As_Profile_triggered();
    void on_action_Load_Profile_triggered();
    void on_chk_wai_world_infra_pre_or_aud_stateChanged(int arg1);

private:
    Ui::win_wai_world *ui;
};
#endif // WIN_WAI_WORLD_H
