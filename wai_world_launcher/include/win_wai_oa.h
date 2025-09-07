#ifndef WIN_WAI_OA_H
#define WIN_WAI_OA_H

#include<ros/ros.h>
#include<ros/package.h>
#include<cstdlib>
#include<dirent.h>
#include<stdio.h>
#include<string>
#include<string.h>
#include<fstream>
#include<stack>
#include<boost/algorithm/string.hpp>
#include<boost/filesystem.hpp>
#include<boost/lambda/bind.hpp>

#include<QMainWindow>
#include<QDialog>
#include<QDialogButtonBox>
#include<QLayout>
#include<QSpinBox>
#include<QGuiApplication>
#include<QMessageBox>
#include<QInputDialog>
#include<QLineEdit>
#include<QImage>
#include<QIcon>
#include<QListWidget>
#include<QListWidgetItem>
#include<QFile>
#include<QFileDialog>
#include<QString>
#include<QSplitter>
#include<QStringList>
#include<QStringAlgorithms>

namespace Ui {
class win_wai_oa;
}

class win_wai_oa : public QMainWindow
{
    Q_OBJECT

    QString qst_path_icons;
    QImage img_wai_oa_settings_logo;
    QImage img_wai_oa_reps_logo;
    QImage img_wai_oa_scene_preview;
    QImage img_wai_oa_scheduler_preview;

    QImage img_wai_oa_camera_graph_3d;
    QImage img_wai_oa_camera_graph_eval;
    QImage img_wai_oa_camera_overview;
    QImage img_wai_oa_camera_presenter;

    int i_scenes_count;
    std::string s_path_wai_oa_launch;
    std::string s_path_wai_oa_sessions_folder;
    std::string s_path_wai_oa_session_folder;
    std::string s_path_wai_oa_session_scenes;
    std::string s_path_wai_oa_session_pptx;
    std::string s_path_wai_oa_session_pdf;
    std::string s_path_wai_os_session_config_template;
    std::string s_path_wai_oa_session_yaml;
    std::string s_path_scene_selected;
    std::string s_session_selected;
    std::string s_scene_selected;

public:
    explicit win_wai_oa(QWidget *parent = nullptr);
    ~win_wai_oa();

    void GetSetArgTextfile(std::string s_filepath,
                           std::string s_arg_name,
                           double* d_arg_value,
                           std::string* s_arg_value_text,
                           std::string s_arg_new_value="");

    std::vector<std::string> GetSubstringsBetweenDelimiters(std::string s, std::string delimiter);
    std::string RemoveChar(std::string str, char c);
    std::string RemoveLeadingZeros(std::string s_input);
    bool CheckIfFilExists(const std::string& name);

    void LoadWAIOASettingsFromFile();
    void SaveWAIOASettingsToFile();

    void LoadWAIOARepsFromFile();
    void SaveWAIOARepsToFile();
    void WAIOARepsLaodPreviewImage();

    void LoadWAIOASchedulerSessionsFromFile();
    void SaveWAIOASchedulerSessionsFromFile();

    void LoadWAIOASessionsFromFile();
    void SaveWAIOASessionsToFile();
    void ImportWAIOASession();
    void GetAvailableSessions(std::string s_filepath);
    void GetAvailableScenes(std::string s_filepath);
    void ValidateSessionPPT(std::string s_filepath);
    void ValidateSessionScenes(std::string s_filepath);
    void ValidateSessionYAML(std::string s_filepath);
    void AddSceneInteractionTextfile(std::string s_filepath);
    void GetAvailableInteractions();
    void GetAvailableTriggers();

    void SelectWAIOASession(std::string s_session);
    void SelectWAIOAScene();

    void LoadWAIOASchedulerTimeSlots();
    void EditWAIOASchedulerTimeSlot();
    void SaveWAIOASchedulerTimeSlots();
    void LoadWAIOASchedulerSessions();
    void EditWAIOASchedulerSession();
    void SaveWAIOASchedulerSessions();

    void RemoveSelectedInteraction();
    void EditSelectedInteraction();
    void AddAvailableInteraction();

private slots:
    void on_btn_wai_oa_close_clicked();

    void on_action_About_triggered();

    void on_action_Close_triggered();

    void on_tab_wai_oa_tabBarClicked(int index);

    void on_lst_wai_oa_sessions_itemClicked(QListWidgetItem *item);

    void on_lst_wai_oa_scenes_itemClicked(QListWidgetItem *item);

    void on_btn_wai_oa_interaction_remove_clicked();

    void on_btn_wai_oa_interaction_edit_clicked();

    void on_btn_wai_oa_interaction_add_clicked();

    void on_lst_wai_oa_interactions_available_itemClicked(QListWidgetItem *item);

    void on_btn_wai_oa_about_clicked();

    void on_btn_wai_oa_settings_load_clicked();

    void on_lst_wai_oa_reps_itemClicked(QListWidgetItem *item);

    void on_btn_wai_oa_reps_load_clicked();

    void on_btn_wai_oa_reps_save_clicked();

    void on_chk_wai_oa_settings_enable_session_scheduler_stateChanged(int arg1);

    void on_btn_wai_oa_settings_save_clicked();

    void on_lne_wai_oa_settings_enable_session_scheduler_textChanged(const QString &arg1);

    void on_btn_wai_oa_camera_graph_3d_clicked();

    void on_btn_wai_oa_camera_graph_eval_clicked();

    void on_btn_wai_oa_camera_overview_clicked();

    void on_btn_wai_oa_camera_presenter_clicked();

    void on_btn_wai_oa_scheduler_load_clicked();

    void on_lst_wai_oa_scheduler_sessions_itemClicked(QListWidgetItem *item);

    void on_cmb_wai_oa_scheduler_weekday_currentTextChanged(const QString &arg1);

    void on_btn_wai_oa_save_session_selected_default_clicked();

    void on_lst_wai_oa_scheduler_time_slots_itemClicked(QListWidgetItem *item);

    void on_btn_wai_oa_scheduler_save_clicked();

    void on_btn_wai_oa_scheduler_time_slots_clicked();

    void on_btn_wai_oa_scheduler_session_settings_clicked();

    void on_lst_wai_oa_scenes_itemSelectionChanged();

    void on_btn_wai_oa_session_import_clicked();

    void on_lst_wai_oa_sessions_itemSelectionChanged();

private:
    Ui::win_wai_oa *ui;
};

#endif // WIN_WAI_OA_H
