#ifndef WAI_REPS_H
#define WAI_REPS_H



/////////////////////////////////////////////////
/// Standard C++ libraries
/////////////////////////////////////////////////
#include<iostream>
#include<math.h>
#include<vector>
#include<string.h>
#include<sstream>
#include<fstream>



/////////////////////////////////////////////////
/// ROS libraries
/////////////////////////////////////////////////
#include<ros/ros.h>
#include<ros/package.h>
#include<tf/transform_listener.h>



/////////////////////////////////////////////////
/// Helper libraries
/////////////////////////////////////////////////
#include<wai_rviz_markers.h>



/////////////////////////////////////////////////
/// Abstract base class of represen. library
/////////////////////////////////////////////////

class WAIReps
{
protected:
    static int i_rep_id;

    ros::NodeHandle* m_hdl_node;
    std::string m_s_namespace;
    std::string m_s_topic;
    std::string m_s_frame;
    WAIRvizMarkers* m_mrk_wai_rep;
    std::vector<WAIRvizMarkers*> m_vec_mrk_wai_rep;
    visualization_msgs::MarkerArray m_mrk_wai_rep_array;
    ros::Publisher m_pub_mrk_wai_rep_array;

    ros::Timer m_tmr_trigger_timeout;
    virtual void cb_tmr_timeout(const ros::TimerEvent& event)=0;

public:
    static WAIReps *create_representative(std::string config);
    virtual void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)=0;
    virtual void UpdateModel()=0;
    virtual void UpdateView()=0;
};



/////////////////////////////////////////////////
/// Helper classes for reps objects
/////////////////////////////////////////////////
class WAIRepTrigger:public WAIReps
{
    tf::Vector3 m_vc3_trigger_position;
    tf::Quaternion m_qua_trigger_orientation;
    tf::Vector3 m_vc3_trigger_scale;
    std_msgs::ColorRGBA m_col_trigger_rgba;

    std_msgs::ColorRGBA m_col_trigger_grey;
    float m_f_trigger_stand_thickness;
    float m_f_trigger_head_diameter;
    float m_f_trigger_label_size;
    std::string m_s_trigger_label;
    bool m_b_trigger_enabled;

    ros::Timer m_tmr_trigger_timeout;
    void cb_tmr_timeout(const ros::TimerEvent& event);

public:
    WAIRepTrigger();
    ~WAIRepTrigger();
    void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame);
    void UpdateModel();
    void UpdateModel(tf::Vector3 vc3_trigger_position,tf::Quaternion qua_trigger_orientation,tf::Vector3 vc3_trigger_scale,std::string s_trigger_label,std_msgs::ColorRGBA col_trigger_rgba,bool b_trigger_enabled,bool b_trigger_timeout,float f_trigger_timeout);
    void UpdateView();
    tf::Vector3 GetPosition();
};



class WAIRepOOI:public WAIReps
{
    tf::Vector3 m_vc3_ooi_position;
    tf::Quaternion m_qua_ooi_orientation;
    tf::Vector3 m_vc3_ooi_scale;
    std_msgs::ColorRGBA m_col_ooi_rgba;

    //tf::TransformListener m_lst_tranforms;
    //tf::StampedTransform m_tfs_world_wrt_rep;
    //tf::Vector3 m_vc3_ooi_position_world;
    //tf::Quaternion m_qua_ooi_orientation_world;

    std_msgs::ColorRGBA m_col_ooi_grey;
    std_msgs::ColorRGBA m_col_ooi_white;
    float m_f_ooi_stand_thickness;
    float m_f_ooi_head_diameter;
    float m_f_ooi_label_size;
    std::string m_s_ooi_label;

    ros::Timer m_tmr_ooi_timeout;
    void cb_tmr_timeout(const ros::TimerEvent& event);

public:
    WAIRepOOI();
    ~WAIRepOOI();
    void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame);
    void UpdateModel();
    void UpdateModel(tf::Vector3 vc3_ooi_position,
                     tf::Quaternion qua_ooi_orientation,
                     tf::Vector3 vc3_ooi_scale,
                     std::string s_ooi_label,
                     std_msgs::ColorRGBA col_ooi_rgba,
                     std::string s_ooi_mesh_label,
                     std::string s_ooi_mesh_path_resources,
                     bool b_update_mesh=false,
                     float f_scale_mesh=1.0,
                     float f_alpha_mesh=1.0,
                     bool b_is_camera=false);
    void UpdateView();
    tf::Vector3 GetPosition();
};



class WAIRepWIM:public WAIReps
{
    tf::Vector3 m_vc3_wim_position;
    tf::Quaternion m_qua_wim_orientation;
    //tf::Vector3 m_vc3_wim_position_world;
    //tf::Quaternion m_qua_wim_orientation_world;
    tf::Vector3 m_vc3_wim_scale;

    //tf::TransformListener m_lst_tranforms;
    //tf::StampedTransform m_tfs_world_wrt_rep;

    float m_f_wim_stand_thickness;
    float m_f_wim_head_diameter;
    float m_f_wim_label_size;

    tf::Vector3 m_vc3_wim_audience_offset;
    tf::Vector3 m_vc3_wim_audience_count;
    tf::Vector3 m_vc3_wim_audience_spacing;
    tf::Vector3 m_vc3_wim_audience_focused;
    int m_i_audience_id_selected;
    int m_i_audience_count_max;
    float m_f_audience_stats_ping[255];

    std_msgs::ColorRGBA m_col_grey;
    std_msgs::ColorRGBA m_col_white_trans;
    std_msgs::ColorRGBA m_col_white;
    std_msgs::ColorRGBA m_col_white_opaque;
    std_msgs::ColorRGBA m_col_oa;
    std_msgs::ColorRGBA m_col_invisible;

    std_msgs::ColorRGBA m_col_wim_rgba;
    std_msgs::ColorRGBA m_col_wim_labels;
    std::string m_s_wim_label;
    std::string m_s_wim_label_cursor;
    std::stringstream m_sst_wim_label_cursor_id;

    ros::Timer m_tmr_trigger_timeout;
    void cb_tmr_timeout(const ros::TimerEvent& event);

public:
    WAIRepWIM();
    ~WAIRepWIM();
    void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame);
    void InitializeWithAudienceCount(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame,int i_audience_count_max,tf::Vector3 vc3_wim_scale);
    void UpdateModel();
    void UpdateModel(std::string s_wim_mesh_label,std::string s_wim_mesh_path_resources);
    void UpdateModel(tf::Vector3 vc3_wim_position,tf::Quaternion qua_wim_orientation,std::string s_wim_label,std::string s_wim_label_cursor,std_msgs::ColorRGBA col_wim_rgba,tf::Vector3 vc3_wim_offset,tf::Vector3 vc3_wim_audience_count,tf::Vector3 vc3_wim_audience_spacing,tf::Vector3 vc3_wim_audience_focused,int i_audience_id_selected,std::string s_wim_mesh_label,std::string s_wim_mesh_path_resources,bool b_update_mesh=false);
    void UpdateStats(float f_audience_stats_ping[255]);
    tf::Vector3 GetScale();
    void UpdateView();
};



class WAIRepGraph3D:public WAIReps
{
    tf::Vector3 m_vc3_position;
    tf::Quaternion m_qua_orientation;
    tf::Vector3 m_vc3_outer_bounds;

    std_msgs::ColorRGBA m_col_grey;
    std_msgs::ColorRGBA m_col_white_trans;
    std_msgs::ColorRGBA m_col_white;
    std_msgs::ColorRGBA m_col_white_opaque;
    std_msgs::ColorRGBA m_col_green_opaque;
    std_msgs::ColorRGBA m_col_cyan_opaque;
    std_msgs::ColorRGBA m_col_oa;
    std_msgs::ColorRGBA m_col_invisible;

    std_msgs::ColorRGBA m_col_axis_x;
    std_msgs::ColorRGBA m_col_axis_y;
    std_msgs::ColorRGBA m_col_axis_z;
    std_msgs::ColorRGBA m_col_grid;
    std_msgs::ColorRGBA m_col_labels;
    std::string m_s_label_title;
    std::string m_s_label_axis_x;
    std::string m_s_label_axis_y;
    std::string m_s_label_axis_z;
    std::string m_s_label_cursor;

    int m_i_grid_count_x;
    int m_i_grid_count_y;
    float m_f_grid_spacing_x;
    float m_f_grid_spacing_y;

    ros::Timer m_tmr_trigger_timeout;
    void cb_tmr_timeout(const ros::TimerEvent& event);

public:
    WAIRepGraph3D();
    ~WAIRepGraph3D();
    void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame);
    void UpdateModel();
    void UpdateModel(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_outer_bounds);
    void UpdateLabelsAndColors(std::string s_label_axis_x,std::string s_label_axis_y,std::string s_label_axis_z,std::string s_label_title,std::string s_label_axis_cursor,std_msgs::ColorRGBA col_axis_x,std_msgs::ColorRGBA col_axis_y,std_msgs::ColorRGBA col_axis_z);
    void InitGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y);
    void UpdateGrid(float f_grid_thickness,std_msgs::ColorRGBA col_grid);
    void UpdateAxes(float f_samples_spacing_x,float f_samples_spacing_y,float f_samples_spacing_z,float f_samples_marker_scale,float f_settings_axis_x_min,float f_settings_axis_x_max,float f_settings_axis_y_min,float f_settings_axis_y_max,float f_settings_axis_z_min,float f_settings_axis_z_max);
    void UpdateGraphDataEval(float* graph_3d_data_part,float* graph_3d_data_exam,float* graph_3d_data_overall,int i_data_size_x,int i_data_size_y,int i_data_size_z,float f_eval_part_scores[],float f_eval_part_fracs[],float f_eval_exam_scores[],float f_eval_exam_fracs[],float f_eval_overall[],std::string s_eval_overall[],int i_id_selected=-1);
    void UpdateGraphDataFunction(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z);
    void UpdateGraphCursor(tf::Vector3 vc3_label_cursor_position,std::string s_label_cursor);
    void UpdateView();
};



class WAIRepGraphStats:public WAIReps
{
    tf::Vector3 m_vc3_position;
    tf::Quaternion m_qua_orientation;
    tf::Vector3 m_vc3_outer_bounds;

    std_msgs::ColorRGBA m_col_grey;
    std_msgs::ColorRGBA m_col_white_trans;
    std_msgs::ColorRGBA m_col_white;
    std_msgs::ColorRGBA m_col_white_opaque;
    std_msgs::ColorRGBA m_col_red_opaque;
    std_msgs::ColorRGBA m_col_green_opaque;
    std_msgs::ColorRGBA m_col_orange;
    std_msgs::ColorRGBA m_col_orange_opaque;
    std_msgs::ColorRGBA m_col_cyan_opaque;
    std_msgs::ColorRGBA m_col_oa;
    std_msgs::ColorRGBA m_col_invisible;

    std_msgs::ColorRGBA m_col_axis_x;
    std_msgs::ColorRGBA m_col_axis_y;
    std_msgs::ColorRGBA m_col_axis_z;
    std_msgs::ColorRGBA m_col_grid;
    std_msgs::ColorRGBA m_col_labels;
    std::string m_s_label_title;
    std::string m_s_label_axis_x;
    std::string m_s_label_axis_y;
    std::string m_s_label_axis_z;
    std::string m_s_label_cursor;

    int m_i_grid_count_x;
    int m_i_grid_count_y;
    float m_f_grid_spacing_x;
    float m_f_grid_spacing_y;

    ros::Timer m_tmr_trigger_timeout;
    void cb_tmr_timeout(const ros::TimerEvent& event);

public:
    WAIRepGraphStats();
    ~WAIRepGraphStats();
    void Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame);
    void UpdateModel();
    void UpdateModel(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_outer_bounds);
    void UpdateLabelsAndColors(std::string s_label_axis_x,std::string s_label_axis_y,std::string s_label_axis_z,std::string s_label_title,std::string s_label_axis_cursor,std_msgs::ColorRGBA col_axis_x,std_msgs::ColorRGBA col_axis_y,std_msgs::ColorRGBA col_axis_z);
    void InitGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y);
    void UpdateGrid(float f_grid_thickness,std_msgs::ColorRGBA col_grid);
    void UpdateAxes(float f_samples_spacing_x,float f_samples_spacing_y,float f_samples_spacing_z,float f_samples_marker_scale,float f_settings_axis_x_min,float f_settings_axis_x_max,float f_settings_axis_y_min,float f_settings_axis_y_max,float f_settings_axis_z_min,float f_settings_axis_z_max);
    void UpdateGraphDataStats(float* graph_3d_data_min,float* graph_3d_data_max,float* graph_3d_data_mean,float* graph_3d_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z);
    void UpdateGraphCursor(tf::Vector3 vc3_label_cursor_position,std::string s_label_cursor);
    void UpdateView();
};

#endif //WAI_REPS_H
