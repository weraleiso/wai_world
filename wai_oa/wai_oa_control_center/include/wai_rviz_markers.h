#ifndef WAI_RVIZ_MARKERS_H
#define WAI_RVIZ_MARKERS_H



/////////////////////////////////////////////////
/// Selective inclusion of common libraries
/////////////////////////////////////////////////
#include<string>
#include<boost/filesystem.hpp>

#include<ros/package.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>



/////////////////////////////////////////////////
/// Abstract base class of marker library
/////////////////////////////////////////////////
class WAIRvizMarkers
{
protected:
    static int i_marker_id;

    visualization_msgs::Marker m_mrk_wai_marker;

    std::string m_s_namespace;
    std::string m_s_frame_id;

    tf::Vector3 m_vc3_marker_position;
    tf::Quaternion m_qua_marker_orientation;
    tf::Vector3 m_vc3_marker_scale;
    std_msgs::ColorRGBA m_col_marker_rgba;

public:
    static WAIRvizMarkers *create_rviz_marker(std::string config);
    virtual void Initialize(
            std::string s_namespace="audit",
            std::string s_frame_id="world",
            std::string s_marker_label="marker",
            std::string s_path_ressources="/home")=0;
    virtual void UpdatePose(
            tf::Vector3 vc3_marker_position=tf::Vector3(0.0,0.0,0.0),
            tf::Quaternion qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0),
            tf::Vector3 vc3_marker_scale=tf::Vector3(0.0,0.0,0.0))=0;
    virtual void UpdateColor(std_msgs::ColorRGBA col_marker_rgba)=0;
    visualization_msgs::Marker GetMarker();
};



/////////////////////////////////////////////////
/// Derived classes of marker library
/////////////////////////////////////////////////
class ArrowMarker:public WAIRvizMarkers
{

public:
    ArrowMarker();
    ~ArrowMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateVector(tf::Vector3 vc3_origin,tf::Vector3 vc3_endpoint);
};


class SphereMarker:public WAIRvizMarkers
{

public:
    SphereMarker();
    ~SphereMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
};


class SpheresMarker:public WAIRvizMarkers
{
    tf::Vector3 m_vc3_spheres_offset;
    tf::Vector3 m_vc3_spheres_count;
    tf::Vector3 m_vc3_spheres_spacing;
    tf::Vector3 m_vc3_spheres_color_thresholds;

public:
    SpheresMarker();
    ~SpheresMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateSpheres(tf::Vector3 vc3_spheres_offset,tf::Vector3 vc3_spheres_count,tf::Vector3 vc3_spheres_spacing,float f_stats[255],tf::Vector3 vc3_spheres_color_thresholds=tf::Vector3(0.1,0.5,1.0));
};


class TextMarker:public WAIRvizMarkers
{
    std::string m_s_marker_text;

public:
    TextMarker();
    ~TextMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateLifetime(ros::Duration dur_lifetime);
    void UpdateText(std::string s_marker_text);
};


class GridLinesMarker:public WAIRvizMarkers
{
    int m_i_grid_count_x;
    int m_i_grid_count_y;
    float m_f_grid_spacing_x;
    float m_f_grid_spacing_y;
    float m_f_grid_max_x;
    float m_f_grid_max_y;

public:
    GridLinesMarker();
    ~GridLinesMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y);
};


class Cursor3DMarker:public WAIRvizMarkers
{

public:
    Cursor3DMarker();
    ~Cursor3DMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateCursor(float f_size,float f_size_corners);
};


class MeshMarker:public WAIRvizMarkers
{
    std::string m_s_path_ressources;
    std::string m_s_name_ressources;
    std::string m_s_path_ressources_default;
    std::string m_s_name_ressources_default;
    float m_f_marker_scale;

public:
    MeshMarker();
    ~MeshMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateAlpha(float f_alpha);
    void UpdateMesh(std::string s_marker_label,std::string s_path_ressources);
    void UpdateMeshAbsolutePath(std::string s_path_mesh_resource);
};



class HeightlineMarker:public WAIRvizMarkers
{

public:
    HeightlineMarker();
    ~HeightlineMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA qua_color_rgba);
    void UpdateHeightline(tf::Vector3 vc3_pos_heightline);
};



class HeightlinesStatsMarker:public WAIRvizMarkers
{
    float m_f_heightlines_spacing_x;
    float m_f_heightlines_spacing_y;
    float m_f_heightlines_spacing_z;

public:
    HeightlinesStatsMarker();
    ~HeightlinesStatsMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA qua_color_rgba);
    void UpdateSettings(float f_heightlines_spacing_x,float f_heightlines_spacing_y,float f_heightlines_spacing_z);
    void UpdateData(float* graph_3d_data_min,float* graph_3d_data_max,float* graph_3d_data_mean,float* graph_3d_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z);
};



class HeightlinesEvalMarker:public WAIRvizMarkers
{
    float m_f_heightlines_spacing_x;
    float m_f_heightlines_spacing_y;
    float m_f_heightlines_spacing_z;

public:
    HeightlinesEvalMarker();
    ~HeightlinesEvalMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateSettings(float f_heightlines_spacing_x,float f_heightlines_spacing_y,float f_heightlines_spacing_z);
    void UpdateData(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z,int i_id_selected=-1,bool b_use_eval_status_overall=false,float f_eval_overall[]=NULL);
};



class Data3DMarker:public WAIRvizMarkers
{
    std::string m_s_graph3d_title;
    std::string m_s_path_ressources;

    float m_f_samples_spacing_x;
    float m_f_samples_spacing_y;
    float m_f_samples_spacing_z;
    float m_f_samples_marker_scale;
    float m_f_settings_axis_x_min;
    float m_f_settings_axis_x_max;
    float m_f_settings_axis_y_min;
    float m_f_settings_axis_y_max;
    float m_f_settings_axis_z_min;
    float m_f_settings_axis_z_max;

public:
    Data3DMarker();
    ~Data3DMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateSettings(float m_f_samples_spacing_x=0.04,
                        float m_f_samples_spacing_y=0.04,
                        float m_f_samples_spacing_z=0.04,
                        float m_f_samples_marker_scale=0.01,
                        float f_settings_axis_x_min=0.0,
                        float f_settings_axis_x_max=6.28,
                        float f_settings_axis_y_min=0.0,
                        float f_settings_axis_y_max=6.28,
                        float f_settings_axis_z_min=-3.0,
                        float f_settings_axis_z_max=3.0);
    void UpdateDataFunction(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z);
    void UpdateDataEval(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z,int i_id_selected=-1,bool b_use_eval_status_overall=false,float f_eval_overall[]=NULL);
    void UpdateDataStats(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z);
};



class DataStdDevMarker:public WAIRvizMarkers
{
    std::string m_s_graph3d_title;
    std::string m_s_path_ressources;

    float m_f_samples_spacing_x;
    float m_f_samples_spacing_y;
    float m_f_samples_spacing_z;
    float m_f_samples_marker_scale;
    float m_f_settings_axis_x_min;
    float m_f_settings_axis_x_max;
    float m_f_settings_axis_y_min;
    float m_f_settings_axis_y_max;
    float m_f_settings_axis_z_min;
    float m_f_settings_axis_z_max;

public:
    DataStdDevMarker();
    ~DataStdDevMarker();
    void Initialize(std::string s_namespace="audit",std::string s_frame_id="world",std::string s_marker_label="marker",std::string s_path_ressources="/home");
    void UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale);
    void UpdateColor(std_msgs::ColorRGBA col_marker_rgba);
    void UpdateSettings(float m_f_samples_spacing_x=0.04,
                        float m_f_samples_spacing_y=0.04,
                        float m_f_samples_spacing_z=0.04,
                        float m_f_samples_marker_scale=0.01,
                        float f_settings_axis_x_min=0.0,
                        float f_settings_axis_x_max=6.28,
                        float f_settings_axis_y_min=0.0,
                        float f_settings_axis_y_max=6.28,
                        float f_settings_axis_z_min=-3.0,
                        float f_settings_axis_z_max=3.0);
    void UpdateDataStdDev(float* graph_stats_data_mean,float* graph_stats_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z);
};


#endif //WAI_RVIZ_MARKERS_H
