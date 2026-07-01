#include<wai_rviz_markers.h>



/////////////////////////////////////////////////
/// Marker IDs of marker library
/////////////////////////////////////////////////
int WAIRvizMarkers::i_marker_id=0;



/////////////////////////////////////////////////
/// Implementation of abstract base class
/////////////////////////////////////////////////
WAIRvizMarkers *WAIRvizMarkers::create_rviz_marker(std::string config)
{
    WAIRvizMarkers::i_marker_id++;

    if(config.compare("SPHERE")==0)
    {
        return new SphereMarker;
    }
    else if(config.compare("TEXT")==0)
    {
        return new TextMarker;
    }
    else if(config.compare("ARROW")==0)
    {
        return new ArrowMarker;
    }
    else if(config.compare("MESH")==0)
    {
        return new MeshMarker;
    }
    else if(config.compare("SPHERES")==0) // Markers with points/colors
    {
        return new SpheresMarker;
    }
    else if(config.compare("HEIGHTLINE")==0)
    {
        return new HeightlineMarker;
    }
    else if(config.compare("HEIGHTLINES_STATS")==0)
    {
        return new HeightlinesStatsMarker;
    }
    else if(config.compare("HEIGHTLINES_EVAL")==0)
    {
        return new HeightlinesEvalMarker;
    }
    else if(config.compare("GRIDLINES")==0)
    {
        return new GridLinesMarker;
    }
    else if(config.compare("CURSOR_3D")==0)
    {
        return new Cursor3DMarker;
    }
    else if(config.compare("DATA_3D")==0)
    {
        return new Data3DMarker;
    }
    else if(config.compare("DATA_STD_DEV")==0)
    {
        return new DataStdDevMarker;
    }
    else
    {
        return NULL;
    }
}
visualization_msgs::Marker WAIRvizMarkers::GetMarker()
{
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    return m_mrk_wai_marker;
}



/////////////////////////////////////////////////
/// Implementations of derived classes
/////////////////////////////////////////////////

SphereMarker::SphereMarker()
{
}
SphereMarker::~SphereMarker()
{
}
void SphereMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::SPHERE;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;
}
void SphereMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
}
void SphereMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}



TextMarker::TextMarker()
{
}
TextMarker::~TextMarker()
{
}
void TextMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.1);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.0; // Set to invisible on init.

    m_mrk_wai_marker.header.frame_id = m_s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;

    m_mrk_wai_marker.text=s_marker_label;
}
void TextMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
}
void TextMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void TextMarker::UpdateText(std::string s_marker_text)
{
    m_s_marker_text=s_marker_text;
    m_mrk_wai_marker.text=m_s_marker_text;
}
void TextMarker::UpdateLifetime(ros::Duration dur_lifetime)
{
    m_mrk_wai_marker.lifetime=dur_lifetime;
}



ArrowMarker::ArrowMarker()
{
}
ArrowMarker::~ArrowMarker()
{
}
void ArrowMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::ARROW;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
}
void ArrowMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
}
void ArrowMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void ArrowMarker::UpdateVector(tf::Vector3 vc3_origin,tf::Vector3 vc3_endpoint)
{
    m_mrk_wai_marker.points.clear();
    geometry_msgs::Point pnt_point;
    pnt_point.x=vc3_origin.getX();
    pnt_point.y=vc3_origin.getY();
    pnt_point.z=vc3_origin.getZ();
    m_mrk_wai_marker.points.push_back(pnt_point);
    pnt_point.x=vc3_endpoint.getX();
    pnt_point.y=vc3_endpoint.getY();
    pnt_point.z=vc3_endpoint.getZ();
    m_mrk_wai_marker.points.push_back(pnt_point);
}



MeshMarker::MeshMarker()
{
}
MeshMarker::~MeshMarker()
{
}
void MeshMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;
    m_s_path_ressources=s_path_ressources;
    m_s_name_ressources=s_marker_label;

    m_s_path_ressources_default=ros::package::getPath("wai_oa_gazebo")+"/resources/reps/";
    m_s_name_ressources_default="logo";

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = m_s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    //m_mrk_wai_marker.color = m_col_marker_rgba; Is skipped for this marker type!

    m_mrk_wai_marker.mesh_use_embedded_materials=true;

    // Check if mesh resource file exists
    std::string s_path_mesh=m_s_path_ressources+m_s_name_ressources+"/"+m_s_name_ressources+".dae";
    if(boost::filesystem::exists(s_path_mesh))
    {
        m_mrk_wai_marker.mesh_resource = "file://"+s_path_mesh;
    }
    else
    {
        m_mrk_wai_marker.mesh_resource = "file://"+m_s_path_ressources_default+m_s_name_ressources_default+"/"+m_s_name_ressources_default+".dae";
    }
}
void MeshMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.mesh_use_embedded_materials=true;
}
void MeshMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    // Do not change color settings here...
    //m_mrk_wai_marker.color=col_marker_rgba;
}
void MeshMarker::UpdateAlpha(float f_alpha)
{
    // Do not change color settings here...
    //m_mrk_wai_marker.color.a=f_alpha;
}
void MeshMarker::UpdateMesh(std::string s_marker_label,std::string s_path_ressources)
{
    m_s_name_ressources=s_marker_label;
    m_s_path_ressources=s_path_ressources;

    std::string s_path_mesh=m_s_path_ressources+m_s_name_ressources+"/"+m_s_name_ressources+".dae";
    if(boost::filesystem::exists(s_path_mesh))
    {
        m_mrk_wai_marker.mesh_resource = "file://"+s_path_mesh;
    }
    else
    {
        m_mrk_wai_marker.mesh_resource = "file://"+m_s_path_ressources_default+m_s_name_ressources_default+"/"+m_s_name_ressources_default+".dae";
    }
    m_mrk_wai_marker.mesh_use_embedded_materials=1;
}
void MeshMarker::UpdateMeshAbsolutePath(std::string s_path_mesh_resource)
{
    if(boost::filesystem::exists(s_path_mesh_resource))
    {
        m_mrk_wai_marker.mesh_resource = "file://"+s_path_mesh_resource;
    }
    else
    {
        m_mrk_wai_marker.mesh_resource = "file://"+m_s_path_ressources_default+m_s_name_ressources_default+"/"+m_s_name_ressources_default+".dae";
    }
    m_mrk_wai_marker.mesh_use_embedded_materials=1;
}



SpheresMarker::SpheresMarker()
{
}
SpheresMarker::~SpheresMarker()
{
}
void SpheresMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);

    m_vc3_spheres_offset=tf::Vector3(0.0,0.0,0.0);
    m_vc3_spheres_count=tf::Vector3(0.0,0.0,0.0);
    m_vc3_spheres_spacing=tf::Vector3(0.0,0.0,0.0);
    m_vc3_spheres_color_thresholds=tf::Vector3(0.1,0.5,1.0);

    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);
}
void SpheresMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
}
void SpheresMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void SpheresMarker::UpdateSpheres(tf::Vector3 vc3_spheres_offset,tf::Vector3 vc3_spheres_count,tf::Vector3 vc3_spheres_spacing,float f_stats[255],tf::Vector3 vc3_spheres_color_thresholds)
{   
    m_vc3_spheres_offset=vc3_spheres_offset;
    m_vc3_spheres_count=vc3_spheres_count;
    m_vc3_spheres_spacing=vc3_spheres_spacing;
    m_vc3_spheres_color_thresholds=vc3_spheres_color_thresholds;

    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    int i_audience_counter=0;

    for(int x=0;x<m_vc3_spheres_count.getX();x++)
    {
        for(int y=0;y<m_vc3_spheres_count.getY();y++)
        {
            for(int z=0;z<m_vc3_spheres_count.getZ();z++)
            {
                geometry_msgs::Point pnt_sphere_new;
                pnt_sphere_new.x=m_vc3_spheres_offset.getX()+x*m_vc3_spheres_spacing.getX();
                pnt_sphere_new.y=m_vc3_spheres_offset.getY()+y*m_vc3_spheres_spacing.getY();
                pnt_sphere_new.z=m_vc3_spheres_offset.getZ()+z*m_vc3_spheres_spacing.getZ();
                m_mrk_wai_marker.points.push_back(pnt_sphere_new);

                std_msgs::ColorRGBA col_sphere_new;
                if(f_stats[i_audience_counter]==0.0)
                {
                    col_sphere_new=m_col_marker_rgba;
                }
                else if(f_stats[i_audience_counter]<m_vc3_spheres_color_thresholds.getX())
                {
                    col_sphere_new.a=0.9;
                    col_sphere_new.r=0.0;
                    col_sphere_new.g=1.0;
                    col_sphere_new.b=0.0;
                }
                else if(f_stats[i_audience_counter]<m_vc3_spheres_color_thresholds.getY())
                {
                    col_sphere_new.a=0.9;
                    col_sphere_new.r=1.0;
                    col_sphere_new.g=0.65;
                    col_sphere_new.b=0.5;
                }
                else if(f_stats[i_audience_counter]<m_vc3_spheres_color_thresholds.getZ())
                {
                    col_sphere_new.a=0.9;
                    col_sphere_new.r=1.0;
                    col_sphere_new.g=0.0;
                    col_sphere_new.b=0.0;
                }
                else
                {
                    col_sphere_new=m_col_marker_rgba;
                }
                m_mrk_wai_marker.colors.push_back(col_sphere_new);

                i_audience_counter++;
            }
        }
    }
}



HeightlineMarker::HeightlineMarker()
{
}
HeightlineMarker::~HeightlineMarker()
{
}
void HeightlineMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::LINE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init two points and colors for 1 HEIGHTLINE
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);
}
void HeightlineMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
}
void HeightlineMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.colors[0]=m_col_marker_rgba;
    m_mrk_wai_marker.colors[1]=m_col_marker_rgba;
}
void HeightlineMarker::UpdateHeightline(tf::Vector3 vc3_pos_heightline)
{
    m_mrk_wai_marker.header.stamp=ros::Time::now();
    m_mrk_wai_marker.points[0].x=vc3_pos_heightline.getX();
    m_mrk_wai_marker.points[0].y=vc3_pos_heightline.getY();
    m_mrk_wai_marker.points[0].z=0.0;
    m_mrk_wai_marker.points[1].x=vc3_pos_heightline.getX();
    m_mrk_wai_marker.points[1].y=vc3_pos_heightline.getY();
    m_mrk_wai_marker.points[1].z=vc3_pos_heightline.getZ();
}



HeightlinesStatsMarker::HeightlinesStatsMarker()
{
}
HeightlinesStatsMarker::~HeightlinesStatsMarker()
{
}
void HeightlinesStatsMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_f_heightlines_spacing_x=0.0;
    m_f_heightlines_spacing_y=0.0;
    m_f_heightlines_spacing_z=0.0;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::LINE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);
}
void HeightlinesStatsMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
}
void HeightlinesStatsMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void HeightlinesStatsMarker::UpdateSettings(float f_heightlines_spacing_x,float f_heightlines_spacing_y,float f_heightlines_spacing_z)
{
    m_f_heightlines_spacing_x=f_heightlines_spacing_x;
    m_f_heightlines_spacing_y=f_heightlines_spacing_y;
    m_f_heightlines_spacing_z=f_heightlines_spacing_z;
}
void HeightlinesStatsMarker::UpdateData(float* graph_3d_data_min,float* graph_3d_data_max,float* graph_3d_data_mean,float* graph_3d_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z)
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Update EVAL DATA HEIGHTLINES
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz_min=*(graph_3d_data_min+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                float f_xyz_max=*(graph_3d_data_max+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                float f_xyz_mean=*(graph_3d_data_mean+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                float f_xyz_stddev=*(graph_3d_data_stddev+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);

                // Draw from MIN to lower bound STDDEV
                geometry_msgs::Point pnt_data_min;
                pnt_data_min.x=x*m_f_heightlines_spacing_x;
                pnt_data_min.y=y*m_f_heightlines_spacing_y;
                pnt_data_min.z=z*m_f_heightlines_spacing_z+f_xyz_min;
                m_mrk_wai_marker.points.push_back(pnt_data_min);
                geometry_msgs::Point pnt_data_stddev_lower;
                pnt_data_stddev_lower.x=x*m_f_heightlines_spacing_x;
                pnt_data_stddev_lower.y=y*m_f_heightlines_spacing_y;
                pnt_data_stddev_lower.z=z*m_f_heightlines_spacing_z+f_xyz_mean-f_xyz_stddev;
                m_mrk_wai_marker.points.push_back(pnt_data_stddev_lower);

                // Draw from lower bound STDDEV to upper bound STDDEV
                m_mrk_wai_marker.points.push_back(pnt_data_stddev_lower);
                geometry_msgs::Point pnt_data_stddev_upper;
                pnt_data_stddev_upper.x=x*m_f_heightlines_spacing_x;
                pnt_data_stddev_upper.y=y*m_f_heightlines_spacing_y;
                pnt_data_stddev_upper.z=z*m_f_heightlines_spacing_z+f_xyz_mean+f_xyz_stddev;
                m_mrk_wai_marker.points.push_back(pnt_data_stddev_upper);

                // Draw from upper bound STDDEV to MAX
                m_mrk_wai_marker.points.push_back(pnt_data_stddev_upper);
                geometry_msgs::Point pnt_data_max;
                pnt_data_max.x=x*m_f_heightlines_spacing_x;
                pnt_data_max.y=y*m_f_heightlines_spacing_y;
                pnt_data_max.z=z*m_f_heightlines_spacing_z+f_xyz_max;
                m_mrk_wai_marker.points.push_back(pnt_data_max);

                // Draw with WHITE and ORANGE colors
                std_msgs::ColorRGBA col_data_min_max;
                col_data_min_max.r=1.0;
                col_data_min_max.g=1.0;
                col_data_min_max.b=1.0;
                col_data_min_max.a=0.5;
                m_mrk_wai_marker.colors.push_back(col_data_min_max);
                m_mrk_wai_marker.colors.push_back(col_data_min_max);
                m_mrk_wai_marker.colors.push_back(m_col_marker_rgba);
                m_mrk_wai_marker.colors.push_back(m_col_marker_rgba);
                m_mrk_wai_marker.colors.push_back(col_data_min_max);
                m_mrk_wai_marker.colors.push_back(col_data_min_max);
            }
        }
    }
}



HeightlinesEvalMarker::HeightlinesEvalMarker()
{
}
HeightlinesEvalMarker::~HeightlinesEvalMarker()
{
}
void HeightlinesEvalMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_f_heightlines_spacing_x=0.0;
    m_f_heightlines_spacing_y=0.0;
    m_f_heightlines_spacing_z=0.0;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::LINE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);
}
void HeightlinesEvalMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
}
void HeightlinesEvalMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void HeightlinesEvalMarker::UpdateSettings(float f_heightlines_spacing_x,float f_heightlines_spacing_y,float f_heightlines_spacing_z)
{
    m_f_heightlines_spacing_x=f_heightlines_spacing_x;
    m_f_heightlines_spacing_y=f_heightlines_spacing_y;
    m_f_heightlines_spacing_z=f_heightlines_spacing_z;
}
void HeightlinesEvalMarker::UpdateData(float* graph_3d_data,
                                       int i_data_size_x,
                                       int i_data_size_y,
                                       int i_data_size_z,
                                       int i_id_selected,
                                       bool b_use_eval_status_overall,
                                       float f_eval_overall[])
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Update EVAL DATA HEIGHTLINES
    for(int x=0;x<i_data_size_x;x++)
    {
        float f_score_count=0;
        float f_score_sum=0.0;

        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz=*(graph_3d_data+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                if(f_xyz<-1.0 || (i_id_selected!=-1 && i_id_selected!=x) ) continue;

                f_score_count=f_score_count+1.0;
                f_score_sum=f_score_sum+f_xyz;

                // First line (score) --> z[0] is empty since we start counting(1) with 1 equals to 1 eval!
                geometry_msgs::Point pnt_score_bottom;
                pnt_score_bottom.x=x*m_f_heightlines_spacing_x;
                pnt_score_bottom.y=y*m_f_heightlines_spacing_y;
                pnt_score_bottom.z=z*m_f_heightlines_spacing_z-m_f_heightlines_spacing_z;
                m_mrk_wai_marker.points.push_back(pnt_score_bottom);
                geometry_msgs::Point pnt_score_top;
                pnt_score_top.x=x*m_f_heightlines_spacing_x;
                pnt_score_top.y=y*m_f_heightlines_spacing_y;
                pnt_score_top.z=z*m_f_heightlines_spacing_z+f_xyz*m_f_heightlines_spacing_z-m_f_heightlines_spacing_z;
                m_mrk_wai_marker.points.push_back(pnt_score_top);
                std_msgs::ColorRGBA col_score_bottom,col_score_top;
                col_score_bottom.r=0.5;
                col_score_bottom.g=f_xyz;
                col_score_bottom.b=0.0;
                col_score_bottom.a=1.0;
                col_score_top=col_score_bottom;
                m_mrk_wai_marker.colors.push_back(col_score_bottom);
                m_mrk_wai_marker.colors.push_back(col_score_top);

                // Second line
                geometry_msgs::Point pnt_score_max_bottom;
                pnt_score_max_bottom.x=x*m_f_heightlines_spacing_x;
                pnt_score_max_bottom.y=y*m_f_heightlines_spacing_y;
                pnt_score_max_bottom.z=z*m_f_heightlines_spacing_z+f_xyz*m_f_heightlines_spacing_z-m_f_heightlines_spacing_z;
                m_mrk_wai_marker.points.push_back(pnt_score_max_bottom);
                geometry_msgs::Point pnt_score_max_top;
                pnt_score_max_top.x=x*m_f_heightlines_spacing_x;
                pnt_score_max_top.y=y*m_f_heightlines_spacing_y;
                pnt_score_max_top.z=z*m_f_heightlines_spacing_z+m_f_heightlines_spacing_z-m_f_heightlines_spacing_z;
                m_mrk_wai_marker.points.push_back(pnt_score_max_top);
                std_msgs::ColorRGBA col_score_max_bottom,col_score_max_top;
                col_score_max_bottom.r=1.0;
                col_score_max_bottom.g=1.0;
                col_score_max_bottom.b=1.0;
                col_score_max_bottom.a=1.0;
                col_score_max_top=col_score_max_bottom;
                m_mrk_wai_marker.colors.push_back(col_score_max_bottom);
                m_mrk_wai_marker.colors.push_back(col_score_max_top);
            }
        }

        // Summary HEIGHTLINE per ID
        geometry_msgs::Point pnt_score_bottom;
        pnt_score_bottom.x=x*m_f_heightlines_spacing_x;
        pnt_score_bottom.y=i_data_size_y*m_f_heightlines_spacing_y;
        pnt_score_bottom.z=0.0;
        m_mrk_wai_marker.points.push_back(pnt_score_bottom);
        if(f_score_count>=1.0) // if enough evals only
        {
            geometry_msgs::Point pnt_score_top;
            pnt_score_top.x=x*m_f_heightlines_spacing_x;
            pnt_score_top.y=i_data_size_y*m_f_heightlines_spacing_y;
            if(b_use_eval_status_overall) pnt_score_top.z=f_eval_overall[x];
            else pnt_score_top.z=f_score_sum/f_score_count;
            m_mrk_wai_marker.points.push_back(pnt_score_top);
            std_msgs::ColorRGBA col_score_bottom,col_score_top;
            col_score_bottom.r = 0.5;
            if(b_use_eval_status_overall) col_score_bottom.g = f_eval_overall[x];
            else col_score_bottom.g = f_score_sum/f_score_count;
            col_score_bottom.b = 0.0;
            col_score_bottom.a = 0.9;
            col_score_top=col_score_bottom;
            m_mrk_wai_marker.colors.push_back(col_score_bottom);
            m_mrk_wai_marker.colors.push_back(col_score_top);
        }
        else
        {
            geometry_msgs::Point pnt_score_top;
            pnt_score_top.x=x*m_f_heightlines_spacing_x;
            pnt_score_top.y=i_data_size_y*m_f_heightlines_spacing_y;
            pnt_score_top.z=1.0;
            m_mrk_wai_marker.points.push_back(pnt_score_top);

            std_msgs::ColorRGBA col_score_bottom,col_score_top;
            col_score_bottom.r = 0.5;
            col_score_bottom.g = 0.5;
            col_score_bottom.b = 0.5;
            col_score_bottom.a = 0.5;
            col_score_top=col_score_bottom;
            m_mrk_wai_marker.colors.push_back(col_score_bottom);
            m_mrk_wai_marker.colors.push_back(col_score_top);
        }

        // Greyed out HEIGHTLINE
        if(f_score_count>=1.0) // if enough evals only
        {
            geometry_msgs::Point pnt_score_bottom_2;
            pnt_score_bottom_2.x=x*m_f_heightlines_spacing_x;
            pnt_score_bottom_2.y=i_data_size_y*m_f_heightlines_spacing_y;
            if(b_use_eval_status_overall) pnt_score_bottom_2.z=f_eval_overall[x];
            else pnt_score_bottom_2.z=f_score_sum/f_score_count;
            m_mrk_wai_marker.points.push_back(pnt_score_bottom_2);

            geometry_msgs::Point pnt_score_top_2;
            pnt_score_top_2.x=x*m_f_heightlines_spacing_x;
            pnt_score_top_2.y=i_data_size_y*m_f_heightlines_spacing_y;
            pnt_score_top_2.z=1.0;
            m_mrk_wai_marker.points.push_back(pnt_score_top_2);
            std_msgs::ColorRGBA col_score_bottom,col_score_top;
            col_score_bottom.r = 1.0;
            col_score_bottom.g = 1.0;
            col_score_bottom.b = 1.0;
            col_score_bottom.a = 1.0;
            col_score_top=col_score_bottom;
            m_mrk_wai_marker.colors.push_back(col_score_bottom);
            m_mrk_wai_marker.colors.push_back(col_score_top);
        }
        else
        {
            geometry_msgs::Point pnt_score_bottom_2;
            pnt_score_bottom_2.x=x*m_f_heightlines_spacing_x;
            pnt_score_bottom_2.y=i_data_size_y*m_f_heightlines_spacing_y;
            pnt_score_bottom_2.z=0.0;
            m_mrk_wai_marker.points.push_back(pnt_score_bottom_2);
            geometry_msgs::Point pnt_score_top_2;
            pnt_score_top_2.x=x*m_f_heightlines_spacing_x;
            pnt_score_top_2.y=i_data_size_y*m_f_heightlines_spacing_y;
            pnt_score_top_2.z=1.0;
            m_mrk_wai_marker.points.push_back(pnt_score_top_2);

            std_msgs::ColorRGBA col_score_bottom,col_score_top;
            col_score_bottom.r = 0.5;
            col_score_bottom.g = 0.5;
            col_score_bottom.b = 0.5;
            col_score_bottom.a = 0.5;
            col_score_top=col_score_bottom;
            m_mrk_wai_marker.colors.push_back(col_score_bottom);
            m_mrk_wai_marker.colors.push_back(col_score_top);
        }
    }
}



GridLinesMarker::GridLinesMarker()
{
}
GridLinesMarker::~GridLinesMarker()
{
}
void GridLinesMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_i_grid_count_x=0;
    m_i_grid_count_y=0;
    m_f_grid_spacing_x=0.0;
    m_f_grid_spacing_y=0.0;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::LINE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    //m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    //std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    //m_mrk_wai_marker.colors.push_back(col_init);
    //m_mrk_wai_marker.colors.push_back(col_init);
}
void GridLinesMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
}
void GridLinesMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void GridLinesMarker::UpdateGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y)
{
    m_mrk_wai_marker.points.clear();
    //m_mrk_wai_marker.colors.clear();

    m_i_grid_count_x=i_grid_count_x;
    m_i_grid_count_y=i_grid_count_y;
    m_f_grid_spacing_x=f_grid_spacing_x;
    m_f_grid_spacing_y=f_grid_spacing_y;

    m_f_grid_max_x=m_i_grid_count_x*m_f_grid_spacing_x;
    m_f_grid_max_y=m_i_grid_count_y*m_f_grid_spacing_y;

    for(int x=1;x<=i_grid_count_x;x++) //First gridline aligned to axis!
    {
        geometry_msgs::Point pnt_gridline_x;
        pnt_gridline_x.x=m_vc3_marker_position.getX()+x*m_f_grid_spacing_x;
        pnt_gridline_x.y=0.0;
        pnt_gridline_x.z=0.0;
        m_mrk_wai_marker.points.push_back(pnt_gridline_x);
        pnt_gridline_x.x=x*m_f_grid_spacing_x;
        pnt_gridline_x.y=m_f_grid_max_y;
        pnt_gridline_x.z=0.0;
        m_mrk_wai_marker.points.push_back(pnt_gridline_x);
    }
    for(int y=1;y<=i_grid_count_y;y++) //First gridline aligned to axis!
    {
        geometry_msgs::Point pnt_gridline_y;
        pnt_gridline_y.x=0.0;
        pnt_gridline_y.y=y*m_f_grid_spacing_y;
        pnt_gridline_y.z=0.0;
        m_mrk_wai_marker.points.push_back(pnt_gridline_y);
        pnt_gridline_y.x=m_f_grid_max_x;
        pnt_gridline_y.y=y*m_f_grid_spacing_y;
        pnt_gridline_y.z=0.0;
        m_mrk_wai_marker.points.push_back(pnt_gridline_y);
    }

    //std_msgs::ColorRGBA col_gridline=m_col_marker_rgba;
    //m_mrk_wai_marker.colors.push_back(col_gridline);
    //m_mrk_wai_marker.colors.push_back(col_gridline);
}


Cursor3DMarker::Cursor3DMarker()
{
}
Cursor3DMarker::~Cursor3DMarker()
{
}
void Cursor3DMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_mrk_wai_marker.header.frame_id = s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::LINE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);
}
void Cursor3DMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = 0.0; // Unused for this marker type!
    m_mrk_wai_marker.scale.z = 0.0; // Unused for this marker type!
}
void Cursor3DMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void Cursor3DMarker::UpdateCursor(float f_size, float f_size_corners)
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    geometry_msgs::Point pnt_c3d;
    float f_scl=f_size/2.0;
    float f_csc=f_size_corners;

    // Top Front Left
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl-f_csc; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl-f_csc; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl-f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Top front right
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl-f_csc; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl+f_csc; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl-f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Top rear left
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl+f_csc; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl-f_csc; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=f_scl-f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Top rear right
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl+f_csc; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl+f_csc; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=f_scl-f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    //BOTTOM
    // Bottom Front Left
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl-f_csc; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl-f_csc; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl+f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Bottom front right
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl-f_csc; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl+f_csc; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl+f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Bottom rear left
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl+f_csc; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl-f_csc; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=f_scl; pnt_c3d.z=-f_scl+f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    // Bottom rear right
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl+f_csc; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl+f_csc; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl; m_mrk_wai_marker.points.push_back(pnt_c3d);
    pnt_c3d.x=-f_scl; pnt_c3d.y=-f_scl; pnt_c3d.z=-f_scl+f_csc; m_mrk_wai_marker.points.push_back(pnt_c3d);

    //Update colors
    for(int i=0;i<m_mrk_wai_marker.points.size();i++)
    {
        std_msgs::ColorRGBA col_cursor=m_col_marker_rgba;
        m_mrk_wai_marker.colors.push_back(col_cursor);
    }
}



Data3DMarker::Data3DMarker()
{
}

Data3DMarker::~Data3DMarker()
{
}

void Data3DMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;
    m_s_graph3d_title=s_marker_label;
    m_s_path_ressources=s_path_ressources;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_f_samples_spacing_x=0.04;
    m_f_samples_spacing_y=0.04;
    m_f_samples_spacing_z=0.04;
    m_f_samples_marker_scale=0.01;
    m_f_settings_axis_x_min=0.0;
    m_f_settings_axis_x_max=6.28;
    m_f_settings_axis_y_min=0.0;
    m_f_settings_axis_y_max=6.28;
    m_f_settings_axis_z_min=-3.0;
    m_f_settings_axis_z_max=3.0;

    m_mrk_wai_marker.header.frame_id = m_s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);

    /* Old initialization for viz. of default straight surface
    for(float x=0;x<m_f_settings_axis_x_max;x=x+m_f_samples_spacing_x)
    {
        for(float y=0;y<m_f_settings_axis_y_max;y=y+m_f_samples_spacing_y)
        {
            geometry_msgs::Point pnt_point;
            pnt_point.x=x;
            pnt_point.y=y;
            pnt_point.z=x+y;
            m_mrk_wai_marker.points.push_back(pnt_point);

            std_msgs::ColorRGBA col_color;
            col_color.r = x/m_f_settings_axis_x_max;
            col_color.g = y/m_f_settings_axis_y_max;
            col_color.b = 0.5;
            col_color.a = x/m_f_settings_axis_x_max;
            m_mrk_wai_marker.colors.push_back(col_color);
        }
    }
    */
}

void Data3DMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = m_vc3_marker_scale.getZ();
}

void Data3DMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}

void Data3DMarker::UpdateSettings(float f_samples_spacing_x,
                                  float f_samples_spacing_y,
                                  float f_samples_spacing_z,
                                   float f_samples_marker_scale,
                                   float f_settings_axis_x_min,
                                   float f_settings_axis_x_max,
                                   float f_settings_axis_y_min,
                                   float f_settings_axis_y_max,
                                   float f_settings_axis_z_min,
                                   float f_settings_axis_z_max)
{
    m_f_samples_spacing_x=f_samples_spacing_x;
    m_f_samples_spacing_y=f_samples_spacing_y;
    m_f_samples_spacing_z=f_samples_spacing_z;
    m_f_samples_marker_scale=f_samples_marker_scale;
    m_vc3_marker_scale.setX(m_f_samples_marker_scale);
    m_vc3_marker_scale.setY(m_f_samples_marker_scale);
    m_vc3_marker_scale.setZ(m_f_samples_marker_scale);
    m_f_settings_axis_x_min=f_settings_axis_x_min;
    m_f_settings_axis_x_max=f_settings_axis_x_max;
    m_f_settings_axis_y_min=f_settings_axis_y_min;
    m_f_settings_axis_y_max=f_settings_axis_y_max;
    m_f_settings_axis_z_min=f_settings_axis_z_min;
    m_f_settings_axis_z_max=f_settings_axis_z_max;
}

void Data3DMarker::UpdateDataFunction(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z)
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Default visualization, e.g. for functions f(x,y)
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz=*(graph_3d_data+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                geometry_msgs::Point pnt_point;
                pnt_point.x=x*m_f_samples_spacing_x;
                pnt_point.y=y*m_f_samples_spacing_y;
                pnt_point.z=f_xyz;
                m_mrk_wai_marker.points.push_back(pnt_point);

                std_msgs::ColorRGBA col_color;
                col_color.r = float(x)/float(i_data_size_x);
                col_color.g = float(y)/float(i_data_size_y);
                col_color.b = 0.5;
                col_color.a = float(x)/float(i_data_size_x);
                m_mrk_wai_marker.colors.push_back(col_color);
            }
        }
    }
}

void Data3DMarker::UpdateDataEval(float* graph_3d_data,
                                  int i_data_size_x,
                                  int i_data_size_y,
                                  int i_data_size_z,
                                  int i_id_selected,
                                  bool b_use_eval_status_overall,
                                  float f_eval_overall[])
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Visualization for evaluation quantities
    for(int x=0;x<i_data_size_x;x++)
    {
        float f_score_count=0;
        float f_score_sum=0.0;

        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz=*(graph_3d_data+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                //if(f_xyz<-1.0) continue;
                if(f_xyz<-1.0 || (i_id_selected!=-1 && i_id_selected!=x) ) continue;

                f_score_count=f_score_count+1.0;
                f_score_sum=f_score_sum+f_xyz;
                //ROS_WARN("score/count: fxy-%3.3f , %3.3f, %3.3f, %3.3f",f_xyz,f_score_sum,f_score_count,f_score_sum/f_score_count);

                geometry_msgs::Point pnt_point;
                pnt_point.x=x*m_f_samples_spacing_x;
                pnt_point.y=y*m_f_samples_spacing_y;
                pnt_point.z=z*m_f_samples_spacing_z;
                m_mrk_wai_marker.points.push_back(pnt_point);
                std_msgs::ColorRGBA col_color;
                if(m_col_marker_rgba.a==0)
                {
                    col_color.r = 0.5;
                    col_color.g = f_xyz;
                    col_color.b = 0.0;
                    col_color.a = 0.9;
                }
                else
                {
                    col_color=m_col_marker_rgba;
                    col_color.a=f_xyz;
                }
                m_mrk_wai_marker.colors.push_back(col_color);
            }
        }

        // Summary per ID
        geometry_msgs::Point pnt_point;
        pnt_point.x=x*m_f_samples_spacing_x;
        pnt_point.y=i_data_size_y*m_f_samples_spacing_y;
        pnt_point.z=1.0;
        m_mrk_wai_marker.points.push_back(pnt_point);

        std_msgs::ColorRGBA col_color;
        if(f_score_count>=1.0)
        {
            if(m_col_marker_rgba.a==0)
            {
                col_color.r = 0.5;
                if(b_use_eval_status_overall) col_color.g = f_eval_overall[x];
                else col_color.g = f_score_sum/f_score_count;
                col_color.b = 0.0;
                col_color.a = 0.9;
            }
            else
            {
                col_color=m_col_marker_rgba;
                col_color.a=f_score_sum/f_score_count;
            }
        }
        else
        {
            col_color.r = 0.5;
            col_color.g = 0.5;
            col_color.b = 0.5;
            col_color.a = 0.5;
        }
        m_mrk_wai_marker.colors.push_back(col_color);
    }
}

void Data3DMarker::UpdateDataStats(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z)
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Default visualization, e.g. for functions f(x,y)
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz=*(graph_3d_data+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                geometry_msgs::Point pnt_point;
                pnt_point.x=x*m_f_samples_spacing_x;
                pnt_point.y=y*m_f_samples_spacing_y;
                pnt_point.z=f_xyz;
                m_mrk_wai_marker.points.push_back(pnt_point);

                /*
                std_msgs::ColorRGBA col_color;
                col_color.r = float(x)/float(i_data_size_x);
                col_color.g = float(y)/float(i_data_size_y);
                col_color.b = 0.5;
                col_color.a = float(x)/float(i_data_size_x);
                */
                m_mrk_wai_marker.colors.push_back(m_col_marker_rgba);
            }
        }
    }
}




DataStdDevMarker::DataStdDevMarker()
{
}
DataStdDevMarker::~DataStdDevMarker()
{
}
void DataStdDevMarker::Initialize(std::string s_namespace,std::string s_frame_id,std::string s_marker_label,std::string s_path_ressources)
{
    m_s_namespace=s_namespace;
    m_s_frame_id=s_frame_id;
    m_s_graph3d_title=s_marker_label;
    m_s_path_ressources=s_path_ressources;

    m_vc3_marker_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_marker_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_marker_scale=tf::Vector3(0.0,0.0,0.0);
    m_col_marker_rgba.r=0.5;
    m_col_marker_rgba.g=0.5;
    m_col_marker_rgba.b=0.5;
    m_col_marker_rgba.a=0.5;

    m_f_samples_spacing_x=0.04;
    m_f_samples_spacing_y=0.04;
    m_f_samples_spacing_z=0.04;
    m_f_samples_marker_scale=0.01;
    m_f_settings_axis_x_min=0.0;
    m_f_settings_axis_x_max=6.28;
    m_f_settings_axis_y_min=0.0;
    m_f_settings_axis_y_max=6.28;
    m_f_settings_axis_z_min=-3.0;
    m_f_settings_axis_z_max=3.0;

    m_mrk_wai_marker.header.frame_id = m_s_frame_id;
    m_mrk_wai_marker.header.stamp = ros::Time::now();
    m_mrk_wai_marker.ns = s_namespace.c_str();
    m_mrk_wai_marker.id = WAIRvizMarkers::i_marker_id;
    m_mrk_wai_marker.type = visualization_msgs::Marker::CUBE_LIST;
    m_mrk_wai_marker.action = visualization_msgs::Marker::ADD;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = 0.01;//m_vc3_marker_scale.getZ();
    m_mrk_wai_marker.color = m_col_marker_rgba;

    // Init points and colors properties
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();
    geometry_msgs::Point pnt_init;
    pnt_init.x=0.0;
    pnt_init.y=0.0;
    pnt_init.z=0.0;
    m_mrk_wai_marker.points.push_back(pnt_init);
    m_mrk_wai_marker.points.push_back(pnt_init);
    std_msgs::ColorRGBA col_init=m_col_marker_rgba;
    m_mrk_wai_marker.colors.push_back(col_init);
    m_mrk_wai_marker.colors.push_back(col_init);

    /* Old initialization for viz. of default straight surface
    for(float x=0;x<m_f_settings_axis_x_max;x=x+m_f_samples_spacing_x)
    {
        for(float y=0;y<m_f_settings_axis_y_max;y=y+m_f_samples_spacing_y)
        {
            geometry_msgs::Point pnt_point;
            pnt_point.x=x;
            pnt_point.y=y;
            pnt_point.z=x+y;
            m_mrk_wai_marker.points.push_back(pnt_point);

            std_msgs::ColorRGBA col_color;
            col_color.r = x/m_f_settings_axis_x_max;
            col_color.g = y/m_f_settings_axis_y_max;
            col_color.b = 0.5;
            col_color.a = x/m_f_settings_axis_x_max;
            m_mrk_wai_marker.colors.push_back(col_color);
        }
    }
    */
}
void DataStdDevMarker::UpdatePose(tf::Vector3 vc3_marker_position,tf::Quaternion qua_marker_orientation,tf::Vector3 vc3_marker_scale)
{
    m_vc3_marker_position=vc3_marker_position;
    m_qua_marker_orientation=qua_marker_orientation;
    m_vc3_marker_scale=vc3_marker_scale;
    m_mrk_wai_marker.pose.position.x = m_vc3_marker_position.getX();
    m_mrk_wai_marker.pose.position.y = m_vc3_marker_position.getY();
    m_mrk_wai_marker.pose.position.z = m_vc3_marker_position.getZ();
    m_mrk_wai_marker.pose.orientation.x = m_qua_marker_orientation.getX();
    m_mrk_wai_marker.pose.orientation.y = m_qua_marker_orientation.getY();
    m_mrk_wai_marker.pose.orientation.z = m_qua_marker_orientation.getZ();
    m_mrk_wai_marker.pose.orientation.w = m_qua_marker_orientation.getW();
    m_mrk_wai_marker.scale.x = m_vc3_marker_scale.getX();
    m_mrk_wai_marker.scale.y = m_vc3_marker_scale.getY();
    m_mrk_wai_marker.scale.z = 0.01;//m_vc3_marker_scale.getZ();
}
void DataStdDevMarker::UpdateColor(std_msgs::ColorRGBA col_marker_rgba)
{
    m_col_marker_rgba=col_marker_rgba;
    m_mrk_wai_marker.color=m_col_marker_rgba;
}
void DataStdDevMarker::UpdateSettings(float f_samples_spacing_x,
                                  float f_samples_spacing_y,
                                  float f_samples_spacing_z,
                                   float f_samples_marker_scale,
                                   float f_settings_axis_x_min,
                                   float f_settings_axis_x_max,
                                   float f_settings_axis_y_min,
                                   float f_settings_axis_y_max,
                                   float f_settings_axis_z_min,
                                   float f_settings_axis_z_max)
{
    m_f_samples_spacing_x=f_samples_spacing_x;
    m_f_samples_spacing_y=f_samples_spacing_y;
    m_f_samples_spacing_z=f_samples_spacing_z;
    m_f_samples_marker_scale=f_samples_marker_scale;
    m_vc3_marker_scale.setX(m_f_samples_marker_scale);
    m_vc3_marker_scale.setY(m_f_samples_marker_scale);
    m_vc3_marker_scale.setZ(m_f_samples_marker_scale);
    m_f_settings_axis_x_min=f_settings_axis_x_min;
    m_f_settings_axis_x_max=f_settings_axis_x_max;
    m_f_settings_axis_y_min=f_settings_axis_y_min;
    m_f_settings_axis_y_max=f_settings_axis_y_max;
    m_f_settings_axis_z_min=f_settings_axis_z_min;
    m_f_settings_axis_z_max=f_settings_axis_z_max;
}

void DataStdDevMarker::UpdateDataStdDev(float* graph_stats_data_mean,float* graph_stats_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z)
{
    m_mrk_wai_marker.points.clear();
    m_mrk_wai_marker.colors.clear();

    // Default visualization, e.g. for functions f(x,y)
    for(int x=0;x<i_data_size_x;x++)
    {
        for(int y=0;y<i_data_size_y;y++)
        {
            for(int z=0;z<i_data_size_z;z++)
            {
                float f_xyz_mean=*(graph_stats_data_mean+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);
                float f_xyz_stddev=*(graph_stats_data_stddev+ x*i_data_size_y*i_data_size_z + y*i_data_size_z + z);

                float f_z=f_xyz_mean-f_xyz_stddev;
                geometry_msgs::Point pnt_point;
                pnt_point.x=x*m_f_samples_spacing_x;
                pnt_point.y=y*m_f_samples_spacing_y;
                pnt_point.z=z*m_f_samples_spacing_z+f_z;
                m_mrk_wai_marker.points.push_back(pnt_point);
                m_mrk_wai_marker.colors.push_back(m_col_marker_rgba);
                f_z=f_xyz_mean+f_xyz_stddev;
                pnt_point.x=x*m_f_samples_spacing_x;
                pnt_point.y=y*m_f_samples_spacing_y;
                pnt_point.z=z*m_f_samples_spacing_z+f_z;
                m_mrk_wai_marker.points.push_back(pnt_point);
                m_mrk_wai_marker.colors.push_back(m_col_marker_rgba);
            }
        }
    }
}
