#include<wai_reps.h>



/////////////////////////////////////////////////
/// Rep IDs of reps library
/////////////////////////////////////////////////
int WAIReps::i_rep_id=0;



/////////////////////////////////////////////////
/// Implementation of abstract base class
/////////////////////////////////////////////////
WAIReps *WAIReps::create_representative(std::string config)
{
    WAIReps::i_rep_id++;

    if(config.compare("TRIGGER")==0)
    {
        return new WAIRepTrigger;
    }
    if(config.compare("OOI")==0)
    {
        return new WAIRepOOI; // Highlighted Mesh
    }
    if(config.compare("GRAPH_3D")==0)
    {
        return new WAIRepGraph3D;
    }
    if(config.compare("GRAPH_STATS")==0)
    {
        return new WAIRepGraphStats;
    }
    if(config.compare("WIM")==0)
    {
        return new WAIRepWIM;
    }
    else
    {
        return NULL;
    }
}


/////////////////////////////////////////////////
/// Implementations of derived classes
/////////////////////////////////////////////////
void WAIRepTrigger::cb_tmr_timeout(const ros::TimerEvent& event)
{
    // Disable trigger
    m_b_trigger_enabled=false;

    ((TextMarker*)m_vec_mrk_wai_rep[3])->UpdateText(m_s_trigger_label+std::string(m_b_trigger_enabled ? " (!)" : ""));

    std_msgs::ColorRGBA col_rgba_basic,col_rgba_disabled;
    col_rgba_basic=m_col_trigger_rgba;
    col_rgba_basic.a=0.3;
    col_rgba_disabled=m_col_trigger_grey;
    m_vec_mrk_wai_rep[2]->UpdateColor(col_rgba_disabled);
    m_vec_mrk_wai_rep[3]->UpdateColor(col_rgba_basic);

    UpdateView();
}
WAIRepTrigger::WAIRepTrigger()
{
}
WAIRepTrigger::~WAIRepTrigger()
{
}
void WAIRepTrigger::Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_vc3_trigger_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_trigger_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_trigger_scale=tf::Vector3(0.0,0.0,0.0);
    m_s_trigger_label="Trigger";
    m_col_trigger_grey.r=0.5;
    m_col_trigger_grey.g=0.5;
    m_col_trigger_grey.b=0.5;
    m_col_trigger_grey.a=0.5;
    m_col_trigger_rgba=m_col_trigger_grey;
    m_b_trigger_enabled=false;

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_trigger_position,m_qua_trigger_orientation,m_vc3_trigger_scale);
    m_mrk_wai_rep->UpdateColor(m_col_trigger_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_trigger_position,m_qua_trigger_orientation,m_vc3_trigger_scale);
    m_mrk_wai_rep->UpdateColor(m_col_trigger_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_trigger_position,m_qua_trigger_orientation,m_vc3_trigger_scale);
    m_mrk_wai_rep->UpdateColor(m_col_trigger_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,"Trigger!");
    m_mrk_wai_rep->UpdatePose(m_vc3_trigger_position,m_qua_trigger_orientation,m_vc3_trigger_scale);
    m_mrk_wai_rep->UpdateColor(m_col_trigger_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
}
void WAIRepTrigger::UpdateModel()
{

}
void WAIRepTrigger::UpdateModel(tf::Vector3 vc3_trigger_position,tf::Quaternion qua_trigger_orientation,tf::Vector3 vc3_trigger_scale,std::string s_trigger_label,std_msgs::ColorRGBA col_trigger_rgba,bool b_trigger_enabled,bool b_trigger_timeout=false,float f_trigger_timeout=3.0)
{
    m_vc3_trigger_position=vc3_trigger_position;
    m_qua_trigger_orientation=qua_trigger_orientation;
    m_vc3_trigger_scale=vc3_trigger_scale;

    m_f_trigger_stand_thickness=m_vc3_trigger_scale.getX();
    m_f_trigger_head_diameter=m_vc3_trigger_scale.getY();
    m_f_trigger_label_size=m_vc3_trigger_scale.getZ();

    m_s_trigger_label=s_trigger_label;
    m_col_trigger_rgba=col_trigger_rgba;
    m_b_trigger_enabled=b_trigger_enabled;

    m_vec_mrk_wai_rep[0]->UpdatePose(
                tf::Vector3(m_vc3_trigger_position.getX(),m_vc3_trigger_position.getY(),0.0),
                m_qua_trigger_orientation,
                tf::Vector3(m_f_trigger_stand_thickness*3.0,m_f_trigger_stand_thickness*3.0,m_f_trigger_stand_thickness*3.0));
    m_vec_mrk_wai_rep[1]->UpdatePose(
                tf::Vector3(0.0,0.0,0.0), // LineArray Marker is given with 0,0,0 offset wrt "world"!
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(m_f_trigger_stand_thickness,m_f_trigger_stand_thickness,m_f_trigger_head_diameter));
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateHeightline(m_vc3_trigger_position);
    m_vec_mrk_wai_rep[2]->UpdatePose(
                m_vc3_trigger_position,
                m_qua_trigger_orientation,
                tf::Vector3(m_f_trigger_head_diameter,m_f_trigger_head_diameter,m_f_trigger_head_diameter));
    m_vec_mrk_wai_rep[3]->UpdatePose(
                tf::Vector3(m_vc3_trigger_position.getX(),m_vc3_trigger_position.getY(),m_vc3_trigger_position.getZ()+m_f_trigger_head_diameter/2.0+m_f_trigger_label_size/2.0),
                m_qua_trigger_orientation,
                tf::Vector3(m_f_trigger_label_size,m_f_trigger_label_size,m_f_trigger_label_size));
    ((TextMarker*)m_vec_mrk_wai_rep[3])->UpdateText(m_s_trigger_label+std::string(m_b_trigger_enabled ? "(ON)" : ""));

    std_msgs::ColorRGBA col_rgba_basic,col_rgba_disabled,col_rgba_enabled;
    col_rgba_basic=m_col_trigger_rgba;
    col_rgba_basic.a=0.3;
    col_rgba_disabled=m_col_trigger_grey;
    col_rgba_enabled=m_col_trigger_rgba;
    col_rgba_enabled.a=0.9;
    m_vec_mrk_wai_rep[0]->UpdateColor(col_rgba_basic);
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateColor(col_rgba_basic);

    if(!b_trigger_timeout)
    {
        if(b_trigger_enabled)
        {
            m_vec_mrk_wai_rep[2]->UpdateColor(col_rgba_enabled);
            m_vec_mrk_wai_rep[3]->UpdateColor(col_rgba_enabled);
        }
        else
        {
            m_vec_mrk_wai_rep[2]->UpdateColor(col_rgba_disabled);
            m_vec_mrk_wai_rep[3]->UpdateColor(col_rgba_basic);
        }
    }
    else if(b_trigger_timeout)
    {
        m_vec_mrk_wai_rep[2]->UpdateColor(col_rgba_enabled);
        m_vec_mrk_wai_rep[3]->UpdateColor(col_rgba_enabled);
        m_tmr_trigger_timeout=m_hdl_node->createTimer(ros::Duration(f_trigger_timeout),&WAIRepTrigger::cb_tmr_timeout,this,true);
    }
}
void WAIRepTrigger::UpdateView()
{
    m_mrk_wai_rep_array.markers.clear();
    for(int i=0;i<m_vec_mrk_wai_rep.size();i++)
    {
        m_mrk_wai_rep_array.markers.push_back(m_vec_mrk_wai_rep[i]->GetMarker());
    }
    m_pub_mrk_wai_rep_array.publish(m_mrk_wai_rep_array);
}
tf::Vector3 WAIRepTrigger::GetPosition()
{
    return m_vc3_trigger_position;
}



void WAIRepOOI::cb_tmr_timeout(const ros::TimerEvent& event)
{
    // Do nothing for now
}
WAIRepOOI::WAIRepOOI()
{
}
WAIRepOOI::~WAIRepOOI()
{
}
void WAIRepOOI::Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_vc3_ooi_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_ooi_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    //m_vc3_ooi_position_world=tf::Vector3(0.0,0.0,0.0);
    //m_qua_ooi_orientation_world=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_ooi_scale=tf::Vector3(0.0,0.0,0.0);
    m_s_ooi_label="OOI";
    m_col_ooi_grey.r=0.5; m_col_ooi_grey.g=0.5; m_col_ooi_grey.b=0.5; m_col_ooi_grey.a=0.5;
    m_col_ooi_white.r=1.0; m_col_ooi_white.g=1.0; m_col_ooi_white.b=1.0; m_col_ooi_white.a=0.9;
    m_col_ooi_rgba=m_col_ooi_grey;

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_ooi_position,m_qua_ooi_orientation,m_vc3_ooi_scale);
    m_mrk_wai_rep->UpdateColor(m_col_ooi_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_ooi_position,m_qua_ooi_orientation,m_vc3_ooi_scale);
    m_mrk_wai_rep->UpdateColor(m_col_ooi_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("MESH");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_ooi_position,m_qua_ooi_orientation,m_vc3_ooi_scale);
    m_mrk_wai_rep->UpdateColor(m_col_ooi_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_ooi_position,m_qua_ooi_orientation,m_vc3_ooi_scale);
    m_mrk_wai_rep->UpdateColor(m_col_ooi_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_ooi_position,m_qua_ooi_orientation,m_vc3_ooi_scale);
    m_mrk_wai_rep->UpdateColor(m_col_ooi_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
}
void WAIRepOOI::UpdateModel()
{

}
void WAIRepOOI::UpdateModel(tf::Vector3 vc3_ooi_position,tf::Quaternion qua_ooi_orientation,tf::Vector3 vc3_ooi_scale,std::string s_ooi_label,std_msgs::ColorRGBA col_ooi_rgba,std::string s_ooi_mesh_label,std::string s_ooi_mesh_path_resources,bool b_update_mesh,float f_scale_mesh,float f_alpha_mesh,bool b_is_camera)
{
    m_vc3_ooi_position=vc3_ooi_position;
    m_qua_ooi_orientation=qua_ooi_orientation;
    m_vc3_ooi_scale=vc3_ooi_scale;

    m_f_ooi_stand_thickness=m_vc3_ooi_scale.getX();
    m_f_ooi_head_diameter=m_vc3_ooi_scale.getY();
    m_f_ooi_label_size=m_vc3_ooi_scale.getZ();

    m_s_ooi_label=s_ooi_label;
    m_col_ooi_rgba=col_ooi_rgba;

    m_vec_mrk_wai_rep[0]->UpdatePose(
                tf::Vector3(m_vc3_ooi_position.getX(),m_vc3_ooi_position.getY(),0.0),
                //tf::Vector3(m_vc3_ooi_position_world.getX(),m_vc3_ooi_position_world.getY(),0.0),
                m_qua_ooi_orientation,
                tf::Vector3(m_f_ooi_stand_thickness*3.0,m_f_ooi_stand_thickness*3.0,m_f_ooi_stand_thickness*3.0));
    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_ooi_rgba);

    m_vec_mrk_wai_rep[1]->UpdatePose(
                tf::Vector3(0.0,0.0,0.0), // LineArray Marker is given with 0,0,0 offset wrt "world"!
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(m_f_ooi_stand_thickness,m_f_ooi_stand_thickness,m_f_ooi_head_diameter));
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateHeightline(m_vc3_ooi_position);
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateColor(m_col_ooi_rgba);

    // Do not update colors or transparency for MESH marker!
    // Transparency must be defined by embedded materials!
    //m_vec_mrk_wai_rep[2]->UpdateColor(m_col_ooi_rgba);
    //((MeshMarker*)m_vec_mrk_wai_rep[2])->UpdateAlpha(f_alpha_mesh);
    if(b_update_mesh)
    {
        std::size_t pos = s_ooi_mesh_path_resources.find(".dae");
        if(pos!=std::string::npos) // .dae found, use absolute path
        {
            ((MeshMarker*)m_vec_mrk_wai_rep[2])->UpdateMeshAbsolutePath(s_ooi_mesh_path_resources);
        }
        else
        {
            ((MeshMarker*)m_vec_mrk_wai_rep[2])->UpdateMesh(s_ooi_mesh_label,s_ooi_mesh_path_resources);
        }
    }
    m_vec_mrk_wai_rep[2]->UpdatePose(
                m_vc3_ooi_position,
                m_qua_ooi_orientation,
                tf::Vector3(f_scale_mesh,f_scale_mesh,f_scale_mesh));

    m_vec_mrk_wai_rep[3]->UpdatePose(
                m_vc3_ooi_position,
                //m_vc3_ooi_position_world,
                m_qua_ooi_orientation,
                tf::Vector3(m_f_ooi_head_diameter,m_f_ooi_head_diameter,m_f_ooi_head_diameter));
    m_vec_mrk_wai_rep[3]->UpdateColor(m_col_ooi_rgba);

    if(b_is_camera)
    {
        m_vec_mrk_wai_rep[4]->UpdatePose(
                    m_vc3_ooi_position+tf::Vector3(0.0,-1.0*(m_f_ooi_head_diameter/2.0+m_f_ooi_label_size/2.0),0.0),
                    //tf::Vector3(m_vc3_ooi_position_world.getX(),m_vc3_ooi_position_world.getY(),m_vc3_ooi_position_world.getZ()+m_f_ooi_head_diameter/2.0+m_f_ooi_label_size/2.0),
                    m_qua_ooi_orientation,
                    tf::Vector3(m_f_ooi_label_size,m_f_ooi_label_size,m_f_ooi_label_size));
    }
    else
    {
        m_vec_mrk_wai_rep[4]->UpdatePose(
                    m_vc3_ooi_position+tf::Vector3(0.0,0.0,m_f_ooi_head_diameter/2.0+m_f_ooi_label_size/2.0),
                    //tf::Vector3(m_vc3_ooi_position_world.getX(),m_vc3_ooi_position_world.getY(),m_vc3_ooi_position_world.getZ()+m_f_ooi_head_diameter/2.0+m_f_ooi_label_size/2.0),
                    m_qua_ooi_orientation,
                    tf::Vector3(m_f_ooi_label_size,m_f_ooi_label_size,m_f_ooi_label_size));
    }

    m_vec_mrk_wai_rep[4]->UpdateColor(m_col_ooi_white);
    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_ooi_label);

    //m_tmr_trigger_timeout=m_hdl_node->createTimer(ros::Duration(f_trigger_timeout),&WAIRepTrigger::cb_tmr_timeout,this,true);
}
void WAIRepOOI::UpdateView()
{
    m_mrk_wai_rep_array.markers.clear();
    for(int i=0;i<m_vec_mrk_wai_rep.size();i++)
    {
        m_mrk_wai_rep_array.markers.push_back(m_vec_mrk_wai_rep[i]->GetMarker());
    }
    m_pub_mrk_wai_rep_array.publish(m_mrk_wai_rep_array);
}
tf::Vector3 WAIRepOOI::GetPosition()
{
    return m_vc3_ooi_position;
    //return m_vc3_ooi_position_world;
}





///////////////////////////////////////////
/// WIM
///////////////////////////////////////////
void WAIRepWIM::cb_tmr_timeout(const ros::TimerEvent& event)
{
    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_oa);
    m_vec_mrk_wai_rep[1]->UpdateColor(m_col_oa);
    std_msgs::ColorRGBA col_wim_bubble=m_col_oa;
    col_wim_bubble.a=0.1;
    m_vec_mrk_wai_rep[3]->UpdateColor(col_wim_bubble);

    m_vec_mrk_wai_rep[6]->UpdateColor(m_col_oa);
    ((Cursor3DMarker*)m_vec_mrk_wai_rep[6])->UpdateCursor(fabs(m_vc3_wim_audience_spacing.getY()),fabs(m_vc3_wim_audience_spacing.getY()/4.0));

    int i_reps=7;
    for(int i=0;i<m_i_audience_count_max;i++)
    {
        m_sst_wim_label_cursor_id.str("");
        m_sst_wim_label_cursor_id << "ID" << i;
        ((TextMarker*)m_vec_mrk_wai_rep[i_reps])->UpdateText(m_sst_wim_label_cursor_id.str());

        if(i==m_i_audience_id_selected)
            m_vec_mrk_wai_rep[i_reps]->UpdateColor(m_col_oa);
        else
            m_vec_mrk_wai_rep[i_reps]->UpdateColor(m_col_wim_labels);
        i_reps++;
    }

    UpdateView();
}
WAIRepWIM::WAIRepWIM()
{
}
WAIRepWIM::~WAIRepWIM()
{
}
void WAIRepWIM::Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_i_audience_count_max=1; // Default initializer
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_vc3_wim_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_wim_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    //m_vc3_wim_position_world=tf::Vector3(0.0,0.0,0.0);
    //m_qua_wim_orientation_world=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_wim_scale=tf::Vector3(0.05,0.05,0.05);

    m_vc3_wim_audience_offset=tf::Vector3(0.0,0.0,0.0);
    m_vc3_wim_audience_count=tf::Vector3(1.0,1.0,1.0);
    m_vc3_wim_audience_spacing=tf::Vector3(1.0,1.0,1.0);
    m_vc3_wim_audience_focused=tf::Vector3(1.0,1.0,1.0);
    m_i_audience_id_selected=0;

    m_s_wim_label="WIM";

    m_col_grey.r=0.5; m_col_grey.g=0.5; m_col_grey.b=0.5; m_col_grey.a=0.5;
    m_col_white_trans.r=1.0; m_col_white_trans.g=1.0; m_col_white_trans.b=1.0; m_col_white_trans.a=0.3;
    m_col_white.r=1.0; m_col_white.g=1.0; m_col_white.b=1.0; m_col_white.a=0.9;
    m_col_white_opaque.r=1.0; m_col_white_opaque.g=1.0; m_col_white_opaque.b=1.0; m_col_white_opaque.a=1.0;
    m_col_oa.r=0.101960784; m_col_oa.g=0.42745098; m_col_oa.b=0.588235294; m_col_oa.a=0.9;
    m_col_invisible.r=0.0;m_col_invisible.g=0.0;m_col_invisible.b=0.0; m_col_invisible.a=0.0;
    m_col_wim_rgba=m_col_grey;
    m_col_wim_labels=m_col_white;


    // WIM ooi
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINE");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("MESH");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);


    // WIM data
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERES");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("CURSOR_3D");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    for(int i=0;i<m_i_audience_count_max;i++)
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
        m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
        m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }

    // Init stats (e.g. PING)
    //m_f_audience_stats_ping = (float*)malloc(m_i_audience_count_max*sizeof(float));
    for(int i=0;i<m_i_audience_count_max;i++)
    {
        m_f_audience_stats_ping[i]=0.0;
    }
}

void WAIRepWIM::InitializeWithAudienceCount(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame,int i_audience_count_max,tf::Vector3 vc3_wim_scale)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_i_audience_count_max=i_audience_count_max;
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_vc3_wim_position=tf::Vector3(0.0,0.0,0.0);
    m_qua_wim_orientation=tf::Quaternion(0.0,0.0,0.0,1.0);
    //m_vc3_wim_position_world=tf::Vector3(0.0,0.0,0.0);
    //m_qua_wim_orientation_world=tf::Quaternion(0.0,0.0,0.0,1.0);
    m_vc3_wim_scale=vc3_wim_scale;

    m_vc3_wim_audience_offset=tf::Vector3(0.0,0.0,0.0);
    m_vc3_wim_audience_count=tf::Vector3(1.0,1.0,1.0);
    m_vc3_wim_audience_spacing=tf::Vector3(1.0,1.0,1.0);
    m_vc3_wim_audience_focused=tf::Vector3(1.0,1.0,1.0);
    m_i_audience_id_selected=0;

    m_s_wim_label="WIM";

    m_col_grey.r=0.5; m_col_grey.g=0.5; m_col_grey.b=0.5; m_col_grey.a=0.5;
    m_col_white_trans.r=1.0; m_col_white_trans.g=1.0; m_col_white_trans.b=1.0; m_col_white_trans.a=0.3;
    m_col_white.r=1.0; m_col_white.g=1.0; m_col_white.b=1.0; m_col_white.a=0.9;
    m_col_white_opaque.r=1.0; m_col_white_opaque.g=1.0; m_col_white_opaque.b=1.0; m_col_white_opaque.a=1.0;
    m_col_oa.r=0.101960784; m_col_oa.g=0.42745098; m_col_oa.b=0.588235294; m_col_oa.a=0.9;
    m_col_invisible.r=0.0;m_col_invisible.g=0.0;m_col_invisible.b=0.0; m_col_invisible.a=0.0;
    m_col_wim_rgba=m_col_grey;
    m_col_wim_labels=m_col_white;


    // WIM ooi
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(tf::Vector3(1.25,1.25,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINE");
    m_mrk_wai_rep->Initialize(m_s_namespace,"world");
    m_mrk_wai_rep->UpdatePose(tf::Vector3(0.0,0.0,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("MESH");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERE");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);


    // WIM data
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("SPHERES");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("CURSOR_3D");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
    m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
    m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    for(int i=0;i<m_i_audience_count_max;i++)
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame,m_s_wim_label);
        m_mrk_wai_rep->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,m_vc3_wim_scale);
        m_mrk_wai_rep->UpdateColor(m_col_wim_rgba);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }

    // Init stats (e.g. PING)
    //m_f_audience_stats_ping = (float*)malloc(m_i_audience_count_max*sizeof(float));
    for(int i=0;i<m_i_audience_count_max;i++)
    {
        m_f_audience_stats_ping[i]=0.0;
    }
}
void WAIRepWIM::UpdateModel()
{

}
tf::Vector3 WAIRepWIM::GetScale()
{
    return m_vc3_wim_scale;
}
void WAIRepWIM::UpdateModel(std::string s_wim_mesh_label,std::string s_wim_mesh_path_resources)
{
    ((MeshMarker*)m_vec_mrk_wai_rep[2])->UpdateMesh(s_wim_mesh_label,s_wim_mesh_path_resources);
}
void WAIRepWIM::UpdateModel(tf::Vector3 vc3_wim_position,tf::Quaternion qua_wim_orientation,std::string s_wim_label,std::string s_wim_label_cursor,std_msgs::ColorRGBA col_wim_rgba,tf::Vector3 vc3_wim_audience_offset,tf::Vector3 vc3_wim_audience_count,tf::Vector3 vc3_wim_audience_spacing,tf::Vector3 vc3_wim_audience_focused,int i_audience_id_selected,std::string s_wim_mesh_label,std::string s_wim_mesh_path_resources,bool b_update_mesh)
{
    m_vc3_wim_position=vc3_wim_position;
    m_qua_wim_orientation=qua_wim_orientation;

    m_vc3_wim_audience_offset=tf::Vector3(
                vc3_wim_audience_offset.getX()*m_vc3_wim_scale.getX(),
                vc3_wim_audience_offset.getY()*m_vc3_wim_scale.getY(),
                vc3_wim_audience_offset.getZ()*m_vc3_wim_scale.getZ());
    m_vc3_wim_audience_spacing=tf::Vector3(
                vc3_wim_audience_spacing.getX()*m_vc3_wim_scale.getX(),
                vc3_wim_audience_spacing.getY()*m_vc3_wim_scale.getY(),
                vc3_wim_audience_spacing.getZ()*m_vc3_wim_scale.getZ());
    m_vc3_wim_audience_focused=tf::Vector3(
                vc3_wim_audience_focused.getX()*m_vc3_wim_scale.getX(),
                vc3_wim_audience_focused.getY()*m_vc3_wim_scale.getY(),
                vc3_wim_audience_focused.getZ()*m_vc3_wim_scale.getZ());
    m_vc3_wim_audience_count=vc3_wim_audience_count;
    m_i_audience_id_selected=i_audience_id_selected;

    m_f_wim_stand_thickness=m_vc3_wim_audience_spacing.getY()/2.0;
    m_f_wim_head_diameter=m_vc3_wim_audience_spacing.getY()*m_vc3_wim_audience_count.getY();//*2.5;
    m_f_wim_label_size=0.1;

    m_s_wim_label=s_wim_label;
    m_s_wim_label_cursor=s_wim_label_cursor;

    m_col_wim_rgba=col_wim_rgba;
    m_col_wim_labels=m_col_white;

    // All WIM markers except supporting reps around OOI mesh, are given in local frame
    /*
    try
    {
        m_lst_tranforms.lookupTransform("world", m_s_frame,ros::Time(0),m_tfs_world_wrt_rep);
        m_vc3_wim_position_world=m_tfs_world_wrt_rep.getOrigin();
        m_qua_wim_orientation_world=m_tfs_world_wrt_rep.getRotation();
    }
    catch(tf::TransformException ex)
    {
        // Do nothing and skip this frame
    }
    */

    // WIM ooi
    m_vec_mrk_wai_rep[0]->UpdatePose(
                tf::Vector3(1.25,1.25,0.0), // Stand bubble Marker given in "world"
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(m_f_wim_stand_thickness*3.0,m_f_wim_stand_thickness*3.0,m_f_wim_stand_thickness*3.0));
    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_wim_rgba);

    m_vec_mrk_wai_rep[1]->UpdatePose(
                tf::Vector3(0.0,0.0,0.0), // Heightline Marker given in "world" with 0,0,0 but heighline is placed relative at 1.25,1.25,0.6
                tf::Quaternion(0.0,0.0,0.0,1.0),
                tf::Vector3(m_f_wim_stand_thickness,m_f_wim_stand_thickness,0.0));
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateHeightline(tf::Vector3(1.25,1.25,0.6)); // Update offset coords given in "world" frame!
    ((HeightlineMarker*)m_vec_mrk_wai_rep[1])->UpdateColor(m_col_wim_rgba);

    m_vec_mrk_wai_rep[2]->UpdatePose(
                m_vc3_wim_position,
                m_qua_wim_orientation,
                m_vc3_wim_scale);
    m_vec_mrk_wai_rep[2]->UpdateColor(m_col_wim_rgba);
    if(b_update_mesh)
    {
        ((MeshMarker*)m_vec_mrk_wai_rep[2])->UpdateMesh(s_wim_mesh_label,s_wim_mesh_path_resources);
    }

    m_vec_mrk_wai_rep[3]->UpdatePose(
                m_vc3_wim_position,
                m_qua_wim_orientation,
                tf::Vector3(m_f_wim_head_diameter,m_f_wim_head_diameter,m_f_wim_head_diameter));
    std_msgs::ColorRGBA col_wim_bubble=m_col_wim_rgba;
    col_wim_bubble.a=0.1;
    m_vec_mrk_wai_rep[3]->UpdateColor(col_wim_bubble);

    m_vec_mrk_wai_rep[4]->UpdatePose(
                tf::Vector3(m_vc3_wim_position.getX(),m_vc3_wim_position.getY(),m_vc3_wim_position.getZ()+m_f_wim_head_diameter/2.0+m_f_wim_label_size/2.0),
                m_qua_wim_orientation,
                tf::Vector3(m_f_wim_label_size,m_f_wim_label_size,m_f_wim_label_size));
    m_vec_mrk_wai_rep[4]->UpdateColor(m_col_oa);
    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_wim_label);


    // WIM data (starts at idx[5])
    /*
    m_vec_mrk_wai_rep[5]->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,tf::Vector3(0.025,0.025,0.025));
    m_vec_mrk_wai_rep[5]->UpdateColor(m_col_grey);
    ((SpheresMarker*)m_vec_mrk_wai_rep[5])->UpdateSpheres(
                m_vc3_wim_audience_offset,
                tf::Vector3(3,6,1),
                m_vc3_wim_audience_spacing,
                m_f_stats,
                tf::Vector3(0.1,0.5,1.0));
    */

    m_vec_mrk_wai_rep[6]->UpdatePose(m_vc3_wim_position+m_vc3_wim_audience_focused,
                                     m_qua_wim_orientation,
                                     tf::Vector3(0.01,0.0,0.0));
    m_vec_mrk_wai_rep[6]->UpdateColor(m_col_wim_rgba);
    ((Cursor3DMarker*)m_vec_mrk_wai_rep[6])->UpdateCursor(fabs(m_vc3_wim_audience_spacing.getY()),fabs(m_vc3_wim_audience_spacing.getY()/4.0));

    // Labels of audience/listeners
    int i_reps=7;
    int i_audience_id=0;
    for(int x=0;x<m_vc3_wim_audience_count.getX();x++)
    {
        for(int y=0;y<m_vc3_wim_audience_count.getY();y++)
        {
            for(int z=0;z<m_vc3_wim_audience_count.getZ();z++)
            {
                if(i_audience_id==m_i_audience_id_selected)
                {
                    m_vec_mrk_wai_rep[i_reps]->UpdatePose(m_vc3_wim_position+
                        tf::Vector3(m_vc3_wim_audience_offset.getX()+x*m_vc3_wim_audience_spacing.getX(),
                                    m_vc3_wim_audience_offset.getY()+y*m_vc3_wim_audience_spacing.getY(),
                                    m_vc3_wim_audience_offset.getZ()+z*m_vc3_wim_audience_spacing.getZ()+fabs(m_vc3_wim_audience_spacing.getY()*2.0)),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.0,0.0,0.075));
                    m_vec_mrk_wai_rep[i_reps]->UpdateColor(m_col_wim_rgba);
                }
                else
                {
                    m_vec_mrk_wai_rep[i_reps]->UpdatePose(m_vc3_wim_position+
                        tf::Vector3(m_vc3_wim_audience_offset.getX()+x*m_vc3_wim_audience_spacing.getX(),
                                    m_vc3_wim_audience_offset.getY()+y*m_vc3_wim_audience_spacing.getY(),
                                    m_vc3_wim_audience_offset.getZ()+z*m_vc3_wim_audience_spacing.getZ()+fabs(m_vc3_wim_audience_spacing.getY())),
                        tf::Quaternion(0.0,0.0,0.0,1.0),
                        tf::Vector3(0.0,0.0,0.04));
                    m_vec_mrk_wai_rep[i_reps]->UpdateColor(m_col_wim_labels);
                }

                if(!m_s_wim_label_cursor.compare("")==0 && i_audience_id==m_i_audience_id_selected)
                {
                    ((TextMarker*)m_vec_mrk_wai_rep[i_reps])->UpdateText(m_s_wim_label_cursor);
                }
                else
                {   
                    m_sst_wim_label_cursor_id.str("");
                    m_sst_wim_label_cursor_id << "ID" << i_audience_id;
                    ((TextMarker*)m_vec_mrk_wai_rep[i_reps])->UpdateText(m_sst_wim_label_cursor_id.str());
                }

                i_reps++;
                i_audience_id++;
            }
        }
    }

    //Reset colors
    m_tmr_trigger_timeout=m_hdl_node->createTimer(ros::Duration(0.5),&WAIRepWIM::cb_tmr_timeout,this,true);
}
void WAIRepWIM::UpdateStats(float f_audience_stats_ping[255])
{
    m_vec_mrk_wai_rep[5]->UpdatePose(m_vc3_wim_position,m_qua_wim_orientation,tf::Vector3(0.025,0.025,0.025));
    m_vec_mrk_wai_rep[5]->UpdateColor(m_col_grey);
    ((SpheresMarker*)m_vec_mrk_wai_rep[5])->UpdateSpheres(
                m_vc3_wim_audience_offset,
                m_vc3_wim_audience_count,
                m_vc3_wim_audience_spacing,
                f_audience_stats_ping,
                tf::Vector3(0.1,0.5,1.0));
}
void WAIRepWIM::UpdateView()
{
    m_mrk_wai_rep_array.markers.clear();
    for(int i=0;i<m_vec_mrk_wai_rep.size();i++)
    {
        m_mrk_wai_rep_array.markers.push_back(m_vec_mrk_wai_rep[i]->GetMarker());
    }
    m_pub_mrk_wai_rep_array.publish(m_mrk_wai_rep_array);
}








void WAIRepGraph3D::cb_tmr_timeout(const ros::TimerEvent& event)
{
    m_s_label_cursor="<CURSOR>";
    ((TextMarker*)m_vec_mrk_wai_rep[14])->UpdateText(m_s_label_cursor);
    UpdateView();
}
WAIRepGraph3D::WAIRepGraph3D()
{
}
WAIRepGraph3D::~WAIRepGraph3D()
{
}
void WAIRepGraph3D::Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_col_grey.r=0.5; m_col_grey.g=0.5; m_col_grey.b=0.5; m_col_grey.a=0.5;
    m_col_white_trans.r=1.0; m_col_white_trans.g=1.0; m_col_white_trans.b=1.0; m_col_white_trans.a=0.3;
    m_col_white.r=1.0; m_col_white.g=1.0; m_col_white.b=1.0; m_col_white.a=0.9;
    m_col_white_opaque.r=1.0; m_col_white_opaque.g=1.0; m_col_white_opaque.b=1.0; m_col_white_opaque.a=1.0;
    m_col_green_opaque.r=0.0; m_col_green_opaque.g=1.0; m_col_green_opaque.b=0.0; m_col_green_opaque.a=1.0;
    m_col_cyan_opaque.r=0.0; m_col_cyan_opaque.g=1.0; m_col_cyan_opaque.b=1.0; m_col_cyan_opaque.a=1.0;
    m_col_oa.r=0.101960784; m_col_oa.g=0.42745098; m_col_oa.b=0.588235294; m_col_oa.a=0.9;
    m_col_invisible.r=0.0;m_col_invisible.g=0.0;m_col_invisible.b=0.0; m_col_invisible.a=0.0;

    m_vc3_outer_bounds=tf::Vector3(0,0,0);

    m_s_label_title="3D-Graph";
    m_s_label_axis_x="X";
    m_s_label_axis_y="Y";
    m_s_label_axis_z="Z";
    m_s_label_cursor="<CURSOR>";
    m_col_axis_x=m_col_grey;
    m_col_axis_y=m_col_grey;
    m_col_axis_z=m_col_grey;
    m_col_grid=m_col_white;

    m_i_grid_count_x=0;
    m_i_grid_count_y=0;
    m_f_grid_spacing_x=1.0;
    m_f_grid_spacing_y=1.0;

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("GRIDLINES");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // X-Label
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Y-Label
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Z-Label
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Title idx[7]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);


    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // Eval PART data
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINES_EVAL");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep); // idx[9]!

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // Eval EXAM data
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINES_EVAL");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep); // idx[11]!

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // Eval OVERALL data
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINES_EVAL");
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep); // idx[13]!

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // CURSOR INFO
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep); // idx[14]
}
void WAIRepGraph3D::UpdateModel()
{

}
void WAIRepGraph3D::UpdateModel(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_outer_bounds)
{
    m_vc3_position=vc3_position;
    m_qua_orientation=qua_orientation;
    m_col_axis_x=m_col_grey;
    m_col_axis_y=m_col_grey;
    m_col_axis_z=m_col_grey;
    m_col_grid=m_col_white;
    m_col_labels=m_col_white;
    m_vc3_outer_bounds=vc3_outer_bounds;

    m_vec_mrk_wai_rep[0]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_axis_x);
    ((ArrowMarker*)m_vec_mrk_wai_rep[0])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(m_vc3_outer_bounds.getX(),0,0));

    m_vec_mrk_wai_rep[1]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[1]->UpdateColor(m_col_axis_y);
    ((ArrowMarker*)m_vec_mrk_wai_rep[1])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(0,m_vc3_outer_bounds.getY(),0));

    m_vec_mrk_wai_rep[2]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[2]->UpdateColor(m_col_axis_z);
    ((ArrowMarker*)m_vec_mrk_wai_rep[2])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(0,0,m_vc3_outer_bounds.getZ()));

    // Grid off by default
    m_vec_mrk_wai_rep[3]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.0,0.0,0.0));
    m_vec_mrk_wai_rep[3]->UpdateColor(m_col_grid);
    ((GridLinesMarker*)m_vec_mrk_wai_rep[3])->UpdateGrid(0,0,0.0,0.0);

    // Labels and title
    m_vec_mrk_wai_rep[4]->UpdatePose(tf::Vector3(m_vc3_position.getX()+m_vc3_outer_bounds.getX()+0.25,m_vc3_position.getY(),m_vc3_position.getZ()),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[4]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_label_axis_x);

    m_vec_mrk_wai_rep[5]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY()+m_vc3_outer_bounds.getY()+0.25,m_vc3_position.getZ()),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[5]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[5])->UpdateText(m_s_label_axis_y);

    m_vec_mrk_wai_rep[6]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY(),m_vc3_position.getZ()+m_vc3_outer_bounds.getZ()+0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[6]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[6])->UpdateText(m_s_label_axis_z);

    m_vec_mrk_wai_rep[7]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY(),m_vc3_position.getZ()-0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[7]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[7])->UpdateText(m_s_label_title);

    m_vec_mrk_wai_rep[14]->UpdatePose(tf::Vector3(m_vc3_position.getX()+0.25,m_vc3_position.getY()+0.25,m_vc3_position.getZ()+0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[14]->UpdateColor(m_col_cyan_opaque);
    ((TextMarker*)m_vec_mrk_wai_rep[14])->UpdateText(m_s_label_cursor);
}
void WAIRepGraph3D::UpdateLabelsAndColors(std::string s_label_axis_x,std::string s_label_axis_y,std::string s_label_axis_z,std::string s_label_title,std::string s_label_axis_cursor,std_msgs::ColorRGBA col_axis_x,std_msgs::ColorRGBA col_axis_y,std_msgs::ColorRGBA col_axis_z)
{
    m_s_label_title=s_label_title;
    m_s_label_axis_x=s_label_axis_x;
    m_s_label_axis_y=s_label_axis_y;
    m_s_label_axis_z=s_label_axis_z;
    m_s_label_cursor=s_label_axis_cursor;
    m_col_axis_x=col_axis_x;
    m_col_axis_y=col_axis_y;
    m_col_axis_z=col_axis_z;

    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_axis_x);
    m_vec_mrk_wai_rep[1]->UpdateColor(m_col_axis_y);
    m_vec_mrk_wai_rep[2]->UpdateColor(m_col_axis_z);

    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_label_axis_x);
    ((TextMarker*)m_vec_mrk_wai_rep[5])->UpdateText(m_s_label_axis_y);
    ((TextMarker*)m_vec_mrk_wai_rep[6])->UpdateText(m_s_label_axis_z);
    ((TextMarker*)m_vec_mrk_wai_rep[7])->UpdateText(m_s_label_title);
    ((TextMarker*)m_vec_mrk_wai_rep[14])->UpdateText(m_s_label_cursor);
}
void WAIRepGraph3D::InitGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y)
{
    m_i_grid_count_x=i_grid_count_x;
    m_i_grid_count_y=i_grid_count_y;
    m_f_grid_spacing_x=f_grid_spacing_x;
    m_f_grid_spacing_y=f_grid_spacing_y;

    for(int x=0;x<m_i_grid_count_x;x++) //IDs idx[12+3]-idx[41+3]
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }
    for(int y=0;y<m_i_grid_count_y;y++) // Months idx[42+3]-idx[53+3]
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }

    // Summary per ID
    for(int x=0;x<m_i_grid_count_x;x++) // Summary idx[54+3]-idx[83+3]
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }
}
void WAIRepGraph3D::UpdateGrid(float f_grid_thickness,std_msgs::ColorRGBA col_grid)
{
    // Update labels of grid along x-axis and y-axis
    for(int x=0;x<m_i_grid_count_x;x++)
    {
        std::stringstream sst_id_to_str;
        sst_id_to_str << m_s_label_axis_x << (x); // Start with audience ID at 0
        ((TextMarker*)m_vec_mrk_wai_rep[15+x])->UpdateText(sst_id_to_str.str());
        m_vec_mrk_wai_rep[15+x]->UpdateColor(m_col_white);
        ((TextMarker*)m_vec_mrk_wai_rep[15+x])->UpdatePose(tf::Vector3(x*m_f_grid_spacing_x,-m_f_grid_spacing_y/2.0,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
    }
    for(int y=0;y<m_i_grid_count_y;y++)
    {
        std::stringstream sst_id_to_str;
        sst_id_to_str << m_s_label_axis_y << (y+1);
        ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+y])->UpdateText(sst_id_to_str.str());
        m_vec_mrk_wai_rep[15+m_i_grid_count_x+y]->UpdateColor(m_col_white);
        ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+y])->UpdatePose(tf::Vector3(-m_f_grid_spacing_x/2.0,y*m_f_grid_spacing_y,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
    }

    m_vec_mrk_wai_rep[3]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_grid_thickness,f_grid_thickness,f_grid_thickness));
    m_vec_mrk_wai_rep[3]->UpdateColor(col_grid);
    ((GridLinesMarker*)m_vec_mrk_wai_rep[3])->UpdateGrid(m_i_grid_count_x,m_i_grid_count_y,m_f_grid_spacing_x,m_f_grid_spacing_y);
}
void WAIRepGraph3D::UpdateAxes(float f_samples_spacing_x,
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
    // Participation
    m_vec_mrk_wai_rep[8]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale,f_samples_marker_scale,f_samples_marker_scale));
    ((Data3DMarker*)m_vec_mrk_wai_rep[8])->UpdateSettings(
            f_samples_spacing_x,
            f_samples_spacing_y,
            f_samples_spacing_z,
            f_samples_marker_scale,
            f_settings_axis_x_min,
            f_settings_axis_x_max,
            f_settings_axis_y_min,
            f_settings_axis_y_max,
            f_settings_axis_z_min,
            f_settings_axis_z_max);
    m_vec_mrk_wai_rep[9]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale/2.0,f_samples_marker_scale/2.0,f_samples_marker_scale/2.0));
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[9])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z);

    // EXAM is shifted along x
    tf::Vector3 vc3_position_exam;
    vc3_position_exam.setX(m_vc3_position.getX()+f_samples_spacing_x/4.0);
    vc3_position_exam.setY(m_vc3_position.getY());
    vc3_position_exam.setZ(m_vc3_position.getZ());
    m_vec_mrk_wai_rep[10]->UpdatePose(vc3_position_exam,m_qua_orientation,tf::Vector3(f_samples_marker_scale,f_samples_marker_scale,f_samples_marker_scale));
    ((Data3DMarker*)m_vec_mrk_wai_rep[10])->UpdateSettings(
            f_samples_spacing_x,
            f_samples_spacing_y,
            f_samples_spacing_z*2.0, // Actually put the weighting between part and exam here! :)
            f_samples_marker_scale,
            f_settings_axis_x_min,
            f_settings_axis_x_max,
            f_settings_axis_y_min,
            f_settings_axis_y_max,
            f_settings_axis_z_min,
            f_settings_axis_z_max);
    m_vec_mrk_wai_rep[11]->UpdatePose(vc3_position_exam,m_qua_orientation,tf::Vector3(f_samples_marker_scale/2.0,f_samples_marker_scale/2.0,f_samples_marker_scale/2.0));
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[11])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z*2.0);

    // OVERALL once again shifted along x
    tf::Vector3 vc3_position_overall;
    vc3_position_overall.setX(m_vc3_position.getX()+f_samples_spacing_x/2.0);
    vc3_position_overall.setY(m_vc3_position.getY());
    vc3_position_overall.setZ(m_vc3_position.getZ());
    m_vec_mrk_wai_rep[12]->UpdatePose(vc3_position_overall,m_qua_orientation,tf::Vector3(f_samples_marker_scale,f_samples_marker_scale,f_samples_marker_scale));
    ((Data3DMarker*)m_vec_mrk_wai_rep[12])->UpdateSettings(
            f_samples_spacing_x,
            f_samples_spacing_y,
            f_samples_spacing_z*2.0, // Actually put the weighting between part and exam here! :)
            f_samples_marker_scale,
            f_settings_axis_x_min,
            f_settings_axis_x_max,
            f_settings_axis_y_min,
            f_settings_axis_y_max,
            f_settings_axis_z_min,
            f_settings_axis_z_max);
    m_vec_mrk_wai_rep[13]->UpdatePose(vc3_position_overall,m_qua_orientation,tf::Vector3(f_samples_marker_scale/2.0,f_samples_marker_scale/2.0,f_samples_marker_scale/2.0));
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[13])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z*2.0);

}
void WAIRepGraph3D::UpdateGraphDataEval(float* graph_3d_data_part,float* graph_3d_data_exam,float* graph_3d_data_overall,int i_data_size_x,int i_data_size_y,int i_data_size_z,float f_eval_part_scores[],float f_eval_part_fracs[],float f_eval_exam_scores[],float f_eval_exam_fracs[],float f_eval_overall[],std::string s_eval_overall[],int i_id_selected)
{
    // Labels for summary per ID
    std::stringstream sst_id_to_str;
    sst_id_to_str.precision(3);

    if(i_id_selected==-1)
    {
        for(int x=0;x<m_i_grid_count_x;x++)
        {
            sst_id_to_str.str("");
            sst_id_to_str << "ID" << x << ": " << s_eval_overall[x] << std::endl
                          << "P: " << f_eval_part_scores[x] << "/" << f_eval_part_fracs[x] << std::endl
                          << "E: " << f_eval_exam_scores[x] << "/" << f_eval_exam_fracs[x] << std::endl
                          << "O: " << f_eval_overall[x];
            ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x])->UpdateText(sst_id_to_str.str());
            m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x]->UpdateColor(m_col_white);
            ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x])->UpdatePose(tf::Vector3(x*m_f_grid_spacing_x+m_f_grid_spacing_x/4.0,m_i_grid_count_y*m_f_grid_spacing_y,1.0+0.35),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
        }
    }
    else
    {
        for(int x=0;x<m_i_grid_count_x;x++)
        {
            sst_id_to_str.str("");
            if(x==i_id_selected)
            {
                sst_id_to_str << "ID" << x << ": " << s_eval_overall[x] << std::endl
                              << "P: " << f_eval_part_scores[x] << "/" << f_eval_part_fracs[x] << std::endl
                              << "E: " << f_eval_exam_scores[x] << "/" << f_eval_exam_fracs[x] << std::endl
                              << "O: " << f_eval_overall[x];
            }
            else
            {
                sst_id_to_str.str("N/A");
            }
            ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x])->UpdateText(sst_id_to_str.str());
            m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x]->UpdateColor(m_col_white);
            ((TextMarker*)m_vec_mrk_wai_rep[15+m_i_grid_count_x+m_i_grid_count_y+x])->UpdatePose(tf::Vector3(x*m_f_grid_spacing_x+m_f_grid_spacing_x/4.0,m_i_grid_count_y*m_f_grid_spacing_y,1.0+0.35),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
        }
    }


    // Data representation for EVAL Participation and Examination
    m_vec_mrk_wai_rep[8]->UpdateColor(m_col_invisible);
    ((Data3DMarker*)m_vec_mrk_wai_rep[8])->UpdateDataEval(graph_3d_data_part,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected);
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[9])->UpdateData(graph_3d_data_part,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected);

    m_vec_mrk_wai_rep[10]->UpdateColor(m_col_invisible);
    ((Data3DMarker*)m_vec_mrk_wai_rep[10])->UpdateDataEval(graph_3d_data_exam,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected);
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[11])->UpdateData(graph_3d_data_exam,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected);

    // Be carefule with mixing per subperiod stats and overall stats with fractions!
    // With FRACTIONS_MOSAIC:
    // Mean of individual subperiods to overall mean gets wronged (of course!)
    // E.g. in one subperiod PART 4/8=0.5 and 16/24=0.666 mean is 0.58333 ...
    // ... unequals (4+16)/(8+24)=0.625 (remember that throughout fraction size we achieve weighting here!)
    // To switch to simple mean calculation in the visualization pipeline...
    // Data3DMarker UpdateDataEval and HeightlinesEvalMarker UpdateData
    // ...disable "b_use_eval_status_overall parameter"!
    m_vec_mrk_wai_rep[12]->UpdateColor(m_col_oa);
    ((Data3DMarker*)m_vec_mrk_wai_rep[12])->UpdateDataEval(graph_3d_data_overall,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected,true,f_eval_overall);
    ((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[13])->UpdateData(graph_3d_data_overall,i_data_size_x,i_data_size_y,i_data_size_z,i_id_selected,true,f_eval_overall);
}
void WAIRepGraph3D::UpdateGraphDataFunction(float* graph_3d_data,int i_data_size_x,int i_data_size_y,int i_data_size_z)
{
    // Common data representation, e.g., 2D-functions f(x,y) with NO heightlines
    ((Data3DMarker*)m_vec_mrk_wai_rep[8])->UpdateDataFunction(graph_3d_data,i_data_size_x,i_data_size_y,i_data_size_z);
    //((HeightlinesEvalMarker*)m_vec_mrk_wai_rep[9])->UpdateData(graph_3d_data,i_data_size_x,i_data_size_y,i_data_size_z);
}
void WAIRepGraph3D::UpdateGraphCursor(tf::Vector3 vc3_label_cursor_position,std::string s_label_cursor)
{
    m_s_label_cursor=s_label_cursor;
    m_vec_mrk_wai_rep[14]->UpdatePose(vc3_label_cursor_position,tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.1));
    ((TextMarker*)m_vec_mrk_wai_rep[14])->UpdateText(m_s_label_cursor);

    //Reset cursor
    m_tmr_trigger_timeout=m_hdl_node->createTimer(ros::Duration(10.0),&WAIRepGraph3D::cb_tmr_timeout,this,true);
}
void WAIRepGraph3D::UpdateView()
{
    m_mrk_wai_rep_array.markers.clear();
    for(int i=0;i<m_vec_mrk_wai_rep.size();i++)
    {
        //m_vec_mrk_wai_rep[i]->GetMarker().header.stamp=ros::Time::now()-ros::Duration(1.0);
        m_mrk_wai_rep_array.markers.push_back(m_vec_mrk_wai_rep[i]->GetMarker());
    }
    m_pub_mrk_wai_rep_array.publish(m_mrk_wai_rep_array);
}





void WAIRepGraphStats::cb_tmr_timeout(const ros::TimerEvent& event)
{
    m_s_label_cursor="<CURSOR>";
    ((TextMarker*)m_vec_mrk_wai_rep[13])->UpdateText(m_s_label_cursor);
    UpdateView();
}
WAIRepGraphStats::WAIRepGraphStats()
{
}
WAIRepGraphStats::~WAIRepGraphStats()
{
}
void WAIRepGraphStats::Initialize(ros::NodeHandle* hdl_node,std::string s_namespace,std::string s_topic,std::string s_frame)
{
    m_hdl_node=hdl_node;
    m_s_namespace=s_namespace;
    m_s_topic=s_topic;
    m_s_frame=s_frame;
    m_pub_mrk_wai_rep_array=m_hdl_node->advertise<visualization_msgs::MarkerArray>(m_s_topic,1);

    m_col_grey.r=0.5; m_col_grey.g=0.5; m_col_grey.b=0.5; m_col_grey.a=0.5;
    m_col_white_trans.r=1.0; m_col_white_trans.g=1.0; m_col_white_trans.b=1.0; m_col_white_trans.a=0.3;
    m_col_white.r=1.0; m_col_white.g=1.0; m_col_white.b=1.0; m_col_white.a=0.9;
    m_col_white_opaque.r=1.0; m_col_white_opaque.g=1.0; m_col_white_opaque.b=1.0; m_col_white_opaque.a=1.0;
    m_col_green_opaque.r=0.0; m_col_green_opaque.g=1.0; m_col_green_opaque.b=0.0; m_col_green_opaque.a=1.0;
    m_col_red_opaque.r=1.0; m_col_red_opaque.g=0.0; m_col_red_opaque.b=0.0; m_col_red_opaque.a=1.0;
    m_col_orange.r=255.0/255.0; m_col_orange.g=165.0/255.0; m_col_orange.b=0.0; m_col_orange.a=0.5;
    m_col_orange_opaque.r=255.0/255.0; m_col_orange_opaque.g=165.0/255.0; m_col_orange_opaque.b=0.0; m_col_orange_opaque.a=1.0;
    m_col_cyan_opaque.r=0.0; m_col_cyan_opaque.g=1.0; m_col_cyan_opaque.b=1.0; m_col_cyan_opaque.a=1.0;
    m_col_oa.r=0.101960784; m_col_oa.g=0.42745098; m_col_oa.b=0.588235294; m_col_oa.a=0.9;
    m_col_invisible.r=0.0;m_col_invisible.g=0.0;m_col_invisible.b=0.0; m_col_invisible.a=0.0;

    m_vc3_outer_bounds=tf::Vector3(0,0,0);

    m_s_label_title="Min,Max\nMu,Sigma";
    m_s_label_axis_x="CND.#";
    m_s_label_axis_y="EVL.#";
    m_s_label_axis_z="STATS";
    m_s_label_cursor="<CURSOR>";
    m_col_axis_x=m_col_grey;
    m_col_axis_y=m_col_grey;
    m_col_axis_z=m_col_grey;
    m_col_grid=m_col_white;

    m_i_grid_count_x=0;
    m_i_grid_count_y=0;
    m_f_grid_spacing_x=1.0;
    m_f_grid_spacing_y=1.0;

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW"); // Arrow X-Axis [0]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW"); // Arrow Y-Axis [1]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("ARROW"); // Arrow Z-Axis [2]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("GRIDLINES");  // GRID Lines [3]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Label X-Label [4]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Label X-Label [5]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // Label X-Label [6]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // // Label Title [7]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    // MIN, MEAN, MAX Data points
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("HEIGHTLINES_STATS"); // [8] From MIN to MAX
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // [9] Sphere at MIN
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // [10] Sphere at MAX
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_3D"); // [11] Sphere at MEAN
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    // STDDEV Data points - From Mean val. to UPPER bound
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("DATA_STD_DEV"); // [12] Cube at MEAN
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);

    // CURSOR marker
    m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT"); // [13]
    m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
    m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
}
void WAIRepGraphStats::UpdateModel()
{

}
void WAIRepGraphStats::UpdateModel(tf::Vector3 vc3_position,tf::Quaternion qua_orientation,tf::Vector3 vc3_outer_bounds)
{
    m_vc3_position=vc3_position;
    m_qua_orientation=qua_orientation;
    m_col_axis_x=m_col_grey;
    m_col_axis_y=m_col_grey;
    m_col_axis_z=m_col_grey;
    m_col_grid=m_col_white;
    m_col_labels=m_col_white;
    m_vc3_outer_bounds=vc3_outer_bounds;

    m_vec_mrk_wai_rep[0]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_axis_x);
    ((ArrowMarker*)m_vec_mrk_wai_rep[0])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(m_vc3_outer_bounds.getX(),0,0));

    m_vec_mrk_wai_rep[1]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[1]->UpdateColor(m_col_axis_y);
    ((ArrowMarker*)m_vec_mrk_wai_rep[1])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(0,m_vc3_outer_bounds.getY(),0));

    m_vec_mrk_wai_rep[2]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.025,0.05,0.125));
    m_vec_mrk_wai_rep[2]->UpdateColor(m_col_axis_z);
    ((ArrowMarker*)m_vec_mrk_wai_rep[2])->UpdateVector(tf::Vector3(0,0,0),tf::Vector3(0,0,m_vc3_outer_bounds.getZ()));

    // Grid off by default
    m_vec_mrk_wai_rep[3]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(0.0,0.0,0.0));
    m_vec_mrk_wai_rep[3]->UpdateColor(m_col_grid);
    ((GridLinesMarker*)m_vec_mrk_wai_rep[3])->UpdateGrid(0,0,0.0,0.0);

    // Labels and title
    m_vec_mrk_wai_rep[4]->UpdatePose(tf::Vector3(m_vc3_position.getX()+m_vc3_outer_bounds.getX()+0.25,m_vc3_position.getY(),m_vc3_position.getZ()),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[4]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_label_axis_x);

    m_vec_mrk_wai_rep[5]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY()+m_vc3_outer_bounds.getY()+0.25,m_vc3_position.getZ()),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[5]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[5])->UpdateText(m_s_label_axis_y);

    m_vec_mrk_wai_rep[6]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY(),m_vc3_position.getZ()+m_vc3_outer_bounds.getZ()+0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[6]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[6])->UpdateText(m_s_label_axis_z);

    m_vec_mrk_wai_rep[7]->UpdatePose(tf::Vector3(m_vc3_position.getX(),m_vc3_position.getY(),m_vc3_position.getZ()-0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[7]->UpdateColor(m_col_labels);
    ((TextMarker*)m_vec_mrk_wai_rep[7])->UpdateText(m_s_label_title);

    m_vec_mrk_wai_rep[13]->UpdatePose(tf::Vector3(m_vc3_position.getX()+0.25,m_vc3_position.getY()+0.25,m_vc3_position.getZ()+0.25),m_qua_orientation,tf::Vector3(0.0,0.0,0.25));
    m_vec_mrk_wai_rep[13]->UpdateColor(m_col_cyan_opaque);
    ((TextMarker*)m_vec_mrk_wai_rep[13])->UpdateText(m_s_label_cursor);
}
void WAIRepGraphStats::UpdateLabelsAndColors(std::string s_label_axis_x,std::string s_label_axis_y,std::string s_label_axis_z,std::string s_label_title,std::string s_label_axis_cursor,std_msgs::ColorRGBA col_axis_x,std_msgs::ColorRGBA col_axis_y,std_msgs::ColorRGBA col_axis_z)
{
    m_s_label_title=s_label_title;
    m_s_label_axis_x=s_label_axis_x;
    m_s_label_axis_y=s_label_axis_y;
    m_s_label_axis_z=s_label_axis_z;
    m_s_label_cursor=s_label_axis_cursor;
    m_col_axis_x=col_axis_x;
    m_col_axis_y=col_axis_y;
    m_col_axis_z=col_axis_z;

    m_vec_mrk_wai_rep[0]->UpdateColor(m_col_axis_x);
    m_vec_mrk_wai_rep[1]->UpdateColor(m_col_axis_y);
    m_vec_mrk_wai_rep[2]->UpdateColor(m_col_axis_z);

    ((TextMarker*)m_vec_mrk_wai_rep[4])->UpdateText(m_s_label_axis_x);
    ((TextMarker*)m_vec_mrk_wai_rep[5])->UpdateText(m_s_label_axis_y);
    ((TextMarker*)m_vec_mrk_wai_rep[6])->UpdateText(m_s_label_axis_z);
    ((TextMarker*)m_vec_mrk_wai_rep[7])->UpdateText(m_s_label_title);
    ((TextMarker*)m_vec_mrk_wai_rep[13])->UpdateText(m_s_label_cursor);
}
void WAIRepGraphStats::InitGrid(int i_grid_count_x,int i_grid_count_y,float f_grid_spacing_x,float f_grid_spacing_y)
{
    m_i_grid_count_x=i_grid_count_x;
    m_i_grid_count_y=i_grid_count_y;
    m_f_grid_spacing_x=f_grid_spacing_x;
    m_f_grid_spacing_y=f_grid_spacing_y;

    for(int x=0;x<m_i_grid_count_x;x++) // Study CONDITIONS
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }
    for(int y=0;y<m_i_grid_count_y;y++) // # of STUDY
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }

    // Summary per Study-Cond
    for(int x=0;x<m_i_grid_count_x;x++) // SUMMARY
    {
        m_mrk_wai_rep=WAIRvizMarkers::create_rviz_marker("TEXT");
        m_mrk_wai_rep->Initialize(m_s_namespace,m_s_frame);
        m_vec_mrk_wai_rep.push_back(m_mrk_wai_rep);
    }
}
void WAIRepGraphStats::UpdateGrid(float f_grid_thickness,std_msgs::ColorRGBA col_grid)
{
    // Update labels of grid along x-axis and y-axis
    for(int x=0;x<m_i_grid_count_x;x++)
    {
        std::stringstream sst_id_to_str;
        sst_id_to_str << m_s_label_axis_x << (x+1); // Start with COND 1
        ((TextMarker*)m_vec_mrk_wai_rep[14+x])->UpdateText(sst_id_to_str.str());
        m_vec_mrk_wai_rep[14+x]->UpdateColor(m_col_white);
        ((TextMarker*)m_vec_mrk_wai_rep[14+x])->UpdatePose(tf::Vector3(x*m_f_grid_spacing_x,-m_f_grid_spacing_y/2.0,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
    }
    for(int y=0;y<m_i_grid_count_y;y++)
    {
        std::stringstream sst_id_to_str;
        sst_id_to_str << m_s_label_axis_y << (y+1); // Start with STUDY 1
        ((TextMarker*)m_vec_mrk_wai_rep[14+m_i_grid_count_x+y])->UpdateText(sst_id_to_str.str());
        m_vec_mrk_wai_rep[14+m_i_grid_count_x+y]->UpdateColor(m_col_white);
        ((TextMarker*)m_vec_mrk_wai_rep[14+m_i_grid_count_x+y])->UpdatePose(tf::Vector3(-m_f_grid_spacing_x/2.0,y*m_f_grid_spacing_y,0.0),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
    }

    m_vec_mrk_wai_rep[3]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_grid_thickness,f_grid_thickness,f_grid_thickness));
    m_vec_mrk_wai_rep[3]->UpdateColor(col_grid);
    ((GridLinesMarker*)m_vec_mrk_wai_rep[3])->UpdateGrid(m_i_grid_count_x,m_i_grid_count_y,m_f_grid_spacing_x,m_f_grid_spacing_y);
}
void WAIRepGraphStats::UpdateAxes(float f_samples_spacing_x,
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
    // Heightline from MIN to MAX
    m_vec_mrk_wai_rep[8]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale/4.0,f_samples_marker_scale/4.0,f_samples_marker_scale/4.0));
    ((HeightlinesStatsMarker*)m_vec_mrk_wai_rep[8])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z);

    // MIN, MAX and MEAN data points
    m_vec_mrk_wai_rep[9]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale*2.0/3.0,f_samples_marker_scale*2.0/3.0,f_samples_marker_scale*2.0/3.0));
    ((Data3DMarker*)m_vec_mrk_wai_rep[9])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z,f_samples_marker_scale*2.0/3.0,f_settings_axis_x_min,f_settings_axis_x_max,f_settings_axis_y_min,f_settings_axis_y_max,f_settings_axis_z_min,f_settings_axis_z_max);
    m_vec_mrk_wai_rep[10]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale*2.0/3.0,f_samples_marker_scale*2.0/3.0,f_samples_marker_scale*2.0/3.0));
    ((Data3DMarker*)m_vec_mrk_wai_rep[10])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z,f_samples_marker_scale*2.0/3.0,f_settings_axis_x_min,f_settings_axis_x_max,f_settings_axis_y_min,f_settings_axis_y_max,f_settings_axis_z_min,f_settings_axis_z_max);
    m_vec_mrk_wai_rep[11]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale,f_samples_marker_scale,f_samples_marker_scale));
    ((Data3DMarker*)m_vec_mrk_wai_rep[11])->UpdateSettings(f_samples_spacing_x,f_samples_spacing_y,f_samples_spacing_z,f_samples_marker_scale,f_settings_axis_x_min,f_settings_axis_x_max,f_settings_axis_y_min,f_settings_axis_y_max,f_settings_axis_z_min,f_settings_axis_z_max);

    // STDDEV Data represented by cube
    m_vec_mrk_wai_rep[12]->UpdatePose(m_vc3_position,m_qua_orientation,tf::Vector3(f_samples_marker_scale/2.0,f_samples_marker_scale/2.0,f_samples_marker_scale));
    ((DataStdDevMarker*)m_vec_mrk_wai_rep[12])->UpdateSettings(
            f_samples_spacing_x,
            f_samples_spacing_y,
            f_samples_spacing_z,
            f_samples_marker_scale,
            f_settings_axis_x_min,
            f_settings_axis_x_max,
            f_settings_axis_y_min,
            f_settings_axis_y_max,
            f_settings_axis_z_min,
            f_settings_axis_z_max);
}
void WAIRepGraphStats::UpdateGraphDataStats(float* graph_3d_data_min,float* graph_3d_data_max,float* graph_3d_data_mean,float* graph_3d_data_stddev,int i_data_size_x,int i_data_size_y,int i_data_size_z)//,float f_eval_part_scores[],float f_eval_part_fracs[],float f_eval_exam_scores[],float f_eval_exam_fracs[],float f_eval_overall[],std::string s_eval_overall[])
{
    // Labels for summary per ID
    std::stringstream sst_id_to_str;
    sst_id_to_str.precision(3);

    // Update text for overall stats
    for(int x=0;x<m_i_grid_count_x;x++)
    {
        sst_id_to_str.str("");
        sst_id_to_str << "COND" << x << ": " << std::endl // << s_eval_overall[x] << std::endl
                      << "M: " << std::endl //<< f_eval_part_scores[x] << "/" << f_eval_part_fracs[x] << std::endl
                      << "S: " << std::endl //<< f_eval_exam_scores[x] << "/" << f_eval_exam_fracs[x] << std::endl
                      << "S2: " ;//<< f_eval_overall[x];
        ((TextMarker*)m_vec_mrk_wai_rep[14+m_i_grid_count_x+m_i_grid_count_y+x])->UpdateText(sst_id_to_str.str());
        m_vec_mrk_wai_rep[14+m_i_grid_count_x+m_i_grid_count_y+x]->UpdateColor(m_col_white);
        ((TextMarker*)m_vec_mrk_wai_rep[14+m_i_grid_count_x+m_i_grid_count_y+x])->UpdatePose(tf::Vector3(x*m_f_grid_spacing_x+m_f_grid_spacing_x/4.0,m_i_grid_count_y*m_f_grid_spacing_y,0.35),tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.125));
    }

    // Plot HEIGHTLINES from MIN to MAX
    m_vec_mrk_wai_rep[8]->UpdateColor(m_col_orange);
    ((HeightlinesStatsMarker*)m_vec_mrk_wai_rep[8])->UpdateData(graph_3d_data_min,graph_3d_data_max,graph_3d_data_mean,graph_3d_data_stddev,i_data_size_x,i_data_size_y,i_data_size_z); // Min max only

    // Plot spheres at MIN, MAX and MEAN
    m_vec_mrk_wai_rep[9]->UpdateColor(m_col_green_opaque);
    ((Data3DMarker*)m_vec_mrk_wai_rep[9])->UpdateDataStats(graph_3d_data_min,i_data_size_x,i_data_size_y,i_data_size_z);
    m_vec_mrk_wai_rep[10]->UpdateColor(m_col_red_opaque);
    ((Data3DMarker*)m_vec_mrk_wai_rep[10])->UpdateDataStats(graph_3d_data_max,i_data_size_x,i_data_size_y,i_data_size_z);
    m_vec_mrk_wai_rep[11]->UpdateColor(m_col_cyan_opaque);
    ((Data3DMarker*)m_vec_mrk_wai_rep[11])->UpdateDataStats(graph_3d_data_mean,i_data_size_x,i_data_size_y,i_data_size_z);

    // Plot STDDEV
    m_vec_mrk_wai_rep[12]->UpdateColor(m_col_orange_opaque);
    ((DataStdDevMarker*)m_vec_mrk_wai_rep[12])->UpdateDataStdDev(graph_3d_data_mean,graph_3d_data_stddev,i_data_size_x,i_data_size_y,i_data_size_z);

}
void WAIRepGraphStats::UpdateGraphCursor(tf::Vector3 vc3_label_cursor_position,std::string s_label_cursor)
{
    m_s_label_cursor=s_label_cursor;
    m_vec_mrk_wai_rep[13]->UpdatePose(vc3_label_cursor_position,tf::Quaternion(0.0,0.0,0.0,1.0),tf::Vector3(0.0,0.0,0.1));
    ((TextMarker*)m_vec_mrk_wai_rep[13])->UpdateText(m_s_label_cursor);

    //Reset cursor
    m_tmr_trigger_timeout=m_hdl_node->createTimer(ros::Duration(10.0),&WAIRepGraphStats::cb_tmr_timeout,this,true);
}
void WAIRepGraphStats::UpdateView()
{
    m_mrk_wai_rep_array.markers.clear();
    for(int i=0;i<m_vec_mrk_wai_rep.size();i++)
    {
        m_mrk_wai_rep_array.markers.push_back(m_vec_mrk_wai_rep[i]->GetMarker());
    }
    m_pub_mrk_wai_rep_array.publish(m_mrk_wai_rep_array);
}
