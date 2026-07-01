#include "rviz_plugin_wai_texture.h"

using namespace rviz; // Necessary ONLY for transport hint signals/slots


namespace rviz_plugin_wai_texture
{
    RVizPluginWAITexture::RVizPluginWAITexture():Display()
    {
    }
    RVizPluginWAITexture::~RVizPluginWAITexture()
    {
        Ogre::TextureManager::getSingleton().remove("VideoTexture"+std::to_string(ui_msh_obj_name_cnt));
        Ogre::MaterialManager::getSingleton().remove("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt));
        //Ogre::MeshManager::getSingleton().unload("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        //Ogre::MeshManager::getSingleton().remove("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        scene_manager_->destroyEntity("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
    }

    void RVizPluginWAITexture::onInitialize()
    {
        // Make resource names unique (Ogre objects and plugin namespace)
        ui_msh_obj_name_cnt++;

        // Initialize node
        std::string m_s_rep_id="";
        ros::get_environment_variable(m_s_rep_id,"WAI_OA_AUDIENCE_ID");
        m_hdl_node=new ros::NodeHandle("/wai_world/oa"+m_s_rep_id+"/rviz_plugin_wai_texture"+std::to_string(ui_msh_obj_name_cnt));
        //ROS_WARN_STREAM("Path:"<< m_hdl_node->getNamespace());

        // Initialize timout icon
        m_s_path_icon_timeout=ros::package::getPath("wai_world_launcher")+"/resources/icons/rviz_plugin_oa_logo.png";
        m_mat_icon_timeout=cv::imread(m_s_path_icon_timeout);
        //cv::cvtColor(m_mat_icon_timeout,m_mat_icon_timeout,CV_BGR2BGR);

        // Initialize RViz properties
        m_b_received_image=false;
        m_i_res_x=320; m_i_res_y=240;
        m_prp_image_topic=new rviz::StringProperty("Image Topic","/usb_cam/image_raw/compressed","Image topic to subscribe to.",this,SLOT(cb_prp_update_image_topic()));
        m_prp_image_res_x=new rviz::IntProperty("Image Res. X",m_i_res_x,"Horizontal X resolution of the image.",this,SLOT(cb_prp_update_image_res_x()));
        m_prp_image_res_y=new rviz::IntProperty("Image Res. Y",m_i_res_y,"Vertical Y resolution of the image.",this,SLOT(cb_prp_update_image_res_y()));
        m_prp_image_res_x->setReadOnly(true);
        m_prp_image_res_y->setReadOnly(true);
        m_prp_image_width=new rviz::FloatProperty("Image Width",1.0,"Width of the image.",this,SLOT(cb_prp_update_image_width()));
        m_prp_image_height=new rviz::FloatProperty("Image Height",1.0,"Height of the image.",this,SLOT(cb_prp_update_image_height()));
        m_prp_image_tf_frame=new rviz::TfFrameProperty("Image TF Frame",rviz::TfFrameProperty::FIXED_FRAME_STRING,"Frame of the rendered image.",this,context_->getFrameManager(),true,SLOT(cb_prp_update_tf_frame()));
        m_prp_image_tf_refresh=new rviz::BoolProperty("Refresh TF Frame",false,"Regularly refresh image TF frame.",this,SLOT(cb_prp_update_tf_refresh()));
        m_prp_border_thickness=new rviz::FloatProperty("Image Border Size",0.01,"Rviz texture border thickness.",this,SLOT(cb_prp_update_image_border_thickness()));
        m_prp_border_thickness->setMin(0.01);
        m_prp_border_thickness->setMax(0.1);
        m_prp_border_color=new rviz::ColorProperty("Image Border Color",QColor(25,108,149),"Color of the image border.",this,SLOT(cb_prp_update_image_border_color()));
        m_prp_image_alpha=new rviz::FloatProperty("Image Alpha",1.0,"Image transparency (Alpha).",this,SLOT(cb_prp_update_image_alpha()));
        m_prp_image_alpha->setMin(0.1);
        m_prp_image_alpha->setMax(1.0);

        // Initialize OGRE Resources
        UpdateOgreResources();

        // Draw image border
        m_shp_border_left=new rviz::Shape(rviz::Shape::Cylinder,scene_manager_,scene_node_);
        m_shp_border_right=new rviz::Shape(rviz::Shape::Cylinder,scene_manager_,scene_node_);
        m_shp_border_top=new rviz::Shape(rviz::Shape::Cylinder,scene_manager_,scene_node_);
        m_shp_border_bottom=new rviz::Shape(rviz::Shape::Cylinder,scene_manager_,scene_node_);
        UpdateImageFrame();

        // Initialize subscribers and publishers
        //m_sub_hints.unreliable();
        m_sub_hints.tcpNoDelay();
        UpdateImageTopicAndTransport();

        // Start timer
        //m_tmr_frame=m_hdl_node->createTimer(ros::Duration(10.0),&RVizPluginWAITexture::cb_tmr_frame,this,true,true);
        m_tmr_image=m_hdl_node->createTimer(ros::Duration(10.0),&RVizPluginWAITexture::cb_tmr_image,this,false,true);
    }
    void RVizPluginWAITexture::onDisable()
    {
        // Simply set alpha to 0.0 once disabled
        m_map_material->setDepthWriteEnabled(false);
        m_tus_texture_unit_state=m_pas_texture->getTextureUnitState(0);
        m_tus_texture_unit_state->setAlphaOperation(
            Ogre::LBX_BLEND_MANUAL,
            Ogre::LBS_MANUAL,
            Ogre::LBS_MANUAL,
            0.0,
            0.0,
            0.0);
        m_tus_texture_unit_state->setColourOperation(Ogre::LBO_REPLACE);
    }
    void RVizPluginWAITexture::onEnable()
    {
        m_map_material->setDepthWriteEnabled(true);
        m_tus_texture_unit_state=m_pas_texture->getTextureUnitState(0);
        m_tus_texture_unit_state->setAlphaOperation(
            Ogre::LBX_BLEND_MANUAL,
            Ogre::LBS_MANUAL,
            Ogre::LBS_MANUAL,
            1.0,
            1.0,
            1.0);
        m_tus_texture_unit_state->setColourOperation(Ogre::LBO_REPLACE);
    }
    void RVizPluginWAITexture::reset()
    {
    }
    void RVizPluginWAITexture::update(float wall_dt,float ros_dt)
    {
        // If the frame is moving, always update the rendered image texture and the border!
        if(m_prp_image_tf_refresh->getBool()==true)
        {
            UpdateImageFrame();
        }
    }


    // Callbacks
    void RVizPluginWAITexture::cb_sub_image(const sensor_msgs::CompressedImageConstPtr& msg)
    {
        m_b_received_image=true;
        //ROS_WARN_STREAM(m_hdl_node->getNamespace() << " callback triggered...");
        // Convert ROS Image Message to OpenCV Image
        try
        {
            // Directly decode image without using image transport (way more efficient!)
            // Convert JPEG compressed image to OGRE texture format!
            m_mat_image_to_render=cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
            cv::cvtColor(m_mat_image_to_render,m_mat_image_to_render,CV_BGR2BGRA);

            if(m_mat_image_to_render.cols==m_i_res_x && m_mat_image_to_render.rows==m_i_res_y)
            {
                // Do nothing...
            }
            else
            {
                // Update actual image resolution properties
                m_i_res_x=m_mat_image_to_render.cols;
                m_i_res_y=m_mat_image_to_render.rows;
                m_prp_image_res_x->setInt(m_mat_image_to_render.cols);
                m_prp_image_res_y->setInt(m_mat_image_to_render.rows);
                UpdateTextureResolution(m_mat_image_to_render.cols,m_mat_image_to_render.rows);
            }

            UpdateTexture();
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_WARN_STREAM("cv_exception: " << e.what());
        }
    }

    /*
    void RVizPluginWAITexture::cb_tmr_frame(const ros::TimerEvent& event)
    {
        UpdateImageFrame();
    }
    */

    void RVizPluginWAITexture::cb_tmr_image(const ros::TimerEvent& event)
    {
        // Do nothing for now...
        if(m_b_received_image==false)
        {
            UpdateImageTopicAndTransport();
        }
        else
        {
            m_tmr_image.stop();
        }
    }

    void RVizPluginWAITexture::UpdateOgreResources()
    {
        // Create texture
        m_tep_texture=Ogre::TextureManager::getSingleton().createManual("VideoTexture"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,m_prp_image_res_x->getInt(),m_prp_image_res_y->getInt(),0,Ogre::PF_BYTE_BGRA,Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE); // TU_DYNAMIC

        // Create proper material that has culling disabled (image is visible from both sides!)
        m_map_material=Ogre::MaterialManager::getSingleton().create("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        m_map_material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

        // Create TexturePass and TextureUnitState
        //m_map_material->setLightingEnabled(false);
        m_pas_texture=m_map_material->getTechnique(0)->getPass(0);
        m_pas_texture->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        m_tus_texture_unit_state=m_pas_texture->createTextureUnitState("VideoTexture"+std::to_string(ui_msh_obj_name_cnt));
        UpdateImageAlpha();

        // Create Plane to render the image to
        m_pla_plane.redefine(Ogre::Vector3::UNIT_Z,Ogre::Vector3(0,0,0));
        Ogre::MeshManager::getSingleton().createPlane("VideoPlane"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,m_pla_plane,m_prp_image_width->getFloat(), m_prp_image_height->getFloat(),1,1,true,1,1.0f,1.0f,Ogre::Vector3::UNIT_Y);

        // Create Entity and attach material
        m_ent_entity=scene_manager_->createEntity("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        m_ent_entity->setMaterialName("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt));

        // Attach Entity to Scene Node
        m_hdl_node_scene=scene_manager_->getRootSceneNode()->createChildSceneNode();
        m_hdl_node_scene->setPosition(Ogre::Vector3(0.0,0.0,0.0));
        m_hdl_node_scene->setOrientation(Ogre::Quaternion(1.0,0.0,0.0,0.0));
        m_hdl_node_scene->attachObject(m_ent_entity);
    }
    void RVizPluginWAITexture::UpdateTexture()
    {
        // Simply Copy Texture Contents To Pixelbuffer
        pixelBuffer=m_tep_texture->getBuffer();
        pixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
        //const Ogre::PixelBox& pixelBox=pixelBuffer->getCurrentLock().data;
        mem_destination=static_cast<uint8_t*>(pixelBuffer->getCurrentLock().data);
        memcpy(mem_destination,m_mat_image_to_render.data,m_mat_image_to_render.total()*m_mat_image_to_render.elemSize());
        pixelBuffer->unlock();
    }
    void RVizPluginWAITexture::UpdateTextureResolution(int i_res_x,int i_res_y)
    {
        //cv::resize(m_mat_image_to_render,m_mat_image_to_render,cv::Size(i_res_x,i_res_y));

        // Detach entity
        m_hdl_node_scene->detachObject(m_ent_entity);

        // Create texture
        Ogre::TextureManager::getSingleton().remove("VideoTexture"+std::to_string(ui_msh_obj_name_cnt));
        m_tep_texture=Ogre::TextureManager::getSingleton().createManual("VideoTexture"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,i_res_x,i_res_y,0,Ogre::PF_BYTE_BGRA,Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

        // Create proper material that has culling disabled (image is visible from both sides!)
        Ogre::MaterialManager::getSingleton().remove("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt));
        m_map_material=Ogre::MaterialManager::getSingleton().create("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        m_map_material->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

        // Create TexturePass and TextureUnitState
        //m_map_material->setLightingEnabled(false);
        m_pas_texture->removeAllTextureUnitStates();
        m_pas_texture=m_map_material->getTechnique(0)->getPass(0);
        m_pas_texture->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        m_tus_texture_unit_state=m_pas_texture->createTextureUnitState("VideoTexture"+std::to_string(ui_msh_obj_name_cnt));
        UpdateImageAlpha();

        // Create Plane to render the image to
        Ogre::MeshManager::getSingleton().remove("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        m_pla_plane.redefine(Ogre::Vector3::UNIT_Z,Ogre::Vector3(0,0,0));
        Ogre::MeshManager::getSingleton().createPlane("VideoPlane"+std::to_string(ui_msh_obj_name_cnt),Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,m_pla_plane,m_prp_image_width->getFloat(), m_prp_image_height->getFloat(),1,1,true,1,1.0f,1.0f,Ogre::Vector3::UNIT_Y);

        // Create Entity and attach material
        scene_manager_->destroyEntity("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        m_ent_entity=scene_manager_->createEntity("VideoPlane"+std::to_string(ui_msh_obj_name_cnt));
        m_ent_entity->setMaterialName("VideoMaterial"+std::to_string(ui_msh_obj_name_cnt));

        // Re-Attach Entity to Scene Node
        m_hdl_node_scene=scene_manager_->getRootSceneNode()->createChildSceneNode();
        m_hdl_node_scene->setPosition(Ogre::Vector3(0.0,0.0,0.0));
        m_hdl_node_scene->setOrientation(Ogre::Quaternion(1.0,0.0,0.0,0.0));
        m_hdl_node_scene->attachObject(m_ent_entity);
    }


    void RVizPluginWAITexture::cb_prp_update_image_topic()
    {
        UpdateImageTopicAndTransport();
    }
    void RVizPluginWAITexture::UpdateImageTopicAndTransport()
    {
        //m_sub_image.shutdown();
        //ros::spinOnce();
        m_sub_image=m_hdl_node->subscribe(m_prp_image_topic->getStdString(),0,&RVizPluginWAITexture::cb_sub_image,this,m_sub_hints);
        //ros::spinOnce();
    }

    void RVizPluginWAITexture::cb_prp_update_image_res_x()
    {
        // Do nothing for now, but keep callback to properly register RViz property!
    }
    void RVizPluginWAITexture::cb_prp_update_image_res_y()
    {
        // Do nothing for now, but keep callback to properly register RViz property!
    }
    void RVizPluginWAITexture::cb_prp_update_image_width()
    {
        m_hdl_node_scene->setScale(m_prp_image_width->getFloat(),m_prp_image_height->getFloat(),1.0);
        UpdateImageBorder();
    }
    void RVizPluginWAITexture::cb_prp_update_image_height()
    {
        m_hdl_node_scene->setScale(m_prp_image_width->getFloat(),m_prp_image_height->getFloat(),1.0);
        UpdateImageBorder();
    }

    void RVizPluginWAITexture::cb_prp_update_tf_frame()
    {
        UpdateImageFrame();
    }
    void RVizPluginWAITexture::cb_prp_update_tf_refresh()
    {
        // Do nothing for now...
    }
    void RVizPluginWAITexture::UpdateImageFrame()
    {
        context_->getFrameManager()->getTransform(m_prp_image_tf_frame->getFrameStd(),ros::Time(0),m_vc3_position,m_qua_orientation);
        m_tf2_image.setOrigin(tf::Vector3(m_vc3_position.x,m_vc3_position.y,m_vc3_position.z));
        m_tf2_image.setRotation(tf::Quaternion(m_qua_orientation.x,m_qua_orientation.y,m_qua_orientation.z,m_qua_orientation.w));
        UpdateImageBorder();
        UpdateImageRendered();
    }

    void RVizPluginWAITexture::cb_prp_update_image_border_thickness()
    {
        UpdateImageBorder();
    }
    void RVizPluginWAITexture::cb_prp_update_image_border_color()
    {
        UpdateImageBorder();
    }
    void RVizPluginWAITexture::UpdateImageBorder()
    {
        m_shp_border_left->setScale(Ogre::Vector3(m_prp_border_thickness->getFloat(),m_prp_image_height->getFloat(),m_prp_border_thickness->getFloat()));
        m_shp_border_right->setScale(Ogre::Vector3(m_prp_border_thickness->getFloat(),m_prp_image_height->getFloat(),m_prp_border_thickness->getFloat()));
        m_shp_border_top->setScale(Ogre::Vector3(m_prp_image_width->getFloat(),m_prp_border_thickness->getFloat(),m_prp_border_thickness->getFloat()));
        m_shp_border_bottom->setScale(Ogre::Vector3(m_prp_image_width->getFloat(),m_prp_border_thickness->getFloat(),m_prp_border_thickness->getFloat()));

        tf::Vector3 vc3_left(m_prp_image_width->getFloat()/2.0,0.0,0.0); vc3_left=m_tf2_image*vc3_left;
        tf::Vector3 vc3_right(-m_prp_image_width->getFloat()/2.0,0.0,0.0); vc3_right=m_tf2_image*vc3_right;
        tf::Vector3 vc3_top(0.0,m_prp_image_height->getFloat()/2.0,0.0); vc3_top=m_tf2_image*vc3_top;
        tf::Vector3 vc3_bottom(0.0,-m_prp_image_height->getFloat()/2.0,0.0); vc3_bottom=m_tf2_image*vc3_bottom;

        m_shp_border_left->setPosition(Ogre::Vector3(vc3_left.getX(),vc3_left.getY(),vc3_left.getZ()));
        m_shp_border_right->setPosition(Ogre::Vector3(vc3_right.getX(),vc3_right.getY(),vc3_right.getZ()));
        m_shp_border_top->setPosition(Ogre::Vector3(vc3_top.getX(),vc3_top.getY(),vc3_top.getZ()));
        m_shp_border_bottom->setPosition(Ogre::Vector3(vc3_bottom.getX(),vc3_bottom.getY(),vc3_bottom.getZ()));
        m_shp_border_left->setOrientation(m_qua_orientation);
        m_shp_border_right->setOrientation(m_qua_orientation);
        m_shp_border_top->setOrientation(m_qua_orientation);
        m_shp_border_bottom->setOrientation(m_qua_orientation);

        Ogre::ColourValue cov_color(m_prp_border_color->getOgreColor().r,
                                    m_prp_border_color->getOgreColor().g,
                                    m_prp_border_color->getOgreColor().b,
                                    m_prp_border_color->getOgreColor().a);

        m_shp_border_left->setColor(cov_color);
        m_shp_border_right->setColor(cov_color);
        m_shp_border_top->setColor(cov_color);
        m_shp_border_bottom->setColor(cov_color);
    }
    void RVizPluginWAITexture::UpdateImageRendered()
    {
        // Update pose of rendered image
        m_hdl_node_scene->setPosition(m_vc3_position);
        m_hdl_node_scene->setOrientation(m_qua_orientation);
    }

    void RVizPluginWAITexture::cb_prp_update_image_alpha()
    {
        UpdateImageAlpha();
    }
    void RVizPluginWAITexture::UpdateImageAlpha()
    {
        if(m_prp_image_alpha->getFloat()==1.0)
        {
            // Actually make texture intransparent no matter what bg
            m_map_material->setDepthWriteEnabled(true);
            m_tus_texture_unit_state=m_pas_texture->getTextureUnitState(0);
        }
        else
        {
            m_map_material->setDepthWriteEnabled(false);
            m_tus_texture_unit_state=m_pas_texture->getTextureUnitState(0);
        }
        m_tus_texture_unit_state->setAlphaOperation(
            Ogre::LBX_BLEND_MANUAL,
            Ogre::LBS_MANUAL,
            Ogre::LBS_MANUAL,
            m_prp_image_alpha->getFloat(),
            m_prp_image_alpha->getFloat(),
            m_prp_image_alpha->getFloat());
        m_tus_texture_unit_state->setColourOperation(Ogre::LBO_REPLACE);
    }
} // namespace rviz_plugin_wai_texture

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_texture::RVizPluginWAITexture,rviz::Display)
