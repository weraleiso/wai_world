#include "rviz_plugin_wai_lights.h"


namespace rviz_plugin_wai_lights
{
    using namespace rviz;

    RVizPluginWAILights::RVizPluginWAILights() : Display()
    {
        i_static_light_id++;

        m_iam_server=new interactive_markers::InteractiveMarkerServer("light_"+std::to_string(i_static_light_id)+"/iam_server");

        m_prp_frame=new TfFrameProperty("Reference Frame",TfFrameProperty::FIXED_FRAME_STRING,"",this,NULL,true);

        m_prp_enu_light_type=new EnumProperty("Light Type","Ambient","",this,SLOT(updateLightType()));
        m_prp_enu_light_type->addOption("Ambient",3);
        m_prp_enu_light_type->addOption("Directional",Ogre::Light::LT_DIRECTIONAL);
        m_prp_enu_light_type->addOption("Point",Ogre::Light::LT_POINT);
        m_prp_enu_light_type->addOption("Spot",Ogre::Light::LT_SPOTLIGHT);
        m_prp_enu_light_type->addOption("Sun",4);

        m_prp_bol_light_shadows_enabled=new BoolProperty("Shadows Enabled",false,"",this,SLOT(updateShadowsEnabled()));
        m_prp_bol_light_markers_enabled=new BoolProperty("Markers Enabled",false,"",this,SLOT(updateMarkerEnabled()));

        m_prp_col_light_ambient=new ColorProperty("Ambient Color",QColor(0,0,0,255),"",this,SLOT(updateAmbientColor()));
        m_prp_col_light_diffuse=new ColorProperty("Diffuse Color",QColor(0,0,0,255),"",this,SLOT(updateDiffuseColor()));
        m_prp_col_light_specular=new ColorProperty("Specular Color",QColor(0,0,0,255),"",this,SLOT(updateSpecularColor()));
        m_prp_vec_hsv_light_diff_spec=new VectorProperty("Diff./Spec. HSV",Ogre::Vector3(0.0,0.0,0.0),"",this,SLOT(updateColorDiffSpecHSV()));
        m_prp_vec_ycbcr_light_diff_spec=new VectorProperty("Diff./Spec. YCbCr",Ogre::Vector3(0.0,0.0,0.0),"",this,SLOT(updateColorDiffSpecYCbCr()));

        m_prp_vec_light_position_offset=new VectorProperty("Position",Ogre::Vector3(0.0,0.0,10.0),"",this, SLOT(updateLightPose()));
        m_prp_qua_light_orientation_offset=new QuaternionProperty("Orientation",Ogre::Quaternion(1.0,0.0,0.0,0.0),"",this,SLOT(updateLightPose()));

        m_prp_f_light_range=new FloatProperty("Range",100.0f,"",this,SLOT(updateRange()));
        m_prp_vec_light_attenuation=new VectorProperty("Attenuation",Ogre::Vector3(1.0,0.0,0.0),"",this,SLOT(updateAttenuation()));
        m_prp_f_light_inner_angle=new FloatProperty("Inner Cone Angle",1.0f,"",this,SLOT(updateInnerAngle()));
        m_prp_f_light_outer_angle=new FloatProperty("Outer Cone Angle",2.0f,"",this,SLOT(updateOuterAngle()));
        m_prp_f_light_falloff_rate=new FloatProperty("Falloff Cone Angles",50.0f,"",this,SLOT(updateFalloff()));
        m_prp_f_light_sun_radial_distance=new FloatProperty("Sunlight Distance",10.0f,"",this,SLOT(updateSunlight()));
        m_prp_i_light_sun_rise_hour=new IntProperty("Sunlight Sunrise Hour",6,"",this,SLOT(updateSunlight()));

        // Subscriber
        m_sub_col_light_color_ambient=m_hdl_node.subscribe("light_"+std::to_string(i_static_light_id)+"/color_ambient",1,&RVizPluginWAILights::cb_sub_col_light_color_ambient,this);
        m_sub_col_light_color_diff_spec=m_hdl_node.subscribe("light_"+std::to_string(i_static_light_id)+"/color_diff_spec",1,&RVizPluginWAILights::cb_sub_col_light_color_diff_spec,this);
        m_sub_bol_light_enable=m_hdl_node.subscribe("light_"+std::to_string(i_static_light_id)+"/enable",1,&RVizPluginWAILights::cb_sub_bol_light_enable,this);

        // POINTLIGHT Marker
        m_msg_mrk_pointlight.header.frame_id="world";
        m_msg_mrk_pointlight.header.stamp=ros::Time::now();
        // m_msg_mrk_pointlight.lifetime=ros::Duration(0.1);
        m_msg_mrk_pointlight.ns="light_"+std::to_string(i_static_light_id);
        m_msg_mrk_pointlight.id=0;
        m_msg_mrk_pointlight.type=visualization_msgs::Marker::MESH_RESOURCE;
        m_msg_mrk_pointlight.action=visualization_msgs::Marker::ADD;
        m_msg_mrk_pointlight.pose.position.x=0.0;
        m_msg_mrk_pointlight.pose.position.y=0.0;
        m_msg_mrk_pointlight.pose.position.z=10.0;
        m_msg_mrk_pointlight.pose.orientation.w=1.0;
        m_msg_mrk_pointlight.pose.orientation.x=0.0;
        m_msg_mrk_pointlight.pose.orientation.y=0.0;
        m_msg_mrk_pointlight.pose.orientation.z=0.0;
        m_msg_mrk_pointlight.scale.x=1.0;
        m_msg_mrk_pointlight.scale.y=1.0;
        m_msg_mrk_pointlight.scale.z=1.0;
        m_msg_mrk_pointlight.mesh_use_embedded_materials=true;
        m_msg_mrk_pointlight.mesh_resource="file://"+ros::package::getPath("rviz_plugin_wai_lights")+"/resources/pointlight/pointlight.dae";

        // SPOTLIGHT Marker
        m_msg_mrk_spotlight.header.frame_id="world";
        m_msg_mrk_spotlight.header.stamp=ros::Time::now();
        // m_msg_mrk_spotlight.lifetime=ros::Duration(0.1);
        m_msg_mrk_spotlight.ns="light_"+std::to_string(i_static_light_id);
        m_msg_mrk_spotlight.id=0;
        m_msg_mrk_spotlight.type=visualization_msgs::Marker::MESH_RESOURCE;
        m_msg_mrk_spotlight.action=visualization_msgs::Marker::ADD;
        m_msg_mrk_spotlight.pose.position.x=0.0;
        m_msg_mrk_spotlight.pose.position.y=0.0;
        m_msg_mrk_spotlight.pose.position.z=10.0;
        m_msg_mrk_spotlight.pose.orientation.w=1.0;
        m_msg_mrk_spotlight.pose.orientation.x=0.0;
        m_msg_mrk_spotlight.pose.orientation.y=0.0;
        m_msg_mrk_spotlight.pose.orientation.z=0.0;
        m_msg_mrk_spotlight.scale.x=1.0;
        m_msg_mrk_spotlight.scale.y=1.0;
        m_msg_mrk_spotlight.scale.z=1.0;
        m_msg_mrk_spotlight.mesh_use_embedded_materials=true;
        m_msg_mrk_spotlight.mesh_resource="file://"+ros::package::getPath("rviz_plugin_wai_lights")+"/resources/spotlight/spotlight.dae";

        // SUNLIGHT Marker
        m_msg_mrk_sunlight.header.frame_id="world";
        m_msg_mrk_sunlight.header.stamp=ros::Time::now();
        // m_msg_mrk_sunlight.lifetime=ros::Duration(0.1);
        m_msg_mrk_sunlight.ns="light_"+std::to_string(i_static_light_id);
        m_msg_mrk_sunlight.id=0;
        m_msg_mrk_sunlight.type=visualization_msgs::Marker::MESH_RESOURCE;
        m_msg_mrk_sunlight.action=visualization_msgs::Marker::ADD;
        m_msg_mrk_sunlight.pose.position.x=0.0;
        m_msg_mrk_sunlight.pose.position.y=0.0;
        m_msg_mrk_sunlight.pose.position.z=10.0;
        m_msg_mrk_sunlight.pose.orientation.w=1.0;
        m_msg_mrk_sunlight.pose.orientation.x=0.0;
        m_msg_mrk_sunlight.pose.orientation.y=0.0;
        m_msg_mrk_sunlight.pose.orientation.z=0.0;
        m_msg_mrk_sunlight.scale.x=0.00004;
        m_msg_mrk_sunlight.scale.y=0.00004;
        m_msg_mrk_sunlight.scale.z=0.00004;
        m_msg_mrk_sunlight.mesh_use_embedded_materials=true;
        m_msg_mrk_sunlight.mesh_resource="file://"+ros::package::getPath("rviz_plugin_wai_lights")+"/resources/sun/sun.dae";
    }
    RVizPluginWAILights::~RVizPluginWAILights()
    {
        m_ogr_scene_manager->destroyLight(m_ogr_light);
    }

    void RVizPluginWAILights::onInitialize()
    {
        Display::onInitialize();
        m_ogr_scene_manager=context_->getSceneManager();

        // Initialize properties
        m_prp_frame->setFrameManager(context_->getFrameManager());
        m_s_frame="world";

        m_b_light_shadows_enabled=false;
        m_b_light_markers_enabled=false;

        qcl_light_ambient_old=m_prp_col_light_ambient->getColor();
        qcl_light_diffuse_old=m_prp_col_light_diffuse->getColor();
        qcl_light_specular_old=m_prp_col_light_specular->getColor();
        m_ogr_vc3_light_position=Ogre::Vector3(0.0,0.0,10.0);
        m_ogr_qua_light_orientation=Ogre::Quaternion(1.0,0.0,0.0,0.0);
        m_ogr_vc3_frame_position=Ogre::Vector3(0.0,0.0,0.0);
        m_ogr_qua_frame_orientation=Ogre::Quaternion(1.0,0.0,0.0,0.0);

        // Initialize light settings
        m_ogr_light=m_ogr_scene_manager->createLight();
        m_ogr_light->setCastShadows(m_b_light_shadows_enabled);
        updateLightType();
        updateLightPose();
        updateInteractiveMarkerType6DOF(m_s_frame,false,visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE);
        updateMarkerPose();
        updateInteractiveMarkerPose();
    }

    void RVizPluginWAILights::reset()
    {
        Display::reset();
    }

    void RVizPluginWAILights::onEnable()
    {
        m_prp_col_light_ambient->setColor(qcl_light_ambient_old);
        m_prp_col_light_diffuse->setColor(qcl_light_diffuse_old);
        m_prp_col_light_specular->setColor(qcl_light_specular_old);
        updateLightColors();

        updateLightType();
    }

    void RVizPluginWAILights::onDisable()
    {
        disableLights();
    }

    void RVizPluginWAILights::cb_sub_col_light_color_ambient(const std_msgs::ColorRGBAConstPtr &msg)
    {
        std_msgs::ColorRGBA col_ambient=*msg;
        m_prp_col_light_ambient->setColor(QColor(col_ambient.r,col_ambient.g,col_ambient.b,col_ambient.a));
    }
    void RVizPluginWAILights::cb_sub_col_light_color_diff_spec(const std_msgs::ColorRGBAConstPtr &msg)
    {
        std_msgs::ColorRGBA col_diff_spec=*msg;
        m_prp_col_light_diffuse->setColor(QColor(col_diff_spec.r,col_diff_spec.g,col_diff_spec.b,255));
        m_prp_col_light_specular->setColor(QColor(col_diff_spec.r,col_diff_spec.g,col_diff_spec.b,255));
    }
    void RVizPluginWAILights::cb_sub_bol_light_enable(const std_msgs::BoolConstPtr &msg)
    {
        std_msgs::Bool msg_b_enable=*msg;
        if(msg_b_enable.data==true)
        {
            onEnable();
        }
        else
        {
            onDisable();
        }
    }


    //void RVizPluginWAILights::fixedFrameChanged() however, only called on demand if frame changes!
    void RVizPluginWAILights::update(float wall_dt,float ros_dt)
    {
        m_s_frame=m_prp_frame->getFrame().toStdString();

        if(context_->getFrameManager()->getTransform(m_s_frame,ros::Time(),m_ogr_vc3_frame_position,m_ogr_qua_frame_orientation))
        {
            // Update correct frame for markers
            m_msg_mrk_pointlight.header.frame_id=m_s_frame;
            m_msg_mrk_spotlight.header.frame_id=m_s_frame;
            m_msg_mrk_sunlight.header.frame_id=m_s_frame;
            updateLightPose();
            if(m_b_light_markers_enabled==true)
            {
                updateMarkerPose();
                updateInteractiveMarkerPose();
            }
            setStatus(StatusProperty::Ok,"TF","Transform OK!");
        }
        else
        {
            std::string error;
            if(context_->getFrameManager()->transformHasProblems(m_s_frame,ros::Time(),error))
            {
                setStatus(StatusProperty::Error,"TF",QString::fromStdString(error));
            }
            else
            {
                setStatus(StatusProperty::Error,"TF","Could not transform from ["+QString::fromStdString(m_s_frame)+"] to Fixed Frame ["+fixed_frame_+"] for an unknown reason!");
            }
        }
    }

    void RVizPluginWAILights::disableLights()
    {
        qcl_light_ambient_old=m_prp_col_light_ambient->getColor();
        m_prp_col_light_ambient->setColor(QColor(0,0,0,255));
        qcl_light_diffuse_old=m_prp_col_light_diffuse->getColor();
        m_prp_col_light_diffuse->setColor(QColor(0,0,0,255));
        qcl_light_specular_old=m_prp_col_light_specular->getColor();
        m_prp_col_light_specular->setColor(QColor(0,0,0,255));
        updateLightColors();

        m_ogr_light->setVisible(false);
    }

    void RVizPluginWAILights::updateShadowsEnabled()
    {
        // Be careful, since shadows break ray-casting for point-tool selection!!
        // After enabling, shadows would need to be re-DISABLED for each and every material!
        m_b_light_shadows_enabled=m_prp_bol_light_shadows_enabled->getBool();
        if(m_b_light_shadows_enabled==true)
        {
            m_ogr_scene_manager->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
        }
        else
        {
            m_ogr_scene_manager->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
        }
        m_ogr_light->setCastShadows(m_b_light_shadows_enabled);
    }

    void RVizPluginWAILights::updateMarkerEnabled()
    {
        m_b_light_markers_enabled=m_prp_bol_light_markers_enabled->getBool();
        if(m_b_light_markers_enabled==true)
        {
            // Markers
            m_pub_mrk_light=m_hdl_node.advertise<visualization_msgs::Marker>("mrk_light_"+std::to_string(i_static_light_id),1);

            // Interactive Markers
            updateMarkerPose();
            updateInteractiveMarkerPose();
        }
        else
        {
            m_pub_mrk_light.shutdown();
        }
    }

    void RVizPluginWAILights::updateMarkerPose()
    {
        // Update pose of normal RViz markers
        geometry_msgs::Pose pos_mrk_pose;
        pos_mrk_pose.position.x=m_ogr_vc3_light_position.x;
        pos_mrk_pose.position.y=m_ogr_vc3_light_position.y;
        pos_mrk_pose.position.z=m_ogr_vc3_light_position.z;
        pos_mrk_pose.orientation.w=m_ogr_qua_light_orientation.w;
        pos_mrk_pose.orientation.x=m_ogr_qua_light_orientation.x;
        pos_mrk_pose.orientation.y=m_ogr_qua_light_orientation.y;
        pos_mrk_pose.orientation.z=m_ogr_qua_light_orientation.z;
        m_msg_mrk_pointlight.pose=pos_mrk_pose;
        m_msg_mrk_spotlight.pose=pos_mrk_pose;
        m_msg_mrk_sunlight.pose=pos_mrk_pose;
        if(m_prp_enu_light_type->getOptionInt()==0) m_pub_mrk_light.publish(m_msg_mrk_pointlight);
        if(m_prp_enu_light_type->getOptionInt()==2) m_pub_mrk_light.publish(m_msg_mrk_spotlight);
        if(m_prp_enu_light_type->getOptionInt()==4) m_pub_mrk_light.publish(m_msg_mrk_sunlight);
    }
    void RVizPluginWAILights::updateInteractiveMarkerPose()
    {
        // Update pose of interactive RViz markers
        geometry_msgs::Pose pos_iam_pose;
        pos_iam_pose.position.x=m_ogr_vc3_light_position.x;
        pos_iam_pose.position.y=m_ogr_vc3_light_position.y;
        pos_iam_pose.position.z=m_ogr_vc3_light_position.z;
        pos_iam_pose.orientation.w=m_ogr_qua_light_orientation.w;
        pos_iam_pose.orientation.x=m_ogr_qua_light_orientation.x;
        pos_iam_pose.orientation.y=m_ogr_qua_light_orientation.y;
        pos_iam_pose.orientation.z=m_ogr_qua_light_orientation.z;
        m_iam_mrk_light_colors.description=m_prp_enu_light_type->getStdString()+"\n6-DOF Control\n"+s_iam_ctl_mode_text;
        m_iam_server->setPose("iam_6dof",pos_iam_pose);
        m_iam_server->applyChanges();
    }

    void RVizPluginWAILights::updateLightType()
    {
        m_ogr_light->setVisible(false);

        switch(m_prp_enu_light_type->getOptionInt())
        {
            case 3: // AMBIENT LIGHT
                updateLightColors();
            break;

            case Ogre::Light::LT_POINT:
                m_ogr_light->setType(Ogre::Light::LT_POINT);
            break;

            case Ogre::Light::LT_DIRECTIONAL:
                m_ogr_light->setType(Ogre::Light::LT_DIRECTIONAL);
            break;

            case Ogre::Light::LT_SPOTLIGHT:
                m_ogr_light->setType(Ogre::Light::LT_SPOTLIGHT);
            break;

            case 4: // SUNLIGHT (TIME DEPENDENT)
                m_ogr_light->setType(Ogre::Light::LT_SPOTLIGHT);
            break;

        default:

            break;
        }

        // Update light properties
        updateLightColors();
        updateRange();
        updateAttenuation();
        updateInnerAngle();
        updateOuterAngle();
        updateFalloff();
        updateSunlight();

        // Re-enable light
        m_ogr_light->setVisible(true);
    }

    void RVizPluginWAILights::resetLightColors()
    {
        m_prp_col_light_ambient->setColor(QColor(0,0,0,255));
        m_prp_col_light_diffuse->setColor(QColor(0,0,0,255));
        m_prp_col_light_specular->setColor(QColor(0,0,0,255));
        updateAmbientColor();
        updateDiffuseColor();
        updateSpecularColor();
    }

    void RVizPluginWAILights::updateLightColors()
    {
        updateAmbientColor();
        updateDiffuseColor();
        updateSpecularColor();
    }
    void RVizPluginWAILights::updateAmbientColor()
    {
        Ogre::ColourValue ocl_color=m_prp_col_light_ambient->getOgreColor();
        context_->getSceneManager()->setAmbientLight(Ogre::ColourValue(ocl_color.r,ocl_color.g,ocl_color.b,1.0f));
    }
    void RVizPluginWAILights::updateDiffuseColor()
    {
        Ogre::ColourValue ocl_color=m_prp_col_light_diffuse->getOgreColor();
        m_ogr_light->setDiffuseColour(Ogre::ColourValue(ocl_color.r,ocl_color.g,ocl_color.b,1.0f));
    }
    void RVizPluginWAILights::updateSpecularColor()
    {
        Ogre::ColourValue ocl_color=m_prp_col_light_specular->getOgreColor();
        m_ogr_light->setSpecularColour(Ogre::ColourValue(ocl_color.r,ocl_color.g,ocl_color.b,1.0f));
    }
    void RVizPluginWAILights::updateColorDiffSpecHSV()
    {
        // HSB is the same as HSV, however HSL is a different color scheme!
        QColor qcl_light_diff_spec_hsv;
        qcl_light_diff_spec_hsv.setHsvF(m_prp_vec_hsv_light_diff_spec->getVector().x/255.0,
                                       m_prp_vec_hsv_light_diff_spec->getVector().y/255.0,
                                       m_prp_vec_hsv_light_diff_spec->getVector().z/255.0,
                                       1.0);

        Ogre::ColourValue ocl_light_diff_spec_rgb(qcl_light_diff_spec_hsv.red()/255.0,
                                                  qcl_light_diff_spec_hsv.green()/255.0,
                                                  qcl_light_diff_spec_hsv.blue()/255.0,
                                                  1.0);

        m_ogr_light->setDiffuseColour(ocl_light_diff_spec_rgb);
        m_ogr_light->setSpecularColour(ocl_light_diff_spec_rgb);

        m_prp_col_light_diffuse->setColor(qcl_light_diff_spec_hsv);
        m_prp_col_light_specular->setColor(qcl_light_diff_spec_hsv);
        updateDiffuseColor();
        updateSpecularColor();
    }
    void RVizPluginWAILights::updateColorDiffSpecYCbCr()
    {
        // According to JPEG standard (Y′, CB and CR have the full 8-bit range of [0...255])
        // 0 -- 255... is equivalent to -1.0 -- +1.0 in the CbCr plane
        tf::Vector3 tf_vc3_light_diff_spec_ycbcr(m_prp_vec_ycbcr_light_diff_spec->getVector().x, // Y'
                                               m_prp_vec_ycbcr_light_diff_spec->getVector().y, // Cb
                                               m_prp_vec_ycbcr_light_diff_spec->getVector().z);// Cr
        tf::Vector3 tf_vc3_light_diff_spec_rgb;
        tf_vc3_light_diff_spec_rgb.setX(tf_vc3_light_diff_spec_ycbcr.getX()+1.402*(tf_vc3_light_diff_spec_ycbcr.getZ()-128.0));
        tf_vc3_light_diff_spec_rgb.setY(tf_vc3_light_diff_spec_ycbcr.getX()-0.344136*(tf_vc3_light_diff_spec_ycbcr.getY()-128.0)-0.714136*(tf_vc3_light_diff_spec_ycbcr.getZ()-128.0));
        tf_vc3_light_diff_spec_rgb.setZ(tf_vc3_light_diff_spec_ycbcr.getX()+1.772*(tf_vc3_light_diff_spec_ycbcr.getY()-128.0));

        // ROS_WARN_STREAM("Color RGB: " << tf_vc3_light_diff_spec_rgb.getX() << "," << tf_vc3_light_diff_spec_rgb.getY()  << "," <<  tf_vc3_light_diff_spec_rgb.getZ());

        Ogre::ColourValue ocl_light_diff_spec_rgb(tf_vc3_light_diff_spec_rgb.getX()/255.0,
                                                  tf_vc3_light_diff_spec_rgb.getY()/255.0,
                                                  tf_vc3_light_diff_spec_rgb.getZ()/255.0,
                                                  1.0);

        m_ogr_light->setDiffuseColour(ocl_light_diff_spec_rgb);
        m_ogr_light->setSpecularColour(ocl_light_diff_spec_rgb);

        QColor qcl_light_diff_spec_rgb(tf_vc3_light_diff_spec_rgb.getX(),
                                       tf_vc3_light_diff_spec_rgb.getY(),
                                       tf_vc3_light_diff_spec_rgb.getZ(),
                                       255);
        m_prp_col_light_diffuse->setColor(qcl_light_diff_spec_rgb);
        m_prp_col_light_specular->setColor(qcl_light_diff_spec_rgb);
        updateDiffuseColor();
        updateSpecularColor();
    }

    void RVizPluginWAILights::updateLightPose()
    {
        if(m_prp_enu_light_type->getOptionInt()==4)
        {
            // Update pose of sunlight:
            // Relate current minute count to 1440minutes of one day,
            // scaled to one full circle with 2*M_PI radians. At sunrise hour, angle is 0rad:
            std::time_t time_now; time(&time_now);
            struct tm tm_now=*std::localtime(&time_now);
            m_ogr_vc3_light_position.x=m_ogr_vc3_frame_position.x+0.0;
            m_ogr_vc3_light_position.y=m_ogr_vc3_frame_position.y-m_f_sunlight_radial_distance*cos( (float(tm_now.tm_hour-m_f_sunlight_sunrise_hour)*60.0+float(tm_now.tm_min))/1440.0 *2.0*M_PI);
            m_ogr_vc3_light_position.z=m_ogr_vc3_frame_position.z+m_f_sunlight_radial_distance*sin( (float(tm_now.tm_hour-m_f_sunlight_sunrise_hour)*60.0+float(tm_now.tm_min))/1440.0 *2.0*M_PI);
            m_ogr_light->setPosition(m_ogr_vc3_light_position);
            m_ogr_light->setDirection(m_ogr_qua_frame_orientation * (-m_ogr_vc3_light_position));
        }
        else
        {
            m_ogr_vc3_light_position=m_ogr_vc3_frame_position+m_prp_vec_light_position_offset->getVector();
            m_ogr_qua_light_orientation=m_ogr_qua_frame_orientation * m_prp_qua_light_orientation_offset->getQuaternion();
            m_ogr_light->setPosition(m_ogr_vc3_light_position);
            m_ogr_light->setDirection(getVector3FromQuaternion(m_ogr_qua_light_orientation));
        }
    }

    Ogre::Vector3 RVizPluginWAILights::getVector3FromQuaternion(Ogre::Quaternion qua_input)
    {
        tf::Vector3 vc3_forward(1.0,0.0,0.0);
        tf::Vector3 vc3_output=tf::quatRotate(tf::Quaternion(qua_input.x,qua_input.y,qua_input.z,qua_input.w),vc3_forward);
        return Ogre::Vector3(vc3_output.getX(),vc3_output.getY(),vc3_output.getZ());
    }

    void RVizPluginWAILights::updateRange()
    {
        m_ogr_light->setAttenuation(m_prp_f_light_range->getFloat(),
                                    m_prp_vec_light_attenuation->getVector().x,
                                    m_prp_vec_light_attenuation->getVector().y,
                                    m_prp_vec_light_attenuation->getVector().z);
    }
    void RVizPluginWAILights::updateAttenuation()
    {
        m_ogr_light->setAttenuation(m_prp_f_light_range->getFloat(),
                                    m_prp_vec_light_attenuation->getVector().x,
                                    m_prp_vec_light_attenuation->getVector().y,
                                    m_prp_vec_light_attenuation->getVector().z);
    }
    void RVizPluginWAILights::updateInnerAngle()
    {
        // Not supported via OpenGL!
        m_ogr_light->setSpotlightInnerAngle(Ogre::Radian(m_prp_f_light_inner_angle->getFloat()));
    }
    void RVizPluginWAILights::updateOuterAngle()
    {
        m_ogr_light->setSpotlightOuterAngle(Ogre::Radian(m_prp_f_light_outer_angle->getFloat()));
    }
    void RVizPluginWAILights::updateFalloff()
    {
        m_ogr_light->setSpotlightFalloff(m_prp_f_light_falloff_rate->getFloat());
    }

    void RVizPluginWAILights::updateSunlight()
    {
        m_f_sunlight_radial_distance=m_prp_f_light_sun_radial_distance->getFloat();
        m_f_sunlight_sunrise_hour=m_prp_i_light_sun_rise_hour->getInt();
    }


    // Interactive markers
    void RVizPluginWAILights::updateInteractiveMarkerType6DOF(std::string s_frame,bool b_iam_fixed,unsigned int ui_interaction_mode)
    {
        m_iam_mrk_light_colors.header.frame_id=s_frame;
        m_iam_mrk_light_colors.pose.position.x=0.0;
        m_iam_mrk_light_colors.pose.position.y=0.0;
        m_iam_mrk_light_colors.pose.position.z=10.0;
        m_iam_mrk_light_colors.pose.orientation.w=1.0;
        m_iam_mrk_light_colors.pose.orientation.x=0.0;
        m_iam_mrk_light_colors.pose.orientation.y=0.0;
        m_iam_mrk_light_colors.pose.orientation.z=0.0;
        m_iam_mrk_light_colors.scale=1;
        m_iam_mrk_light_colors.name="LIGHT_"+std::to_string(i_static_light_id);
        m_iam_mrk_light_colors.controls.clear();
        addInteractiveMarkerTriggerControl(m_iam_mrk_light_colors); // Insert sphere marker as control trigger

        // Add controls
        if(ui_interaction_mode!=visualization_msgs::InteractiveMarkerControl::NONE)
        {
            if(ui_interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_3D)
            {

            }
            else if(ui_interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
            {

            }
            else if(ui_interaction_mode==visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE)
            {
                s_iam_ctl_mode_text="MOVE_ROTATE_AXIS";

                // Not necessary since direction looses 1DOF, in this case tf_vc3_vector_forward=(1.0,0.0,0.0)
                visualization_msgs::InteractiveMarkerControl iam_ctl_rotate_3d_x;
                if(b_iam_fixed) iam_ctl_rotate_3d_x.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_rotate_3d_x.orientation.w=1.0/sqrt(2);
                iam_ctl_rotate_3d_x.orientation.x=1.0/sqrt(2);
                iam_ctl_rotate_3d_x.orientation.y=0;
                iam_ctl_rotate_3d_x.orientation.z=0;
                iam_ctl_rotate_3d_x.name="rotate_x";
                iam_ctl_rotate_3d_x.interaction_mode=visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_rotate_3d_x);
                visualization_msgs::InteractiveMarkerControl iam_ctl_move_3d_x;
                if(b_iam_fixed) iam_ctl_move_3d_x.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_move_3d_x.orientation.w=1.0/sqrt(2);
                iam_ctl_move_3d_x.orientation.x=1.0/sqrt(2);
                iam_ctl_move_3d_x.orientation.y=0;
                iam_ctl_move_3d_x.orientation.z=0;
                iam_ctl_move_3d_x.name="move_x";
                iam_ctl_move_3d_x.interaction_mode=visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_move_3d_x);

                visualization_msgs::InteractiveMarkerControl iam_ctl_rotate_3d_y;
                if(b_iam_fixed) iam_ctl_rotate_3d_y.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_rotate_3d_y.orientation.w=1.0/sqrt(2);
                iam_ctl_rotate_3d_y.orientation.x=0;
                iam_ctl_rotate_3d_y.orientation.y=0;
                iam_ctl_rotate_3d_y.orientation.z=1.0/sqrt(2);
                iam_ctl_rotate_3d_y.name="rotate_y";
                iam_ctl_rotate_3d_y.interaction_mode=visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_rotate_3d_y);
                visualization_msgs::InteractiveMarkerControl iam_ctl_move_3d_y;
                if(b_iam_fixed) iam_ctl_move_3d_y.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_move_3d_y.orientation.w=1.0/sqrt(2);
                iam_ctl_move_3d_y.orientation.x=0;
                iam_ctl_move_3d_y.orientation.y=0;
                iam_ctl_move_3d_y.orientation.z=1.0/sqrt(2);
                iam_ctl_move_3d_y.name="move_y";
                iam_ctl_move_3d_y.interaction_mode=visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_move_3d_y);

                visualization_msgs::InteractiveMarkerControl iam_ctl_rotate_3d_z;
                if(b_iam_fixed) iam_ctl_rotate_3d_z.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_rotate_3d_z.orientation.w=1.0/sqrt(2);
                iam_ctl_rotate_3d_z.orientation.x=0;
                iam_ctl_rotate_3d_z.orientation.y=1.0/sqrt(2);
                iam_ctl_rotate_3d_z.orientation.z=0;
                iam_ctl_rotate_3d_z.name="rotate_z";
                iam_ctl_rotate_3d_z.interaction_mode=visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_rotate_3d_z);
                visualization_msgs::InteractiveMarkerControl iam_ctl_move_3d_z;
                if(b_iam_fixed) iam_ctl_move_3d_z.orientation_mode=visualization_msgs::InteractiveMarkerControl::FIXED;
                iam_ctl_move_3d_z.orientation.w=1.0/sqrt(2);
                iam_ctl_move_3d_z.orientation.x=0;
                iam_ctl_move_3d_z.orientation.y=1.0/sqrt(2);
                iam_ctl_move_3d_z.orientation.z=0;
                iam_ctl_move_3d_z.name="move_z";
                iam_ctl_move_3d_z.interaction_mode=visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
                m_iam_mrk_light_colors.controls.push_back(iam_ctl_move_3d_z);
            }
            else
            {
                // Do nothing...
            }
            m_iam_mrk_light_colors.description=m_iam_mrk_light_colors.name+"\n6-DOF Control\n"+s_iam_ctl_mode_text;
        }

        m_iam_server->insert(m_iam_mrk_light_colors);
        m_iam_server->setCallback(m_iam_mrk_light_colors.name,boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
        if(ui_interaction_mode!=visualization_msgs::InteractiveMarkerControl::NONE)
        {
            m_iam_menu_handler=new interactive_markers::MenuHandler();

            m_iam_menu_handler->insert("WHITE Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            m_iam_menu_handler->insert("RED Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            m_iam_menu_handler->insert("GREEN Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            m_iam_menu_handler->insert("BLUE Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            m_iam_menu_handler->insert("WARM Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            m_iam_menu_handler->insert("COLD Light",boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1));
            /*
            interactive_markers::MenuHandler::EntryHandle sub_menu_handle = m_iam_menu_handler.insert( "Submenu" );
            m_iam_menu_handler.insert( sub_menu_handle, "First Entry", boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1) );
            m_iam_menu_handler.insert( sub_menu_handle, "Second Entry", boost::bind(&RVizPluginWAILights::cb_iam_process_feedback,this,_1) );
            */
            m_iam_menu_handler->apply(*m_iam_server,m_iam_mrk_light_colors.name);
        }

        m_iam_server->applyChanges();
    }
    visualization_msgs::InteractiveMarkerControl& RVizPluginWAILights::addInteractiveMarkerTriggerControl(visualization_msgs::InteractiveMarker &msg)
    {
        visualization_msgs::InteractiveMarkerControl iam_ctl_trigger;
        iam_ctl_trigger.always_visible=true;
        iam_ctl_trigger.interaction_mode=visualization_msgs::InteractiveMarkerControl::MENU;
        iam_ctl_trigger.markers.push_back(addInteractiveMarkerTrigger(msg));
        msg.controls.push_back(iam_ctl_trigger);
        return msg.controls.back();
    }
    visualization_msgs::Marker RVizPluginWAILights::addInteractiveMarkerTrigger(visualization_msgs::InteractiveMarker &msg)
    {
        visualization_msgs::Marker mrk_iam_trigger;
        mrk_iam_trigger.type=visualization_msgs::Marker::CUBE;
        mrk_iam_trigger.scale.x=msg.scale*0.5;
        mrk_iam_trigger.scale.y=msg.scale*0.5;
        mrk_iam_trigger.scale.z=msg.scale*0.5;
        mrk_iam_trigger.color.r=0.101960784;
        mrk_iam_trigger.color.g=0.42745098;
        mrk_iam_trigger.color.b=0.588235294;
        mrk_iam_trigger.color.a=0.3;
        return mrk_iam_trigger;
    }
    void RVizPluginWAILights::cb_iam_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &iam_feedback_msg)
    {

        std::ostringstream s;
        s << "Feedback from marker '" << iam_feedback_msg->marker_name << "' " << " / control '" << iam_feedback_msg->control_name << "'";

        /*
        std::ostringstream mouse_point_ss;
        if(iam_feedback_msg->mouse_point_valid)
        {
            mouse_point_ss << " at " << iam_feedback_msg->mouse_point.x
                           << ", " << iam_feedback_msg->mouse_point.y
                           << ", " << iam_feedback_msg->mouse_point.z
                           << " in frame " << iam_feedback_msg->header.frame_id;
        }
        */

        switch(iam_feedback_msg->event_type)
        {
            case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
                // ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
            break;

            case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
                if(iam_feedback_msg->menu_entry_id==1)
                {
                    m_prp_col_light_diffuse->setColor(QColor(136,138,133));
                    m_prp_col_light_specular->setColor(QColor(136,138,133));
                }
                else if(iam_feedback_msg->menu_entry_id==2)
                {
                    m_prp_col_light_diffuse->setColor(QColor(128,0,0,255));
                    m_prp_col_light_specular->setColor(QColor(128,0,0,255));
                }
                else if(iam_feedback_msg->menu_entry_id==3)
                {
                    m_prp_col_light_diffuse->setColor(QColor(0,128,0,255));
                    m_prp_col_light_specular->setColor(QColor(0,128,0,255));
                }
                else if(iam_feedback_msg->menu_entry_id==4)
                {
                    m_prp_col_light_diffuse->setColor(QColor(0,0,128,255));
                    m_prp_col_light_specular->setColor(QColor(0,0,128,255));
                }
                else if(iam_feedback_msg->menu_entry_id==5)
                {
                    m_prp_col_light_diffuse->setColor(QColor(233,185,110,255));
                    m_prp_col_light_specular->setColor(QColor(233,185,110,255));
                }
                else if(iam_feedback_msg->menu_entry_id==6)
                {
                    m_prp_col_light_diffuse->setColor(QColor(114,159,207));
                    m_prp_col_light_specular->setColor(QColor(114,159,207));
                }
                else
                {
                    // Do nothing...
                }
            break;

            case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            {

                m_prp_vec_light_position_offset->setVector(Ogre::Vector3(iam_feedback_msg->pose.position.x,
                                                                         iam_feedback_msg->pose.position.y,
                                                                         iam_feedback_msg->pose.position.z));
                m_prp_qua_light_orientation_offset->setQuaternion(Ogre::Quaternion(iam_feedback_msg->pose.orientation.w,
                                                                                   iam_feedback_msg->pose.orientation.x,
                                                                                   iam_feedback_msg->pose.orientation.y,
                                                                                   iam_feedback_msg->pose.orientation.z));
                /*
                ROS_INFO_STREAM( s.str() << ": pose changed"
                << "\nposition="
                << iam_feedback_msg->pose.position.x
                << ", " << iam_feedback_msg->pose.position.y
                << ", " << iam_feedback_msg->pose.position.z
                << "\norientation="
                << iam_feedback_msg->pose.orientation.w
                << ", " << iam_feedback_msg->pose.orientation.x
                << ", " << iam_feedback_msg->pose.orientation.y
                << ", " << iam_feedback_msg->pose.orientation.z
                << "\nframe: " << iam_feedback_msg->header.frame_id
                << " time: " << iam_feedback_msg->header.stamp.sec << "sec, "
                << iam_feedback_msg->header.stamp.nsec << " nsec" );
                */
            }break;

            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
                // ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
            break;

            case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
                // ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
            break;

            default:
                // Do nothing...
            break;
        }
    }
} // end namespace rviz_plugin_wai_lights

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_lights::RVizPluginWAILights,rviz::Display)
