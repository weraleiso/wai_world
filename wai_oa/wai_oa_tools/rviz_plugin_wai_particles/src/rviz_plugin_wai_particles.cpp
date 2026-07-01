#include "rviz_plugin_wai_particles.h"



namespace rviz_plugin_wai_particles
{
    using namespace rviz;

    RVizPluginWAIParticles::RVizPluginWAIParticles() : Display()
    {
        m_prp_tf_particles_frame=new TfFrameProperty("Frame",TfFrameProperty::FIXED_FRAME_STRING,"",this,NULL,true);
        m_prp_vec_particles_position=new VectorProperty("Position",Ogre::Vector3(0.0,0.0,0.0),"",this, SLOT(UpdateParticlesPosition()));
        m_prp_vec_particles_direction=new VectorProperty("Direction",Ogre::Vector3(0.0,0.0,1.0),"",this,SLOT(UpdateParticlesDirection()));
        m_prp_enu_particles_preset=new EnumProperty("Preset","Fire","",this,SLOT(UpdateParticlesPreset()));
        m_prp_enu_particles_preset->addOption("Fire",0);
        m_prp_enu_particles_preset->addOption("Gas Leak",1);
        m_prp_enu_particles_preset->addOption("Water Leak",2);
        m_prp_enu_particles_type=new EnumProperty("Type","Point","",this,SLOT(UpdateParticlesType()));
        m_prp_enu_particles_type->addOption("Point",0);
        m_prp_enu_particles_type->addOption("Area",1);
        m_prp_enu_particles_type->addOption("Box",2);
        m_prp_enu_particles_type->addOption("Cylinder",3);
        m_prp_enu_particles_type->addOption("Ellipsoid",4);
        m_prp_enu_particles_type->addOption("HollowEllipsoid",5);
        m_prp_enu_particles_type->addOption("Ring",6);
        m_prp_i_particles_quota=new IntProperty("Quota",1000,"",this,SLOT(UpdateParticlesQuota()));
        m_prp_i_particles_emission_rate=new IntProperty("Emission Rate",200,"",this,SLOT(UpdateParticlesEmissionRate()));
        m_prp_f_particles_angle=new FloatProperty("Angle",0.5f,"",this,SLOT(UpdateParticlesAngle()));
        m_prp_f_particles_size=new FloatProperty("Size",0.01f,"",this,SLOT(UpdateParticlesSize()));
        //m_prp_col_particles_color=new ColorProperty("Color",QColor(255,255,255,255),"",this,SLOT(UpdateParticlesColor()));
        m_prp_col_particles_color_range_start=new ColorProperty("Color Range Start",QColor(255,255,255,255),"",this,SLOT(UpdateParticlesColor()));
        m_prp_col_particles_color_range_end=new ColorProperty("Color Range End",QColor(255,255,255,255),"",this,SLOT(UpdateParticlesColor()));
        m_prp_f_particles_time_to_live=new FloatProperty("Time To Live",5.0f,"",this,SLOT(UpdateParticlesTimeToLive()));
        m_prp_f_particles_velocity_max=new FloatProperty("Velocity Max.",0.25f,"",this,SLOT(UpdateParticlesVelocityMax()));
        m_prp_vec_particles_colour_fader=new VectorProperty("Colour Fader",Ogre::Vector3(0.0,0.0,0.0),"",this,SLOT(UpdateParticlesColourFader()));
        m_prp_f_particles_colour_fader_alpha=new FloatProperty("Alpha Fader",0.0f,"",this,SLOT(UpdateParticlesColourFader()));
        m_prp_vec_particles_linear_force=new VectorProperty("Linear Force",Ogre::Vector3(0.0,0.5,0.0),"",this,SLOT(UpdateParticlesLinearForce()));
        m_prp_f_particles_scaler=new FloatProperty("Scaler",0.0f,"",this,SLOT(UpdateParticlesScaler()));
        m_prp_i_particles_randomness=new IntProperty("Randomness",3,"",this,SLOT(UpdateParticlesRandomness()));
        m_prp_f_particles_scope=new FloatProperty("Scope",0.5f,"",this,SLOT(UpdateParticlesRandomness()));
        m_prp_b_particles_keep_velocity=new BoolProperty("Keep Velocity",false,"",this,SLOT(UpdateParticlesRandomness()));
    }

    void RVizPluginWAIParticles::onInitialize()
    {
        Display::onInitialize();
        m_ogr_scene_manager=context_->getSceneManager();
        m_ogr_scene_node=context_->getSceneManager()->getRootSceneNode();
        m_prp_tf_particles_frame->setFrameManager(context_->getFrameManager());
        m_ogr_vc3_frame_position=Ogre::Vector3(0.0,0.0,0.0);
        m_ogr_qua_frame_orientation=Ogre::Quaternion(1.0,0.0,0.0,0.0);

        // Initialize particlesystem
        /* Particle System Test with Script:
        Ogre::ParticleSystem* partSystem=context_->getSceneManager()->createParticleSystem("Smoke","Smoke");
        Ogre::SceneNode* m_ogr_node_scene=context_->getSceneManager()->getRootSceneNode()->createChildSceneNode(Ogre::Vector3(1,1,1));
        m_ogr_node_scene->attachObject((Ogre::MovableObject*)partSystem);
        */
        /*
        Ogre::ResourceGroupManager::getSingletonPtr()->destroyResourceGroup("UserDefinedMaterials");
        Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup("UserDefinedMaterials");
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation("/home/ias/catkin_ws/src/wai_world/wai_world_tools/rviz/ogre_media/materials", "FileSystem", "UserDefinedMaterials", true);
        Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup("UserDefinedMaterials");
        Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup("UserDefinedMaterials");
        */
        /*
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setCullingMode(Ogre::CULL_NONE);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
        material->getTechnique(0)->setLightingEnabled(false);
        Ogre::TextureUnitState* tu = material->getTechnique(0)->getPass(0)->createTextureUnitState();
        tu->setTextureName(texture_->getTexture()->getName());
        tu->setTextureFiltering(Ogre::TFO_NONE);
        tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
        */
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("flatvertexcolor",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setCullingMode(Ogre::CULL_NONE);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
        material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        material->getTechnique(0)->setLightingEnabled(false); // Necessary to allow for vertexcolor!

        m_ogr_particlesystem=m_ogr_scene_manager->createParticleSystem();
        m_ogr_scene_node->attachObject(m_ogr_particlesystem);
        m_ogr_particlesystem->setDefaultDimensions(0.01,0.01);
        m_ogr_particlesystem->setSortingEnabled(true);
        m_ogr_particlesystem->setCullIndividually(true);
        m_ogr_particlesystem->setParticleQuota(1000);
        m_ogr_particlesystem->setRenderer("billboard");
        m_ogr_particlesystem->setKeepParticlesInLocalSpace(false);
        m_ogr_particlesystem->setDebugDisplayEnabled(false);
        m_ogr_particlesystem->setVisible(true);
        m_ogr_particlesystem->setMaterialName("flatvertexcolor");
        m_ogr_particlesystem->clear();

        m_ogr_particlesystem->addAffector("ColourFader");
        m_ogr_particlesystem->getAffector(0)->setParameter("red","0.0");
        m_ogr_particlesystem->getAffector(0)->setParameter("green","0.0");
        m_ogr_particlesystem->getAffector(0)->setParameter("blue","0.0");
        m_ogr_particlesystem->getAffector(0)->setParameter("alpha","0.0");
        m_ogr_particlesystem->addAffector("LinearForce");
        m_ogr_particlesystem->getAffector(1)->setParameter("force_vector","0 0.5 0");
        m_ogr_particlesystem->addAffector("Scaler");
        m_ogr_particlesystem->getAffector(2)->setParameter("rate","0.0");  
        m_ogr_particlesystem->addAffector("DirectionRandomiser");
        m_ogr_particlesystem->getAffector(3)->setParameter("randomness","3");
        m_ogr_particlesystem->getAffector(3)->setParameter("scope","0.5");
        m_ogr_particlesystem->getAffector(3)->setParameter("keep_velocity","false");

        m_ogr_particlesystem->addEmitter("Point");
        m_ogr_particlesystem->getEmitter(0)->setPosition(Ogre::Vector3(0.0,0.0,0.0));
        m_ogr_particlesystem->getEmitter(0)->setDirection(Ogre::Vector3(0.0,0.0,1.0));
        m_ogr_particlesystem->getEmitter(0)->colour=Ogre::ColourValue(1.0,0.0,0.0,1.0);
        m_ogr_particlesystem->getEmitter(0)->setColour(Ogre::ColourValue(1.0,0.0,0.0,1.0));
        m_ogr_particlesystem->getEmitter(0)->setColourRangeStart(Ogre::ColourValue(1.0,0.0,0.0,1.0));
        m_ogr_particlesystem->getEmitter(0)->setColourRangeEnd(Ogre::ColourValue(1.0,0.0,0.0,1.0));
        m_ogr_particlesystem->getEmitter(0)->setAngle(Ogre::Radian(0.5));
        m_ogr_particlesystem->getEmitter(0)->setEmissionRate(200);
        m_ogr_particlesystem->getEmitter(0)->setMaxParticleVelocity(0.25);
        m_ogr_particlesystem->getEmitter(0)->setTimeToLive(5.0);
        m_ogr_particlesystem->getEmitter(0)->setParameter("colour","1 1 1 1");
        m_ogr_particlesystem->getEmitter(0)->setParameter("colour_range_start","1 1 1 1");
        m_ogr_particlesystem->getEmitter(0)->setParameter("colour_range_end","1 1 1 1");
        m_ogr_particlesystem->getEmitter(0)->setEnabled(true);

        m_ogr_particlesystem->addEmitter("Ellipsoid");
        m_ogr_particlesystem->getEmitter(1)->setParameter("width","0.5");
        m_ogr_particlesystem->getEmitter(1)->setParameter("height","0.5");
        m_ogr_particlesystem->getEmitter(1)->setParameter("depth","0.5");
        m_ogr_particlesystem->getEmitter(1)->setParameter("angle","30");
        m_ogr_particlesystem->getEmitter(1)->setParameter("emission_rate","200");
        m_ogr_particlesystem->getEmitter(1)->setParameter("time_to_live_min","2");
        m_ogr_particlesystem->getEmitter(1)->setParameter("time_to_live_max","5");
        m_ogr_particlesystem->getEmitter(1)->setParameter("direction","0 0 1");
        m_ogr_particlesystem->getEmitter(1)->setParameter("velocity","0.1");
        m_ogr_particlesystem->getEmitter(1)->setParameter("colour","0.15 0.1 0.0");
        m_ogr_particlesystem->getEmitter(1)->setEnabled(false);

        // TODO: Add mor emitters with all particle types here!

        UpdateParticlesPreset();
    }

    RVizPluginWAIParticles::~RVizPluginWAIParticles()
    {
        m_ogr_particlesystem->getEmitter(0)->setEnabled(false);
        m_ogr_particlesystem->cleanupDictionary();
        m_ogr_particlesystem->clear();
        m_ogr_scene_manager->destroyParticleSystem(m_ogr_particlesystem);
    }

    void RVizPluginWAIParticles::reset()
    {
        Display::reset();
    }

    void RVizPluginWAIParticles::onEnable()
    {
        m_ogr_particlesystem->getEmitter(0)->setEnabled(true);
    }

    void RVizPluginWAIParticles::onDisable()
    {
        m_ogr_particlesystem->getEmitter(0)->setEnabled(false);
    }

    void RVizPluginWAIParticles::fixedFrameChanged()
    {
        QString qframe=m_prp_tf_particles_frame->getFrame();
        std::string frame=qframe.toStdString();
        context_->getFrameManager()->getTransform(frame,ros::Time(),m_ogr_vc3_frame_position,m_ogr_qua_frame_orientation);
        UpdateParticlesPosition();
        UpdateParticlesDirection();
    }

    void RVizPluginWAIParticles::update(float wall_dt,float ros_dt)
    {
        fixedFrameChanged();
        m_ogr_particlesystem->_update(ros_dt);
    }

    void RVizPluginWAIParticles::UpdateParticlesPosition()
    {
        m_ogr_particlesystem->getEmitter(0)->setPosition(m_ogr_vc3_frame_position+m_prp_vec_particles_position->getVector());
    }
    void RVizPluginWAIParticles::UpdateParticlesDirection()
    {
        m_ogr_particlesystem->getEmitter(0)->setDirection(m_ogr_qua_frame_orientation * m_prp_vec_particles_direction->getVector());
    }
    void RVizPluginWAIParticles::UpdateParticlesPreset()
    {
        switch(m_prp_enu_particles_preset->getOptionInt())
        {
            case 0: // FIRE
                m_prp_i_particles_quota->setInt(2000);
                m_prp_i_particles_emission_rate->setInt(500);
                m_prp_f_particles_angle->setFloat(0.25);
                m_prp_f_particles_size->setFloat(0.04);
                m_prp_col_particles_color_range_start->setColor(QColor(252,233,79));
                m_prp_col_particles_color_range_end->setColor(QColor(245,121,0));
                m_prp_f_particles_time_to_live->setFloat(4.0);
                m_prp_f_particles_velocity_max->setFloat(0.25);
                m_prp_vec_particles_colour_fader->setVector(Ogre::Vector3(-0.5,-0.5,-0.5));
                m_prp_f_particles_colour_fader_alpha->setFloat(-0.2);
                m_prp_vec_particles_linear_force->setVector(Ogre::Vector3(0.0,0.5,0.0));
                m_prp_f_particles_scaler->setFloat(-0.01);
                m_prp_i_particles_randomness->setInt(2);
                m_prp_f_particles_scope->setFloat(0.5);
                m_prp_b_particles_keep_velocity->setBool(false);

                UpdateParticlesPropertiesAll();
            break;

            case 1: // GAS LEAK
                m_prp_i_particles_quota->setInt(1000);
                m_prp_i_particles_emission_rate->setInt(1000);
                m_prp_f_particles_angle->setFloat(0.25);
                m_prp_f_particles_size->setFloat(0.05);
                m_prp_col_particles_color_range_start->setColor(QColor(255,255,255));
                m_prp_col_particles_color_range_end->setColor(QColor(255,255,255));
                m_prp_f_particles_time_to_live->setFloat(4.0);
                m_prp_f_particles_velocity_max->setFloat(0.25);
                m_prp_vec_particles_colour_fader->setVector(Ogre::Vector3(-0.1,-0.1,-0.1));
                m_prp_f_particles_colour_fader_alpha->setFloat(-0.25);
                m_prp_vec_particles_linear_force->setVector(Ogre::Vector3(0.0,0.25,0.0));
                m_prp_f_particles_scaler->setFloat(-0.005);
                m_prp_i_particles_randomness->setInt(3);
                m_prp_f_particles_scope->setFloat(0.75);
                m_prp_b_particles_keep_velocity->setBool(false);

                UpdateParticlesPropertiesAll();
            break;

            case 2: // WATER LEAK
                m_prp_i_particles_quota->setInt(2000);
                m_prp_i_particles_emission_rate->setInt(1000);
                m_prp_f_particles_angle->setFloat(0.125);
                m_prp_f_particles_size->setFloat(0.025);
                m_prp_col_particles_color_range_start->setColor(QColor(32,74,135));
                m_prp_col_particles_color_range_end->setColor(QColor(32,74,135));
                m_prp_f_particles_time_to_live->setFloat(2.0);
                m_prp_f_particles_velocity_max->setFloat(0.25);
                m_prp_vec_particles_colour_fader->setVector(Ogre::Vector3(0.2,0.2,0.2));
                m_prp_f_particles_colour_fader_alpha->setFloat(-0.25);
                m_prp_vec_particles_linear_force->setVector(Ogre::Vector3(0.0,0.0,0.0));
                m_prp_f_particles_scaler->setFloat(-0.005);
                m_prp_i_particles_randomness->setInt(1);
                m_prp_f_particles_scope->setFloat(0.25);
                m_prp_b_particles_keep_velocity->setBool(true);

                UpdateParticlesPropertiesAll();
            break;

            default:
            break;
        }
    }
    void RVizPluginWAIParticles::UpdateParticlesType()
    {
        switch(m_prp_enu_particles_type->getOptionInt())
        {
            case 0: // POINT
                m_ogr_particlesystem->getEmitter(0)->setEnabled(true);
                m_ogr_particlesystem->getEmitter(1)->setEnabled(false);
            break;

            case 4: // ELLIPSOID
                m_ogr_particlesystem->getEmitter(0)->setEnabled(false);
                m_ogr_particlesystem->getEmitter(1)->setEnabled(true);
            break;
        }
    }
    void RVizPluginWAIParticles::UpdateParticlesQuota()
    {
        m_ogr_particlesystem->setParticleQuota(m_prp_i_particles_quota->getInt());
    }
    void RVizPluginWAIParticles::UpdateParticlesEmissionRate()
    {
        m_ogr_particlesystem->getEmitter(0)->setEmissionRate(m_prp_i_particles_emission_rate->getInt());
    }
    void RVizPluginWAIParticles::UpdateParticlesAngle()
    {
        m_ogr_particlesystem->getEmitter(0)->setAngle(Ogre::Radian(m_prp_f_particles_angle->getFloat()));
    }
    void RVizPluginWAIParticles::UpdateParticlesSize()
    {
        m_ogr_particlesystem->setDefaultDimensions(m_prp_f_particles_size->getFloat(),m_prp_f_particles_size->getFloat());
    }
    void RVizPluginWAIParticles::UpdateParticlesColor()
    {
        //m_ogr_particlesystem->getEmitter(0)->setColour(m_prp_col_particles_color->getOgreColor());
        m_ogr_particlesystem->getEmitter(0)->setColourRangeStart(m_prp_col_particles_color_range_start->getOgreColor());
        m_ogr_particlesystem->getEmitter(0)->setColourRangeEnd(m_prp_col_particles_color_range_end->getOgreColor());
    }
    void RVizPluginWAIParticles::UpdateParticlesTimeToLive()
    {
        m_ogr_particlesystem->getEmitter(0)->setTimeToLive(m_prp_f_particles_time_to_live->getFloat());
    }
    void RVizPluginWAIParticles::UpdateParticlesVelocityMax()
    {
        m_ogr_particlesystem->getEmitter(0)->setMaxParticleVelocity(m_prp_f_particles_velocity_max->getFloat());
    }
    void RVizPluginWAIParticles::UpdateParticlesColourFader()
    {
        m_ogr_particlesystem->getAffector(0)->setParameter("red",std::to_string(m_prp_vec_particles_colour_fader->getVector().x));
        m_ogr_particlesystem->getAffector(0)->setParameter("green",std::to_string(m_prp_vec_particles_colour_fader->getVector().y));
        m_ogr_particlesystem->getAffector(0)->setParameter("blue",std::to_string(m_prp_vec_particles_colour_fader->getVector().z));
        m_ogr_particlesystem->getAffector(0)->setParameter("alpha",std::to_string(m_prp_f_particles_colour_fader_alpha->getFloat()));
    }
    void RVizPluginWAIParticles::UpdateParticlesLinearForce()
    {
        m_ogr_particlesystem->getAffector(1)->setParameter("force_vector",
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().x)+" "+
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().y)+" "+
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().z) );
    }
    void RVizPluginWAIParticles::UpdateParticlesScaler()
    {
        m_ogr_particlesystem->getAffector(2)->setParameter("rate",std::to_string(m_prp_f_particles_scaler->getFloat()));
    }
    void RVizPluginWAIParticles::UpdateParticlesRandomness()
    {
        m_ogr_particlesystem->getAffector(3)->setParameter("randomness",std::to_string(m_prp_i_particles_randomness->getInt()));
        m_ogr_particlesystem->getAffector(3)->setParameter("scope",std::to_string(m_prp_f_particles_scope->getFloat()));
        m_ogr_particlesystem->getAffector(3)->setParameter("keep_velocity",std::to_string(m_prp_b_particles_keep_velocity->getBool()));
    }

    void RVizPluginWAIParticles::UpdateParticlesPropertiesAll()
    {
        m_ogr_particlesystem->setParticleQuota(m_prp_i_particles_quota->getInt());
        m_ogr_particlesystem->getEmitter(0)->setEmissionRate(m_prp_i_particles_emission_rate->getInt());
        m_ogr_particlesystem->getEmitter(0)->setAngle(Ogre::Radian(m_prp_f_particles_angle->getFloat()));
        m_ogr_particlesystem->setDefaultDimensions(m_prp_f_particles_size->getFloat(),m_prp_f_particles_size->getFloat());
        m_ogr_particlesystem->getEmitter(0)->setColourRangeStart(m_prp_col_particles_color_range_start->getOgreColor());
        m_ogr_particlesystem->getEmitter(0)->setColourRangeEnd(m_prp_col_particles_color_range_end->getOgreColor());
        m_ogr_particlesystem->getEmitter(0)->setTimeToLive(m_prp_f_particles_time_to_live->getFloat());
        m_ogr_particlesystem->getEmitter(0)->setMaxParticleVelocity(m_prp_f_particles_velocity_max->getFloat());
        m_ogr_particlesystem->getAffector(0)->setParameter("red",std::to_string(m_prp_vec_particles_colour_fader->getVector().x));
        m_ogr_particlesystem->getAffector(0)->setParameter("green",std::to_string(m_prp_vec_particles_colour_fader->getVector().y));
        m_ogr_particlesystem->getAffector(0)->setParameter("blue",std::to_string(m_prp_vec_particles_colour_fader->getVector().z));
        m_ogr_particlesystem->getAffector(0)->setParameter("alpha",std::to_string(m_prp_f_particles_colour_fader_alpha->getFloat()));
        m_ogr_particlesystem->getAffector(1)->setParameter("force_vector",
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().x)+" "+
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().y)+" "+
                                                            std::to_string(m_prp_vec_particles_linear_force->getVector().z) );
        m_ogr_particlesystem->getAffector(2)->setParameter("rate",std::to_string(m_prp_f_particles_scaler->getFloat()));
        m_ogr_particlesystem->getAffector(3)->setParameter("randomness",std::to_string(m_prp_i_particles_randomness->getInt()));
        m_ogr_particlesystem->getAffector(3)->setParameter("scope",std::to_string(m_prp_f_particles_scope->getFloat()));
        m_ogr_particlesystem->getAffector(3)->setParameter("keep_velocity",std::to_string(m_prp_b_particles_keep_velocity->getBool()));
    }

} // end namespace rviz_plugin_wai_particles

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_particles::RVizPluginWAIParticles,rviz::Display)
