#include "rviz_plugin_wai_texture.h"

using namespace rviz; // Necessary ONLY for transport hint signals/slots


namespace rviz_plugin_wai_texture
{
    RVizPluginWAITexture::RVizPluginWAITexture() : Display() , m_hdl_it(m_hdl_node)
    {
        // Initialize helper members
        m_col_border.r=0.101960784;
        m_col_border.g=0.42745098;
        m_col_border.b=0.588235294;
        m_col_border.a=1.0;
        m_ogr_vc3_texture_position.x=0.0;
        m_ogr_vc3_texture_position.y=0.0;
        m_ogr_vc3_texture_position.z=0.0;
        m_ogr_qua_texture_orientation.w=1.0;
        m_ogr_qua_texture_orientation.x=0.0;
        m_ogr_qua_texture_orientation.y=0.0;
        m_ogr_qua_texture_orientation.z=0.0;
        m_tf_texture.setOrigin(tf::Vector3(m_ogr_vc3_texture_position.x,m_ogr_vc3_texture_position.y,m_ogr_vc3_texture_position.z));
        m_tf_texture.setRotation(tf::Quaternion(m_ogr_qua_texture_orientation.x,m_ogr_qua_texture_orientation.y,m_ogr_qua_texture_orientation.z,m_ogr_qua_texture_orientation.w));
        m_tf_frame=m_tf_texture;
        m_tf_from_property=m_tf_texture;
        m_i_image_width_pixels=80;
        m_i_image_height_pixels=45;
        m_f_image_width_meters=1.6; // Invoke an update of whole image/texture geometry here at first received image!
        m_f_image_height_meters=0.9; // Invoke an update of whole image/texture geometry here at first received image!
        m_i_image_width_pixels_old=8;
        m_i_image_height_pixels_old=5;
        m_f_image_alpha=1.0;
        m_f_border_size=0.005;
        m_f_pixels_to_meters=0.001;
        m_i_img_tra_theory_counter=0;
        m_b_img_received=false;

        m_prp_image_topic=new rviz::RosTopicProperty("Image Topic","image_raw",QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),"Image topic to subscribe to.",this,SLOT(onImageTopicChanged()));
        m_prp_image_transport=new rviz::EnumProperty("Image Transport Hint","raw","Preferred method of sending images.",this,SLOT(onImageTopicChanged()));
        m_prp_image_alpha=new rviz::FloatProperty("Image Alpha",m_f_image_alpha,"Alpha transparency of the image.",this,SLOT(onImageAlphaChanged()));
        m_prp_image_alpha->setMin(0.5); m_prp_image_alpha->setMax(1.0);
        connect(m_prp_image_transport,SIGNAL(requestOptions(EnumProperty*)),this,SLOT(fillTransportOptionList(EnumProperty*)));
        m_prp_tf_frame=new rviz::TfFrameProperty("TF Frame","projector","Frame in which the image is visualized.",this,0,true,SLOT(onTFFrameChanged())); //TfFrameProperty::FIXED_FRAME_STRING
        m_prp_vec_image_position=new VectorProperty("Position",Ogre::Vector3(0.0,0.0,0.0),"",this, SLOT(onImagePositionChanged()));
        m_prp_qua_image_orientation=new QuaternionProperty("Orientation",Ogre::Quaternion(1.0,0.0,0.0,0.0),"",this,SLOT(onImageOrientationChanged()));
        m_prp_pixels_to_meters=new rviz::FloatProperty("Pixels To Meters",m_f_pixels_to_meters,"Meters per image pixel.",this,SLOT(onMetersPerPixelChanged()));
        m_prp_pixels_to_meters->setMin(0.0001); m_prp_pixels_to_meters->setMax(1.0);
        m_prp_border_size=new rviz::FloatProperty("Border Size",m_f_border_size,"Rviz texture border size.",this,SLOT(onBorderSizeChanged()));
        m_prp_border_size->setMin(0.005); m_prp_border_size->setMax(1.0);
        m_prp_border_color=new rviz::ColorProperty("Border Color",QColor(m_col_border.r*255.0,m_col_border.g*255.0,m_col_border.b*255.0,m_col_border.a*255.0),"Color of border.",this,SLOT(onBorderColorChanged()));

        setStatus(rviz::StatusProperty::Warn,"TF Frame",QString("Not inizialized!"));
        setStatus(rviz::StatusProperty::Warn,"Image Topic",QString("Not inizialized!"));
        setStatus(rviz::StatusProperty::Warn,"Image Width [pix]",QString("Not inizialized!"));
        setStatus(rviz::StatusProperty::Warn,"Image Height [pix]",QString("Not inizialized!"));
        setStatus(rviz::StatusProperty::Warn,"Image Width [m]",QString("Not inizialized!"));
        setStatus(rviz::StatusProperty::Warn,"Image Height [m]",QString("Not inizialized!"));
    }
    RVizPluginWAITexture::~RVizPluginWAITexture()
    {
    }

    void RVizPluginWAITexture::onInitialize()
    {
        m_prp_tf_frame->setFrameManager(context_->getFrameManager());

        setStatus(rviz::StatusProperty::Ok,"TF Frame",QString("Transform Ok!"));
        setStatus(rviz::StatusProperty::Ok,"Image Topic","Using default image!");
        setStatus(rviz::StatusProperty::Ok,"Image Width [pix]",QString::number(m_i_image_width_pixels));
        setStatus(rviz::StatusProperty::Ok,"Image Height [pix]",QString::number(m_i_image_height_pixels));
        setStatus(rviz::StatusProperty::Ok,"Image Width [m]",QString::number(m_f_image_width_meters));
        setStatus(rviz::StatusProperty::Ok,"Image Height [m]",QString::number(m_f_image_height_meters));

        tim_img_tra_theora=m_hdl_node.createTimer(ros::Duration(5.0),&RVizPluginWAITexture::cb_tmr_img_tra_theora,this,false,false);

        // Initialize image transport
        pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader("image_transport","image_transport::SubscriberPlugin");
        BOOST_FOREACH(const std::string& lookup_name,sub_loader.getDeclaredClasses())
        {
            // lookup_name is formatted as "pkg/transport_sub", for instance
            // "image_transport/compressed_sub" for the "compressed"
            // transport.  This code removes the "_sub" from the tail and
            // everything up to and including the "/" from the head, leaving
            // "compressed" (for example) in transport_name.
            std::string transport_name=boost::erase_last_copy(lookup_name, "_sub");
            transport_name=transport_name.substr(lookup_name.find('/') + 1);
            // If the plugin loads without throwing an exception, add its
            // transport name to the list of valid plugins, otherwise ignore
            // it.
            try
            {
                boost::shared_ptr<image_transport::SubscriberPlugin> sub=sub_loader.createInstance(lookup_name);
                transport_plugin_types_.insert(transport_name);
            }
            catch (const pluginlib::LibraryLoadException& e)
            {
                // Do nothing...
            }
            catch (const pluginlib::CreateClassException& e)
            {
                // Do nothing...
            }
        }

        // Initialize visualization nodes and frustums
        m_scn_nodes_meshes=this->scene_node_->createChildSceneNode();
        m_fru_frustum_decal=new Ogre::Frustum();
        m_scn_nodes_projector=scene_manager_->getRootSceneNode()->createChildSceneNode();
        m_scn_nodes_projector->attachObject(m_fru_frustum_decal);
        static uint32_t ui_msh_obj_name_cnt=0;
        ui_msh_obj_name_cnt++;
        m_man_manual_object=context_->getSceneManager()->createManualObject("MeshObject"+std::to_string(ui_msh_obj_name_cnt));//"MeshObject0Index0");
        m_scn_nodes_meshes->attachObject(m_man_manual_object);

        // Initialize visualization objects for material and mesh
        Ogre::MaterialManager& material_manager=Ogre::MaterialManager::getSingleton();
        Ogre::ResourceGroupManager& rg_mgr=Ogre::ResourceGroupManager::getSingleton();
        Ogre::String resource_group_name="MeshNode"+std::to_string(ui_msh_obj_name_cnt);
        Ogre::String material_name=resource_group_name+"MeshMaterial";
        rg_mgr.createResourceGroup(resource_group_name);
        m_mem_material_mesh=material_manager.create(material_name,resource_group_name);
        //m_mem_material_mesh->setReceiveShadows(false);
        m_mem_material_mesh->setCullingMode(Ogre::CULL_NONE);

        // Initialize passes for image texture
        pas_material_image_texture=m_mem_material_mesh->getTechnique(0)->getPass(0);//createPass();
        pas_material_image_texture->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
        //pas_material_image_texture->setDepthBias(1);
        //pas_material_image_texture->setDepthWriteEnabled(false);
        //pas_material_image_texture->setLightingEnabled(false);
        //pas_material_image_texture->setDepthCheckEnabled(false);
        //pas_material_image_texture->setAlphaRejectValue(160);
        tex_state=pas_material_image_texture->createTextureUnitState();
        m_rit_ros_image_texture=new rviz::ROSImageTexture();
        tex_state->setTextureName(m_rit_ros_image_texture->getTexture()->getName());
        tex_state->setProjectiveTexturing(true,m_fru_frustum_decal);
        tex_state->setTextureBorderColour(m_col_border);
        tex_state->setTextureAddressingMode(Ogre::TextureUnitState::TAM_BORDER);
        UpdateTextureAlpha();
    }
    void RVizPluginWAITexture::reset()
    {
        m_sub_image_texture.shutdown();
        m_rit_ros_image_texture->clear();

        if(m_prp_image_topic->getTopic().isEmpty() ||
           m_prp_image_transport->getString().isEmpty())
        {
            m_prp_image_topic->setStdString("texture/image_raw");
            m_prp_image_transport->setStdString("raw");
            setStatus(rviz::StatusProperty::Warn,"Image Topic","No image received!");
        }
        else
        {
            try
            {
                m_sub_image_texture.shutdown();
                std::string s_image_topic=m_prp_image_topic->getTopicStd();
                std::string s_image_transport=m_prp_image_transport->getStdString();
                image_transport::TransportHints hints(s_image_transport, ros::TransportHints());
                m_sub_image_texture=m_hdl_it.subscribe(s_image_topic,0,&RVizPluginWAITexture::cb_img_texture,this,hints);
                setStatus(rviz::StatusProperty::Ok,"Image Topic","Ok");

                if(s_image_transport.compare("theora")==0 && m_i_img_tra_theory_counter<10)
                {
                    m_b_img_received=false;
                    setStatus(rviz::StatusProperty::Warn,"Image Topic","Subscribing to theora stream!");
                    tim_img_tra_theora.stop();
                    tim_img_tra_theora.start();
                    m_i_img_tra_theory_counter++;
                }
                else if(s_image_transport.compare("theora")==0 && m_i_img_tra_theory_counter>=10)
                {
                    tim_img_tra_theora.stop();
                    setStatus(rviz::StatusProperty::Error,"Image Topic","Subscribing to theora stream failed!");
                }
                else
                {
                    // Do nothing...
                }
            }
            catch(ros::Exception& e)
            {
                setStatus(rviz::StatusProperty::Error,"Image Topic",QString("Error subscribing: ")+e.what());
            }
        }
    }
    void RVizPluginWAITexture::update(float wall_dt,float ros_dt)
    {
        // Sandbox (new rendering method)
        //UpdateTextureSandbox();
    }


    // Callbacks
    void RVizPluginWAITexture::cb_img_texture(const sensor_msgs::Image::ConstPtr& msg_img_texture)
    {
        if(m_b_img_received==false ||
            msg_img_texture->width!=m_i_image_width_pixels_old ||
            msg_img_texture->height!=m_i_image_height_pixels_old)
        {
            m_i_image_width_pixels=msg_img_texture->width;
            m_i_image_height_pixels=msg_img_texture->height;
            m_f_image_width_meters=m_i_image_width_pixels*m_f_pixels_to_meters;
            m_f_image_height_meters=m_i_image_height_pixels*m_f_pixels_to_meters;

            // Update status properties
            setStatus(rviz::StatusProperty::Ok,"Image Topic","Ok");
            setStatus(rviz::StatusProperty::Ok,"Image Width [pix]",QString::number(m_i_image_width_pixels));
            setStatus(rviz::StatusProperty::Ok,"Image Height [pix]",QString::number(m_i_image_height_pixels));
            setStatus(rviz::StatusProperty::Ok,"Image Width [m]",QString::number(m_f_image_width_meters));
            setStatus(rviz::StatusProperty::Ok,"Image Height [m]",QString::number(m_f_image_height_meters));

            // Update old image geometry
            m_i_image_width_pixels_old=m_i_image_width_pixels;
            m_i_image_height_pixels_old=m_i_image_height_pixels;

            m_b_img_received=true;
        }

        // If image data changes also update whole mesh geometry, extrinsics and intrinsics...
        UpdateTextureGeometry();
        UpdateTextureRendering();

        //cv_bridge::CvImagePtr cv_ptr;
        //cv_ptr=cv_bridge::toCvCopy(msg_img_texture,sensor_msgs::image_encodings::RGBA8);
        /* Changing alpha channel of RGBD opencv image
        std::vector<cv::Mat>channels(4);
        cv::split(cv_ptr->image,channels);
        channels[3]=100;
        cv::merge(channels,cv_ptr->image);
        */
        m_rit_ros_image_texture->addMessage(msg_img_texture);//cv_ptr->toImageMsg());
        m_rit_ros_image_texture->update();
    }
    void RVizPluginWAITexture::cb_tmr_img_tra_theora(const ros::TimerEvent& event)
    {
        if(m_b_img_received)
        {
            m_i_img_tra_theory_counter=0;
            tim_img_tra_theora.stop();
            setStatus(rviz::StatusProperty::Ok,"Image Topic","Ok");
        }
        else
        {
            reset();
        }
    }


    // Sandbox method
    void RVizPluginWAITexture::UpdateTextureSandbox()
    {
        // Notes: Maybe a better/more efficient approach is to get the local camera coordinates from desired world coordinates
        // and then render the texture in camera coordinates...

        // Sandbox to render image as pixelbox
        cv::Mat mat_img_test=cv::imread("/home/ias/test.png");
        int image_width=640;
        int image_height=480;
        const Ogre::PixelFormat pixel_format=Ogre::PF_BYTE_BGR;
        const auto bytes_per_pixel=Ogre::PixelUtil::getNumElemBytes(pixel_format);
        auto image_data=new unsigned char[image_width*image_height*bytes_per_pixel];
        Ogre::Box image_extents(0,0,image_width,image_height);
        std::shared_ptr<Ogre::PixelBox> pixel_box=std::make_shared<Ogre::PixelBox>(image_extents,pixel_format,image_data);
        size_t img_size=image_width*image_height*bytes_per_pixel;
        //memcpy(pixel_box->data,mat_img_test.data,img_size);

        Ogre::MaterialPtr material=Ogre::MaterialManager::getSingleton().create("mat",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        Ogre::TextureUnitState *tus = material->getTechnique(0)->getPass(0)->createTextureUnitState();
        material->setDepthCheckEnabled(false);
        material->setDepthWriteEnabled(false);
        material->setLightingEnabled(false);
        material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);


        Ogre::TexturePtr image_texture_ = Ogre::TextureManager::getSingleton().createManual("DynamicImageTexture",
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            Ogre::TEX_TYPE_2D,
            image_width,image_height,0,
            Ogre::PF_BYTE_RGB,
            Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
        Ogre::HardwarePixelBufferSharedPtr buffer = image_texture_->getBuffer(0,0);
            buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
            const Ogre::PixelBox &pb = buffer->getCurrentLock();
            // Obtain a pointer to the texture's buffer
            uint8_t *data = static_cast<uint8_t*>(pb.data);
            // Copy the data
            memcpy( (uint8_t*)pb.data, mat_img_test.data, img_size );
            buffer->unlock();
        tus->setTextureName(image_texture_->getName());
        tus->setTextureFiltering(Ogre::TFO_NONE);


        // create ManualObject and SceneNode
        Ogre::ManualObject *mo=new Ogre::ManualObject("myMo");
        Ogre::SceneNode *sn=this->scene_node_->createChildSceneNode();
        // setup the ManualObject.
        float x = -0.5;		// position x
        float y = 0.5;		// position y
        float w = 1;		// width
        float h = 1;		// height
        float z = 0;
        mo->setUseIdentityProjection(true);
        mo->setUseIdentityView(true);
        mo->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
        mo->begin(material->getName(),Ogre::RenderOperation::OT_TRIANGLE_LIST); // begin the Manual Object update.
        mo->position( Ogre::Vector3(x, y, z) ); // left top
        mo->textureCoord(0,0);
        mo->position( Ogre::Vector3( x + w, y, z) ); // right top
        mo->textureCoord(1,0);
        mo->position( Ogre::Vector3( x + w, y - h, z) ); // right bottom
        mo->textureCoord(1,1);
        mo->position( Ogre::Vector3( x, y - h, z) ); // left bottom
        mo->textureCoord(0,1);
        mo->triangle(2,1,0);
        mo->triangle(3,2,0);
        mo->end();
        //Ogre::AxisAlignedBox aabb; // infinite AxisAlignedBox.
        //aabb.setInfinite();
        //mo->setBoundingBox();
        sn->attachObject(mo); // attacsh the Manual Object to the SceneNodes
        sn->setOrientation(0.9159756,-0.0499502, 0.1205905, 0.3794095);
        sn->setPosition(0,0,0);

        /*
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Image","General");
        material->getTechnique(0)->getPass(0)->createTextureUnitState("DynamicImageTexture");
        material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
        material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
        material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
        */
    }


    // Callbacks if properties changed
    void RVizPluginWAITexture::onImageTopicChanged()
    {
        reset();
    }
    void RVizPluginWAITexture::onImageAlphaChanged()
    {
        m_f_image_alpha=m_prp_image_alpha->getFloat();
        UpdateTextureAlpha();
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onMetersPerPixelChanged()
    {
        m_f_pixels_to_meters=m_prp_pixels_to_meters->getFloat();
        m_f_image_width_meters=m_i_image_width_pixels*m_f_pixels_to_meters;
        m_f_image_height_meters=m_i_image_height_pixels*m_f_pixels_to_meters;
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onBorderSizeChanged()
    {
        m_f_border_size=m_prp_border_size->getFloat();
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onBorderColorChanged()
    {
        m_col_border=m_prp_border_color->getOgreColor();
        tex_state->setTextureBorderColour(m_col_border);
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onTFFrameChanged()
    {
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onImagePositionChanged()
    {
        m_b_img_received=false;
    }
    void RVizPluginWAITexture::onImageOrientationChanged()
    {
        m_b_img_received=false;
    }



    // Helper methods
    void RVizPluginWAITexture::UpdateTextureAlpha()
    {
        if(m_f_image_alpha==1.0)
        {
            // Actually make texture intransparent no matter what bg
            pas_material_image_texture->setDepthWriteEnabled(true);
            tex_state=pas_material_image_texture->getTextureUnitState(0);
        }
        else
        {
            pas_material_image_texture->setDepthWriteEnabled(false);
            tex_state=pas_material_image_texture->getTextureUnitState(0);
        }
        tex_state->setAlphaOperation(Ogre::LBX_BLEND_MANUAL,Ogre::LBS_MANUAL,Ogre::LBS_MANUAL,m_f_image_alpha,m_f_image_alpha,m_f_image_alpha);
        tex_state->setColourOperation(Ogre::LBO_REPLACE);
        //tex_state->setAlphaOperation(Ogre::LBX_MODULATE,Ogre::LBS_MANUAL,Ogre::LBS_TEXTURE,m_f_image_alpha);
        //tex_state->setTextureFiltering(Ogre::FO_POINT,Ogre::FO_LINEAR,Ogre::FO_NONE);
    }
    void RVizPluginWAITexture::UpdateTextureGeometry()
    {
        // Update extrinsics
        if(context_->getFrameManager()->getTransform(m_prp_tf_frame->getFrameStd(),ros::Time(0),m_ogr_vc3_frame_position,m_ogr_qua_frame_orientation))
        {
            m_tf_frame.setOrigin(tf::Vector3(m_ogr_vc3_frame_position.x,m_ogr_vc3_frame_position.y,m_ogr_vc3_frame_position.z));
            m_tf_frame.setRotation(tf::Quaternion(m_ogr_qua_frame_orientation.x,m_ogr_qua_frame_orientation.y,m_ogr_qua_frame_orientation.z,m_ogr_qua_frame_orientation.w));
            m_tf_from_property.setOrigin(tf::Vector3(m_prp_vec_image_position->getVector().x,m_prp_vec_image_position->getVector().y,m_prp_vec_image_position->getVector().z));
            m_tf_from_property.setRotation(tf::Quaternion(m_prp_qua_image_orientation->getQuaternion().x,m_prp_qua_image_orientation->getQuaternion().y,m_prp_qua_image_orientation->getQuaternion().z,m_prp_qua_image_orientation->getQuaternion().w));
            m_tf_texture=m_tf_frame*m_tf_from_property;
            m_ogr_vc3_texture_position.x=m_tf_texture.getOrigin().getX();
            m_ogr_vc3_texture_position.y=m_tf_texture.getOrigin().getY();
            m_ogr_vc3_texture_position.z=m_tf_texture.getOrigin().getZ();
            m_ogr_qua_texture_orientation.w=m_tf_texture.getRotation().getW();
            m_ogr_qua_texture_orientation.x=m_tf_texture.getRotation().getX();
            m_ogr_qua_texture_orientation.y=m_tf_texture.getRotation().getY();
            m_ogr_qua_texture_orientation.z=m_tf_texture.getRotation().getZ();
            setStatus(rviz::StatusProperty::Ok,"TF Frame",QString("Transform Ok!"));
        }
        else
        {
            m_ogr_vc3_texture_position.x=0.0;
            m_ogr_vc3_texture_position.y=0.0;
            m_ogr_vc3_texture_position.z=0.0;
            m_ogr_qua_texture_orientation.w=1.0;
            m_ogr_qua_texture_orientation.x=0.0;
            m_ogr_qua_texture_orientation.y=0.0;
            m_ogr_qua_texture_orientation.z=0.0;
            setStatus(rviz::StatusProperty::Error,"TF Frame",QString("Error getting transform!"));
        }

        // Update mesh geometry based on extrinsics
        tf::Vector3 vc3_mesh_corner_top_left(-m_f_image_width_meters/2.0f-m_f_border_size,0.0f,-m_f_image_height_meters/2.0f-m_f_border_size);
        tf::Vector3 vc3_mesh_corner_top_right(m_f_image_width_meters/2.0f+m_f_border_size,0.0f,-m_f_image_height_meters/2.0f-m_f_border_size);
        tf::Vector3 vc3_mesh_corner_bottom_left(-m_f_image_width_meters/2.0f-m_f_border_size,0.0f,m_f_image_height_meters/2.0f+m_f_border_size);
        tf::Vector3 vc3_mesh_corner_bottom_right(m_f_image_width_meters/2.0f+m_f_border_size,0.0f,m_f_image_height_meters/2.0f+m_f_border_size);
        tf::Vector3 vc3_mesh_vertice_top_left=m_tf_texture*vc3_mesh_corner_top_left;
        tf::Vector3 vc3_mesh_vertice_top_right=m_tf_texture*vc3_mesh_corner_top_right;
        tf::Vector3 vc3_mesh_vertice_bottom_left=m_tf_texture*vc3_mesh_corner_bottom_left;
        tf::Vector3 vc3_mesh_vertice_bottom_right=m_tf_texture*vc3_mesh_corner_bottom_right;
        ver_mesh_texture.clear();
        tri_mesh_texture.clear();
        geometry_msgs::Point pnt_temp;
        pnt_temp.x=vc3_mesh_vertice_top_left.getX(); pnt_temp.y=vc3_mesh_vertice_top_left.getY(); pnt_temp.z=vc3_mesh_vertice_top_left.getZ(); ver_mesh_texture.push_back(pnt_temp);
        pnt_temp.x=vc3_mesh_vertice_top_right.getX(); pnt_temp.y=vc3_mesh_vertice_top_right.getY(); pnt_temp.z=vc3_mesh_vertice_top_right.getZ(); ver_mesh_texture.push_back(pnt_temp);
        pnt_temp.x=vc3_mesh_vertice_bottom_left.getX(); pnt_temp.y=vc3_mesh_vertice_bottom_left.getY(); pnt_temp.z=vc3_mesh_vertice_bottom_left.getZ(); ver_mesh_texture.push_back(pnt_temp);
        pnt_temp.x=vc3_mesh_vertice_bottom_right.getX(); pnt_temp.y=vc3_mesh_vertice_bottom_right.getY(); pnt_temp.z=vc3_mesh_vertice_bottom_right.getZ(); ver_mesh_texture.push_back(pnt_temp);
        shape_msgs::MeshTriangle tri_temp;
        tri_temp.vertex_indices[0]=0; tri_temp.vertex_indices[1]=1; tri_temp.vertex_indices[2]=2; tri_mesh_texture.push_back(tri_temp);
        tri_temp.vertex_indices[0]=1; tri_temp.vertex_indices[1]=2; tri_temp.vertex_indices[2]=3; tri_mesh_texture.push_back(tri_temp);
        msh_mesh_texture.vertices=ver_mesh_texture;
        msh_mesh_texture.triangles=tri_mesh_texture;

        // Update intrinsics of projector frustum
        float z_offset=(m_i_image_width_pixels > m_i_image_height_pixels) ? m_i_image_width_pixels : m_i_image_height_pixels;
        float scale_factor=1.0/((m_f_image_width_meters>m_f_image_height_meters) ? m_f_image_width_meters : m_f_image_height_meters);
        tf::Vector3 projector_origin(0.0,0.0,1.0/(z_offset*scale_factor));
        tf::Vector3 projector_point=
                tf::Transform(tf::Quaternion(m_ogr_qua_texture_orientation.x,m_ogr_qua_texture_orientation.y,m_ogr_qua_texture_orientation.z,m_ogr_qua_texture_orientation.w),
                              tf::Vector3(m_ogr_vc3_texture_position.x,m_ogr_vc3_texture_position.y,m_ogr_vc3_texture_position.z))*
                tf::Transform(tf::Quaternion(0.70710678,0.0,0.0,0.70710678),tf::Vector3(0.0,0.0,0.0))*
                              projector_origin;
        m_scn_nodes_projector->setPosition(projector_point.getX(),projector_point.getY(),projector_point.getZ());
        m_scn_nodes_projector->setOrientation(m_ogr_qua_texture_orientation*Ogre::Quaternion(Ogre::Degree(90),Ogre::Vector3::UNIT_X));
        float P[12]={1.0,0.0,m_i_image_width_pixels/2.0f,0.0,0.0,1.0,m_i_image_height_pixels/2.0f,0.0,0.0,0.0,1.0,0.0};
        double far_plane=100;
        double near_plane=0.01;
        Ogre::Matrix4 proj_matrix;
        proj_matrix=Ogre::Matrix4::ZERO;
        proj_matrix[0][0]=2.0 * P[0]/m_i_image_width_pixels;
        proj_matrix[1][1]=2.0 * P[5]/m_i_image_height_pixels;
        proj_matrix[0][2]=2.0 * (+0.5-P[2]/m_i_image_width_pixels);
        proj_matrix[1][2]=2.0 * (-0.5+P[6]/m_i_image_height_pixels);
        proj_matrix[2][2]=-(far_plane+near_plane)/(far_plane-near_plane);
        proj_matrix[2][3]=-2.0 * far_plane*near_plane/(far_plane-near_plane);
        proj_matrix[3][2]=-1.0;
        m_fru_frustum_decal->setCustomProjectionMatrix(true,proj_matrix);
    }

    void RVizPluginWAITexture::UpdateTextureRendering()
    {
        m_man_manual_object->clear();
        m_man_manual_object->estimateVertexCount(msh_mesh_texture.vertices.size());
        m_man_manual_object->begin(m_mem_material_mesh->getName(),Ogre::RenderOperation::OT_TRIANGLE_LIST);
        // m_man_manual_object->beginUpdate(0);
        const std::vector<geometry_msgs::Point>& points=msh_mesh_texture.vertices;
        for(int i=0;i<msh_mesh_texture.triangles.size();i++)
        {
            std::vector<Ogre::Vector3> corners(3);
            for(int c=0;c<3;c++)
            {
                corners[c]=Ogre::Vector3(points[msh_mesh_texture.triangles[i].vertex_indices[c]].x,
                                         points[msh_mesh_texture.triangles[i].vertex_indices[c]].y,
                                         points[msh_mesh_texture.triangles[i].vertex_indices[c]].z);
            }
            Ogre::Vector3 normal=(corners[1]-corners[0]).crossProduct(corners[2]-corners[0]);
            if(i==0) normal=-normal;
            normal.normalise();
            for(int c=0;c<3;c++)
            {
                m_man_manual_object->position(corners[c]);
                m_man_manual_object->normal(normal);
            }
        }
        m_man_manual_object->end();
        m_mem_material_mesh->setCullingMode(Ogre::CULL_NONE);
    }

    void RVizPluginWAITexture::fillTransportOptionList(EnumProperty* property)
    {
        if(m_prp_image_topic->getTopic().isEmpty())
        {
            setStatus(rviz::StatusProperty::Warn,"Image Topic","No image received!");
        }
        else
        {
            property->clearOptions();
            std::vector<std::string> choices;
            choices.push_back("raw");
            // Loop over all current ROS topic names
            ros::master::V_TopicInfo topics;
            ros::master::getTopics(topics);
            ros::master::V_TopicInfo::iterator it=topics.begin();
            ros::master::V_TopicInfo::iterator end=topics.end();
            for (; it != end; ++it)
            {
                // If the beginning of this topic name is the same as topic_,
                // and the whole string is not the same,
                // and the next character is /
                // and there are no further slashes from there to the end,
                // then consider this a possible transport topic.
                const ros::master::TopicInfo& ti=*it;
                const std::string& topic_name=ti.name;
                const std::string& topic=m_prp_image_topic->getStdString();
                if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/' &&
                topic_name.find('/', topic.size() + 1) == std::string::npos)
                {
                    std::string transport_type=topic_name.substr(topic.size() + 1);
                    // If the transport type string found above is in the set of
                    // supported transport type plugins, add it to the list.
                    if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
                    {
                        choices.push_back(transport_type);
                    }
                }
            }
            for (size_t i=0; i < choices.size(); i++)
            {
                property->addOptionStd(choices[i]);
            }
        }
    }
}  // namespace rviz_plugin_wai_texture

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_texture::RVizPluginWAITexture,rviz::Display)
