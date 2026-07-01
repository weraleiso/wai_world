#include "rviz_plugin_wai_vr_hmd_portable.h"



namespace rviz_plugin_wai_vr_hmd_portable
{
    using namespace rviz;

    RVizPluginWAIVRHmdPortable::RVizPluginWAIVRHmdPortable() : Display(), m_hdl_node(), m_hdl_it(m_hdl_node)
    {
        m_prp_top_hmd_view_image_topic=new rviz::RosTopicProperty("View Image Topic","vr_hmd_portable/image_raw",QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),"",this,SLOT(onVRHMDPortableViewImageTopicChanged()));
        m_prp_i_hmd_view_width=new IntProperty("Per Eye View Width",640,"",this,SLOT(UpdateVRHMDPortableResultion()));
        m_prp_i_hmd_view_height=new IntProperty("Per Eye View Height",720,"",this,SLOT(UpdateVRHMDPortableResultion()));
        m_prp_f_hmd_ipd=new FloatProperty("Interpupillary Distance",0.065,"",this,SLOT(UpdateVRHMDPortableIPD()));
        m_prp_f_hmd_dist=new FloatProperty("Radial Distortion",0.3,"",this,SLOT(UpdateVRHMDPortableDistortion()));
    }

    void RVizPluginWAIVRHmdPortable::onInitialize()
    {
        Display::onInitialize();

        m_b_cam_found=false;
        m_ogr_scene_manager=context_->getSceneManager();
        try
        {
            ROS_WARN("VRHmdPortable: Proper RViz camera \"ViewControllerCamera1\" found!");
            m_b_cam_found=true;
            //m_ogr_camera_current=m_ogr_scene_manager->getCurrentViewport()->getCamera();
            m_ogr_camera_current=m_ogr_scene_manager->getCamera("ViewControllerCamera1");
        }
        catch(const Ogre::Exception& exception)
        {
            ROS_WARN("VRHmdPortable: RViz camera NOT found, retrying in a couple of seconds!");
            m_b_cam_found=false;
            tmr_init_delayed=m_hdl_node.createTimer(ros::Duration(3.0),&RVizPluginWAIVRHmdPortable::cb_tmr_init_delayed,this,true,true);
            return;
        }

        m_i_hmd_view_width=640;
        m_i_hmd_view_height=720;

        onVRHMDPortableViewImageTopicChanged();
        InitVRHMDPortableTexture();
        UpdateVRHMDPortableResultion();
        UpdateVRHMDPortableIPD();
        UpdateVRHMDPortableDistortion();
        UpdateVRHMDPortableCameraPose();
        UpdateVRHMDPortableCameraIntrinsics();
    }
    void RVizPluginWAIVRHmdPortable::cb_tmr_init_delayed(const ros::TimerEvent& event)
    {
        RVizPluginWAIVRHmdPortable::onInitialize();
    }

    RVizPluginWAIVRHmdPortable::~RVizPluginWAIVRHmdPortable()
    {
        CleanupVRHMDPortableTexture();
    }

    void RVizPluginWAIVRHmdPortable::reset()
    {
        Display::reset();
    }

    void RVizPluginWAIVRHmdPortable::onEnable()
    {

    }

    void RVizPluginWAIVRHmdPortable::onDisable()
    {

    }

    void RVizPluginWAIVRHmdPortable::fixedFrameChanged()
    {

    }

    void RVizPluginWAIVRHmdPortable::update(float wall_dt,float ros_dt)
    {
        if(m_b_cam_found)
        {
            UpdateVRHMDPortableCameraPose();
            UpdateVRHMDPortableStream();
        }
    }

    void RVizPluginWAIVRHmdPortable::onVRHMDPortableViewImageTopicChanged()
    {
        if(m_pub_img_vr_hmd_portable) m_pub_img_vr_hmd_portable.shutdown();
        m_pub_img_vr_hmd_portable=m_hdl_it.advertise(m_prp_top_hmd_view_image_topic->getTopicStd(),1);
    }

    void RVizPluginWAIVRHmdPortable::InitVRHMDPortableTexture()
    {
        // Init Ogre3D textures with cameras
        m_ogr_camera_eye_left=context_->getSceneManager()->createCamera("vr_hmd_portable_camera_eye_left");
        m_ogr_camera_eye_left->setNearClipDistance(m_ogr_camera_current->getNearClipDistance());
        m_ogr_texture_ptr_eye_left=Ogre::TextureManager::getSingleton().createManual(
                                        "vr_hmd_portable_camera_eye_left",
                                        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                        Ogre::TEX_TYPE_2D,
                                        m_i_hmd_view_width,
                                        m_i_hmd_view_height,
                                        0,
                                        Ogre::PF_R8G8B8,
                                        Ogre::TU_RENDERTARGET);
        m_ogr_render_texture_eye_left=m_ogr_texture_ptr_eye_left->getBuffer()->getRenderTarget();
        m_ogr_render_texture_eye_left->addViewport(m_ogr_camera_eye_left);
        m_ogr_render_texture_eye_left->getViewport(0)->setClearEveryFrame(true);
        m_ogr_render_texture_eye_left->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
        m_ogr_render_texture_eye_left->getViewport(0)->setOverlaysEnabled(false);
        m_ogr_render_texture_eye_left->setAutoUpdated(true);
        m_ogr_render_texture_eye_left->setActive(true);

        m_ogr_camera_eye_right=context_->getSceneManager()->createCamera("vr_hmd_portable_camera_eye_right");
        m_ogr_camera_eye_right->setNearClipDistance(m_ogr_camera_eye_left->getNearClipDistance());
        m_ogr_texture_ptr_eye_right=Ogre::TextureManager::getSingleton().createManual(
                                        "vr_hmd_portable_camera_eye_right",
                                        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                        Ogre::TEX_TYPE_2D,
                                        m_i_hmd_view_width,
                                        m_i_hmd_view_height,
                                        0,
                                        Ogre::PF_R8G8B8,
                                        Ogre::TU_RENDERTARGET);
        m_ogr_render_texture_eye_right=m_ogr_texture_ptr_eye_right->getBuffer()->getRenderTarget();
        m_ogr_render_texture_eye_right->addViewport(m_ogr_camera_eye_right);
        m_ogr_render_texture_eye_right->getViewport(0)->setClearEveryFrame(true);
        m_ogr_render_texture_eye_right->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
        m_ogr_render_texture_eye_right->getViewport(0)->setOverlaysEnabled(false);
        m_ogr_render_texture_eye_right->setAutoUpdated(true);
        m_ogr_render_texture_eye_right->setActive(true);


        // Init OpenCV textures
        mat_img_hmd_eye_left=new cv::Mat(m_i_hmd_view_height,m_i_hmd_view_width,CV_8UC3);
        mat_img_hmd_eye_right=new cv::Mat(m_i_hmd_view_height,m_i_hmd_view_width,CV_8UC3);
        mat_img_hmd_eye_left_dist=new cv::Mat(m_i_hmd_view_height,m_i_hmd_view_width,CV_8UC3);
        mat_img_hmd_eye_right_dist=new cv::Mat(m_i_hmd_view_height,m_i_hmd_view_width,CV_8UC3);
        mat_img_vr_hmd_portable=new cv::Mat(m_i_hmd_view_height,m_i_hmd_view_width*2,CV_8UC3);
    }

    void RVizPluginWAIVRHmdPortable::CleanupVRHMDPortableTexture()
    {
        m_ogr_render_texture_eye_left->setAutoUpdated(false);
        m_ogr_render_texture_eye_left->setActive(false);
        Ogre::TextureManager::getSingleton().remove(m_ogr_texture_ptr_eye_left->getName());
        m_ogr_scene_manager->destroyCamera("vr_hmd_portable_camera_eye_left");
        m_ogr_texture_ptr_eye_left->freeInternalResources();

        m_ogr_render_texture_eye_right->setAutoUpdated(false);
        m_ogr_render_texture_eye_right->setActive(false);
        Ogre::TextureManager::getSingleton().remove(m_ogr_texture_ptr_eye_right->getName());
        m_ogr_scene_manager->destroyCamera("vr_hmd_portable_camera_eye_right");
        m_ogr_texture_ptr_eye_right->freeInternalResources();
    }

    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableTexture()
    {
        CleanupVRHMDPortableTexture();
        InitVRHMDPortableTexture();
    }

    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableResultion()
    {
        m_i_hmd_view_width=m_prp_i_hmd_view_width->getInt();
        m_i_hmd_view_height=m_prp_i_hmd_view_height->getInt();
        UpdateVRHMDPortableTexture();
    }
    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableIPD()
    {
        m_f_hmd_ipd=m_prp_f_hmd_ipd->getFloat();
        UpdateVRHMDPortableCameraIntrinsics();
    }
    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableDistortion()
    {
        m_f_hmd_dist=m_prp_f_hmd_dist->getFloat();
        UpdateVRHMDPortableCameraIntrinsics();
    }
    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableCameraPose()
    {
            m_ogr_camera_eye_left->setPosition(m_ogr_camera_current->getPosition());
            m_ogr_camera_eye_left->setOrientation(m_ogr_camera_current->getOrientation());
            Ogre::Vector3 ogr_vce3_camera_pos=m_ogr_camera_eye_left->getPosition();
            Ogre::Vector3 ogr_vce3_camera_ipd(m_f_hmd_ipd,0.0,0.0);
            Ogre::Quaternion ogr_qua_camera_ori=m_ogr_camera_eye_left->getOrientation();
            ogr_vce3_camera_pos=ogr_vce3_camera_pos+(ogr_qua_camera_ori * ogr_vce3_camera_ipd);
            m_ogr_camera_eye_right->setPosition(ogr_vce3_camera_pos);
            m_ogr_camera_eye_right->setOrientation(ogr_qua_camera_ori);
    }
    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableCameraIntrinsics()
    {
        m_mat_camera_matrix=cv::Mat(3,3,cv::DataType<double>::type);
        m_mat_camera_matrix.at<double>(0,0)=m_i_hmd_view_width/2.0;
        m_mat_camera_matrix.at<double>(0,1)=0;
        m_mat_camera_matrix.at<double>(0,2)=m_i_hmd_view_width/2.0;
        m_mat_camera_matrix.at<double>(1,0)=0;
        m_mat_camera_matrix.at<double>(1,1)=m_i_hmd_view_height/2.0;
        m_mat_camera_matrix.at<double>(1,2)=m_i_hmd_view_height/2.0;
        m_mat_camera_matrix.at<double>(2,0)=0;
        m_mat_camera_matrix.at<double>(2,1)=0;
        m_mat_camera_matrix.at<double>(2,2)=1;
        m_mat_distortion_coefficients=cv::Mat(4,1,cv::DataType<double>::type);
        m_mat_distortion_coefficients.at<double>(0,0)=m_f_hmd_dist;
        m_mat_distortion_coefficients.at<double>(1,0)=0.0;
        m_mat_distortion_coefficients.at<double>(2,0)=0.0;
        m_mat_distortion_coefficients.at<double>(3,0)=0.0;
    }
    void RVizPluginWAIVRHmdPortable::UpdateVRHMDPortableStream()
    {
        m_i_hmd_view_width=m_ogr_render_texture_eye_left->getWidth();
        m_i_hmd_view_height=m_ogr_render_texture_eye_left->getHeight();

        Ogre::PixelFormat ogr_pf_pixel_format=Ogre::PF_BYTE_RGB;
        uint pixelsize=Ogre::PixelUtil::getNumElemBytes(ogr_pf_pixel_format);
        uint uc_data_size=m_i_hmd_view_width*m_i_hmd_view_height*pixelsize;

        uchar* uc_data_eye_left=OGRE_ALLOC_T(uchar,static_cast<int>(uc_data_size),Ogre::MEMCATEGORY_RENDERSYS);
        uchar* uc_data_eye_right=OGRE_ALLOC_T(uchar,static_cast<int>(uc_data_size),Ogre::MEMCATEGORY_RENDERSYS);
        Ogre::PixelBox ogr_pb_pixelbox_eye_left(m_i_hmd_view_width,m_i_hmd_view_height,1,ogr_pf_pixel_format,uc_data_eye_left);
        Ogre::PixelBox ogr_pb_pixelbox_eye_right(m_i_hmd_view_width,m_i_hmd_view_height,1,ogr_pf_pixel_format,uc_data_eye_right);

        m_ogr_render_texture_eye_left->copyContentsToMemory(ogr_pb_pixelbox_eye_left,Ogre::RenderTarget::FB_AUTO);
        m_ogr_render_texture_eye_right->copyContentsToMemory(ogr_pb_pixelbox_eye_right,Ogre::RenderTarget::FB_AUTO);

        memcpy(&mat_img_hmd_eye_left->data[0],uc_data_eye_left,uc_data_size);
        memcpy(&mat_img_hmd_eye_right->data[0],uc_data_eye_right,uc_data_size);

        m_mat_distortion_coefficients.at<double>(0,0)=m_f_hmd_dist; // Update k1 to enable barrel distortion
        cv::undistort(*mat_img_hmd_eye_left,*mat_img_hmd_eye_left_dist,m_mat_camera_matrix,m_mat_distortion_coefficients);
        cv::undistort(*mat_img_hmd_eye_right,*mat_img_hmd_eye_right_dist,m_mat_camera_matrix,m_mat_distortion_coefficients);
        cv::hconcat(*mat_img_hmd_eye_left_dist,*mat_img_hmd_eye_right_dist,*mat_img_vr_hmd_portable);

        m_msg_img_vr_hmd_portable=cv_bridge::CvImage(std_msgs::Header(),"rgb8",*mat_img_vr_hmd_portable).toImageMsg();
        m_pub_img_vr_hmd_portable.publish(m_msg_img_vr_hmd_portable);

        OGRE_FREE(uc_data_eye_left,Ogre::MEMCATEGORY_RENDERSYS);
        OGRE_FREE(uc_data_eye_right,Ogre::MEMCATEGORY_RENDERSYS);
    }

} // end namespace rviz_plugin_wai_vr_hmd_portable

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_vr_hmd_portable::RVizPluginWAIVRHmdPortable,rviz::Display)
