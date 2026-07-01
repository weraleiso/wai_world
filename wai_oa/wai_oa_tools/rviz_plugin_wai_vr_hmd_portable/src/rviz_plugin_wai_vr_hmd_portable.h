#ifndef RVIZ_PLUGIN_WAI_VR_HMD_PORTABLE_H
#define RVIZ_PLUGIN_WAI_VR_HMD_PORTABLE_H

#ifndef Q_MOC_RUN
    #include<ros/ros.h>
    #include<ros/package.h>
    #include<std_msgs/String.h>
    #include<rviz/display.h>
#endif

#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<cv_bridge/cv_bridge.h>

#include<rviz/display_context.h>
#include<rviz/frame_manager.h>
#include<rviz/visualization_manager.h>
#include<rviz/properties/ros_topic_property.h>
#include<rviz/properties/color_property.h>
#include<rviz/properties/enum_property.h>
#include<rviz/properties/bool_property.h>
#include<rviz/properties/int_property.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/vector_property.h>

#include<OGRE/OgreSceneManager.h>
#include<OGRE/OgreMaterial.h>
#include<OGRE/OgreRenderTargetListener.h>
#include<OGRE/OgreSharedPtr.h>
#include<OGRE/OgreTexture.h>
#include<OGRE/OgreSceneNode.h>
#include<OGRE/OgreSceneManager.h>
#include<OGRE/OgreTextureManager.h>
#include<OGRE/OgreHardwarePixelBuffer.h>
#include<OGRE/OgreManualObject.h>
#include<OGRE/OgreCamera.h>
#include<OGRE/OgreRenderWindow.h>
#include<OGRE/OgreViewport.h>
#include<OGRE/OgreVector3.h>
#include<OGRE/OgreQuaternion.h>



namespace rviz_plugin_wai_vr_hmd_portable
{
    class RVizPluginWAIVRHmdPortable : public rviz::Display
    {
        Q_OBJECT
    public:
        // RViz requires plugins to use default constructor with no arguments
        RVizPluginWAIVRHmdPortable();
        virtual ~RVizPluginWAIVRHmdPortable();

    protected:
        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void reset() override;
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt,float ros_dt);

    private Q_SLOTS:
        void onVRHMDPortableViewImageTopicChanged();
        void CleanupVRHMDPortableTexture();
        void InitVRHMDPortableTexture();
        void UpdateVRHMDPortableTexture();
        void UpdateVRHMDPortableResultion();
        void UpdateVRHMDPortableIPD();
        void UpdateVRHMDPortableDistortion();
        void UpdateVRHMDPortableCameraPose();
        void UpdateVRHMDPortableCameraIntrinsics();
        void UpdateVRHMDPortableStream();

        void cb_tmr_init_delayed(const ros::TimerEvent& event);

    private:
        ros::NodeHandle m_hdl_node;
        ros::Timer tmr_init_delayed;
        image_transport::ImageTransport m_hdl_it;
        image_transport::TransportHints hints;
        sensor_msgs::ImagePtr m_msg_img_vr_hmd_portable;
        std::string s_hmd_view_image_topic;
        cv::Mat* mat_img_hmd_eye_left;
        cv::Mat* mat_img_hmd_eye_left_dist;
        cv::Mat* mat_img_hmd_eye_right;
        cv::Mat* mat_img_hmd_eye_right_dist;
        cv::Mat* mat_img_vr_hmd_portable;
        cv::Mat m_mat_camera_matrix;
        cv::Mat m_mat_distortion_coefficients;

        ros::Subscriber m_sub_hmd_camera_pose;
        image_transport::Publisher m_pub_img_vr_hmd_portable;

        Ogre::SceneManager* m_ogr_scene_manager;
        Ogre::SceneNode* m_ogr_scene_node;
        Ogre::Camera* m_ogr_camera_current;
        Ogre::Camera* m_ogr_camera_eye_left;
        Ogre::Camera* m_ogr_camera_eye_right;
        Ogre::TexturePtr m_ogr_texture_ptr_eye_left;
        Ogre::TexturePtr m_ogr_texture_ptr_eye_right;
        Ogre::RenderTexture* m_ogr_render_texture_eye_left;
        Ogre::RenderTexture* m_ogr_render_texture_eye_right;

        rviz::RosTopicProperty* m_prp_top_hmd_view_image_topic;
        rviz::IntProperty* m_prp_i_hmd_view_width;
        rviz::IntProperty* m_prp_i_hmd_view_height;
        rviz::FloatProperty* m_prp_f_hmd_ipd;
        rviz::FloatProperty* m_prp_f_hmd_dist;

        bool m_b_cam_found;
        int m_i_hmd_view_width;
        int m_i_hmd_view_height;
        float m_f_hmd_ipd;
        float m_f_hmd_dist;
    };
} // end namespace rviz_plugin_wai_vr_hmd_portable

#endif // RVIZ_PLUGIN_WAI_VR_HMD_PORTABLE_H
