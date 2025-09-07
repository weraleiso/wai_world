#ifndef RVIZ_PLUGIN_WAI_TEXTURE_H
#define RVIZ_PLUGIN_WAI_TEXTURE_H

#include<QObject>
#ifndef Q_MOC_RUN
#include<OGRE/OgreEntity.h>
#include<OGRE/OgreFrustum.h>
#include<OGRE/OgreMaterialManager.h>
#include<OGRE/OgreMovableObject.h>
#include<OGRE/OgreSceneManager.h>
#include<OGRE/OgreSceneNode.h>
#include<OGRE/OgreManualObject.h>
#include<OGRE/OgreRenderQueueListener.h>
#include<OGRE/OgreRenderSystem.h>
#include<OGRE/OgreRenderTargetListener.h>
#include<OGRE/OgreRenderWindow.h>
#include<OGRE/OgreRoot.h>
#include<OGRE/OgreSceneNode.h>
#include<OGRE/OgreVector3.h>
#include<OGRE/OgreHardwarePixelBuffer.h>
#include<OGRE/OgreWindowEventUtilities.h>
#include<rviz/display.h>
#include<rviz/frame_manager.h>
#include<rviz/image/image_display_base.h>
#include<rviz/image/ros_image_texture.h>
#include<rviz/display_context.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/ros_topic_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/color_property.h>
#include<rviz/properties/vector_property.h>
#include<rviz/properties/quaternion_property.h>
#include<rviz/render_panel.h>
#include<rviz/robot/robot.h>
#include<rviz/robot/tf_link_updater.h>
#include<rviz/validate_floats.h>
#include<rviz/view_manager.h>
#include<rviz/visualization_manager.h>
#include<message_filters/subscriber.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<image_transport/image_transport.h>
#include<image_transport/subscriber_plugin.h>
#include<image_transport/subscriber_filter.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/CameraInfo.h>
#include<sensor_msgs/Image.h>
#include<shape_msgs/Mesh.h>
#include<tf/message_filter.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf_conversions/tf_eigen.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/camera_common.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<vector>
#include<map>
#include<QMap>
#include<string>
#include<vector>
#endif  // Q_MOC_RUN

using namespace rviz; // Necessary ONLY for transport hint signals/slots



namespace rviz_plugin_wai_texture
{
    /**
    * \class RVizPluginWAITexture
    * \brief Uses a pose from topic + offset to render a bounding object with shape, size and color
    */
    class RVizPluginWAITexture: public rviz::Display,  public Ogre::RenderTargetListener, public Ogre::RenderQueueListener
    {
        Q_OBJECT

    private:
        rviz::RosTopicProperty* m_prp_image_topic;
        rviz::FloatProperty* m_prp_image_alpha;
        rviz::EnumProperty* m_prp_image_transport;
        rviz::TfFrameProperty* m_prp_tf_frame;
        rviz::VectorProperty* m_prp_vec_image_position;
        rviz::QuaternionProperty* m_prp_qua_image_orientation;
        rviz::FloatProperty* m_prp_pixels_to_meters;
        rviz::FloatProperty* m_prp_border_size;
        rviz::ColorProperty* m_prp_border_color;

        Ogre::SceneNode* m_scn_nodes_meshes;
        Ogre::SceneNode* m_scn_nodes_projector;
        Ogre::Frustum* m_fru_frustum_decal;
        Ogre::MaterialPtr m_mem_material_mesh;
        Ogre::Pass* pas_material_image_texture;
        Ogre::TextureUnitState* tex_state;
        Ogre::ManualObject* m_man_manual_object;
        rviz::ROSImageTexture* m_rit_ros_image_texture;
        shape_msgs::Mesh msh_mesh_texture;
        std::vector<geometry_msgs::Point> ver_mesh_texture;
        std::vector<shape_msgs::MeshTriangle> tri_mesh_texture;

        ros::NodeHandle m_hdl_node;
        image_transport::ImageTransport m_hdl_it;
        image_transport::Subscriber m_sub_image_texture;
        ros::Timer tim_img_tra_theora;

        Ogre::Vector3 m_ogr_vc3_frame_position;
        Ogre::Quaternion m_ogr_qua_frame_orientation;
        Ogre::Vector3 m_ogr_vc3_texture_position;
        Ogre::Quaternion m_ogr_qua_texture_orientation;
        tf::Transform m_tf_texture;
        tf::Transform m_tf_frame;
        tf::Transform m_tf_from_property;

        Ogre::ColourValue m_col_border;
        int m_i_image_width_pixels;
        int m_i_image_height_pixels;
        int m_i_image_width_pixels_old;
        int m_i_image_height_pixels_old;
        float m_f_image_alpha;
        float m_f_image_width_meters;
        float m_f_image_height_meters;
        float m_f_border_size;
        float m_f_pixels_to_meters;
        int m_i_img_tra_theory_counter;
        bool m_b_img_received;

        void cb_img_texture(const sensor_msgs::Image::ConstPtr& image);
        void cb_tmr_img_tra_theora(const ros::TimerEvent& event);

    private Q_SLOTS:
        void onImageTopicChanged();
        void onImageAlphaChanged();
        void onTFFrameChanged();
        void onImagePositionChanged();
        void onImageOrientationChanged();
        void onBorderSizeChanged();
        void onBorderColorChanged();
        void onMetersPerPixelChanged();
        void fillTransportOptionList(EnumProperty* property);
        void UpdateTextureAlpha();
        void UpdateTextureGeometry();
        void UpdateTextureRendering();
        void UpdateTextureSandbox();

    protected:
        std::set<std::string> transport_plugin_types_;

    public:
        RVizPluginWAITexture();
        ~RVizPluginWAITexture();
        virtual void onInitialize();
        virtual void reset();
        virtual void update(float wall_dt, float ros_dt);
    };
}  // namespace rviz_plugin_wai_texture

#endif  // RVIZ_PLUGIN_WAI_TEXTURE_H
