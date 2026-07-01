#ifndef RVIZ_PLUGIN_WAI_TEXTURE_H
#define RVIZ_PLUGIN_WAI_TEXTURE_H


#ifndef Q_MOC_RUN
#include<QObject>
#include<QMap>

#include<image_transport/image_transport.h>
#include<image_transport/subscriber_plugin.h>
#include<image_transport/subscriber_filter.h>
#include<message_filters/subscriber.h>

#include<std_msgs/Float64.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/CameraInfo.h>
#include<sensor_msgs/Image.h>
#include<shape_msgs/Mesh.h>

#include<rviz/display.h>
#include<rviz/display_context.h>
#include<rviz/render_panel.h>
#include<rviz/image/image_display_base.h>
#include<rviz/view_manager.h>
#include<rviz/visualization_manager.h>
#include<rviz/frame_manager.h>
#include<rviz_visual_tools/rviz_visual_tools.h>
#include<rviz/ogre_helpers/shape.h>

#include<rviz/properties/ros_topic_property.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/color_property.h>
#include<rviz/properties/vector_property.h>
#include<rviz/properties/quaternion_property.h>

#include<rviz/robot/robot.h>
#include<rviz/robot/tf_link_updater.h>
#include<rviz/validate_floats.h>

#include<tf/message_filter.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<tf_conversions/tf_eigen.h>

#include<cv_bridge/cv_bridge.h>
#include<image_transport/camera_common.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<OGRE/OgreEntity.h>
#include<OGRE/OgreFrustum.h>
#include<OGRE/OgreMaterialManager.h>
#include<OGRE/OgreMovableObject.h>
#include<OGRE/OgreSceneManager.h>
#include<OGRE/OgreSceneNode.h>
#include<OGRE/OgreMeshManager.h>
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

#include<vector>
#include<map>
#include<string>
#endif  // Q_MOC_RUN

using namespace rviz; // Necessary ONLY for transport hint signals/slots


static int ui_msh_obj_name_cnt=0;

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

        // ROS Basics
        ros::NodeHandle* m_hdl_node;

        // Subscribers And Publishers
        ros::TransportHints m_sub_hints;
        ros::Subscriber m_sub_image;
        //ros::Timer m_tmr_frame;
        ros::Timer m_tmr_image;

        // RViz Properties
        rviz::StringProperty* m_prp_image_topic;
        rviz::EnumProperty* m_prp_image_transport;
        rviz::IntProperty* m_prp_image_res_x;
        rviz::IntProperty* m_prp_image_res_y;
        rviz::FloatProperty* m_prp_image_width;
        rviz::FloatProperty* m_prp_image_height;
        rviz::TfFrameProperty* m_prp_image_tf_frame;
        rviz::BoolProperty* m_prp_image_tf_refresh;
        rviz::FloatProperty* m_prp_border_thickness;
        rviz::ColorProperty* m_prp_border_color;
        rviz::FloatProperty* m_prp_image_alpha;

        // RViz Shapes
        rviz::Shape* m_shp_border_left;
        rviz::Shape* m_shp_border_right;
        rviz::Shape* m_shp_border_top;
        rviz::Shape* m_shp_border_bottom;

        // OGRE Objects
        Ogre::SceneNode* m_hdl_node_scene;
        Ogre::TexturePtr m_tep_texture;
        Ogre::MaterialPtr m_map_material;
        Ogre::TextureUnitState* m_tus_texture_unit_state;
        Ogre::Pass* m_pas_texture;
        Ogre::Entity* m_ent_entity;
        Ogre::Plane m_pla_plane;
        Ogre::HardwarePixelBufferSharedPtr pixelBuffer;
        Ogre::PixelBox* m_pib_pixel_box;
        uint8_t* mem_destination;
        Ogre::Vector3 m_vc3_position;
        Ogre::Quaternion m_qua_orientation;
        tf::Transform m_tf2_image;

        // OpenCV
        cv::Mat m_mat_image_to_render;
        cv::Mat m_mat_icon_timeout;
        std::string m_s_path_icon_timeout;

        // Other helper members
        bool m_b_received_image;
        int m_i_res_x;
        int m_i_res_y;

    private Q_SLOTS:
        void cb_prp_update_image_topic();
        void cb_prp_update_image_res_x();
        void cb_prp_update_image_res_y();
        void cb_prp_update_image_width();
        void cb_prp_update_image_height();
        void cb_prp_update_tf_frame();
        void cb_prp_update_tf_refresh();
        void cb_prp_update_image_border_thickness();
        void cb_prp_update_image_border_color();
        void cb_prp_update_image_alpha();

    protected:
        void onInitialize() override;
        void onDisable();
        void onEnable();
        void reset() override;
        void update(float wall_dt, float ros_dt) override;

        void UpdateOgreResources();
        void UpdateTexture();
        void UpdateTextureResolution(int i_res_x,int i_res_y);
        void UpdateImageFrame();
        void UpdateImageRendered();
        void UpdateImageTopicAndTransport();
        void UpdateImageBorder();
        void UpdateImageAlpha();

        void cb_sub_image(const sensor_msgs::CompressedImageConstPtr&);
        //void cb_tmr_frame(const ros::TimerEvent& event);
        void cb_tmr_image(const ros::TimerEvent& event);

    public:
        RVizPluginWAITexture();
        ~RVizPluginWAITexture();
    };
}  // namespace rviz_plugin_wai_texture

#endif  // RVIZ_PLUGIN_WAI_TEXTURE_H
