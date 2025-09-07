#ifndef RVIZ_PLUGIN_WAI_LIGHTS_H
#define RVIZ_PLUGIN_WAI_LIGHTS_H

#ifndef Q_MOC_RUN
    #include<ros/ros.h>
    #include<ros/package.h>
    #include<std_msgs/String.h>
    #include<rviz/display.h>
#endif

#include<rviz/properties/color_property.h>
#include<rviz/display_context.h>
#include<rviz/frame_manager.h>
#include<rviz/visualization_manager.h>
#include<rviz/properties/bool_property.h>
#include<rviz/properties/color_property.h>
#include<rviz/properties/enum_property.h>
#include<rviz/properties/int_property.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/vector_property.h>
#include<rviz/properties/quaternion_property.h>
#include<OGRE/OgreSceneManager.h>
#include<OgreQuaternion.h>
#include<OgreVector3.h>
#include<OgreLight.h>

#include<std_msgs/Bool.h>
#include<geometry_msgs/Pose.h>
#include<interactive_markers/interactive_marker_server.h>
#include<interactive_markers/menu_handler.h>
#include<visualization_msgs/Marker.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<visualization_msgs/InteractiveMarkerControl.h>
#include<visualization_msgs/InteractiveMarkerFeedback.h>
#include<tf/transform_datatypes.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>



static int i_static_light_id=0;


namespace rviz_plugin_wai_lights
{
    class RVizPluginWAILights : public rviz::Display
    {
        Q_OBJECT
    public:
        // RViz requires plugins to use default constructor with no arguments
        RVizPluginWAILights();
        virtual ~RVizPluginWAILights();

    protected:
        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void reset() override;
        //virtual void fixedFrameChanged() override;
        virtual void update(float wall_dt,float ros_dt);

    private Q_SLOTS:
        void updateLightType();
        void updateShadowsEnabled();
        void updateMarkerEnabled();
        void updateLightColors();
        void updateAmbientColor();
        void updateDiffuseColor();
        void updateSpecularColor();
        void updateColorDiffSpecHSV();
        void updateColorDiffSpecYCbCr();
        void updateLightPose();
        void updateMarkerPose();
        void updateRange();
        void updateAttenuation();
        void updateOuterAngle();
        void updateInnerAngle();
        void updateFalloff();
        void updateSunlight();
        void disableLights();
        void resetLightColors();

        void cb_sub_col_light_color_ambient(const std_msgs::ColorRGBAConstPtr &msg);
        void cb_sub_col_light_color_diff_spec(const std_msgs::ColorRGBAConstPtr &msg);
        void cb_sub_bol_light_enable(const std_msgs::BoolConstPtr &msg);

        void updateInteractiveMarkerType6DOF(std::string s_frame,bool b_iam_fixed,unsigned int ui_interaction_mode);
        visualization_msgs::InteractiveMarkerControl& addInteractiveMarkerTriggerControl(visualization_msgs::InteractiveMarker &msg);
        visualization_msgs::Marker addInteractiveMarkerTrigger(visualization_msgs::InteractiveMarker &msg);
        void cb_iam_process_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &iam_feedback_msg);
        void updateInteractiveMarkerPose();

        tf::Quaternion getQuaternionFromDirection(tf::Vector3 vc3_input);
        Ogre::Vector3 getVector3FromQuaternion(Ogre::Quaternion qua_input);

    private:
        Ogre::SceneManager* m_ogr_scene_manager;
        Ogre::Light* m_ogr_light;
        Ogre::Vector3 m_ogr_vc3_light_position;
        Ogre::Quaternion m_ogr_qua_light_orientation;
        Ogre::Vector3 m_ogr_vc3_frame_position;
        Ogre::Quaternion m_ogr_qua_frame_orientation;
        QColor qcl_light_ambient_old;
        QColor qcl_light_diffuse_old;
        QColor qcl_light_specular_old;
        std::string m_s_frame;
        std::string s_iam_ctl_mode_text;
        bool m_b_light_shadows_enabled;
        bool m_b_light_markers_enabled;
        float m_f_sunlight_radial_distance;
        int m_f_sunlight_sunrise_hour;

        rviz::TfFrameProperty* m_prp_frame;
        rviz::EnumProperty* m_prp_enu_light_type;
        rviz::BoolProperty* m_prp_bol_light_shadows_enabled;
        rviz::BoolProperty* m_prp_bol_light_markers_enabled;
        rviz::ColorProperty* m_prp_col_light_ambient;
        rviz::ColorProperty* m_prp_col_light_diffuse;
        rviz::ColorProperty* m_prp_col_light_specular;
        rviz::VectorProperty* m_prp_vec_hsv_light_diff_spec;
        rviz::VectorProperty* m_prp_vec_ycbcr_light_diff_spec;
        rviz::QuaternionProperty* m_prp_qua_light_orientation_offset;
        rviz::VectorProperty* m_prp_vec_light_position_offset;
        rviz::FloatProperty* m_prp_f_light_range;
        rviz::VectorProperty* m_prp_vec_light_attenuation;
        rviz::FloatProperty* m_prp_f_light_inner_angle;
        rviz::FloatProperty* m_prp_f_light_outer_angle;
        rviz::FloatProperty* m_prp_f_light_falloff_rate;
        rviz::FloatProperty* m_prp_f_light_sun_radial_distance;
        rviz::IntProperty* m_prp_i_light_sun_rise_hour;

        ros::NodeHandle m_hdl_node;
        interactive_markers::InteractiveMarkerServer* m_iam_server;
        interactive_markers::MenuHandler* m_iam_menu_handler;
        ros::Subscriber m_sub_col_light_color_ambient;
        ros::Subscriber m_sub_col_light_color_specular;
        ros::Subscriber m_sub_col_light_color_diff_spec;
        ros::Subscriber m_sub_bol_light_enable;
        ros::Publisher m_pub_mrk_light;
        visualization_msgs::Marker m_msg_mrk_pointlight;
        visualization_msgs::Marker m_msg_mrk_spotlight;
        visualization_msgs::Marker m_msg_mrk_sunlight;
        visualization_msgs::InteractiveMarker m_iam_mrk_light_colors;
    };
} // end namespace rviz_plugin_wai_lights

#endif // RVIZ_PLUGIN_WAI_LIGHTS_H
