#ifndef RVIZ_PLUGIN_WAI_PARTICLES_H
#define RVIZ_PLUGIN_WAI_PARTICLES_H

#ifndef Q_MOC_RUN
    #include<ros/ros.h>
    #include<ros/package.h>
    #include<std_msgs/String.h>
    #include<rviz/display.h>
#endif

#include<visualization_msgs/Marker.h>
#include<rviz/properties/color_property.h>
#include<rviz/display_context.h>
#include<rviz/frame_manager.h>
#include<rviz/visualization_manager.h>
#include<rviz/properties/color_property.h>
#include<rviz/properties/enum_property.h>
#include<rviz/properties/bool_property.h>
#include<rviz/properties/int_property.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/vector_property.h>
#include<OGRE/OgreSceneManager.h>
#include<OgreQuaternion.h>
#include<OgreVector3.h>
#include<OgreLight.h>
#include<OgreParticleSystem.h>
#include<OgreParticle.h>
#include<OgreParticleEmitter.h>
#include<OgreParticleAffector.h>
#include<boost/random.hpp>



namespace rviz_plugin_wai_particles
{
    class RVizPluginWAIParticles : public rviz::Display
    {
        Q_OBJECT
    public:
        // RViz requires plugins to use default constructor with no arguments
        RVizPluginWAIParticles();
        virtual ~RVizPluginWAIParticles();

    protected:
        virtual void onInitialize() override;
        virtual void onEnable() override;
        virtual void onDisable() override;
        virtual void reset() override;
        virtual void fixedFrameChanged();
        virtual void update(float wall_dt,float ros_dt);

    private Q_SLOTS:
        void UpdateParticlesPosition();
        void UpdateParticlesDirection();
        void UpdateParticlesPreset();
        void UpdateParticlesType();
        void UpdateParticlesQuota();
        void UpdateParticlesEmissionRate();
        void UpdateParticlesAngle();
        void UpdateParticlesSize();
        void UpdateParticlesColor();
        void UpdateParticlesTimeToLive();
        void UpdateParticlesVelocityMax();
        void UpdateParticlesColourFader();
        void UpdateParticlesLinearForce();
        void UpdateParticlesScaler();
        void UpdateParticlesRandomness();

        void UpdateParticlesPropertiesAll();

    private:
        Ogre::SceneManager* m_ogr_scene_manager;
        Ogre::SceneNode* m_ogr_scene_node;
        Ogre::ParticleSystem* m_ogr_particlesystem;
        Ogre::Vector3 m_ogr_vc3_frame_position;
        Ogre::Quaternion m_ogr_qua_frame_orientation;

        rviz::TfFrameProperty* m_prp_tf_particles_frame;
        rviz::VectorProperty* m_prp_vec_particles_position;
        rviz::VectorProperty* m_prp_vec_particles_direction;
        rviz::EnumProperty* m_prp_enu_particles_preset;
        rviz::EnumProperty* m_prp_enu_particles_type;
        rviz::IntProperty* m_prp_i_particles_quota;
        rviz::IntProperty* m_prp_i_particles_emission_rate;
        rviz::FloatProperty* m_prp_f_particles_angle;
        rviz::FloatProperty* m_prp_f_particles_size;
        //rviz::ColorProperty* m_prp_col_particles_color;
        rviz::ColorProperty* m_prp_col_particles_color_range_start;
        rviz::ColorProperty* m_prp_col_particles_color_range_end;
        rviz::FloatProperty* m_prp_f_particles_time_to_live;
        rviz::FloatProperty* m_prp_f_particles_velocity_max;
        rviz::VectorProperty* m_prp_vec_particles_colour_fader;
        rviz::FloatProperty* m_prp_f_particles_colour_fader_alpha;
        rviz::VectorProperty* m_prp_vec_particles_linear_force;
        rviz::FloatProperty* m_prp_f_particles_scaler;
        rviz::IntProperty* m_prp_i_particles_randomness;
        rviz::FloatProperty* m_prp_f_particles_scope;
        rviz::BoolProperty* m_prp_b_particles_keep_velocity;
    };
} // end namespace rviz_plugin_wai_particles

#endif // RVIZ_PLUGIN_WAI_PARTICLES_H
