//
// Created by Denys Kotelovych on 04.09.18.
// Heavily modified by Werner Alexander Isop in 2019
//

#ifndef RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H
#define RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H

#include <rviz/display.h>

#ifndef Q_MOC_RUN

#include <OGRE/OgreTexture.h>
#include <GL/glew.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include "rviz_vive_plugin/vive.h"
#include "rviz_vive_plugin/vive_conversions.h"
#include <rviz_vive_plugin_msgs/ControllerVibration.h>

#endif

namespace Ogre
{
    class SceneNode;
    class Viewport;
    class Camera;
    class RenderWindow;
}

namespace rviz
{
    class RenderWidget;
    class StringProperty;
    class RosTopicProperty;
    class TfFrameProperty;
}

namespace rviz_vive_plugin
{

    class ViveDisplay : public rviz::Display
    {
    Q_OBJECT

    public:
        ViveDisplay();
        virtual ~ViveDisplay();
        virtual void onInitialize() override;
        virtual void onDisable() override;
        virtual void update(float wall_dt, float ros_dt) override;

    private:
        void InitProperties();
        void InitOgre();

    private Q_SLOTS:
        void LeftStateTopicPropertyChanged();
        void RightStateTopicPropertyChanged();
        void LeftVibrationTopicPropertyChanged();
        void RightVibrationTopicPropertyChanged();
        void OffsetTopicPropertyChanged();

    private:
        void LeftVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg);
        void RightVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg);
        void HMDOffsetMessageReceived(const geometry_msgs::PoseStampedPtr &msg);

    private:
        struct
        {
            struct
            {
                rviz::StringProperty *Frame;
                rviz::StringProperty *StateTopic;
                rviz::RosTopicProperty *VibrationTopic;
            }Left,Right;
            struct
            {
                rviz::StringProperty *Frame;
            }HMD;
            rviz::RosTopicProperty *OffsetTopic;
            rviz::TfFrameProperty *RootFrame;
        }properties_;

        struct
        {
            rviz::RenderWidget *RenderWidget;
            Ogre::RenderWindow *RenderWindow;
            struct
            {
                GLuint TextureGLID;
                Ogre::TexturePtr Texture;
                Ogre::Viewport *Viewport;
                Ogre::Camera *EyeCamera;
            }Left,Right;
            struct
            {
                Ogre::SceneNode *Node;
            }HMD;
        }ogre_;

        tf::TransformBroadcaster tb_;
        tf::TransformListener tl_;
        tf::StampedTransform stf_offset_root_frame;
        tf::StampedTransform stf_offset_message;
        tf::StampedTransform stf_offset_message_straighten;
        tf::Transform stf_offset;
        tf::StampedTransform stf_hmd;
        tf::StampedTransform stf_controller_left;
        tf::StampedTransform stf_controller_right;
        ros::NodeHandle node_;
        struct
        {
            ros::Publisher LeftHand;
            ros::Publisher RightHand;
            //ros::Publisher HTC;
        }pubs_;
        struct
        {
            ros::Subscriber VibrationLeft;
            ros::Subscriber VibrationRight;
            ros::Subscriber HMDOffset;
        }subs_;

        ros::Time leftVibrationTimestamp_, rightVibrationTimestamp_;
        double leftVibrationDuration_, rightVibrationDuration_;
        std::string s_frame_id_msg;
        std::string s_frame_id_msg_old;
        tf::Quaternion qua_hmd_pose_avg;
        tf::Quaternion qua_hmd_pose_avg_old;

        Vive vive_;
        Vive::HMD viveHMD{};
        HMD hmd;
        Controller controller_left;
        Controller controller_right;

        bool b_enable_motion_smooting;
        bool b_enable_htc_vive_view_window;
    };

} // namespace rviz_vive_plugin

#endif
