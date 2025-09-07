//
// Created by Denys Kotelovych on 04.09.18.
//

#include <random>
#include <ros/package.h>
#include <QApplication>
#include <QDesktopWidget>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreCompositorManager.h>
#include <OGRE/OgreCompositorInstance.h>
#include <OGRE/OgreCompositionTargetPass.h>
#include <OGRE/OgreCompositionPass.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/RenderSystems/GL/OgreGLTextureManager.h>
#include <OGRE/RenderSystems/GL/OgreGLRenderSystem.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <rviz_vive_plugin_msgs/Controller.h>
#include <rviz_vive_plugin/vive_display.h>
#include <dirent.h>

#include "rviz_vive_plugin/vive_display.h"
#include "rviz_vive_plugin/vive_conversions.h"
#include "rviz_vive_plugin/vive.h"

#include <OgreMeshManager.h>
#include "boost/filesystem.hpp"
#include <iostream>

static inline tf::StampedTransform BuildTransform(const rviz_vive_plugin::Pose &pose,
                                                  const std::string &parentFrameId,
                                                  const std::string &childFrameId)
{
    return tf::StampedTransform(
                tf::Transform(
                    tf::Quaternion(pose.Orientation.x, pose.Orientation.y, pose.Orientation.z, pose.Orientation.w),
                    tf::Vector3(pose.Position.x, pose.Position.y, pose.Position.z)
                ),
                ros::Time::now(),
                parentFrameId,
                childFrameId);
}

static inline rviz_vive_plugin_msgs::Controller BuildMessage(const rviz_vive_plugin::Controller &controller,
                                                             const std::string &frameId)
{
    rviz_vive_plugin_msgs::Controller msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frameId;
    msg.trackpad_position.x = controller.TrackpadPosition.x;
    msg.trackpad_position.y = controller.TrackpadPosition.y;
    msg.trackpad_position.z = 0.0f;
    msg.trigger = controller.TriggerPressed; // TBD: trigger value
    msg.trackpad_touched = 0; // TBD: trackpad touched
    msg.trackpad_pressed = static_cast<unsigned char>(controller.TrackpadPressed);
    msg.menu_pressed = static_cast<unsigned char>(controller.MenuPressed);
    msg.grip_pressed = static_cast<unsigned char>(controller.GripPressed);
    return msg;
};

namespace rviz_vive_plugin
{

    ViveDisplay::ViveDisplay()
    {
        ogre_.HMD.Node = nullptr;
        ogre_.Left.Texture.setNull();
        ogre_.Right.Texture.setNull();
        ogre_.Left.TextureGLID = 0;
        ogre_.Right.TextureGLID = 0;
        ogre_.Left.Viewport = nullptr;
        ogre_.Right.Viewport = nullptr;
        leftVibrationDuration_ = 0.0;
        rightVibrationDuration_ = 0.0;
        leftVibrationTimestamp_ = ros::Time::now();
        rightVibrationTimestamp_ = ros::Time::now();
        s_frame_id_msg="world";
        s_frame_id_msg_old="world";
        stf_offset_root_frame.stamp_=ros::Time::now();
        stf_offset_root_frame.setOrigin(tf::Vector3(0.0,0.0,0.0));
        stf_offset_root_frame.setRotation(tf::Quaternion(0.0,0.0,0.0,1.0));
        stf_offset_message=stf_offset_root_frame;
        stf_offset_message_straighten=stf_offset_root_frame;
        stf_offset=stf_offset_root_frame;
        stf_hmd=stf_offset_root_frame;
        stf_controller_left=stf_offset_root_frame;
        stf_controller_right=stf_offset_root_frame;
        stf_offset_message_straighten.setOrigin(tf::Vector3(0.0,0.0,0.0));
        stf_offset_message_straighten.setRotation(tf::Quaternion(0.5,-0.5,0.5,0.5));
        qua_hmd_pose_avg.setValue(0.0,0.0,0.0,1.0);
        qua_hmd_pose_avg_old.setValue(0.0,0.0,0.0,1.0);
        b_enable_motion_smooting=false;
        b_enable_htc_vive_view_window=false;
    }

    ViveDisplay::~ViveDisplay()
    {
        onDisable();
    };

    void ViveDisplay::onInitialize()
    {
        InitProperties();

        pubs_.LeftHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
                properties_.Left.StateTopic->getStdString(), 1);
        pubs_.RightHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
                properties_.Right.StateTopic->getStdString(), 1);
        //pubs_.HTC = node_.advertise<geometry_msgs::Pose>("/wai_world/world/htc_vive/hmd_position", 1);

        subs_.VibrationLeft = node_.subscribe(properties_.Left.VibrationTopic->getStdString(),1, &ViveDisplay::LeftVibrationMessageReceived,this);
        subs_.VibrationRight = node_.subscribe(properties_.Right.VibrationTopic->getStdString(),1, &ViveDisplay::RightVibrationMessageReceived,this);
        subs_.HMDOffset = node_.subscribe(properties_.OffsetTopic->getStdString(),1, &ViveDisplay::HMDOffsetMessageReceived,this);

        if (!vive_.Init(ros::package::getPath("rviz_vive_plugin") + "/media/actions.json"))
        {
            ROS_ERROR_STREAM_NAMED("vive_display", "Failed to initialize Vive");
            return;
        }

        InitOgre();
    }

    void ViveDisplay::onDisable()
    {
        pubs_.LeftHand.shutdown();
        pubs_.RightHand.shutdown();
        //pubs_.HTC.shutdown();
        subs_.VibrationLeft.shutdown();
        subs_.VibrationRight.shutdown();
        scene_manager_->destroyCamera(ogre_.Left.EyeCamera);
        scene_manager_->destroyCamera(ogre_.Right.EyeCamera);
        scene_manager_->destroySceneNode(ogre_.HMD.Node);
        if(b_enable_htc_vive_view_window)
        {
            ogre_.RenderWindow->removeAllListeners();
            ogre_.RenderWindow->removeAllViewports();
            delete ogre_.RenderWidget;
        }
    }

    void ViveDisplay::InitProperties()
    {

        properties_.RootFrame=new rviz::TfFrameProperty(
                    "Root frame",
                    rviz::TfFrameProperty::FIXED_FRAME_STRING,
                    "Root frame for each transform",
                    this,
                    0,
                    true);
    /*
        properties_.RootFrame = new rviz::TfFrameProperty(
                    "Root frame",
                    "server/camera_rviz",
                    "Root frame for each transform",
                    this,
                    0,
                    true);
    */
        properties_.Left.Frame=new rviz::StringProperty(
                    "Left controller frame",
                    "htc_vive_left_controller",
                    "Left controller frame",
                    this);

        properties_.Right.Frame=new rviz::StringProperty(
                    "Right controller frame",
                    "htc_vive_right_controller",
                    "Right controller frame",
                    this);

        properties_.Left.StateTopic=new rviz::StringProperty(
                    "Left controller state topic",
                    "/wai_world/world/htc_vive/left_controller/state",
                    "Left controller state topic",
                    this,
                    SLOT(LeftStateTopicPropertyChanged()));

        properties_.Right.StateTopic=new rviz::StringProperty(
                    "Right controller state topic",
                    "/wai_world/world/htc_vive/right_controller/state",
                    "Right controller state topic",
                    this,
                    SLOT(RightStateTopicPropertyChanged()));

        properties_.Left.VibrationTopic=new rviz::RosTopicProperty(
                    "Left controller vibration topic",
                    "/wai_world/world/htc_vive/left_controller_vibration",
                    QString::fromStdString(ros::message_traits::datatype<rviz_vive_plugin_msgs::ControllerVibration>()),
                    "Left controller vibration topic",
                    this,
                    SLOT(LeftVibrationTopicPropertyChanged()));

        properties_.Right.VibrationTopic=new rviz::RosTopicProperty(
                    "Right controller vibration topic",
                    "/wai_world/world/htc_vive/right_controller_vibration",
                    QString::fromStdString(ros::message_traits::datatype<rviz_vive_plugin_msgs::ControllerVibration>()),
                    "Right controller vibration topic",
                    this,
                    SLOT(RightVibrationTopicPropertyChanged()));

        properties_.HMD.Frame=new rviz::StringProperty(
                    "HMD frame",
                    "htc_vive_hmd",
                    "HMD frame",
                    this);

        // Subscribe to camera Rviz pose as offset for HMD
        std::string m_s_rep_id="";
        std::string s_hmd_offset_topic="";
        ros::get_environment_variable(m_s_rep_id,"WAI_OA_AUDIENCE_ID");
        s_hmd_offset_topic="/wai_world/oa"+m_s_rep_id+"/camera_rviz/pose";
        //s_hmd_offset_topic="/wai_world/world/htc_vive/hmd_offset";
        properties_.OffsetTopic = new rviz::RosTopicProperty(
                "HMD offset topic",QString::fromStdString(s_hmd_offset_topic),
                QString::fromStdString(ros::message_traits::datatype<geometry_msgs::PoseStamped>()),
                "HMD offset topic", this, SLOT(OffsetTopicPropertyChanged()));

        properties_.RootFrame->setFrameManager(context_->getFrameManager());
    }

    void ViveDisplay::InitOgre()
    {
        ogre_.HMD.Node=scene_manager_->getRootSceneNode()->createChildSceneNode();

        ogre_.Left.EyeCamera=scene_manager_->createCamera("left_camera");
        ogre_.Left.EyeCamera->setAutoAspectRatio(true);
        ogre_.HMD.Node->attachObject(ogre_.Left.EyeCamera);

        ogre_.Right.EyeCamera=scene_manager_->createCamera("right_camera");
        ogre_.Right.EyeCamera->setAutoAspectRatio(true);
        ogre_.HMD.Node->attachObject(ogre_.Right.EyeCamera);

        ogre_.Left.EyeCamera->setPosition(Convert(vive_.LeftEyeTransform()).getTrans());
        ogre_.Left.EyeCamera->setOrientation(Ogre::Quaternion::IDENTITY);

        ogre_.Right.EyeCamera->setPosition(Convert(vive_.RightEyeTransform()).getTrans());
        ogre_.Right.EyeCamera->setOrientation(Ogre::Quaternion::IDENTITY);

        auto textureManager=dynamic_cast<Ogre::GLTextureManager *>(Ogre::TextureManager::getSingletonPtr());

        ogre_.Left.Texture=textureManager->createManual(
                "left_texture",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D,
                vive_.RenderWidth(),
                vive_.RenderHeight(),
                0,
                Ogre::PF_R8G8B8,
                Ogre::TU_RENDERTARGET,
                nullptr,
                false);

        ogre_.Right.Texture=textureManager->createManual(
                "right_texture",
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                Ogre::TEX_TYPE_2D,
                vive_.RenderWidth(),
                vive_.RenderHeight(),
                0,
                Ogre::PF_R8G8B8,
                Ogre::TU_RENDERTARGET,
                nullptr,
                false);

        ogre_.Left.Texture->getCustomAttribute("GLID",&ogre_.Left.TextureGLID);
        ogre_.Right.Texture->getCustomAttribute("GLID",&ogre_.Right.TextureGLID);

        ogre_.Left.Viewport=ogre_.Left.Texture->getBuffer()->getRenderTarget()->addViewport(ogre_.Left.EyeCamera);
        ogre_.Right.Viewport=ogre_.Right.Texture->getBuffer()->getRenderTarget()->addViewport(ogre_.Right.EyeCamera);

        static const float nearClip=0.01f;
        static const float farClip=1000.0f;

        const auto &projLeft=Convert(vive_.LeftEyeProjectionMatrix(nearClip, farClip));
        const auto &projRight=Convert(vive_.RightEyeProjectionMatrix(nearClip, farClip));

        ogre_.Left.EyeCamera->setCustomProjectionMatrix(true,projLeft);
        ogre_.Right.EyeCamera->setCustomProjectionMatrix(true,projRight);

        vive_.RegisterGLTextures((void *)ogre_.Left.TextureGLID,(void *)ogre_.Right.TextureGLID);

        // OGRE Render window
        if(b_enable_htc_vive_view_window)
        {
            ogre_.RenderWidget=new rviz::RenderWidget(rviz::RenderSystem::get());
            ogre_.RenderWidget->setWindowTitle("Vive");
            ogre_.RenderWidget->setParent(context_->getWindowManager()->getParentWindow());
            ogre_.RenderWidget->setWindowFlags(Qt::Window|Qt::CustomizeWindowHint|Qt::WindowTitleHint|Qt::WindowMaximizeButtonHint);
            ogre_.RenderWidget->setVisible(true);
            ogre_.RenderWindow=ogre_.RenderWidget->getRenderWindow();
            ogre_.RenderWindow->setVisible(true);
            ogre_.RenderWindow->setAutoUpdated(false);
            Ogre::Viewport *port;
            port=ogre_.RenderWindow->addViewport(ogre_.Left.EyeCamera, 0, 0.0, 0.0f, 0.5f, 1.0f);
            port->setClearEveryFrame(true);
            port->setBackgroundColour(Ogre::ColourValue::Black);
            port=ogre_.RenderWindow->addViewport(ogre_.Right.EyeCamera, 1, 0.5f, 0.0f, 0.5f, 1.0f);
            port->setClearEveryFrame(true);
            port->setBackgroundColour(Ogre::ColourValue::Black);
        }
    }

    void ViveDisplay::update(float wall_dt, float ros_dt)
    {
        const auto now=ros::Time::now();
        const auto dt1=now-leftVibrationTimestamp_;
        const auto dt2=now-rightVibrationTimestamp_;

        if(dt1.toSec()<leftVibrationDuration_)
        {
            vive_.VibrateLeft(0.0f,1.0f,4.0f,1.0f);
        }
        if(dt2.toSec()<rightVibrationDuration_)
        {
            vive_.VibrateRight(0.0f,1.0f,4.0f,1.0f);
        }
        if(!vive_.ReadAll())
        {
            ROS_ERROR_STREAM_NAMED("vive_display","Failed to all Vive data");
        }

        //Vive::HMD viveHMD{};
        if(vive_.ReadHMD(viveHMD))
        {
            hmd = Convert(viveHMD);
            stf_hmd.setOrigin(tf::Vector3(hmd.Pose.Position.x,hmd.Pose.Position.y,hmd.Pose.Position.z));
            if(b_enable_motion_smooting)
            {
                qua_hmd_pose_avg.setW((hmd.Pose.Orientation.w+qua_hmd_pose_avg_old.getW())/2.0);
                qua_hmd_pose_avg.setX((hmd.Pose.Orientation.x+qua_hmd_pose_avg_old.getX())/2.0);
                qua_hmd_pose_avg.setY((hmd.Pose.Orientation.y+qua_hmd_pose_avg_old.getY())/2.0);
                qua_hmd_pose_avg.setZ((hmd.Pose.Orientation.z+qua_hmd_pose_avg_old.getZ())/2.0);
                stf_hmd.setRotation(qua_hmd_pose_avg);
                qua_hmd_pose_avg_old=qua_hmd_pose_avg;
            }
            else
            {
                stf_hmd.setRotation(tf::Quaternion(hmd.Pose.Orientation.x,hmd.Pose.Orientation.y,hmd.Pose.Orientation.z,hmd.Pose.Orientation.w));
            }
            stf_offset=stf_offset_root_frame*stf_offset_message*stf_offset_message_straighten*stf_hmd;
            hmd.Pose.Position.x=stf_offset.getOrigin().getX();
            hmd.Pose.Position.y=stf_offset.getOrigin().getY();
            hmd.Pose.Position.z=stf_offset.getOrigin().getZ();
            hmd.Pose.Orientation.w=stf_offset.getRotation().getW();
            hmd.Pose.Orientation.x=stf_offset.getRotation().getX();
            hmd.Pose.Orientation.y=stf_offset.getRotation().getY();
            hmd.Pose.Orientation.z=stf_offset.getRotation().getZ();
            ogre_.HMD.Node->setPosition(hmd.Pose.Position);
            ogre_.HMD.Node->setOrientation(hmd.Pose.Orientation);
            /*
            geometry_msgs::Pose msg;
            msg.position.x = hmd.Pose.Position.x;
            msg.position.y = hmd.Pose.Position.y;
            msg.position.z = hmd.Pose.Position.z;
            msg.orientation.x = hmd.Pose.Orientation.x;
            msg.orientation.y = hmd.Pose.Orientation.y;
            msg.orientation.z = hmd.Pose.Orientation.z;
            msg.orientation.w = hmd.Pose.Orientation.w;
            pubs_.HTC.publish(msg);
            */
            const auto &transform = BuildTransform(hmd.Pose,properties_.RootFrame->getFrame().toStdString(),properties_.HMD.Frame->getStdString());
            tb_.sendTransform(transform);
        }
        else
        {
            // ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read HMD from Vive");
        }

        Vive::Controller viveLeftController{};
        if(vive_.ReadLeftController(viveLeftController))
        {
            controller_left = Convert(viveLeftController);
            stf_controller_left.setOrigin(tf::Vector3(controller_left.Pose.Position.x,controller_left.Pose.Position.y,controller_left.Pose.Position.z));
            stf_controller_left.setRotation(tf::Quaternion(controller_left.Pose.Orientation.x,controller_left.Pose.Orientation.y,controller_left.Pose.Orientation.z,controller_left.Pose.Orientation.w));
            stf_offset=stf_offset_root_frame*stf_offset_message*stf_offset_message_straighten*stf_controller_left;
            controller_left.Pose.Position.x=stf_offset.getOrigin().getX();
            controller_left.Pose.Position.y=stf_offset.getOrigin().getY();
            controller_left.Pose.Position.z=stf_offset.getOrigin().getZ();
            controller_left.Pose.Orientation.w=stf_offset.getRotation().getW();
            controller_left.Pose.Orientation.x=stf_offset.getRotation().getX();
            controller_left.Pose.Orientation.y=stf_offset.getRotation().getY();
            controller_left.Pose.Orientation.z=stf_offset.getRotation().getZ();

            // Republish corrected poses and TFs
            const auto &msg = BuildMessage(controller_left, "htc_vive_left_controller");
            pubs_.LeftHand.publish(msg);
            const auto &transform = BuildTransform(controller_left.Pose,properties_.RootFrame->getFrame().toStdString(),properties_.Left.Frame->getStdString());
            tb_.sendTransform(transform);
        }
        else
        {
            // ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read left controller data from Vive");
        }

        Vive::Controller viveRightController{};
        if(vive_.ReadRightController(viveRightController))
        {
            controller_right = Convert(viveRightController);
            stf_controller_right.setOrigin(tf::Vector3(controller_right.Pose.Position.x,controller_right.Pose.Position.y,controller_right.Pose.Position.z));
            stf_controller_right.setRotation(tf::Quaternion(controller_right.Pose.Orientation.x,controller_right.Pose.Orientation.y,controller_right.Pose.Orientation.z,controller_right.Pose.Orientation.w));
            stf_offset=stf_offset_root_frame*stf_offset_message*stf_offset_message_straighten*stf_controller_right;
            controller_right.Pose.Position.x=stf_offset.getOrigin().getX();
            controller_right.Pose.Position.y=stf_offset.getOrigin().getY();
            controller_right.Pose.Position.z=stf_offset.getOrigin().getZ();
            controller_right.Pose.Orientation.w=stf_offset.getRotation().getW();
            controller_right.Pose.Orientation.x=stf_offset.getRotation().getX();
            controller_right.Pose.Orientation.y=stf_offset.getRotation().getY();
            controller_right.Pose.Orientation.z=stf_offset.getRotation().getZ();

            // Republish corrected poses and TFs
            const auto &msg = BuildMessage(controller_right, "htc_vive_right_controller");
            pubs_.RightHand.publish(msg);
            const auto &transform=BuildTransform(controller_right.Pose,properties_.RootFrame->getFrame().toStdString(),properties_.Right.Frame->getStdString());
            tb_.sendTransform(transform);
        }
        else
        {
            ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read right controller data from Vive");
        }

        //ogre_.RenderWindow->update(true);
        vive_.SubmitGLTextures();
    }

    void ViveDisplay::LeftStateTopicPropertyChanged()
    {
        pubs_.LeftHand.shutdown();
        pubs_.LeftHand=node_.advertise<rviz_vive_plugin_msgs::Controller>(properties_.Left.StateTopic->getStdString(),1);
    }

    void ViveDisplay::RightStateTopicPropertyChanged()
    {
        pubs_.RightHand.shutdown();
        pubs_.RightHand=node_.advertise<rviz_vive_plugin_msgs::Controller>(properties_.Right.StateTopic->getStdString(),1);
    }

    void ViveDisplay::LeftVibrationTopicPropertyChanged()
    {
        subs_.VibrationLeft.shutdown();
        subs_.VibrationLeft=node_.subscribe(properties_.Left.VibrationTopic->getStdString(),1,&ViveDisplay::LeftVibrationMessageReceived,this);
    }

    void ViveDisplay::RightVibrationTopicPropertyChanged()
    {
        subs_.VibrationRight.shutdown();
        subs_.VibrationRight=node_.subscribe(properties_.Right.VibrationTopic->getStdString(),1,&ViveDisplay::RightVibrationMessageReceived,this);
    }

    void ViveDisplay::OffsetTopicPropertyChanged()
    {
        subs_.HMDOffset.shutdown();
        subs_.HMDOffset=node_.subscribe(properties_.OffsetTopic->getStdString(),1,&ViveDisplay::HMDOffsetMessageReceived,this);
    }

    void ViveDisplay::LeftVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg)
    {
        leftVibrationTimestamp_=ros::Time::now();
        leftVibrationDuration_=msg->durationSeconds;
        vive_.VibrateLeft(msg->startSecondsFromNow,msg->durationSeconds,msg->frequency, msg->amplitude);
    }

    void ViveDisplay::RightVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg)
    {
        rightVibrationTimestamp_=ros::Time::now();
        rightVibrationDuration_=msg->durationSeconds;
        vive_.VibrateRight(msg->startSecondsFromNow,msg->durationSeconds,msg->frequency, msg->amplitude);
    }

    void ViveDisplay::HMDOffsetMessageReceived(const geometry_msgs::PoseStampedPtr &msg)
    {
        stf_offset_message.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z));
        stf_offset_message.setRotation(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
        s_frame_id_msg=msg->header.frame_id;
        if(s_frame_id_msg.compare(s_frame_id_msg_old))
        {
            try
            {
                tl_.waitForTransform(properties_.RootFrame->getFrame().toStdString(),s_frame_id_msg,ros::Time(0),ros::Duration(1.0));
                tl_.lookupTransform(properties_.RootFrame->getFrame().toStdString(),s_frame_id_msg,ros::Time(0),stf_offset_root_frame);
                /* ROS_WARN_STREAM("Updated TF..." <<
                                stf_offset_root_frame.getOrigin().getX() << ", " <<
                                stf_offset_root_frame.getOrigin().getX() << ", " <<
                                stf_offset_root_frame.getOrigin().getX() << ", " <<
                                stf_offset_root_frame.getRotation().getW() << ", " <<
                                stf_offset_root_frame.getRotation().getX() << ", " <<
                                stf_offset_root_frame.getRotation().getY() << ", " <<
                                stf_offset_root_frame.getRotation().getZ() << ", "
                                );*/
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
            s_frame_id_msg_old=s_frame_id_msg;
        }
    }

} // namespace rviz_vive_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_vive_plugin::ViveDisplay, rviz::Display)
