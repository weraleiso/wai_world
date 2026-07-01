#ifndef RVIZ_PLUGIN_WAI_VIRTUAL_CAMERA_H
#define RVIZ_PLUGIN_WAI_VIRTUAL_CAMERA_H

#include<rviz/view_controller.h>
#include<rviz/view_manager.h>
#include<rviz/frame_manager.h>
#include<rviz/render_panel.h>
#include<rviz/load_resource.h>
#include<rviz/uniform_string_stream.h>
#include<rviz/display_context.h>
#include<rviz/viewport_mouse_event.h>
#include<rviz/geometry.h>
#include<rviz/ogre_helpers/shape.h>
#include<rviz/properties/float_property.h>
#include<rviz/properties/vector_property.h>
#include<rviz/properties/bool_property.h>
#include<rviz/properties/int_property.h>
#include<rviz/properties/tf_frame_property.h>
#include<rviz/properties/editable_enum_property.h>
#include<rviz/properties/ros_topic_property.h>

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

#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Duration.h>
#include<geometry_msgs/Pose.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_datatypes.h>
#include<view_controller_msgs/CameraMovement.h>
#include<view_controller_msgs/CameraPlacement.h>
#include<view_controller_msgs/CameraTrajectory.h>



namespace rviz_plugin_wai_virtual_camera
{
    class RVizPluginWAIVirtualCamera : public rviz::ViewController
    {
        Q_OBJECT
        public:
        RVizPluginWAIVirtualCamera();
        virtual ~RVizPluginWAIVirtualCamera();

        virtual void handleMouseEvent(rviz::ViewportMouseEvent& evt);
        virtual void onInitialize();
        virtual void onActivate();
        virtual void lookAt(const Ogre::Vector3& point);
        virtual void reset();
        virtual void transitionFrom(ViewController* previous_view);

        void MoveCameraEyeFocus(float x,float y,float z);
        void MoveCameraEye(float x,float y,float z);
        void SetCameraYawPitchRoll(float yaw,float pitch,float roll);
        void PublishCameraPose();
        void PublishCameraTransitionFinished();
        void orbitCameraTo(const Ogre::Vector3& point);
        void moveEyeWithFocusTo(const Ogre::Vector3& point);


        protected Q_SLOTS:
        void UpdateCameraPlacementTopic();
        void UpdateCameraTrajectoryTopic();
        void UpdateCameraTransitionPauseTopic();
        void UpdateCameraTransitionFinishedTopic();
        void UpdateCameraPoseTopic();
        void UpdateCameraLiveviewTopic();

        virtual void onFramePropertyChanged();
        virtual void onDistancePropertyChanged();
        virtual void onFocusPropertyChanged();
        virtual void onEyePropertyChanged();
        virtual void onUpPropertyChanged();
        virtual void onLiveviewPropertyChanged();


        protected:
        ros::NodeHandle m_hdl_node;
        image_transport::ImageTransport m_hdl_it;
        //tf::TransformBroadcaster brc_transforms; // Publish TFs to visualize camera in RViz

        ros::Subscriber m_sub_plc_camera_placement;
        ros::Subscriber m_sub_trj_camera_trajectory;
        ros::Subscriber m_sub_dur_camera_transition_pause;
        ros::Publisher m_pub_pst_camera;
        ros::Publisher m_pub_bol_camera_transition_finished;
        image_transport::Publisher m_pub_img_camera_liveview;

        std::vector<view_controller_msgs::CameraMovement> vec_camera_movements;

        rviz::EditableEnumProperty* m_prp_str_mouse_control_mode;
        rviz::BoolProperty* m_prp_bol_mouse_control_enabled;
        rviz::BoolProperty* m_prp_bol_camera_lock_up_axis_enabled;
        rviz::BoolProperty* m_prp_bol_camera_pose_enabled;
        rviz::BoolProperty* m_prp_bol_camera_liveview_enabled;
        rviz::FloatProperty* m_prp_f_camera_distance; ///< The camera's distance from the focal point
        rviz::VectorProperty* m_prp_vc3_camera_eye; ///< The position of the camera.
        rviz::VectorProperty* m_prp_vc3_camera_focus; ///< The point around which the camera "orbits".
        rviz::VectorProperty* m_prp_vc3_camera_up_axis; ///< The up vector for the camera.
        rviz::FloatProperty* m_prp_f_camera_transition_time; ///< A default time for any animation requests.
        rviz::RosTopicProperty* m_prp_top_camera_placement;
        rviz::RosTopicProperty* m_prp_top_camera_trajectory;
        rviz::RosTopicProperty* m_prp_top_camera_transition_pause;
        rviz::RosTopicProperty* m_prp_top_camera_transition_finished;
        rviz::RosTopicProperty* m_prp_top_camera_liveview;
        rviz::RosTopicProperty* m_prp_top_camera_pose;
        rviz::IntProperty* m_prp_i_camera_window_width; ///< The width of the rviz visualization window in pixels.
        rviz::IntProperty* m_prp_i_camera_window_height; ///< The height of the rviz visualization window in pixels.

        rviz::TfFrameProperty* m_prp_frm_camera_target;
        Ogre::SceneNode* m_ogr_node_scene;

        Ogre::Quaternion m_qua_pose_reference; ///< Used to store the orientation of the attached frame relative to <Fixed Frame>
        Ogre::Vector3 m_vc3_pose_reference; ///< Used to store the position of the attached frame relative to <Fixed Frame>

        ros::WallTime m_tim_camera_transition_start;
        ros::WallDuration m_dur_camera_transition_pause;

        rviz::Shape* m_ogr_shape_focal;

        std::string m_s_camera_rviz_namespace;

        QCursor m_ico_mouse_control_disabled;

        std::string m_s_rep_id;
        float m_f_camera_distance;
        int m_i_rep_id;
        int m_i_target_fps;
        int m_i_frames_rendered;
        int m_i_view_diff_x;
        int m_i_view_diff_y;
        bool m_b_mouse_drag_enabled;
        bool m_b_transition_enabled;
        bool m_b_frame_by_frame_enabled;

        void SetSubAndPubTopics();
        void SetCameraLiveviewImageResolution();

        virtual void update(float dt, float ros_dt);

        /** @brief Convert the relative progress in time to the corresponding relative progress in space wrt. the interpolation speed profile.
        *
        * Example:
        *   The camera has to move from point A to point B in a duration D.
        *   The camera should accelerate slowly and arrive at full speed - RISING speed profile.
        *   At exactly half of the duration D the camera wouldn't be right in the middle between A and B, because it needed
        *   time to accelerate.
        *   This method converts the relative progress in time specified by a number between zero and one, to the relative
        *   progress in space as a number between zero and one.
        *
        * Interpolation speed profiles:
        * RISING    = 0 # Speed of the camera rises smoothly - resembles the first quarter of a sinus wave.
        * DECLINING = 1 # Speed of the camera declines smoothly - resembles the second quarter of a sinus wave.
        * FULL      = 2 # Camera is always at full speed - depending on transition_duration.
        * WAVE      = 3 # RISING and DECLINING concatenated in one movement.
        *
        * @params[in] relative_progress_in_time  the relative progress in time, between 0 and 1.
        * @params[in] ui8_movement_mode        speed profile.
        *
        * @returns relative progress in space as a float between 0 and 1.
        */
        float CalculateCameraTransitionIteration(double relative_progress_in_time,uint8_t ui8_movement_mode);
        void PublishCameraLiveviewImage();
        void UpdateCameraAttachedSceneNode();

        void cb_sub_plc_camera_placement(const view_controller_msgs::CameraPlacementConstPtr &cp_ptr);
        void cb_sub_trj_camera_trajectory(const view_controller_msgs::CameraTrajectoryConstPtr& ct_ptr);
        void cb_sub_dur_camera_transition_pause(const std_msgs::Duration::ConstPtr& pause_duration_msg);

        void transformCameraToAttachedFrame(geometry_msgs::PointStamped& eye,
                                          geometry_msgs::PointStamped& focus,
                                          geometry_msgs::Vector3Stamped& up);

        /** @brief Begins a camera movement animation to the given goal point.
        *
        * @param[in] eye                     goal position of camera.
        * @param[in] focus                   goal focus point of camera.
        * @param[in] up                      goal vector of camera pointing up.
        * @param[in] transition_duration     duration needed for transition.
        * @param[in] ui8_movement_mode     the interpolation speed profile.
        */
        void InitCameraTransition(view_controller_msgs::CameraMovement cam_movement);
        void AbortCameraTransition();
        void UpdateCameraPose();

        Ogre::Vector3 fixedFrameToAttachedLocal(const Ogre::Vector3 &v);
        Ogre::Vector3 attachedLocalToFixedFrame(const Ogre::Vector3 &v);
        float GetCameraEyeFocusDistance();
    };

}

#endif // RVIZ_PLUGIN_WAI_VIRTUAL_CAMERA_H
