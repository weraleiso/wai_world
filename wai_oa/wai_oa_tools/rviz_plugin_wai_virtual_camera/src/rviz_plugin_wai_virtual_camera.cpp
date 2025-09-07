#include "rviz_plugin_wai_virtual_camera.h"



namespace rviz_plugin_wai_virtual_camera
{
    using namespace view_controller_msgs;
    using namespace rviz;

    // Strings for selecting control mode styles
    static const std::string MODE_ORBIT="Orbit";
    static const std::string MODE_FPS="FPS";

    // Limits to prevent orbit controller singularity, but not currently used.
    static const Ogre::Radian PITCH_LIMIT_LOW=Ogre::Radian(0.02);
    static const Ogre::Radian PITCH_LIMIT_HIGH=Ogre::Radian(Ogre::Math::PI-0.02);



    RVizPluginWAIVirtualCamera::RVizPluginWAIVirtualCamera():m_hdl_node(),m_hdl_it(m_hdl_node)
    {
        ros::get_environment_variable(m_s_rep_id,"WAI_OA_AUDIENCE_ID");
        m_s_camera_rviz_namespace="/wai_world/oa"+m_s_rep_id+"/camera_rviz/";

        m_b_transition_enabled=false;
        m_b_mouse_drag_enabled=false;
        m_b_frame_by_frame_enabled=false;
        m_i_target_fps=20;
        m_i_frames_rendered=0;
        m_i_view_diff_x=0;
        m_i_view_diff_y=0;
        m_dur_camera_transition_pause.fromSec(0.0);
        m_f_camera_distance=0.0;

        m_ico_mouse_control_disabled=makeIconCursor("package://rviz/icons/forbidden.svg");

        m_prp_bol_mouse_control_enabled=new BoolProperty("Mouse Control Enabled",true,"Enable mouse control of the camera.",this);
        m_prp_str_mouse_control_mode=new EditableEnumProperty("Control Mode",QString::fromStdString(MODE_ORBIT),"Select mouse control mode.",this);
        m_prp_str_mouse_control_mode->addOptionStd(MODE_ORBIT);
        m_prp_str_mouse_control_mode->addOptionStd(MODE_FPS);

        m_prp_i_camera_window_width=new IntProperty("Liveview Width",960,"Camera liveview width in pixels.",this);
        m_prp_i_camera_window_height=new IntProperty("Liveview Height",540,"Camera liveview height in pixels.",this);

        m_prp_frm_camera_target=new TfFrameProperty("Target Frame",TfFrameProperty::FIXED_FRAME_STRING,"Target frame of the camera.",this,NULL,true,SLOT(onFramePropertyChanged()));

        m_prp_vc3_camera_eye=new VectorProperty("Eye",Ogre::Vector3::UNIT_SCALE,"Eye position of the camera.",this,SLOT(onEyePropertyChanged()));
        m_prp_vc3_camera_focus=new VectorProperty("Focus",Ogre::Vector3::ZERO,"Focus position of the camera.",this,SLOT(onFocusPropertyChanged()));
        m_prp_vc3_camera_up_axis=new VectorProperty("Up Axis",Ogre::Vector3::UNIT_Z,"The vector representing the up axis of the camera.",this);
        m_prp_f_camera_distance=new FloatProperty("Distance",1.0,"The distance between the camera position and the focus point.",this,SLOT(onDistancePropertyChanged()));//);
        m_prp_f_camera_distance->setMin(0.1);

        m_prp_bol_camera_lock_up_axis_enabled=new BoolProperty("Lock Up Axis",true,"If enabled, roll axis Z is locked.",this,SLOT(onUpPropertyChanged()));
        m_prp_bol_camera_pose_enabled=new BoolProperty("Publish Camera Pose",true,"If enabled, publish liveview of the camera transiton.",this);
        m_prp_bol_camera_liveview_enabled=new BoolProperty("Publish Camera Liveview",true,"If enabled, publish liveview of the camera transiton.",this,SLOT(onLiveviewPropertyChanged()));

        m_prp_f_camera_transition_time=new FloatProperty("Camera Transition Time",3.0,"The default time to use for camera transitions.",this);

        m_prp_top_camera_placement=new RosTopicProperty("Camera Placement Topic",QString::fromStdString(m_s_camera_rviz_namespace+"placement"),QString::fromStdString(ros::message_traits::datatype<view_controller_msgs::CameraPlacement>()),"Topic for subscribing to CameraPlacement messages.",this,SLOT(UpdateCameraPlacementTopic()));
        m_prp_top_camera_trajectory=new RosTopicProperty("Camera Trajectory Topic",QString::fromStdString(m_s_camera_rviz_namespace+"trajectory"),QString::fromStdString(ros::message_traits::datatype<view_controller_msgs::CameraTrajectory>()),"Topic for subscribing to CameraTrajectory messages.",this,SLOT(UpdateCameraTrajectoryTopic()));
        m_prp_top_camera_transition_pause=new RosTopicProperty("Camera Pause Topic",QString::fromStdString(m_s_camera_rviz_namespace+"transition_pause"),QString::fromStdString(ros::message_traits::datatype<std_msgs::Duration>()),"Topic for subscribing to pause duration messages.",this,SLOT(UpdateCameraTransitionPauseTopic()));
        m_prp_top_camera_transition_finished=new RosTopicProperty("Camera Finished Topic",QString::fromStdString(m_s_camera_rviz_namespace+"transition_finished"),QString::fromStdString(ros::message_traits::datatype<std_msgs::Bool>()),"Topic for publishing finished messages.",this,SLOT(UpdateCameraTransitionFinishedTopic()));
        m_prp_top_camera_pose=new RosTopicProperty("Camera Pose Topic",QString::fromStdString(m_s_camera_rviz_namespace+"pose"),QString::fromStdString(ros::message_traits::datatype<geometry_msgs::Pose>()),"Topic for publishing camera pose.",this,SLOT(UpdateCameraPoseTopic()));
        m_prp_top_camera_liveview=new RosTopicProperty("Camera Liveview Topic",QString::fromStdString(m_s_camera_rviz_namespace+"liveview"),QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),"Topic for publishing camera liveview images.",this,SLOT(UpdateCameraLiveviewTopic()));
    }

    RVizPluginWAIVirtualCamera::~RVizPluginWAIVirtualCamera()
    {
        delete m_ogr_shape_focal;
        context_->getSceneManager()->destroySceneNode(m_ogr_node_scene);
    }

    void RVizPluginWAIVirtualCamera::onInitialize()
    {
        m_prp_frm_camera_target->setFrameManager(context_->getFrameManager());
        m_ogr_node_scene=context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
        camera_->detachFromParent();
        m_ogr_node_scene->attachObject(camera_);
        camera_->setProjectionType(Ogre::PT_PERSPECTIVE);

        m_ogr_shape_focal=new Shape(Shape::Sphere,context_->getSceneManager(),m_ogr_node_scene);
        m_ogr_shape_focal->setScale(Ogre::Vector3(0.05f,0.05f,0.01f));
        m_ogr_shape_focal->setColor(1.0f,1.0f,0.0f,0.5f);
        m_ogr_shape_focal->getRootNode()->setVisible(false);
    }

    void RVizPluginWAIVirtualCamera::onActivate()
    {
        reset();
        UpdateCameraAttachedSceneNode();
        SetCameraLiveviewImageResolution();
        SetSubAndPubTopics();
    }

    void RVizPluginWAIVirtualCamera::reset()
    {
        m_prp_bol_mouse_control_enabled->setBool(true);
        m_prp_str_mouse_control_mode->setStdString(MODE_ORBIT);
        m_prp_vc3_camera_eye->setVector(Ogre::Vector3(-5.0,10.0,25.0)); // Old starting pose 3.85,1.70,12.65
        m_prp_vc3_camera_focus->setVector(Ogre::Vector3(0.0,0.0,0.0)); // Old starting pose -0.90,1.70,2.0
        m_prp_vc3_camera_up_axis->setVector(Ogre::Vector3::UNIT_Z);
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }

    void RVizPluginWAIVirtualCamera::onFramePropertyChanged()
    {
        Ogre::Vector3 old_reference_position=m_ogr_node_scene->getPosition();
        Ogre::Quaternion old_reference_orientation=m_ogr_node_scene->getOrientation();
        UpdateCameraAttachedSceneNode();

        Ogre::Vector3 fixed_frame_focus_position=old_reference_position+old_reference_orientation*m_prp_vc3_camera_focus->getVector();
        Ogre::Vector3 fixed_frame_eye_position=old_reference_position+old_reference_orientation*m_prp_vc3_camera_eye->getVector();
        Ogre::Vector3 new_focus_position=fixedFrameToAttachedLocal(fixed_frame_focus_position);
        Ogre::Vector3 new_eye_position=fixedFrameToAttachedLocal(fixed_frame_eye_position);
        Ogre::Vector3 new_up_vector=m_qua_pose_reference.Inverse()*old_reference_orientation*m_prp_vc3_camera_up_axis->getVector();

        m_prp_vc3_camera_focus->setVector(new_focus_position);
        m_prp_vc3_camera_eye->setVector(new_eye_position);
        //m_prp_vc3_camera_up_axis->setVector(m_prp_bol_camera_lock_up_axis_enabled->getBool() ? Ogre::Vector3::UNIT_Z : new_up_vector);
        m_prp_vc3_camera_up_axis->setVector(new_up_vector);
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());

        // force orientation to match up vector; first call doesn't actually change the quaternion
        //camera_->setFixedYawAxis(true, m_qua_pose_reference * m_prp_vc3_camera_up_axis->getVector());
        //camera_->setDirection( m_qua_pose_reference * (m_prp_vc3_camera_focus->getVector() - m_prp_vc3_camera_eye->getVector()));
        camera_->setFixedYawAxis(m_prp_bol_camera_lock_up_axis_enabled->getBool(),m_qua_pose_reference*m_prp_vc3_camera_up_axis->getVector());
    }
    void RVizPluginWAIVirtualCamera::onEyePropertyChanged()
    {
        camera_->setPosition(m_prp_vc3_camera_eye->getVector());
        camera_->setDirection(m_qua_pose_reference*(m_prp_vc3_camera_focus->getVector()-m_prp_vc3_camera_eye->getVector()));
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }
    void RVizPluginWAIVirtualCamera::onFocusPropertyChanged()
    {
        camera_->setDirection(m_qua_pose_reference*(m_prp_vc3_camera_focus->getVector()-m_prp_vc3_camera_eye->getVector()));
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }
    void RVizPluginWAIVirtualCamera::onDistancePropertyChanged()
    {
        // Do nothing...
    }
    void RVizPluginWAIVirtualCamera::onUpPropertyChanged()
    {
        camera_->setFixedYawAxis(m_prp_bol_camera_lock_up_axis_enabled->getBool(),m_qua_pose_reference*m_prp_vc3_camera_up_axis->getVector());
        camera_->setDirection(m_qua_pose_reference*(m_prp_vc3_camera_focus->getVector()-m_prp_vc3_camera_eye->getVector()));
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }
    void RVizPluginWAIVirtualCamera::onLiveviewPropertyChanged()
    {
        SetCameraLiveviewImageResolution();
    }

    void RVizPluginWAIVirtualCamera::handleMouseEvent(ViewportMouseEvent& event)
    {
        if(!m_prp_bol_mouse_control_enabled->getBool())
        {
            setCursor(m_ico_mouse_control_disabled);
            return;
        }
        else if(event.shift())
        {
            // Do nothing...
        }
        else if(event.control())
        {
            // Do nothing...
        }
        else
        {
            // Do nothing...
        }

        if(event.type==QEvent::MouseButtonPress)
        {
            AbortCameraTransition();
            m_ogr_shape_focal->getRootNode()->setVisible(true);
            m_b_mouse_drag_enabled=true;
        }
        else if(event.type==QEvent::MouseButtonRelease)
        {
            m_ogr_shape_focal->getRootNode()->setVisible(false);
            m_b_mouse_drag_enabled=false;
        }
        else if(m_b_mouse_drag_enabled && event.type==QEvent::MouseMove)
        {
            m_i_view_diff_x=event.x-event.last_x;
            m_i_view_diff_y=event.y-event.last_y;
        }

        // regular left-button drag
        if(event.left() && !event.shift())
        {
            setCursor(Rotate3D);
            SetCameraYawPitchRoll(-m_i_view_diff_x*0.005,-m_i_view_diff_y*0.005,0);
        }
        // middle or shift-left drag
        else if(event.middle() || (event.shift() && event.left()))
        {
            setCursor(MoveXY);
            if(m_prp_str_mouse_control_mode->getStdString()==MODE_ORBIT)  // Orbit style
            {
                float fovY=camera_->getFOVy().valueRadians();
                float fovX=2.0f*atan(tan(fovY/2.0f)*camera_->getAspectRatio());

                int width=camera_->getViewport()->getActualWidth();
                int height=camera_->getViewport()->getActualHeight();

                MoveCameraEyeFocus(-((float)m_i_view_diff_x/(float)width)*m_f_camera_distance*tan(fovX/2.0f)*2.0f,
                      ((float)m_i_view_diff_y/(float)height)*m_f_camera_distance*tan(fovY/2.0f)*2.0f,0.0f);
            }
            else if(m_prp_str_mouse_control_mode->getStdString()==MODE_FPS)  // Orbit style
            {
                MoveCameraEyeFocus(m_i_view_diff_x*0.01,-m_i_view_diff_y*0.01,0.0f);
            }
        }
        else if(event.right())
        {
            if(event.shift()||(m_prp_str_mouse_control_mode->getStdString()==MODE_FPS))
            {
                setCursor(MoveZ);
                MoveCameraEyeFocus(0.0f,0.0f,m_i_view_diff_y*0.01*m_f_camera_distance);
            }
            else
            {
                setCursor(Zoom);
                MoveCameraEye(0,0,m_i_view_diff_y*0.01*m_f_camera_distance);
            }
        }
        else
        {
            setCursor(event.shift() ? MoveXY : Rotate3D);
        }
        if(event.wheel_delta!=0)
        {
            int diff=event.wheel_delta;

            if(event.shift())
            {
                MoveCameraEyeFocus(0,0,-diff*0.001*m_f_camera_distance);
            }
            else if(event.control())
            {
                SetCameraYawPitchRoll(0,0,diff*0.001);
            }
            else
            {
                MoveCameraEye(0,0,-diff*0.001*m_f_camera_distance);
            }
        }
        if(event.type==QEvent::MouseButtonPress && event.left() && event.control() && event.shift())
        {
            bool b_was_mode_orbit=(m_prp_str_mouse_control_mode->getStdString()==MODE_ORBIT);
            m_prp_str_mouse_control_mode->setStdString(b_was_mode_orbit ? MODE_FPS : MODE_ORBIT );
        }

        // Render view window
        context_->queueRender();
    }

    void RVizPluginWAIVirtualCamera::transitionFrom(ViewController* previous_view)
    {
        RVizPluginWAIVirtualCamera* avc_anim_view_cont=dynamic_cast<RVizPluginWAIVirtualCamera*>(previous_view);

        if(avc_anim_view_cont)
        {
            view_controller_msgs::CameraMovement cam_movement;
            cam_movement.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
            cam_movement.eye.point.x=m_prp_vc3_camera_eye->getVector().x;
            cam_movement.eye.point.y=m_prp_vc3_camera_eye->getVector().y;
            cam_movement.eye.point.z=m_prp_vc3_camera_eye->getVector().z;
            cam_movement.focus.point.x=m_prp_vc3_camera_focus->getVector().x;
            cam_movement.focus.point.y=m_prp_vc3_camera_focus->getVector().y;
            cam_movement.focus.point.z=m_prp_vc3_camera_focus->getVector().z;
            cam_movement.up.vector.x=m_prp_vc3_camera_up_axis->getVector().x;
            cam_movement.up.vector.y=m_prp_vc3_camera_up_axis->getVector().y;
            cam_movement.up.vector.z=m_prp_vc3_camera_up_axis->getVector().z;
            cam_movement.transition_duration=ros::Duration(m_prp_f_camera_transition_time->getFloat());
            InitCameraTransition(cam_movement);
        }
    }

    void RVizPluginWAIVirtualCamera::update(float dt,float ros_dt)
    {
        UpdateCameraAttachedSceneNode();

        // Moved here to constantly publish
        if(m_prp_bol_camera_pose_enabled->getBool() && m_pub_pst_camera.getNumSubscribers()>0)
        {
            PublishCameraPose();
        }

        if(m_prp_bol_camera_liveview_enabled->getBool() && m_pub_img_camera_liveview.getNumSubscribers()>0)
        {
            PublishCameraLiveviewImage();
        }

        // Transition, if movements are still available (at least one start pose and one end pose)
        if(m_b_transition_enabled && vec_camera_movements.size()>=2)
        {
            if(m_dur_camera_transition_pause.toSec()>0.0)
            {
                m_dur_camera_transition_pause.sleep();
                m_tim_camera_transition_start+=m_dur_camera_transition_pause;
                m_dur_camera_transition_pause.fromSec(0.0);
            }

            view_controller_msgs::CameraMovement cam_movement_origin=vec_camera_movements.front();
            view_controller_msgs::CameraMovement cam_movement_goal=vec_camera_movements.begin()[1];

            double d_time_delta=0.0;
            if(m_b_frame_by_frame_enabled)
            {
                d_time_delta=m_i_frames_rendered/(m_i_target_fps*cam_movement_goal.transition_duration.toSec());
                m_i_frames_rendered++;
            }
            else
            {
                ros::WallDuration dur_elapsed=ros::WallTime::now()-m_tim_camera_transition_start;
                d_time_delta=dur_elapsed.toSec()/cam_movement_goal.transition_duration.toSec();
            }

            // make sure we get all the way there before turning off
            bool finished_current_movement=false;
            if(d_time_delta>=1.0)
            {
                d_time_delta=1.0;
                finished_current_movement=true;
                PublishCameraTransitionFinished();
            }

            // Update camera pose based on calculated iterations
            float f_cam_tran_iter=CalculateCameraTransitionIteration(d_time_delta,cam_movement_goal.interpolation_speed);
            Ogre::Vector3 vc3_eye_new;
            vc3_eye_new.x=cam_movement_origin.eye.point.x+f_cam_tran_iter*(cam_movement_goal.eye.point.x-cam_movement_origin.eye.point.x);
            vc3_eye_new.y=cam_movement_origin.eye.point.y+f_cam_tran_iter*(cam_movement_goal.eye.point.y-cam_movement_origin.eye.point.y);
            vc3_eye_new.z=cam_movement_origin.eye.point.z+f_cam_tran_iter*(cam_movement_goal.eye.point.z-cam_movement_origin.eye.point.z);
            m_prp_vc3_camera_eye->setVector(vc3_eye_new);

            Ogre::Vector3 vc3_focus_new;
            vc3_focus_new.x=cam_movement_origin.focus.point.x+f_cam_tran_iter*(cam_movement_goal.focus.point.x-cam_movement_origin.focus.point.x);
            vc3_focus_new.y=cam_movement_origin.focus.point.y+f_cam_tran_iter*(cam_movement_goal.focus.point.y-cam_movement_origin.focus.point.y);
            vc3_focus_new.z=cam_movement_origin.focus.point.z+f_cam_tran_iter*(cam_movement_goal.focus.point.z-cam_movement_origin.focus.point.z);
            m_prp_vc3_camera_focus->setVector(vc3_focus_new);

            Ogre::Vector3 vc3_up_new;
            vc3_up_new.x=cam_movement_origin.up.vector.x+f_cam_tran_iter*(cam_movement_goal.up.vector.x-cam_movement_origin.up.vector.x);
            vc3_up_new.y=cam_movement_origin.up.vector.y+f_cam_tran_iter*(cam_movement_goal.up.vector.y-cam_movement_origin.up.vector.y);
            vc3_up_new.z=cam_movement_origin.up.vector.z+f_cam_tran_iter*(cam_movement_goal.up.vector.z-cam_movement_origin.up.vector.z);
            m_prp_vc3_camera_up_axis->setVector(vc3_up_new);

            // This needs to happen so that the camera orientation will update properly when fixed_up_property==false
            camera_->setFixedYawAxis(true, m_qua_pose_reference*m_prp_vc3_camera_up_axis->getVector());
            camera_->setDirection(m_qua_pose_reference*(m_prp_vc3_camera_focus->getVector()-m_prp_vc3_camera_eye->getVector()));

            if(finished_current_movement)
            {
                // delete current start element in buffer
                vec_camera_movements.erase(vec_camera_movements.begin());
                if(vec_camera_movements.size()>=2) // Movements are still available (at least one start pose and one end pose)
                {
                    m_tim_camera_transition_start+=ros::WallDuration(cam_movement_goal.transition_duration.toSec());
                    m_i_frames_rendered=0;
                }
                else
                {
                    AbortCameraTransition();
                }
            }
        }
        UpdateCameraPose();
    }

    float RVizPluginWAIVirtualCamera::CalculateCameraTransitionIteration(double d_time_delta,uint8_t ui8_movement_mode)
    {
        switch(ui8_movement_mode)
        {
            case view_controller_msgs::CameraMovement::RISING:
                return 1.0f-static_cast<float>(cos(d_time_delta*M_PI_2));
            break;

            case view_controller_msgs::CameraMovement::DECLINING:
                return static_cast<float>(-cos(d_time_delta*M_PI_2+M_PI_2));
            break;

            case view_controller_msgs::CameraMovement::FULL:
                return static_cast<float>(d_time_delta);
            break;

            case view_controller_msgs::CameraMovement::WAVE:
                return 0.5f*(1.0f-static_cast<float>(cos(d_time_delta*M_PI)));
            break;

            default:
                return 0.5f*(1.0f-static_cast<float>(cos(d_time_delta*M_PI)));
            break;
        }
    }

    void RVizPluginWAIVirtualCamera::SetSubAndPubTopics()
    {
        // Correctly update properties for subscriber/publisher topics (does not work in constructor!)
        m_prp_top_camera_placement->setStdString(m_s_camera_rviz_namespace+"placement");
        m_prp_top_camera_trajectory->setStdString(m_s_camera_rviz_namespace+"trajectory");
        m_prp_top_camera_transition_pause->setStdString(m_s_camera_rviz_namespace+"transition_pause");
        m_prp_top_camera_transition_finished->setStdString(m_s_camera_rviz_namespace+"transition_finished");
        m_prp_top_camera_pose->setStdString(m_s_camera_rviz_namespace+"pose");
        m_prp_top_camera_liveview->setStdString(m_s_camera_rviz_namespace+"liveview");

        // Initialize Subscribers and Publishers
        m_sub_plc_camera_placement=m_hdl_node.subscribe(m_prp_top_camera_placement->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_plc_camera_placement,this);
        m_sub_trj_camera_trajectory=m_hdl_node.subscribe(m_prp_top_camera_trajectory->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_trj_camera_trajectory,this);
        m_sub_dur_camera_transition_pause=m_hdl_node.subscribe(m_prp_top_camera_transition_pause->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_dur_camera_transition_pause,this);
        m_pub_bol_camera_transition_finished=m_hdl_node.advertise<std_msgs::Bool>(m_prp_top_camera_transition_finished->getStdString(),1);
        m_pub_pst_camera=m_hdl_node.advertise<geometry_msgs::PoseStamped>(m_prp_top_camera_pose->getStdString(),1);
        m_pub_img_camera_liveview=m_hdl_it.advertise(m_prp_top_camera_liveview->getStdString(),1);
    }

    void RVizPluginWAIVirtualCamera::UpdateCameraPlacementTopic()
    {
        m_sub_plc_camera_placement.shutdown();
        m_sub_plc_camera_placement=m_hdl_node.subscribe(m_prp_top_camera_placement->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_plc_camera_placement,this);
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraTrajectoryTopic()
    {
        //m_sub_trj_camera_trajectory.shutdown();
        m_sub_trj_camera_trajectory=m_hdl_node.subscribe(m_prp_top_camera_trajectory->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_trj_camera_trajectory,this);
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraTransitionPauseTopic()
    {
        //m_sub_dur_camera_transition_pause.shutdown();
        m_sub_dur_camera_transition_pause=m_hdl_node.subscribe(m_prp_top_camera_transition_pause->getStdString(),1,&RVizPluginWAIVirtualCamera::cb_sub_dur_camera_transition_pause,this);
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraTransitionFinishedTopic()
    {
        //m_pub_bol_camera_transition_finished.shutdown();
        m_pub_bol_camera_transition_finished=m_hdl_node.advertise<std_msgs::Bool>(m_prp_top_camera_transition_finished->getStdString(),1);
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraPoseTopic()
    {
        //m_pub_pst_camera.shutdown();
        m_pub_pst_camera=m_hdl_node.advertise<geometry_msgs::PoseStamped>(m_prp_top_camera_pose->getStdString(),1);
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraLiveviewTopic()
    {
        //m_pub_img_camera_liveview.shutdown();
        m_pub_img_camera_liveview=m_hdl_it.advertise(m_prp_top_camera_liveview->getStdString(),1);
    }

    void RVizPluginWAIVirtualCamera::UpdateCameraAttachedSceneNode()
    {
        Ogre::Vector3 new_reference_position;
        Ogre::Quaternion new_reference_orientation;

        if(context_->getFrameManager()->getTransform(m_prp_frm_camera_target->getFrameStd(),
                                                     ros::Time(),
                                                     new_reference_position,
                                                     new_reference_orientation))
        {
            m_ogr_node_scene->setPosition(new_reference_position);
            m_ogr_node_scene->setOrientation(new_reference_orientation);
            m_vc3_pose_reference=new_reference_position;
            m_qua_pose_reference=new_reference_orientation;

            context_->queueRender();
        }
    }
    float RVizPluginWAIVirtualCamera::GetCameraEyeFocusDistance()
    {
        m_f_camera_distance=(m_prp_vc3_camera_eye->getVector()-m_prp_vc3_camera_focus->getVector()).length();
        return m_f_camera_distance;
    }
    void RVizPluginWAIVirtualCamera::UpdateCameraPose()
    {
        camera_->setPosition(m_prp_vc3_camera_eye->getVector());
        camera_->setFixedYawAxis(m_prp_bol_camera_lock_up_axis_enabled->getBool(),m_qua_pose_reference*m_prp_vc3_camera_up_axis->getVector());
        camera_->setDirection(m_qua_pose_reference*(m_prp_vc3_camera_focus->getVector()-m_prp_vc3_camera_eye->getVector()));
        m_ogr_shape_focal->setPosition(m_prp_vc3_camera_focus->getVector());
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }
    void RVizPluginWAIVirtualCamera::PublishCameraPose()
    {
        geometry_msgs::PoseStamped pst_camera;
        pst_camera.header.frame_id=m_prp_frm_camera_target->getFrameStd();
        pst_camera.header.stamp=ros::Time::now();
        pst_camera.pose.position.x=camera_->getPosition().x;
        pst_camera.pose.position.y=camera_->getPosition().y;
        pst_camera.pose.position.z=camera_->getPosition().z;
        pst_camera.pose.orientation.w=camera_->getOrientation().w;
        pst_camera.pose.orientation.x=camera_->getOrientation().x;
        pst_camera.pose.orientation.y=camera_->getOrientation().y;
        pst_camera.pose.orientation.z=camera_->getOrientation().z;
        // Transform into OpenCV frame (Z-axis pointing forward!)
        tf::Quaternion q_rot_origin=tf::Quaternion(pst_camera.pose.orientation.x,pst_camera.pose.orientation.y,pst_camera.pose.orientation.z,pst_camera.pose.orientation.w);
        tf::Quaternion q_rot_roll;
        q_rot_roll.setEulerZYX(0.0,0.0,3.14159);
        tf::Quaternion q_rot_result=q_rot_origin*q_rot_roll;
        q_rot_result.normalize();
        pst_camera.pose.orientation.w=q_rot_result.getW();
        pst_camera.pose.orientation.x=q_rot_result.getX();
        pst_camera.pose.orientation.y=q_rot_result.getY();
        pst_camera.pose.orientation.z=q_rot_result.getZ();
        m_pub_pst_camera.publish(pst_camera);
    }

    void RVizPluginWAIVirtualCamera::SetCameraYawPitchRoll(float yaw,float pitch,float roll)
    {
        Ogre::Quaternion old_camera_orientation=camera_->getOrientation();
        Ogre::Radian old_pitch=old_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

        if(m_prp_bol_camera_lock_up_axis_enabled->getBool())
        {
            yaw=cos(old_pitch.valueRadians()-Ogre::Math::HALF_PI)*yaw; // helps to reduce crazy spinning!
        }

        Ogre::Quaternion yaw_quat,pitch_quat,roll_quat;
        yaw_quat.FromAngleAxis(Ogre::Radian(yaw),Ogre::Vector3::UNIT_Y);
        pitch_quat.FromAngleAxis(Ogre::Radian(pitch),Ogre::Vector3::UNIT_X);
        roll_quat.FromAngleAxis(Ogre::Radian(roll),Ogre::Vector3::UNIT_Z);
        Ogre::Quaternion orientation_change=yaw_quat*pitch_quat*roll_quat;
        Ogre::Quaternion new_camera_orientation=old_camera_orientation * orientation_change;
        Ogre::Radian new_pitch=new_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

        if(m_prp_bol_camera_lock_up_axis_enabled->getBool() &&
          ((new_pitch>PITCH_LIMIT_HIGH && new_pitch>old_pitch) ||
           (new_pitch<PITCH_LIMIT_LOW && new_pitch<old_pitch))
          )
        {
            orientation_change=yaw_quat*roll_quat;
            new_camera_orientation=old_camera_orientation*orientation_change;
        }

        //  Ogre::Radian new_roll=new_camera_orientation.getRoll(false);
        //  Ogre::Radian new_yaw=new_camera_orientation.getYaw(false);
        //ROS_INFO("old_pitch: %.3f, new_pitch: %.3f", old_pitch.valueRadians(), new_pitch.valueRadians());

        camera_->setOrientation(new_camera_orientation);
        if(m_prp_str_mouse_control_mode->getStdString()==MODE_ORBIT)
        {
            // In orbit mode the focal point stays fixed, so we need to compute the new camera position.
            Ogre::Vector3 new_eye_position=m_prp_vc3_camera_focus->getVector()+m_f_camera_distance*new_camera_orientation.zAxis();
            m_prp_vc3_camera_eye->setVector(new_eye_position);
            camera_->setPosition(new_eye_position);
            //setPropertiesFromCamera(camera_);
        }
        else
        {
            // In FPS mode the camera stays fixed, so we can just apply the rotations and then rely on the property update to set the new focal point.
            //setPropertiesFromCamera(camera_);
        }
    }
    void RVizPluginWAIVirtualCamera::MoveCameraEyeFocus(float x,float y,float z)
    {
        Ogre::Vector3 translate(x,y,z);
        m_prp_vc3_camera_eye->add(camera_->getOrientation()*translate);
        m_prp_vc3_camera_focus->add(camera_->getOrientation()*translate);
    }
    void RVizPluginWAIVirtualCamera::MoveCameraEye(float x,float y,float z)
    {
        Ogre::Vector3 translate(x,y,z);
        // Only update the camera position if it won't "pass through" the origin
        Ogre::Vector3 new_position=m_prp_vc3_camera_eye->getVector()+camera_->getOrientation()*translate;
        if((new_position-m_prp_vc3_camera_focus->getVector()).length()>m_prp_f_camera_distance->getMin())
        {
            m_prp_vc3_camera_eye->setVector(new_position);
        }
        m_prp_f_camera_distance->setFloat(GetCameraEyeFocusDistance());
    }

    void RVizPluginWAIVirtualCamera::InitCameraTransition(view_controller_msgs::CameraMovement cam_movement)
    {
        if(cam_movement.transition_duration.toSec()<0.0)
        {
            ROS_WARN("Negative transition duration, dropping movement.");
            return;
        }
        else if(cam_movement.transition_duration.isZero())
        {
            // convert positional jumps to very fast movements to prevent numerical problems
            cam_movement.transition_duration=ros::Duration(0.001);
        }

        // If the buffer is empty we set the first element in it to the current camera pose
        if(vec_camera_movements.empty())
        {
            m_tim_camera_transition_start=ros::WallTime::now(); // Reset time if transitioning to one pose only
            view_controller_msgs::CameraMovement cam_movement_empty;
            cam_movement_empty.eye.point.x=m_prp_vc3_camera_eye->getVector().x;
            cam_movement_empty.eye.point.y=m_prp_vc3_camera_eye->getVector().y;
            cam_movement_empty.eye.point.z=m_prp_vc3_camera_eye->getVector().z;
            cam_movement_empty.focus.point.x=m_prp_vc3_camera_focus->getVector().x;
            cam_movement_empty.focus.point.y=m_prp_vc3_camera_focus->getVector().y;
            cam_movement_empty.focus.point.z=m_prp_vc3_camera_focus->getVector().z;
            cam_movement_empty.up.vector.x=m_prp_vc3_camera_up_axis->getVector().x;
            cam_movement_empty.up.vector.y=m_prp_vc3_camera_up_axis->getVector().y;
            cam_movement_empty.up.vector.z=m_prp_vc3_camera_up_axis->getVector().z;
            cam_movement_empty.transition_duration=ros::Duration(0.001); // Set to small default value
            cam_movement_empty.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
            vec_camera_movements.push_back(cam_movement_empty);
        }
        vec_camera_movements.push_back(cam_movement);
        m_b_transition_enabled=true;
    }
    void RVizPluginWAIVirtualCamera::AbortCameraTransition()
    {
        m_b_transition_enabled=false;
        vec_camera_movements.clear();
        m_i_frames_rendered=0;

        if(m_b_frame_by_frame_enabled)
        {
            m_b_frame_by_frame_enabled=false;
            PublishCameraTransitionFinished();
        }
    }
    void RVizPluginWAIVirtualCamera::PublishCameraTransitionFinished()
    {
        std_msgs::Bool msg_bol_transition_finished;
        msg_bol_transition_finished.data=1;
        m_pub_bol_camera_transition_finished.publish(msg_bol_transition_finished);
    }

    void RVizPluginWAIVirtualCamera::cb_sub_plc_camera_placement(const CameraPlacementConstPtr &cp_ptr)
    {
        CameraPlacement cpl_camera=*cp_ptr;

        // Handle control parameters
        m_prp_bol_mouse_control_enabled->setBool(!cpl_camera.interaction_disabled);
        m_prp_bol_camera_lock_up_axis_enabled->setBool(!cpl_camera.allow_free_yaw_axis);
        if(cpl_camera.mouse_interaction_mode!=cpl_camera.NO_CHANGE)
        {
            std::string name="";
            if(cpl_camera.mouse_interaction_mode==cpl_camera.ORBIT) name=MODE_ORBIT;
            else if(cpl_camera.mouse_interaction_mode==cpl_camera.FPS) name=MODE_FPS;
            m_prp_str_mouse_control_mode->setStdString(name);
        }
        if(cpl_camera.target_frame!="")
        {
            m_prp_frm_camera_target->setStdString(cpl_camera.target_frame);
        }

        if(cpl_camera.time_from_start.toSec()>=0.0)
        {
            transformCameraToAttachedFrame(cpl_camera.eye,cpl_camera.focus,cpl_camera.up);
            view_controller_msgs::CameraMovement cam_movement;
            cam_movement.eye=cpl_camera.eye;
            cam_movement.focus=cpl_camera.focus;
            cam_movement.up.vector=cpl_camera.up.vector;
            cam_movement.transition_duration=cpl_camera.time_from_start;
            cam_movement.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
            InitCameraTransition(cam_movement);
        }
    }

    void RVizPluginWAIVirtualCamera::cb_sub_trj_camera_trajectory(const view_controller_msgs::CameraTrajectoryConstPtr& ct_ptr)
    {
        view_controller_msgs::CameraTrajectory ct=*ct_ptr;

        if(ct.trajectory.empty())
        {
            return;
        }

        // Handle control parameters
        m_prp_bol_mouse_control_enabled->setBool(!ct.interaction_disabled);
        m_prp_bol_camera_lock_up_axis_enabled->setBool(!ct.allow_free_yaw_axis);

        if(ct.mouse_interaction_mode!=view_controller_msgs::CameraTrajectory::NO_CHANGE)
        {
            std::string name="";
            if(ct.mouse_interaction_mode==view_controller_msgs::CameraTrajectory::ORBIT)
            {
                name=MODE_ORBIT;
            }
            else if(ct.mouse_interaction_mode==view_controller_msgs::CameraTrajectory::FPS)
            {
                name=MODE_FPS;
            }
            m_prp_str_mouse_control_mode->setStdString(name);
        }

        if(ct.render_frame_by_frame > 0)
        {
            m_b_frame_by_frame_enabled=true;
            m_i_target_fps=static_cast<int>(ct.frames_per_second);
            m_prp_bol_camera_liveview_enabled->setBool(true);
        }

        for(auto& cam_movement:ct.trajectory)
        {
            if(cam_movement.transition_duration.toSec()>=0.0)
            {
                if(ct.target_frame!="")
                {
                    m_prp_frm_camera_target->setStdString(ct.target_frame);
                }
                transformCameraToAttachedFrame(cam_movement.eye,cam_movement.focus,cam_movement.up);
                InitCameraTransition(cam_movement); // ui8_movement_mode is aquivalent to "interpolation_speed"
            }
            else
            {
                ROS_WARN("Negative transition duration, dropping movement.");
            }
        }
    }

    void RVizPluginWAIVirtualCamera::cb_sub_dur_camera_transition_pause(const std_msgs::Duration::ConstPtr& pause_duration_msg)
    {
        m_dur_camera_transition_pause.sec=pause_duration_msg->data.sec;
        m_dur_camera_transition_pause.nsec=pause_duration_msg->data.nsec;
    }

    void RVizPluginWAIVirtualCamera::transformCameraToAttachedFrame(geometry_msgs::PointStamped& eye,
                                                                geometry_msgs::PointStamped& focus,
                                                                geometry_msgs::Vector3Stamped& up)
    {
        Ogre::Vector3 position_fixed_eye, position_fixed_focus, position_fixed_up;
        Ogre::Quaternion rotation_fixed_eye, rotation_fixed_focus, rotation_fixed_up;

        context_->getFrameManager()->getTransform(eye.header.frame_id,ros::Time(0),position_fixed_eye,rotation_fixed_eye);
        context_->getFrameManager()->getTransform(focus.header.frame_id,ros::Time(0),position_fixed_focus,rotation_fixed_focus);
        context_->getFrameManager()->getTransform(up.header.frame_id,ros::Time(0),position_fixed_up,rotation_fixed_up);

        Ogre::Vector3 ogre_eye=Ogre::Vector3(eye.point.x,eye.point.y,eye.point.z);
        Ogre::Vector3 ogre_focus=Ogre::Vector3(focus.point.x,focus.point.y,focus.point.z);
        Ogre::Vector3 ogre_up=Ogre::Vector3(up.vector.x,up.vector.y,up.vector.z);

        ogre_eye=fixedFrameToAttachedLocal(position_fixed_eye+rotation_fixed_eye*ogre_eye);
        ogre_focus=fixedFrameToAttachedLocal(position_fixed_focus+rotation_fixed_focus*ogre_focus);
        ogre_up=m_qua_pose_reference.Inverse()*rotation_fixed_up*ogre_up;

        eye.point.x=ogre_eye.x; eye.point.y=ogre_eye.y; eye.point.z=ogre_eye.z;
        focus.point.x=ogre_focus.x; focus.point.y=ogre_focus.y; focus.point.z=ogre_focus.z;
        up.vector.x=ogre_up.x; up.vector.y=ogre_up.y; up.vector.z=ogre_up.z;
        eye.header.frame_id=m_prp_frm_camera_target->getStdString();
        focus.header.frame_id=m_prp_frm_camera_target->getStdString();
        up.header.frame_id=m_prp_frm_camera_target->getStdString();
    }
    Ogre::Vector3 RVizPluginWAIVirtualCamera::fixedFrameToAttachedLocal(const Ogre::Vector3 &v)
    {
        return m_qua_pose_reference.Inverse()*(v-m_vc3_pose_reference);
    }
    Ogre::Vector3 RVizPluginWAIVirtualCamera::attachedLocalToFixedFrame(const Ogre::Vector3 &v)
    {
        return m_vc3_pose_reference+(m_qua_pose_reference*v);
    }

    // We must assume that this point is in the Rviz Fixed frame since it came from Rviz...
    void RVizPluginWAIVirtualCamera::lookAt(const Ogre::Vector3& point)
    {
        if( !m_prp_bol_mouse_control_enabled->getBool() ) return;

        Ogre::Vector3 vc3_focus_new=fixedFrameToAttachedLocal(point);

        view_controller_msgs::CameraMovement cam_movement;
        cam_movement.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
        cam_movement.eye.point.x=m_prp_vc3_camera_eye->getVector().x; cam_movement.eye.point.y=m_prp_vc3_camera_eye->getVector().y; cam_movement.eye.point.z=m_prp_vc3_camera_eye->getVector().z;
        cam_movement.focus.point.x=vc3_focus_new.x; cam_movement.focus.point.y=vc3_focus_new.y; cam_movement.focus.point.z=vc3_focus_new.z;
        cam_movement.up.vector.x=m_prp_vc3_camera_up_axis->getVector().x; cam_movement.up.vector.y=m_prp_vc3_camera_up_axis->getVector().y; cam_movement.up.vector.z=m_prp_vc3_camera_up_axis->getVector().z;
        cam_movement.transition_duration=ros::Duration(m_prp_f_camera_transition_time->getFloat());
        InitCameraTransition(cam_movement);

        //  // Just for easily testing the other movement styles:
        //  orbitCameraTo(point);
        //  moveCameraWithFocusTo(point);
    }
    void RVizPluginWAIVirtualCamera::orbitCameraTo(const Ogre::Vector3& point)
    {
        view_controller_msgs::CameraMovement cam_movement;
        cam_movement.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
        cam_movement.eye.point.x=point.x; cam_movement.eye.point.y=point.y; cam_movement.eye.point.z=point.z;
        cam_movement.focus.point.x=m_prp_vc3_camera_focus->getVector().x; cam_movement.focus.point.y=m_prp_vc3_camera_focus->getVector().y; cam_movement.focus.point.z=m_prp_vc3_camera_focus->getVector().z;
        cam_movement.up.vector.x=m_prp_vc3_camera_up_axis->getVector().x; cam_movement.up.vector.y=m_prp_vc3_camera_up_axis->getVector().y; cam_movement.up.vector.z=m_prp_vc3_camera_up_axis->getVector().z;
        cam_movement.transition_duration=ros::Duration(m_prp_f_camera_transition_time->getFloat());
        InitCameraTransition(cam_movement);
    }
    void RVizPluginWAIVirtualCamera::moveEyeWithFocusTo( const Ogre::Vector3& point)
    {
        view_controller_msgs::CameraMovement cam_movement;
        cam_movement.interpolation_speed=view_controller_msgs::CameraMovement::WAVE;
        cam_movement.eye.point.x=point.x; cam_movement.eye.point.y=point.y; cam_movement.eye.point.z=point.z;

        cam_movement.focus.point.x=m_prp_vc3_camera_focus->getVector().x+(point.x-m_prp_vc3_camera_eye->getVector().x);
        cam_movement.focus.point.y=m_prp_vc3_camera_focus->getVector().y+(point.y-m_prp_vc3_camera_eye->getVector().y);
        cam_movement.focus.point.z=m_prp_vc3_camera_focus->getVector().z+(point.z-m_prp_vc3_camera_eye->getVector().z);

        cam_movement.up.vector.x=m_prp_vc3_camera_up_axis->getVector().x; cam_movement.up.vector.y=m_prp_vc3_camera_up_axis->getVector().y; cam_movement.up.vector.z=m_prp_vc3_camera_up_axis->getVector().z;
        cam_movement.transition_duration=ros::Duration(m_prp_f_camera_transition_time->getFloat());
        InitCameraTransition(cam_movement);
    }

    void RVizPluginWAIVirtualCamera::SetCameraLiveviewImageResolution()
    {
        m_prp_i_camera_window_width->setInt(context_->getViewManager()->getRenderPanel()->getRenderWindow()->getWidth());
        m_prp_i_camera_window_height->setInt(context_->getViewManager()->getRenderPanel()->getRenderWindow()->getHeight());
    }
    void RVizPluginWAIVirtualCamera::PublishCameraLiveviewImage()
    {
        // No "custom" resolution possible, since it has to fit to render window!
        SetCameraLiveviewImageResolution(); // Thus, update width and height with current view
        const unsigned int image_height=m_prp_i_camera_window_height->getInt();
        const unsigned int image_width=m_prp_i_camera_window_width->getInt();

        // Create a PixelBox to store the rendered view image
        const Ogre::PixelFormat pixel_format=Ogre::PF_BYTE_BGR;
        const auto bytes_per_pixel=Ogre::PixelUtil::getNumElemBytes(pixel_format);
        auto image_data=new unsigned char[image_width*image_height*bytes_per_pixel];
        Ogre::Box image_extents(0,0,image_width,image_height);
        std::shared_ptr<Ogre::PixelBox> pixel_box=std::make_shared<Ogre::PixelBox>(image_extents,pixel_format,image_data);
        context_->getViewManager()->getRenderPanel()->getRenderWindow()->copyContentsToMemory(*pixel_box,Ogre::RenderTarget::FB_AUTO);

        sensor_msgs::ImagePtr image_msg=sensor_msgs::ImagePtr(new sensor_msgs::Image());
        image_msg->header.frame_id=m_prp_frm_camera_target->getStdString();
        image_msg->header.stamp=ros::Time::now();
        image_msg->height=image_height;
        image_msg->width=image_width;
        image_msg->encoding=sensor_msgs::image_encodings::BGR8;
        image_msg->is_bigendian=false;
        image_msg->step=static_cast<unsigned int>(image_width*bytes_per_pixel);
        size_t size=image_width*image_height*bytes_per_pixel;
        image_msg->data.resize(size);
        memcpy((char*)(&image_msg->data[0]),pixel_box->data,size);
        m_pub_img_camera_liveview.publish(*image_msg);

        delete[] (unsigned char*)pixel_box->data;
    }
}

#include<pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_wai_virtual_camera::RVizPluginWAIVirtualCamera,rviz::ViewController)
