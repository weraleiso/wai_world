/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboRosForce plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce()
{
  this->wrench_msg_.force.x = 0;
  this->wrench_msg_.force.y = 0;
  this->wrench_msg_.force.z = 0;
  this->wrench_msg_.torque.x = 0;
  this->wrench_msg_.torque.y = 0;
  this->wrench_msg_.torque.z = 0;
  this->tim_iteration_last=ros::Time::now();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();
  this->model_=_model;
  str_world_name=this->world_->Name();
  str_model_name=_model->GetName();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }
  else
  {
    // Do nothing...
  }

  if (!_sdf->HasElement("bodyName"))
  {
    this->link_name_="link_base";
    //ROS_FATAL_NAMED("force", "force plugin missing <bodyName>, cannot proceed");
    //return;
  }
  else
  {
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  // Choose to pub TF only!
  if (!_sdf->HasElement("pubTFOnly"))
  {
    this->m_b_tf_only=true;
  }
  else
  {
    this->m_b_tf_only = _sdf->GetElement("pubTFOnly")->Get<bool>();
  }

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    this->topic_name_ = str_model_name+"_force";
    //ROS_FATAL_NAMED("force", "force plugin missing <topicName>, cannot proceed");
    //return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Interface to set state of model
  if (!_sdf->HasElement("topicNameState"))
  {
    this->topic_name_state_ = str_model_name+"_state";
  }
  else
  {
    this->topic_name_state_ = _sdf->GetElement("topicNameState")->Get<std::string>();
  }


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  if(m_b_tf_only==false)
  {
      ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
        this->topic_name_,1,
        boost::bind( &GazeboRosForce::UpdateObjectForce,this,_1),
        ros::VoidPtr(), &this->queue_);
      this->sub_ = this->rosnode_->subscribe(so);

      ros::SubscribeOptions so_state = ros::SubscribeOptions::create<geometry_msgs::Pose>(
        this->topic_name_state_,1,
        boost::bind( &GazeboRosForce::UpdateObjectState,this,_1),
        ros::VoidPtr(), &this->queue_);
      this->sub_state_ = this->rosnode_->subscribe(so_state);
  }

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosForce::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosForce::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrench_msg_.force.x = _msg->force.x;
  this->wrench_msg_.force.y = _msg->force.y;
  this->wrench_msg_.force.z = _msg->force.z;
  this->wrench_msg_.torque.x = _msg->torque.x;
  this->wrench_msg_.torque.y = _msg->torque.y;
  this->wrench_msg_.torque.z = _msg->torque.z;
}

void GazeboRosForce::UpdateObjectState(const geometry_msgs::Pose::ConstPtr& _msg)
{
    this->pose_msg_=*_msg;

    // Reset force
    this->wrench_msg_.force.x=0.0;
    this->wrench_msg_.force.y=0.0;
    this->wrench_msg_.force.z=0.0;
    this->wrench_msg_.torque.x=0.0;
    this->wrench_msg_.torque.y=0.0;
    this->wrench_msg_.torque.z=0.0;
    this->model_->ResetPhysicsStates();

    // Set state
    ignition::math::Pose3d pos_model;
    pos_model.Pos().Set(pose_msg_.position.x,pose_msg_.position.y,pose_msg_.position.z);
    pos_model.Rot().Set(pose_msg_.orientation.w,pose_msg_.orientation.x,pose_msg_.orientation.y,pose_msg_.orientation.z);
    this->link_->SetWorldPose(pos_model);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{
/* New plugin IF
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosForce::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  this->lock_.lock();
  ignition::math::Vector3d force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
  ignition::math::Vector3d torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
  this->link_->AddForce(force);
  this->link_->AddTorque(torque);
  this->lock_.unlock();
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
*/

#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosForce::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

  //this->lock_.lock(); //THIS GUY ACTUALLY DELAYS SPAWNING QUEUE IN GAZEBO!! However be careful parallel calling methods!
  
  if(m_b_tf_only==false)
  {
      ignition::math::Vector3d force(this->wrench_msg_.force.x,this->wrench_msg_.force.y,this->wrench_msg_.force.z);
      ignition::math::Vector3d torque(this->wrench_msg_.torque.x,this->wrench_msg_.torque.y,this->wrench_msg_.torque.z);
      this->link_->SetForce(force);
      this->link_->SetTorque(torque);
  }
  
  if((ros::Time::now()-this->tim_iteration_last).toSec()>0.05)
  {
	  this->tim_iteration_last=ros::Time::now();

      ignition::math::Pose3d pos_model_this = link_->WorldPose();
	  brc_transforms.sendTransform(
          tf::StampedTransform(
              tf::Transform(tf::Quaternion(pos_model_this.Rot().X(),pos_model_this.Rot().Y(),pos_model_this.Rot().Z(),pos_model_this.Rot().W()),tf::Vector3(pos_model_this.Pos().X(),pos_model_this.Pos().Y(),pos_model_this.Pos().Z())),
              ros::Time::now(),str_world_name,str_model_name+"/"+this->link_name_) );
  }

  //this->lock_.unlock();

#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForce::QueueThread()
{
  static const double timeout = 0.01; // 0.1 was old timout

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
