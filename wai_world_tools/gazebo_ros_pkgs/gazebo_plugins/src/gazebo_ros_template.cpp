/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author W. A. Isop
 *  \desc   Simple base_link TF publisher
 */

#include <gazebo_plugins/gazebo_ros_template.h>
#include <ros/ros.h>



namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTemplate::GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTemplate::~GazeboRosTemplate()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTemplate::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
    // Get the world name.
    this->world_=_parent->GetWorld();
    this->model_=_parent;

    if(_sdf->HasElement("worldFrame"))
    {
        s_world_name_=_sdf->GetElement("worldFrame")->Get<std::string>();
    }
    else
    {
        s_world_name_=this->world_->Name();
    }
    if(_sdf->HasElement("linkName"))
    {
        s_link_name_=_sdf->GetElement("linkName")->Get<std::string>();
    }
    else
    {
        s_link_name_="link_base";
    }
    s_model_name_=_parent->GetName();

    // Get link from model
    link_=this->world_->ModelByName(s_model_name_)->GetLink(s_link_name_);

    if(_sdf->HasElement("TFpubInterval"))
    {
        tf_pub_interval_=_sdf->GetElement("TFpubInterval")->Get<double>();
    }
    else
    {
        tf_pub_interval_=0.05; // Default is 20Hz
    }


    // Make sure the ROS node for Gazebo has already been initalized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_=event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosTemplate::UpdateChild,this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTemplate::UpdateChild()
{
    // Publish TF
    if((ros::Time::now()-this->tim_iteration_last_).toSec()>=tf_pub_interval_)
    {
        this->tim_iteration_last_=ros::Time::now();
        brc_transforms_.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(link_->WorldPose().Rot().X(),
                                             link_->WorldPose().Rot().Y(),
                                             link_->WorldPose().Rot().Z(),
                                             link_->WorldPose().Rot().W()),
                              tf::Vector3(link_->WorldPose().Pos().X(),
                                          link_->WorldPose().Pos().Y(),
                                          link_->WorldPose().Pos().Z())),
                              this->tim_iteration_last_,
                              s_world_name_,
                              s_model_name_+"/"+s_link_name_) );
    }
}

}
