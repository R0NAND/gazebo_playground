#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <ros/ros.h>
#include <thread>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <rodrone_core/Setpoints.h>

namespace gazebo
{
  class RodronePlugin : public ModelPlugin
  {

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&RodronePlugin::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(0.3, 0, 0));
      gazebo::physics::LinkPtr link =  this->model->GetChildLink("pos_pos_rotor");
      gazebo::physics::JointPtr joint = this->model->GetJoint("pos_pos_rotor_joint");
      link->AddRelativeForce(gazebo::math::Vector3(0, 0, -1*(joint->GetVelocity(2)))); 

      link =  this->model->GetChildLink("pos_neg_rotor");
      joint = this->model->GetJoint("pos_neg_rotor_joint");
      link->AddRelativeForce(gazebo::math::Vector3(0, 0, 1*(joint->GetVelocity(2))));

      link =  this->model->GetChildLink("neg_pos_rotor");
      joint = this->model->GetJoint("neg_pos_rotor_joint");
      link->AddRelativeForce(gazebo::math::Vector3(0, 0, 1*(joint->GetVelocity(2))));

      link =  this->model->GetChildLink("neg_neg_rotor");
      joint = this->model->GetJoint("neg_neg_rotor_joint");
      link->AddRelativeForce(gazebo::math::Vector3(0, 0, -1*(joint->GetVelocity(2)))); 
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(RodronePlugin)
}