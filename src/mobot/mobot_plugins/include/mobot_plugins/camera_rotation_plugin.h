#include <string>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace gazebo
{
  class CameraPos : public ModelPlugin
  {
  public:
    // Constructor
    CameraPos();
    // Called after constructor to initialize using sdf
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  public:   
    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/);

    void setAngle(double angle);  
    // Function called when received msg
    void callBack(const std_msgs::Float32::ConstPtr& msg);

  private:
    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    double _angle;
    std::string _jointName, _topicName;
    physics::JointPtr _joint;
    sdf::ElementPtr sdf;

    double _updateRate;
    ros::NodeHandle _nh;
    ros::Subscriber sub;
  };
}