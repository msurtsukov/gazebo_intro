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
	class CameraPos;
}

class PosSubscriber
{
  std::string topic_name;
  ros::Subscriber sub;
  gazebo::CameraPos *control;

  void callBack(const std_msgs::Float32::ConstPtr& msg);
	public: PosSubscriber(ros::NodeHandle nh, std::string topic_name, gazebo::CameraPos* _cp);
 	public: PosSubscriber();
};

namespace gazebo
{
  class CameraPos : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

		public: CameraPos();
    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/);

    public: void setAngle(double angle);	
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    double _angle;
    std::string _jointName, _topicName;
    gazebo::physics::JointPtr _joint;
		sdf::ElementPtr sdf;
    double _updateRate;
		ros::NodeHandle _nh;
		PosSubscriber _posSubscriber;
  };
}
