#include "/home/mikhail/catkin_ws/src/mobot/mobot_plugins/include/camera_rotation_plugin.h"

namespace gazebo
{
  CameraPos::CameraPos():_nh("camera_pos_plugin"){
  };

  void CameraPos::setAngle(double angle){
    ROS_INFO("was angle: %g", this->_angle);
    ROS_INFO("setting angle: %g", angle);
    this->_angle = angle;
  }

  void CameraPos::callBack(const std_msgs::Float32::ConstPtr& msg)
  {    
    this->setAngle(msg->data);
  }

  void CameraPos::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;
    this->sdf = _sdf;

    // Get parameters from sdf
    if (!this->sdf->HasElement("cameraJointName"))
    {
      ROS_FATAL_STREAM("cameraJointName tag is not specified, quitting");
      exit(1);
    } else {
      this->_jointName = this->sdf->Get<std::string>("cameraJointName");
      ROS_INFO("inside");
      this->_joint = this->model->GetJoint(this->_jointName);
    }
    if (!this->sdf->HasElement("topicName"))
    {
      // default
      this->_topicName = "set_camera_angle";
    }
    else {
      this->_topicName = this->sdf->Get<std::string>("topicName");
    }

    sub = _nh.subscribe(_topicName, 2, &CameraPos::callBack, this);
    
    if (!this->sdf->HasElement("updateRate"))
    {
      ROS_INFO("joint trajectory plugin missing <updateRate>, defaults"
               " to 0.0 (as fast as possible)");
      this->_updateRate = 0;
    }
    else
      this->_updateRate = this->sdf->Get<double>("updateRate");

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&CameraPos::OnUpdate, this, _1));
  }

  // Called by the world update start event
  void CameraPos::OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    // Change joint angle
    this->_joint->SetAngle(0, _angle);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraPos)
}