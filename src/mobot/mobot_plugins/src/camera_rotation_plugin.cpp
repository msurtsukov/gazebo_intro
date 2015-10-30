#include "/home/mikhail/catkin_ws/src/mobot/mobot_plugins/include/camera_rotation_plugin.h"

void PosSubscriber::callBack(const std_msgs::Float32::ConstPtr& msg)
{
  control->setAngle(msg->data);
}
PosSubscriber::PosSubscriber(ros::NodeHandle nh, std::string topic_name, gazebo::CameraPos* _cp)
{
  sub = nh.subscribe(topic_name,1000, &PosSubscriber::callBack, this);
  control = _cp;  
}

namespace gazebo
{

  CameraPos::CameraPos():_nh("camera_pos_plugin"){};

  void CameraPos::setAngle(double angle){
    this->_angle = angle;
  }
  void CameraPos::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;
    this->sdf = _sdf;

    if (!this->sdf->HasElement("cameraJointName"))
    {
      ROS_FATAL_STREAM("cameraJointName tag is not specified, quitting");
      exit(1);
    } else {
      this->_jointName = this->sdf->Get<std::string>("cameraJointName");
      this->_joint = this->model->GetJoint(this->_jointName);
    }
    // Get parameters from sdf
    if (!this->sdf->HasElement("topicName"))
    {
      // default
      this->_topicName = "set_camera_angle";
    }
    else {
      this->_topicName = this->sdf->Get<std::string>("topicName");
      this->_posSubscriber = PosSubscriber(this->_nh, this->_topicName, this);
    }

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
    // Apply a small linear velocity to the model.
    this->_joint->SetAngle(0, _angle);
  }
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraPos)
}