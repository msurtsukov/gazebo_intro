#Написание плагина

В этой часте мы напишем небольшой плагин, который будет менять угол поворота камеры.

Переходим в папку src/mobot и создаем там новый пакет:
~~~~
$ catkin_init_pkg mobot_plugins roscpp gazebo_ros
$ cd mobot_plugins
$ mkdir include src
~~~~
Создаем файл `include/camera_rotaion_plugin.h` и добавляем в него следующее содержимое:
~~~~
#include <string>

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace gazebo {
  class CameraPos : public ModelPlugin {
  public:
    // Constructor
    CameraPos();
    // Called after constructor to initialize using sdf ang get model pointer which we want to interuct with
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

    // Pointer to the joint 
    physics::JointPtr _joint;

    // Pointer to the representation of sdf parameters
    sdf::ElementPtr sdf;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    double _updateRate;	
    
    // Angle of the joint which we want to set
    double _angle;

    std::string _jointName, _topicName;
   
    // NodeHandler and Subscriber to connect to ROS
    ros::NodeHandle _nh;
    ros::Subscriber sub;
  };
}
~~~~
Функция `Load` переопределяется чтобы получить указатель на физическую модель `physics::ModelPtr model`, с которой можно взаимодействовать, а так же чтобы получить параметры `sdf::ElementPtr sdf` из sdf файла из которого плагин запускается.

Функция `onUpdate` будет вызываться Gazebo пре пересчете параметров мира с частотой `_updateRate`.

Более подробно в комментариях в коде.

Теперь создадим файл `src/camera_rotation_plugin.cpp` с таким содержимым (не забываем поменять путь в своему .h файлу):
~~~~
#include "/home/mikhail/catkin_ws/src/mobot/mobot_plugins/include/camera_rotation_plugin.h"

namespace gazebo {
  // Constructor with NodeHandler inititalization 
  CameraPos::CameraPos():_nh("camera_pos_plugin") {
  	_angle = 0.0;
  };

  void CameraPos::setAngle(double angle) {
    ROS_INFO("was angle: %g", this->_angle);
    ROS_INFO("setting angle: %g", angle);
    this->_angle = angle;
  }

  // Called when ROS msg received
  void CameraPos::callBack(const std_msgs::Float32::ConstPtr& msg) {    
    this->setAngle(msg->data);
  }

  // Called to load plugin
  void CameraPos::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;
    this->sdf = _sdf;

    // Get parameters from sdf
    if(!this->sdf->HasElement("cameraJointName")) {
      ROS_FATAL_STREAM("cameraJointName tag is not specified, quitting");
      exit(1);
    } else {
      this->_jointName = this->sdf->Get<std::string>("cameraJointName");
      ROS_INFO("inside");
      this->_joint = this->model->GetJoint(this->_jointName);
    }
    if(!this->sdf->HasElement("topicName")) {
      // default
      this->_topicName = "set_camera_angle";
    } else {
      this->_topicName = this->sdf->Get<std::string>("topicName");
    }

    // Create subscriber to connect with ROS
    sub = _nh.subscribe(_topicName, 2, &CameraPos::callBack, this);
    
    if(!this->sdf->HasElement("updateRate")) {
      ROS_INFO("joint trajectory plugin missing <updateRate>, defaults"
               " to 0.0 (as fast as possible)");
      this->_updateRate = 0;
    }
    else
      this->_updateRate = this->sdf->Get<double>("updateRate");

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()) {
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
  void CameraPos::OnUpdate(const common::UpdateInfo & /*_info*/) {
    // Change joint angle
    this->_joint->SetAngle(0, _angle);
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(CameraPos)
}
~~~~
Внимательно изучите код, я постарался его подробно закоментировать.
Обратите внимание на строчку
~~~~
	this->_joint->SetAngle(0, _angle);
~~~~
Поподробнее прочитать, про то что и как можно задать или считать можно вот [здесь](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Joint.html).
Там же находится и остальное API взаимодействия Gazebo и C++. Однако за актуальность не могу ручаться. Но это единственное место где я нашел описание API.  
Впринципе этот код может являться основой для написания своих плагинов.

Осталось только подключить этот плагин в sdf файле модели, откроем его и допишем в конец за предыдущим плагином вот такой код:
~~~~
    <plugin name="camera_pos" filename="libmobot_plugins.so">
      <cameraJointName>camera_joint</cameraJointName>
    </plugin>
~~~~
`camera_joint` это названия того самого сочленения, которым мы будем управлять.

Так как мы писали код на C++, то пакет надо откомпилировать с помощью `catkin_make`!

##Вращение камерой

Отправим из терминала сообщение которым повернем камеру:
~~~~
$ rostopic pub -1 /camera_pos_plugin/set_camera_angle std_msgs/Float32 "data: 0.2"
~~~~
Увидим, что камера действительно певернулась: 

![Mobot_camera](/figs/model_camera_rotated.png)

В этом репозитории вы можете скачать папку src со всем кодом.
