#Добавление камеры
В предыдущей части мы сконструировали простенького двухколесного робота, теперь добавим ему немного зрения

В файл model.sdf.xacro добавляем следующий код:
~~~~
    <link name="camera_link">
      <pose>${chassisLength/2 - cameraSize/2} 0 ${wheelRadius + chassisHeight/2 + cameraSize/2} 0 0 0</pose>
      <inertial> 
        <mass>${cameraMass}</mass> 
        <box_inertia m="${cameraMass}" x="${cameraSize}" y="${cameraSize}" z="${cameraSize}"/>
      </inertial>

      <collision name='camera_collision'>
        <geometry>
          <box>
            <size>${cameraSize} ${cameraSize} ${cameraSize}</size>
          </box>
        </geometry>
      </collision>

      <visual name='camera_visual'>
        <geometry>
          <box>
            <size>${cameraSize} ${cameraSize} ${cameraSize}</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Red</name>
            <uri>__default__</uri>
          </script>
        </material>
      </visual>
~~~~
В коде выше ничего нового.
Ниже добавляем объект сенсор. Их много типов, рассмотрим на примере камеры. [Еще примеры.](http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors)
~~~~
      <sensor name='camera1' type='camera'>
        <update_rate>30</update_rate>
        <camera name='head'>
          <horizontal_fov>1.39626</horizontal_fov>
          <image>
            <width>800</width>
            <height>800</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
~~~~
Обсудим тэги:
- `horizontal_fov` задает широкоугольность камеры, 
- `clip` - область рендеринга, то есть камеры видит объекты от `near` до `far`,
- `noise` - добавляет шум.

Теперь добавим плагин, который подключит камеру к ROS.
~~~~
        <plugin name='camera_controller' filename='libgazebo_ros_camera.so'>
          <alwaysOn>true</alwaysOn>
          <updateRate>0.0</updateRate>
          <cameraName>mobot/camera1</cameraName>
          <imageTopicName>image_raw</imageTopicName>
          <cameraInfoTopicName>camera_info</cameraInfoTopicName>
          <frameName>camera_link</frameName>
          <hackBaseline>0.07</hackBaseline>
          <distortionK1>0.0</distortionK1>
          <distortionK2>0.0</distortionK2>
          <distortionK3>0.0</distortionK3>
          <distortionT1>0.0</distortionT1>
          <distortionT2>0.0</distortionT2>
        </plugin>
      </sensor>
    </link>
~~~~
`name` - название плагина, который надо подключить, а `filename` - имя файла библиотеки в которой он находится. Тэги внутри `plugin`, это указания на входные параметры в исполняемый код плагина, будет более понятно, когда мы напишем свой простенький плагин.

Теперь присоединим нашу камеру к базе робота:
~~~~
    <joint name="camera_joint" type="revolute">
      <child>camera_link</child>
      <parent>chassis</parent>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <upper>0</upper>
          <lower>0</lower>
        </limit>
      </axis>
    </joint>
~~~~
Мы уже встречались с таким типом соединения. К сожалению в sdf нету жесткого соединения, обычно можно обойтись добавлением объектов прямо в нужный линк напрямую. Когда же это все-таки нужно, то можно использовать соединение типа `revolute` с нулевыми пределами.

Запускаем Gazebo, добавляем нашего робота, и ставим напротив него какой-нибудь объект, примерно вот так.
![Mobot_cont3](/figs/model_added_camera.png)

Теперь, если открыть утилиту `rqt` в консоле, и выбрать там `image_view`, то можно увидеть видео-изображение нашего объекта! Наконец-то мы как-то взаимодействуем с нашим роботом из ROS'а!
![Mobot_cont3](/figs/rqt_camera.png)

Если вам интересна документация по имеющимся плагинам, то я в сети ее не нашел. С ros.org/wiki ее убрали, а в gazebosim не добавили, пока разве что изучать исходники [вот тут](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/jade-devel/gazebo_plugins/src)

#Добавим двигатель
Изображение с камеры мы получили, теперь бы проехаться.
Добавим в модель плагин вращающий колеса:
~~~~
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>100</updateRate>
      <leftJoint>left_wheel_hinge</leftJoint>
      <rightJoint>right_wheel_hinge</rightJoint>
      <wheelSeparation>${chassisWidth+wheelWidth}</wheelSeparation>
      <wheelDiameter>${2*wheelRadius}</wheelDiameter>
      <torque>20</torque>
      <commandTopic>mobot/cmd_vel</commandTopic>
      <odometryTopic>mobot/odom_diffdrive</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>chassis</robotBaseFrame>
    </plugin>
~~~~
Названия входных параметров говорят сами за себя. `leftJoint` и `rightJoint` это ссылки на соединители колес с базой. `torque` - вращающий момент силы. `commandTopic` - топик на который ему надо посылать команды и т.д.

Заного добавим нашу модель, предварительно удалив старую. А в новом терминале введем команду:
~~~~
$ 	rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/mobot/cmd_vel
~~~~
Это небольшая учебная утилита, которая отправляет geometry_msgs/Twist при нажатии клавиш стрелок на клавиатуре, попробуйте!
Особенно интересно управлять им, если смотреть на изображение с камеры.

В следующей части мы напишем простенький плагин на C++.
