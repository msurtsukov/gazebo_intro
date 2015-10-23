# Описание

Gazebo - это программный пакет, моделирующий взаимодействие робота или даже популяции роботов с физическим миром.
Детально описав робота можно тестировать как работу алгоритмов, так и физическую реализацию робота в виртуальной среде, до того, как собирать его в железе.
Физический движок использующийся в Gazebo позволяет учитывать такие тонкости взаимодействия робота со средой как трение, мощность моторов, имитация сенсоров и камеры и т.д.

##Вкратце о работе с Gazebo
Сначала в xml-формате создается модель робота, каждая функциональная часть (вроде базы, колес, камеры и т.д.) описывается отдельно, затем эти части соединяются вместе в модель робота.

К таким частям работа, как камеры, моторы, сенсоры и т.д. необходимо подключать плагины, которые пишутся на C++ и описывают функциональность этих частей.
Например простейший плагин на мотор прикладывает момент силы к некоторой оси, при получении соответствующей команды (ROS-сообщения как вариант).

Можно подключать плагины не только непосредственно к модели и сенсорам, но и к миру целиком.

##Gazebo и ROS
Замечание: Gazebo не нужен ROS для работы, в нем успешно можно работать и вообще без ROS'а. Однако это намного менее удобно.
Именно наличие некоторого количества уже написанных плагинов взаимодействующих методами ROS (сообщения, сервисы, параметры) и интегрируют Gazebo в ROS.
ROS Indigo работает с Gazebo 2.2, ROS Jade работает с Gazebo 5.0.


##Установка и настройка
###ROS
Если у вас настроена полная версия ROS, то пропускаем этот шаг.

Устанавливаем ROS-indigo-desktop-full, как написано здесь: установка ROS,
во время установки обязательно устанавливаем полную версию
Настраиваем рабочую папку catkin, как написано здесь: настройка рабочей папки
Если вы не знакомы с ROS, то желательно сначала пройти уроки для начинающих здесь: beginner tutorials
Gazebo
Gazebo установился вместе с ROS'ом на предыдущем шаге. Убедитесь, что он работает, введя команду в терминал:
~~~~
$ roslaunch gazebo_ros empty_world.launch
~~~~
Вы должны увидеть, как запускается окно Gazebo.
![Gazebo](/figs/gazebo_overview.png)
Запуск через roslaunch запускает как ядро ROS roscore, так и пару клиент-сервер Gazebo.
Обновите путь по которому Gazebo ищет модели:
~~~~
$ mkdir ~/catkin_ws/src/models 
$ echo “export GAZEBO_MODEL_PATH=~/catkin_ws/src/models:GAZEBO_MODEL_PATH” >> ~/.bashrc
~~~~

На этом первоначальную настройку можно считать законченной.
#Создание робота
На примере создания небольшой рабочей модели робота познакомимся с основными моментами Gazebo. 
Начнем с создания структуры файлов и пакетов для нашего робота в рабочей папке.
В терминале:
~~~~
$ mkdir ~/catkin_ws/src/mobot
$ cd ~/catkin_ws/src/mobot
$ catkin_create_pkg mobot_gazebo gazebo_ros
$ mkdir mobot_gazebo/launch mobot_gazebo/worlds
$ cd ../models
$ catkin_create_pkg mobot_description xacro
$ mkdir mobot_description/sdf
~~~~
В папке src/mobot будут находится пакеты связанные с роботом, включая mobot_gazebo, который интергрирует робота в Gazebo, в папке src/models/mobot_description будет находится описание модели робота. 
##Описание модели робота 
3-х мерная физическая модель робота в Gazebo описывается с помощью файла формата sdf, близкого родственника urdf, который обычно используется в ROS’е. В сети множество туториалов как использовать urdf в Gazebo, однако все они основаны на невероятно забагованной стандартной утилите, которая преобразует один формат в другой. Я не рекомендую пользоваться urdf для Gazebo, если urdf файл все же необходим, то проще написать его отдельно.

Создадим файл model.config в папке mobot_description со следующим содержанием 
~~~~
<?xml version="1.0"?>
<model>
  <name>Mobot</name>
  <version>1.0</version>
  <sdf version='1.4'>sdf/model.sdf</sdf>

  <author>
   <name>My Name</name>
   <email>me@my.email</email>
  </author>

  <description>
    My awesome robot.
  </description>
</model>
~~~~
В этом файле самое главное указать путь до sdf файла модели.
Создание sdf файла модели робота с помощью утилиты xacro
Теперь создадим в папке sdf файл с названием model.sdf.xacro и добавим в него следующее содержание:
~~~~
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="mobot" xmlns:xacro="http://www.ros.org/wiki/xacro">


    <xacro:property name="PI" value="3.1415926535897931"/>

    <xacro:property name="chassisHeight" value="0.1"/>
    <xacro:property name="chassisLength" value="0.4"/>
    <xacro:property name="chassisWidth" value="0.2"/>
    <xacro:property name="chassisMass" value="50"/>

    <xacro:property name="casterRadius" value="0.05"/>
    <xacro:property name="casterMass" value="5"/>

    <xacro:property name="wheelWidth" value="0.05"/>
    <xacro:property name="wheelRadius" value="0.1"/>
    <xacro:property name="wheelPos" value="0.12"/>
    <xacro:property name="wheelMass" value="5"/>
    <xacro:property name="wheelDamping" value="1"/>

    <xacro:property name="cameraSize" value="0.05"/>
    <xacro:property name="cameraMass" value="0.1"/>

    
    <xacro:include filename="$(find mobot_description)/sdf/macros.xacro" />

    <static>false</static>

    <link name='chassis'>
      <pose>0 0 ${wheelRadius} 0 0 0</pose>
      <collision name='collision'>
        <geometry>
          <box>
            <size>${chassisLength} ${chassisWidth} ${chassisHeight}</size>
          </box>
        </geometry>
      </collision>

      <visual name='visual'>
        <geometry>
          <box>
            <size>${chassisLength} ${chassisWidth} ${chassisHeight}</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Orange</name>
            <uri>__default__</uri>
          </script>
        </material>
      </visual>

      <inertial> 
        <mass>${chassisMass}</mass> 
        <box_inertia m="${chassisMass}" x="${chassisLength}" y="${chassisWidth}" z="${chassisHeight}"/>
      </inertial>

      <collision name='caster_collision'>
        <pose>${-chassisLength/3} 0 ${-chassisHeight/2} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>${casterRadius}</radius>
          </sphere>
        </geometry>

        <surface>
          <friction>
            <ode>
              <mu>0</mu>
              <mu2>0</mu2>
              <slip1>1.0</slip1>
              <slip2>1.0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name='caster_visual'>
        <pose>${-chassisLength/3} 0 ${-chassisHeight/2} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>${casterRadius}</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Red</name>
            <uri>__default__</uri>
          </script>
        </material>
      </visual>
    </link>

  </model>
</sdf>
~~~~
