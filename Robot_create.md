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
В папке src/mobot будут находится пакеты связанные с роботом, включая mobot_gazebo, который интегрирует робота в Gazebo, в папке src/models/mobot_description будет находится описание модели робота. 
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
##Введение в sfd формат
Создадим файл sfd/model.sdf и добавим в него содержимое:
~~~~
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="mobot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  </model>
</sdf>
~~~~
Описание самой модели прописывается между тэгами `model`, добавим между тэгами следующий код:
~~~~
<link name='chassis'>
  <pose>0 0 0.1 0 0 0</pose>
  <collision name='collision'>
    <geometry>
      <box>
        <size>0.5 0.2 0.1</size>
      </box>
    </geometry>
  </collision>

  <visual name='visual'>
    <geometry>
      <box>
        <size>0.5 0.2 0.1</size>
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
    <mass>5</mass> 
    <inertia>
      <ixx>0.2</ixx>
      <iyy>0.3</iyy>
      <izz>0.2</izz>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </inertial>
</link>
~~~~
Тэг `link` определяет базовый структурный элемент, их может быть несколько в одной моделе.
Внутри `link` нужно определить тэги:
- `pose` в формате xyz rpy для определения позиции относительно начала координат модели, этот тэг используется очень часто и определяет позицию относительно родительского тэга!
- `collision`(их может быть много внутри однога линка) используется для просчета столкновений, в нем можно использовать как стандартные примитивы, так и импортировать модель форматов .dae или .stl. Здесь же описываются свойства поверхности вроде трения. Добавив сюда `pose` можно сдвинуть мэш относительно родительского линка
- `visual`(их так же может быть много) добавляется для рендеринга, здесь также можно использовать как стандартные примитивы, так и импортировать модель форматов .dae или .stl. Здесь же добавляются текстуры. Добавив сюда `pose` можно сдвинуть мэш относительно родительского линка
- `inertial`(он в линке один) описывает физические инерциальные свойства линка: его массу, тензор инерции и т.д. Добавив сюда `pose` можно сдвинуть координаты центра масс относительно родительского линка

Тэги `link`, `collision`, `visual` обязательно должны быть снабжены уникальными именами! 

Подробнее про спецификации sdf формата написано [здесь](http://sdformat.org/spec) (в нашей версии Gazebo нужно выбрать 1.4 версию sdf)

Запустим Gazebo командой:
~~~~
roslaunch gazebo_ros empty_world.launch 
~~~~
Выберем вкладку insert и добавим оттуда модель Mobot. Должно появиться что-то такое:
![Mobot_begin](/figs/model_begin.png)


##Создание sdf файла модели робота с помощью утилиты xacro

Теперь, когда структура sdf файла стала примерно понятна, продолжим её изучение, попутно познакомившись с утилитой xacro. 

Название xacro происходит от Xml mACROs. Это простая утилита, однако она существенно помогает в создании сложных xml файлов в том числе sdf и urdf. 
Xacro позволяет импортировать один sdf файл в другой, определять переменные, создавать макросы, производить не сложные арифметические вычисления и т.д. 


Удалим файл model.sdf и создадим в папке sdf файл с названием model.sdf.xacro, добавим в него следующее содержание:
~~~~
<?xml version='1.0'?>
<sdf version='1.4'>
  <model name="mobot" xmlns:xacro="http://www.ros.org/wiki/xacro">
~~~~
Далее добавим определения переменных:
~~~~
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
~~~~
Импортируем файл с макросами, который мы напишем позже,
обратите внимание на $(find mobot_description)
~~~~
    <xacro:include filename="$(find mobot_description)/sdf/macros.xacro" />
~~~~
Ниже добавим описание линка базы робота, тэг `static` указывает на то, что модель пока статична, это полезно указывать на момент формирования модели
~~~~
    <static>True</static>
~~~~
Обратите внимание, что в тексте ниже несколько `visual` и `collision` в одном линке, так же в `collision` кастера изменены свойства поверхности, а именно добавлено проскальзывание и убрано трение.
Кастер это сфера без трения, которая добавлена, чтобы роботу хватило только двух колес.
~~~~
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
Теперь создадим там же файл model.xacro и определим там несколько макросов, синтаксис говорит сам за себя.
~~~~
<?xml version='1.0'?>
<model>

  <macro name="cylinder_inertia" params="m r h">
    <inertia>
      <ixx>${m*(3*r*r+h*h)/12}</ixx>
      <iyy>${m*(3*r*r+h*h)/12}</iyy>
      <izz>${m*r*r/2}</izz>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </macro>

  <macro name="box_inertia" params="m x y z">
    <inertia>
      <ixx>${m*(y*y+z*z)/12}</ixx>
      <iyy>${m*(x*x+z*z)/12}</iyy>
      <izz>${m*(x*x+y*y)/12}</izz>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </macro>

  <macro name="sphere_inertia" params="m r">
    <inertia>
      <ixx>${2*m*r*r/5}</ixx>
      <iyy>${2*m*r*r/5}</iyy>
      <izz>${2*m*r*r/5}</izz>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyz>0</iyz>
    </inertia>
  </macro>
</model>
~~~~
Далее с помощью xacro создаем sdf файл:
~~~~
rosrun xacro xacro -o model.sdf model.sdf.xacro
~~~~
Запустим Gazebo и посмотрим, что получилось.
![Mobot_cont1](/figs/model_cont1.png)

###Добавление колес
Так как колеса отдельные физические объекты в нашем роботе, их надо добавлять в свои линки. Добавим для этого макросы в файл macros.xacro:
~~~~
<macro name="wheel" params="lr tY">
  <link name="${lr}_wheel">
    <pose>${wheelPos} ${tY*wheelWidth/2+tY*chassisWidth/2} ${wheelRadius} 0 ${PI/2} ${PI/2}</pose>
    <collision name="${lr}_wheel_collision">
      <geometry>
        <cylinder>
          <radius>${wheelRadius}</radius>
          <length>${wheelWidth}</length>
        </cylinder>
      </geometry>
~~~~
Обратите внимание на параметры. "lr" следует задать "left" или "rigth", а tY - смещение по оси Y, 1 или -1 в зависимости от колеса.
Зададим колесам трение:
~~~~
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
            <slip1>0.0</slip1>
            <slip2>0.0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>
~~~~
Описываем визуальную и инерциальную часть:
~~~~
    <visual name="${lr}_wheel_visual">
      <geometry>
        <cylinder>
          <radius>${wheelRadius}</radius>
          <length>${wheelWidth}</length>
        </cylinder>
      </geometry>
      <material>
        <script>
          <name>Gazebo/Black</name>
          <uri>__default__</uri>
        </script>
      </material>
    </visual>

    <inertial>
      <mass>${wheelMass}</mass>
      <cylinder_inertia m="${wheelMass}" r="${wheelRadius}" h="${wheelWidth}"/>
    </inertial>
  </link>
~~~~
Мы задали колеса, однако теперь их надо соединить с базой:
~~~~
  <joint name="${lr}_wheel_hinge" type="revolute">
    <parent>chassis</parent>
    <child>${lr}_wheel</child>
    <pose>0 0 ${wheelWidth/2} 0 0 0</pose>
    <axis>
      <xyz>0 1 0</xyz>
      <dynamics>
        <damping>${wheelDamping}</damping>
      </dynamics>
      <limit>
        <effort>100</effort>
        <velocity>100</velocity>
      </limit>
    </axis>
  </joint>
</macro>
~~~~
Тэг `joint` используется чтобы соединять линки между собой. Ему нужно задать уникальное имя и тип.
Наиболее часто испольюзуются типы revolute и continuous. `child` крепится к 'parent' 
- `xyz` определяет по какой оси будет вращение, 
- `limit` задает максимальные моменты сил Н*м которые можно задать по оси, 
- `velocity` - максимальную скорость. 
- `damping` определяет возникающее сопротивление пропорционально скорости. Варьированием этого параметра и момента силы которая будет прикладываться на ось задается максимальная скорость вращения сочленения и его динамика.
Подробнее [тут](http://sdformat.org/spec?ver=1.4&elem=joint#joint_type).

Теперь добавим эти колеса в основной файл сразу после "chassis" линка:
~~~~
    <wheel lr="left" tY="1"/>
    <wheel lr="right" tY="-1"/>
~~~~
Снова вызываем утилиту xacro - не забываем это делать каждый раз, когда меняем файлы xacro.
~~~~
rosrun xacro xacro -o model.sdf model.sdf.xacro
~~~~
Импортируем модель в Gazebo и смотрим, что получилось:
![Mobot_cont2](/figs/model_cont2.png)

В следующей части мы добавим камеру и научимся взаимодействовать им из ROS'а.
