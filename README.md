# Chase_ball
### Setting up my_robot

1. Create the my_robot Package

（1）Create and initialize acatkin_ws

```
  $ mkdir -p /home/workspace/catkin_ws/src
  $ cd /home/workspace/catkin_ws/src
  $ catkin_init_workspace
```
（2）Navigate to thesrcdirectory of yourcatkin_wsand create themy_robotpackage:

```
$ cd /home/workspace/catkin_ws/src/
$ catkin_create_pkg my_robot
```

（3）Next, create aworldsdirectory and alaunchdirectory, that will further define the structure of your package:

```
$ cd /home/workspace/catkin_ws/src/my_robot/
$ mkdir launch
$ mkdir worlds
```
2. Create and Store an Empty Gazebo World File

（1）Create an empty Gazebo world

```
$ cd /home/workspace/catkin_ws/src/my_robot/worlds/
$ touch empty.world
```

（2）Add the following toempty.world

```
<?xml version="1.0" ?>
```

### Robot Basic Setup
1. Create the URDF File

（1）Create aurdfdirectory in themy_robotpackage

```
$ cd /home/workspace/catkin_ws/src/my_robot/
$ mkdir urdf
```

（2）Create the robot’sxacrofile inside theurdfdirectory

```
$ cd /home/workspace/catkin_ws/src/my_robot/urdf/
$ touch my_robot.xacro
```

（3）Copy the following code intomy_robot.xacrofile

```
<?xml version='1.0'?>

<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

</robot>
```

2. Launch the Robot

（1）Create a new launch file to load theURDFmodel file

```
$ cd /home/workspace/catkin_ws/src/my_robot/launch/
$ touch robot_description.launch
```

（2）Copy the following code intorobot_description.launchfile

```
<?xml version="1.0"?>
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'" />

</launch>
```

To generate the URDF file from the Xacro file, you must first define a parameter,robot_description. This parameter will set a single command to use the xacro packageto generate the URDF from the xacro file.

（3）Update the world.launch file created earlier so that Gazebo can load the robot URDF model

```
<!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>

  <!-- Launch other relevant files-->
  <include file="$(find my_robot)/launch/robot_description.launch"/>
  ```
  
Add the following to the launch file (before</launch>):

```
<!-- Find my robot Description-->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'"/>

  <!-- Spawn My Robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" 
        args="-urdf -param robot_description -model my_robot 
              -x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>
 ``` 
 
（4）Launch

```
$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

### Robot Enhancement
1. Create Wheel Links

```
<!--Link (left_wheel)-->  
  <link name='left_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='left_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='left_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link> 
<!--Link (right_wheel)-->  
  <link name='right_wheel'>    
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>
    
    <collision name='right_wheel_collision'>
      <origin  xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>
    </collision>
    
    <visual name='right_wheel_visual'>
      <origin xyz="0 0 0" rpy="0 1.5707 1.5707"/>
      <geometry>
        <cylinder length="0.05" radius="0.1"/>
      </geometry>  
    </visual>
    
  </link>
  ```
  
2. Create Joints for the two wheels

```
<!--Joint (left_wheel_hinge)-->
  <joint name="left_wheel_hinge" type="continuous">
    <origin xyz="0 0.125 0" rpy="0 0 0"/>
    <child link="left_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>
<!--Joint (right_wheel_hinge)-->
  <joint name="right_wheel_hinge" type="continuous">
    <origin xyz="0 -0.125 0" rpy="0 0 0"/>
    <child link="right_wheel" />
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0" />
    <limit effort="10000" velocity="1000"/>
    <joint_properties damping="1.0" friction="1.0" />
  </joint>

<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```

3. Create a Launch File

（1）Create theworld.launchfile

```
$ cd /home/workspace/catkin_ws/src/my_robot/launch/
$ touch world.launch
```

（2）Add the following toworld.launch

```
<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <!-- World File -->
  <arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

</launch>
```

4. Launch empty.world

```
$ cd /home/workspace/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
