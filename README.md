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
