# Mobile_Robot (Chase_ball) 

![demo](https://user-images.githubusercontent.com/54982873/211189309-9cfc9164-1d02-4022-8299-cb2a36fb7da3.gif)

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

### Robot Sensors
1. Add a Camera

First, add the camera link and a corresponding joint. Open themy_robot.xacrofile and add a camera sensor based on the following specifications:

```
<!--Link (camera)-->
  <link name='camera'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <box_inertia m="0.1" x="0.05" y="0.05" z="0.05"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='camera_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </visual>

    <collision name='camera_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".05 .05 .05"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (camera)-->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.225 0 0" rpy="0 0 0"/>
    <child link="camera"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>
  ```
  
2. Add a Lidar

The Hokuyo sensor can be added to your robot model just like the camera sensor, except that you first need to add a mesh file to your robot model. Mesh files define the shape of the object or model you are working with. There are some basic shapes, like the box or cylinder, that do not require a mesh file. However, for more advanced designs, mesh files are necessary. The mesh file should be located in a directory calledmeshesthat you can create in your package folder,my_robot.

（1）Createmeshesdirectory

```
$ cd /home/workspace/catkin_ws/src/my_robot/
$ mkdir meshes
```

（2）now, download thishokuyo.daefile and place it under themeshesdirectory you just created.

（3）Add the Hokuyo sensor tomy_robot.xacro

```
<!--Links (hokuyo)-->
  <link name='hokuyo'>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <visual name='hokuyo_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://my_robot/meshes/hokuyo.dae"/>
      </geometry>
    </visual>

    <collision name='hokuyo_collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".1 .1 .1"/>
      </geometry>
    </collision>
  </link>

  <!--Joint (hokuyo)-->
  <joint type="fixed" name="hokuyo_joint">
    <origin xyz="0.175 0 .085" rpy="0 0 0"/>
    <child link="hokuyo"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0"/>
  </joint>
  ```
  
3. Launch

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

### Gazebo Plugins
Gazebo allows for plugins that implement specific use-cases.

Sensor and Actuators Plugins
We will cover the use of three such plugins:

A plugin for the camera sensor.

A plugin for the Hokuyo lidar sensor.

A plugin for the wheel joints actuator.

1. Add Plugins

Download themy_robot.gazebofile, which includes the 3 plugins mentioned above, and place it inside theurdfdirectory ofmy_robot.

2. Gazebo Plugin Files

```
<?xml version="1.0"?>
<robot>

  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <legacyMode>false</legacyMode>
      <alwaysOn>true</alwaysOn>
      <updateRate>10</updateRate>
      <leftJoint>left_wheel_hinge</leftJoint>
      <rightJoint>right_wheel_hinge</rightJoint>
      <wheelSeparation>0.35</wheelSeparation>
      <wheelDiameter>0.2</wheelDiameter>
      <torque>10</torque>
      <commandTopic>cmd_vel</commandTopic>
      <odometryTopic>odom</odometryTopic>
      <odometryFrame>odom</odometryFrame>
      <robotBaseFrame>robot_footprint</robotBaseFrame>
      <publishWheelTF>false</publishWheelTF>
      <publishWheelJointState>false</publishWheelJointState>
      <rosDebugLevel>na</rosDebugLevel>
      <wheelAcceleration>0</wheelAcceleration>
      <wheelTorque>5</wheelTorque>
      <odometrySource>world</odometrySource>
      <publishTf>1</publishTf>
      <publishOdomTF>true</publishOdomTF>
    </plugin>
  </gazebo>

  <!-- camera -->
  <gazebo reference="camera">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>800</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>rgb/image_raw</imageTopicName>
        <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
        <frameName>camera</frameName>
        <hackBaseline>0.07</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>

  <!-- hokuyo -->
  <gazebo reference="hokuyo">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>hokuyo</frameName>
      </plugin>
    </sensor>
  </gazebo>


</robot>
```

3. ROS Communication

You need to define the topics to which each sensor publishes.

For the wheel joints, it's the cmd_vel topic.

```
<commandTopic>cmd_vel</commandTopic>
```

For the camera, it's the rgb/image_raw topic.

```
<imageTopicName>rgb/image_raw</imageTopicName>
```

And for the lidar, it's the scan topic

```
<topicName>/scan</topicName>
```

4. Import Plugins

Import the sensors plugins by adding the following code to the top of the file (immediately before you define therobot_footprintlink):

```
<!--Import Plugins-->
  <xacro:include filename="$(find my_robot)/urdf/my_robot.gazebo" />
```

### RViz Integration
you will display your model into RViz and visualize data from thecameraandlidarsensors. You will alsoactuateyour robot and drive it around!

1. Modify robot_description

Start by modifying therobot_description.launchfile. Open it and add these lines after the firstparamdefinition.

```
<!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
```

Those elements add two nodes - the joint_state_publisher and the robot_state_publisher.

joint_state_publisher: Publishes joint state messages for the robot, such as the angles for the non-fixed joints.
robot_state_publisher: Publishes the robot's state to tf (transform tree). Your robot model has several frames corresponding to each link/joint. The robot_state_publisher publishes the 3D poses of all of these links. This offers a convenient and efficient advantage, especially for more complicated robots.

2. Modify world.launch

Next, you need to launch RViz along with Gazebo. Open theworld.launchfile and add these elements after theurdf_spawnernode definition:

```
<!--launch rviz-->
<node name="rviz" pkg="rviz" type="rviz" respawn="false"/> 
```

3. Launch!

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

4. RViz Setup

Setup RViz to visualize the sensor readings. On the left side of RViz, under Displays:

Select odom for fixed frame
Click the Add button and
add RobotModel and your robot model should load up in RViz.
add Camera and select the Image topic that was defined in the camera Gazebo plugin
add LaserScan and select the topic that was defined in the Hokuyo Gazebo plugin

5. Add Objects

In Gazebo, add a box, sphere or cylinder object in front of your robot. Your robot’s sensors should be able to visualize it. You can check theCameraviewer on the bottom left side of RViz to see a picture of the new object. Also, you can see a redLaserscan inside the scene, reflecting from your object.

6. Drive Around

While everything above is still running, test the robot’s actuators and drive it around. Open a new terminal window and publish velocity commands to the robot’s wheel actuators:

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
  
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```

### House your Robot
1. Adding the World File

Copy the <yourname>.world file from the world directory of the Build My World project and paste it in the worlds directory of my_robot.

Inside your package’s worlds directory you should now see two files - the empty.world that we created earlier and the <yourname>.world file that you just added.

Feel free to delete the empty.world file. We won’t need it anymore.

2. Launch the World

Edit the world.launch file and add a reference to the <yourname>.world file that you just added. To do so, open the world.launch file and edit this line:

  ```
<arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>
  ```
  
Replace it with this:
  
```
<arg name="world_file" default="$(find my_robot)/worlds/<yourname>.world"/>
```
  
now

```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
 ``` 

![image](https://user-images.githubusercontent.com/54982873/211188329-e5b1366f-a7bf-4092-b1bc-00a3f93f1bb7.png)
  
 3. Initialize the Robot’s Position and Orientation

As you can see, my robot’s initial position is outside of my world! You might face the same problem. I have to change my robot’s initial pose: its position and orientation. This can be done through editing theworld.launchfile:

  ```
<!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>
  ```
  
4. my_robot.xacro

  ```
<!--give your robot some color-->
  <gazebo reference="chassis">
    <material>Gazebo/Yellow</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="camera">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="hokuyo">
    <material>Gazebo/Black</material>
  </gazebo>
  ```
  
### Setting up ball_chaser
	
1. Package Nodes
The ball_chaser package will have two C++ nodes: the drive_bot and process_image

**drive_bot**: This server node will provide a ball_chaser/command_robotservice to drive the robot around by controlling its linear x and angular z velocities. The service will publish a message containing the velocities to the wheel joints.
process_image: This client node will subscribe to the robot’s camera images and analyze each image to determine the position of the white ball. Once ball position is determined, the client node will request a service to drive the robot either left, right or forward.
Now, follow along with the steps to set up ball_chaser.

2. Create the ball_chaser Package

（1）Navigate to thesrcdirectory of yourcatkin_wsand create theball_chaserpackage:

  ```
$ cd /home/workspace/catkin_ws/src/
$ catkin_create_pkg ball_chaser roscpp std_msgs message_generation
  ```
  
（2）Next, create ansrvand alaunchfolder, which will further define the structure of your package:

  ```
$ cd /home/workspace/catkin_ws/src/ball_chaser/
$ mkdir srv
$ mkdir launch
  ```
  
（3）Build the Package

  ```
$ cd /home/workspace/catkin_ws/
$ catkin_make
  ```
  
3. ROS Node: drive_bot

（1）ROS Service File

Write theDriveToTarget.srvfile

  ```
float64 linear_x
float64 angular_z
---
string msg_feedback
  ```
  
（2）TestDriveToTarget.srv

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rossrv show DriveToTarget
  ```
  
（3）drive_bot.cpp Node

Now it’s time to write thedrive_bot.cppserver node that will provide theball_chaser/command_robotservice. Create the script under thesrcdirectory of yourball_chaserpackage.

  ```
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
  
// Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

//  Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
// This callback function executes whenever a safe_move service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Publish angles to drive the robot
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Velocities set - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //  Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    //  Handle ROS communication events
    ros::spin();

    return 0;
}
  
  ```
  
（4）Edit CMakeLists.txt

After you write the server node in C++, you’ll have to add the following dependencies:

- Add the ```add_compile_options``` for C++ 11 dependency, this step is optional and depends on your code
  
- Add the ```add_service_files``` dependency which defines the DriveToTarget.srv file
  
- Add the ```generate_messages``` dependency
  
- Add ```include_directories``` dependency
  
- Add the ```add_executable```, ```target_link_libraries```, and ```add_dependencies``` dependency for your drive_bot.cppscript

	```
cmake_minimum_required(VERSION 2.8.3)
  
project(ball_chaser)

  
## Compile as C++11, supported in ROS Kinetic and newer
  
add_compile_options(-std=c++11)

## Find catkin macros and libraries
  
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  
## is used, also find other catkin packages
  
find_package(catkin REQUIRED COMPONENTS
  
  message_generation
  
  roscpp
  
  std_msgs
)
################################################
	
 ## Declare ROS messages, services and actions ##
	
################################################
	
## Generate services in the 'srv' folder

	```
add_service_files(
   FILES
   DriveToTarget.srv
)
	
## Generate added messages and services with any dependencies listed here
	
generate_messages(
   DEPENDENCIES
   std_msgs
)
	
###################################
	
## catkin specific configuration ##
	
###################################
	
	
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ball_chaser
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)
	
###########
	
## Build ##
	
###########
	
## Specify additional locations of header files
## The package locations should be listed before other locations
  
	
include_directories(
  
# include
  
  ${catkin_INCLUDE_DIRS}
)
  
  
## Declare a C++ executable
 
  
add_executable(drive_bot src/drive_bot.cpp)

  
## Add cmake target dependencies of the executable
  
  
add_dependencies(drive_bot ball_chaser_generate_messages_cpp)

  
## Specify libraries to link a library or executable target against
	
  
target_link_libraries(drive_bot ${catkin_LIBRARIES})
 ```
  
（5）Build Package

  ```
$ cd /home/workspace/catkin_ws/
$ catkin_make
  ```
  
4. Test drive_bot.cpp

（1）Launch the robot inside your world

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch 
  ```
  
（2）Run thedrive_botnode

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun ball_chaser drive_bot
  ```
  
（3）Request aball_chaser/command_robotservice

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash

$ rosservice call /ball_chaser/command_robot "linear_x: 0.5
angular_z: 0.0"  # This request should drive your robot forward

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.5"  # This request should drive your robot left

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: -0.5"  # This request should drive your robot right

$ rosservice call /ball_chaser/command_robot "linear_x: 0.0
angular_z: 0.0"  # This request should bring your robot to a complete stop
  ```
  
5.Launch Files

Let’s add thedrive_botnode to a launch file. Create aball_chaser.launchfile under thelaunchdirectory of yourball_chaserpackage and then copy this code to it:

  ```
<launch>

 <!-- The drive_bot node -->
  <node name="drive_bot" type="drive_bot" pkg="ball_chaser" output="screen">
  </node>

</launch>
  ```
  
### Model a White Ball
1. Model Editor

  ```
$ gazebo # then Edit-> Model Editor
  ```
  
2. Insert Sphere

Under thesimple shapesmenu of the Model Editor tool, click on a sphere and insert it anywhere in the scene.

3.Edit Size

Double click on the sphere, and change its radius to0.1both inVisualandCollision.

4. Change Color

To change the ball’s color to white, set itsVisualAmbient, Diffuse, Specular, and EmissiveRGBAvalues to 1.

5. Save

Save the white ball model asmy_ballunder the/home/workspacedirectory. Then exit the Model Editor tool and go back to the Gazebo main world. 
  
6. Insert Ball

Now that you are back in the Gazebo main world, you can click on “Insert” and drop the white ball anywhere in the scene.
  
  7. Relaunch Nodes

Now that you modeled the white ball, relaunch the nodes insideworld.launch. Then verify that you can insert amy_ballanywhere inside your world.

8. Save

Place the white ball anywhere outside of your building structure, so that the robot would not see it. Then, save a copy of this new world under/home/workspace/catkin_ws/src/my_robot/worldsby replacing your old<yourname>.worldfile. Whenever you launch this newly saved world you should be able to see your building environment, in addition, the white ball.
  
### ROS Node: process_image
  
1. Write process_image.cpp

  ```
#include <iostream>
#include <algorithm>
#include <vector>
using namespace std;

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    //  Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
	
    // Call the command_robot service and pass the requested joint angles
    if (!client.call(srv)){
        ROS_ERROR("Failed to call service command_robot");
    }	
}

void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int left_counter = 0;
    int front_counter = 0;
    int right_counter = 0;
	
    //  
    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < img.height * img.step; i += 3) {
        int position_index = i % (img.width * 3) / 3;
	
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            if(position_index <= 265) {
		left_counter += 1;                
            }
            if(position_index > 265 && position_index <= 533) {
		front_counter += 1;               
            }
            if(position_index > 533) {
		right_counter += 1;                
            }
	}
    }
		
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    vector<int> position_counter{left_counter, front_counter, right_counter};
    int where_to_move = *max_element(position_counter.begin(), position_counter.end());

    // Depending on the white ball position, call the drive_bot function and pass velocities to it.
    // Request a stop when there's no white ball seen by the camera.
    if (where_to_move == 0){
        drive_robot(0.0, 0.0); // This request brings my_robot to a complete stop
    }
    else if (where_to_move == left_counter) {
	drive_robot(0.0, 0.5);  // This request should drive my_robot left
    }
    else if (where_to_move == front_counter) {
        drive_robot(0.5, 0.0);  // This request drives my_robot robot forward
    }
    else if (where_to_move == right_counter) {
        drive_robot(0.0, -0.5); // This request drives my_robot right
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
  
  ```
 
2. Edit CMakeLists.txt

In addition to all the dependencies you added earlier for drive_bot.cpp, these are the dependencies that you should add for process_image.cpp :

  ```
Add add_executable
Add target_link_libraries
Add add_dependencies
add_executable(process_image src/process_image.cpp)
add_dependencies(process_image ball_chaser_generate_messages_cpp)
target_link_libraries(process_image ${catkin_LIBRARIES})
```
  
3. Build Package

  ```
$ cd /home/workspace/catkin_ws/
$ catkin_make
  ```
  
4. Launch File

Edit theball_chaser.launchfile

  ```
<!-- The process_image node -->
  <node name="process_image" type="process_image" pkg="ball_chaser" output="screen">
  </node>
  
  ```
  
5. Test process_image

（1）Launch the robot inside your world

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch my_robot world.launch
  ```
  
（2）Rundrive_botandprocess_image

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
  ```
  
（3）Visualize

  ```
$ cd /home/workspace/catkin_ws/
$ source devel/setup.bash
$ rosrun rqt_image_view rqt_image_view
  ```
 ![image](https://user-images.githubusercontent.com/54982873/211188962-4ec1daef-ca02-4428-9e89-7643e497690b.png)
 
