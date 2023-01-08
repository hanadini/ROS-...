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
