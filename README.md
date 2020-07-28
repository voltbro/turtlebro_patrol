# Turtlebro_patrol ROS package for Turtlebro patrolbot

### Dependecies
Dependences
To perform patrolling tasks, you should have navigation packages installed on the robot:

* turtlebro_navigation
* amcl
* dwa_local_planner
* global_planner
* gmapping
* map_server
* move_base
* move_base_msgs


### Package installation
Install the package on RaspberryPi in the "standard" way:

```
cd ros_catkin_ws/src
git clone https://github.com/voltbro/turtlebro_patrol
cd ..
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=turtlebro_patrol
```

### Launch
```
#  Navigation and patrol node launch
roslaunch turtlebro_patrol patrol.launch

#  Patrol node launch only
roslaunch turtlebro_patrol patrol_run.launch
```

### Configuring patrolling
The coordinates of the points where the robot starts patrolling are located in the file /data/goals.xml 
After changing the data in the file, you should rebuild the package.

### Patrol control
The control of the patrol bot is performed by sending messages of the std_msgs/String type to the topic /patrol_control

Accepted commands:
1. Start -starts the patrol cycle
2. Pause -pauses patrolling at any point
3. Resume -resumes a patrol that was stopped by the Pause command
4. Home -stops patrolling and sends the robot to a point with coordinates 0, 0
5. Stop -stops patrolling and executing the package
