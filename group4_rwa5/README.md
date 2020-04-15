## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
# copy group4_rwa5 package here
cd ..
catkin_make
```

## Running the code

Instructions for running package:-
1. In terminal 1, do:-
```
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch group4_rwa5 group4_rwa5.launch 
```
2. In terminal 2, to start moveit for arm1 (https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/moveit_interface), do:-

```
source ~/catkin_ws/devel/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1
```

3. In terminal 3, to start moveit for arm2 (https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/moveit_interface), do:-

```
source ~/catkin_ws/devel/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm2
```

4. In terminal 4, to start the node, do:-
```
source ~/catkin_ws/devel/setup.bash
rosrun group4_rwa5 main_node
```