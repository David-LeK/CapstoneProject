sudo apt install ros-noetic-amcl ros-noetic-navigation

#Execute at this directory

```
mv src/jetbot_navigation .
catkin_make
mv jetbot_navigation src/
catkin_make
source devel/setup.bash
roslaunch jetbot_control control.launch
```
