if pgrep -x "roscore" > /dev/null
then
    echo "roscore is running"
else
    echo "roscore is not running. Starting roscore ..."
    gnome-terminal -- "roscore"
fi
source devel/setup.bash
sudo chmod 666 /dev/ttyUSB0
wait
gnome-terminal -- bash -c "source devel/setup.bash; echo 'Publishing roll/pitch/yaw'; rosrun my_robot_visualization read_serial.py; exec bash"
gnome-terminal -- bash -c "source devel/setup.bash; rosrun my_robot_visualization rviz_visualization.py; exec bash"
rosrun rviz rviz -d stm32_rviz.rviz &
