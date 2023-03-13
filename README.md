HOW TO DOWNLOAD:
```
sudo apt install git -y
git clone git@github.com:David-LeK/CapstoneProject.git
```
Qt version: 5.15.2

# UBLOX ROS
To install the ublox package, you can use the following command in a terminal:
```
sudo apt-get install ros-$ROS_DISTRO-ublox
```
Ex:
```
sudo apt-get install ros-noetic-ublox
```
Alternatively, you can clone the repository from https://github.com/KumarRobotics/ublox and build it with catkin_make.

To configure your u-blox GPS receiver with the UBX protocol and the desired settings, you can use u-center or any other tool that can communicate with the device. For example, you can use minicom to send UBX commands to the device via serial port. You can find some UBX commands in ublox_msgs/msg folder. For example, to set the baud rate to 115200 bps, you can send this command:
```
B5 62 06 00 14 00 01 00 00 00 D0 08 00 00 80 25
00 00 07 00 03 00 00 00 C0 A8 B9 B5
```
You can also use rosservice call /ublox_gps/set_baudrate [baudrate] if you have already launched the node.

To connect your u-blox GPS receiver to your computer via serial port, USB, TCP or UDP, you need to specify the connection type and parameters in your .yaml configuration file. For example, if your device is connected via USB with /dev/ttyACM0 port name, you can use this line in your .yaml file:
```
device: /dev/ttyACM0
```
You can also specify other parameters such as frame_id, rate, nav_rate, etc. in your .yaml file.

To launch the ublox_gps node with a .yaml configuration file that matches your device and settings, you can use roslaunch command with the name of your .yaml file as an argument. For example, if your .yaml file is named c94_m8p_rover.yaml and it is located in ublox_gps/config folder, you can use this command:
```
roslaunch ublox_gps ublox_device.launch param_file_name:=c94_m8p_rover.yaml
```
This will launch the node and load the parameters from your .yaml file.

The node will publish GPS data on several topics, such as /fix, /navsat/fix, /navsat/vel etc. You can use rostopic echo [topic] to see the data on any topic. For example,
```
rostopic echo /fix
```
will show you the latitude, longitude and altitude of your device.

You can also subscribe to some services to set or get parameters from the device. For example,
```
rosservice call /ublox_gps/get_version "{}"
```
will show you the firmware version of your device.

# MPU9250
## Arduino
* https://github.com/twogoldteeth/mpu9250
* https://www.hackster.io/30503/using-the-mpu9250-to-get-real-time-motion-data-08f011
## STM32
* https://github.com/desertkun/MPU9250
* https://www.youtube.com/watch?v=UEnWlSgGPiE
* https://github.com/xtr0d3m0n/MPU9250-STM32-HAL-libary
