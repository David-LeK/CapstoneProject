#!/usr/bin/env python

from roslibpy import Message, Ros, Topic
from std_msgs.msg import Float32MultiArray

ros = Ros('103.200.20.166', 9090)
ros.run()

motor = Topic(ros, '/encoder', 'std_msgs/Float32MultiArray')
while ros.is_connected:
    arg = Float32MultiArray(data=[1.0, 2.0, 3.0])
    motor.publish(Message(arg))
