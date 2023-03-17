import serial
import rospy
from std_msgs.msg import Float64

ser = serial.Serial('/dev/ttyUSB0', 115200) # adjust baudrate and serial port as necessary
roll_pub = rospy.Publisher('roll', Float64, queue_size=10)
pitch_pub = rospy.Publisher('pitch', Float64, queue_size=10)
yaw_pub = rospy.Publisher('yaw', Float64, queue_size=10)

rospy.init_node('read_serial_node')

def parse_data(data):
    data = data.decode('utf-8')
    data = data.strip().split(', ')
    roll = float(data[0].split(': ')[1])
    pitch = float(data[1].split(': ')[1])
    yaw = float(data[2].split(': ')[1])
    return roll, pitch, yaw

def read():
    while not rospy.is_shutdown():
        data = ser.readline()
        # Parse roll, pitch, yaw from data
        roll, pitch, yaw = parse_data(data)
        roll_pub.publish(roll)
        pitch_pub.publish(pitch)
        yaw_pub.publish(yaw)

if __name__ == '__main__':
    try:
        read()
    except UnicodeDecodeError:
        read()
    except IndexError:
        read()




