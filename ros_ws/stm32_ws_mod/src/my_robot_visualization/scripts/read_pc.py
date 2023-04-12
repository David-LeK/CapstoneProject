import serial
import rospy
from std_msgs.msg import Float64
import time
import re
from std_msgs.msg import Float32MultiArray

ser_pc = serial.Serial('/home/tien/Documents/COM10', 115200) # Serial port for PC
data = ''

rospy.init_node('read_pc_node')

def parse_and_publish(string):
    # Parse the string to extract the values
    start_idx = string.index("<easting>") + len("<easting>")
    end_idx = string.index("</easting>")
    easting_values = string[start_idx:end_idx].split(",")
    
    start_idx = string.index("<northing>") + len("<northing>")
    end_idx = string.index("</northing>")
    northing_values = string[start_idx:end_idx].split(",")
    
    start_idx = string.index("<angle>") + len("<angle>")
    end_idx = string.index("</angle>")
    angle_values = string[start_idx:end_idx].split(",")
    
    # Convert the values to float and publish to ROS topics
    ref_easting_pub = rospy.Publisher('ref_easting', Float32MultiArray, queue_size=10)
    ref_northing_pub = rospy.Publisher('ref_northing', Float32MultiArray, queue_size=10)
    ref_angle_pub = rospy.Publisher('ref_angle', Float32MultiArray, queue_size=10)
    
    ref_easting_msg = Float32MultiArray(data=[float(x) for x in easting_values])
    ref_northing_msg = Float32MultiArray(data=[float(x) for x in northing_values])
    ref_angle_msg = Float32MultiArray(data=[float(x) for x in angle_values])
    
    ref_easting_pub.publish(ref_easting_msg)
    ref_northing_pub.publish(ref_northing_msg)
    ref_angle_pub.publish(ref_angle_msg)


def read():
    global data
    while not rospy.is_shutdown():
        print("It is running")
        if ser_pc.inWaiting() > 0:
            print("Went here")
            line = ser_pc.readline().decode('utf-8').strip()
            print(line)
            data += line
            match = re.search('<ref><easting>(.*?)</angle></ref>', data)
            if match:
                data = match.group(0)
                # Process data
                print(data)
                parse_and_publish(data)
                # Reset data
                data = ''
        else:
            time.sleep(0.1)
        

if __name__ == '__main__':
    read()




