import rospy
from std_msgs.msg import Float64
from serial import Serial
import utm

def publish_topic(topic_name, message):
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    #rospy.loginfo(message)
    pub.publish(message)

def parse_rmc_sentence(rmc_sentence):
    rmc_fields = rmc_sentence.split(',')
    if len(rmc_fields) < 12 or rmc_fields[2] != 'A':
        return

    # Parse latitude and longitude
    latitude_degrees = int(rmc_fields[3][:2])
    latitude_minutes = float(rmc_fields[3][2:])
    longitude_degrees = int(rmc_fields[5][:3])
    longitude_minutes = float(rmc_fields[5][3:])
    longitude = longitude_degrees + (longitude_minutes / 60)
    latitude = latitude_degrees + (latitude_minutes / 60)
    if rmc_fields[4] == 'S':
        latitude *= -1
    if rmc_fields[6] == 'W':
        longitude *= -1

    utm_coord = utm.from_latlon(latitude, longitude)
    easting = utm_coord[0]
    northing = utm_coord[1]
    # Convert speed from knots to km/h
    speed_knots = float(rmc_fields[7])
    speed_kmh = speed_knots * 1.852

    # Publish each RMC field to a separate topic
    # publish_topic('time_utc', rmc_fields[1])
    # publish_topic('status', rmc_fields[2])
    publish_topic('latitude', latitude)
    publish_topic('longitude', longitude)
    publish_topic('speed_kmh', speed_kmh)
    publish_topic('track_angle', rmc_fields[8])
    # publish_topic('date', rmc_fields[9])
    # publish_topic('mode_indicator', rmc_fields[11])
    # publish_topic('nav_status', rmc_fields[12])
    publish_topic('easting', easting)
    publish_topic('northing', northing)
    

def serial_talker():
    rospy.init_node('gps_parse', anonymous=True)
    serial_port = Serial("/home/tien/Documents/COM9", baudrate=115200)
    while not rospy.is_shutdown():
        serial_data = serial_port.read(serial_port.inWaiting()).decode()
        nmea_sentences = serial_data.split("\n")
        for nmea_sentence in nmea_sentences:
            if "$GPRMC" in nmea_sentence:
                parse_rmc_sentence(nmea_sentence)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        serial_talker()
    except rospy.ROSInterruptException:
        pass

