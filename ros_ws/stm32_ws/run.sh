#!/bin/bash

# Set the name to search for
name_imu="Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001"
name_gps="Prolific_Technology_Inc._USB-Serial_Controller"
name_stm="1a86_USB_Serial"

port_stm=""
port_gps=""
port_imu=""

# Search for USB devices and filter by name
for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && exit
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && exit

        # Check if the name matches and assign to the variable
        if [[ "$ID_MODEL" == "$name_imu" ]]; then
            port_stm="/dev/$devname"
            echo "Port IMU: $port_imu"
        fi

        if [[ "$ID_MODEL" == "$name_gps" ]]; then
            port_stm="/dev/$devname"
            echo "Port GPS: $port_gps"
        fi

        if [[ "$ID_MODEL" == "$name_stm" ]]; then
            port_stm="/dev/$devname"
            echo "Port STM: $port_stm"
        fi
    )
done

source devel/setup.bash
roslaunch my_robot_visualization robot_run.launch port_stm:="$port_stm" port_gps:="$port_gps" port_imu:="$port_imu"

