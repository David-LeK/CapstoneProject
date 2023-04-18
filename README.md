HOW TO DOWNLOAD:
```
sudo apt install git -y
git clone git@github.com:David-LeK/CapstoneProject.git
```

RANDOM SCRIPTS

```
sudo nano /etc/udev/rules.d/50-myusb.rules


KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"

sudo udevadm control --reload

sudo chmod 666 /dev/ttyUSB0
sudo adduser user_name dialout

BEGIN:10.7729,10.773,10.7729,10.7728,10.7728,10.773;106.66,106.66,106.66,106.66,106.66,106.66:END
```
Qt version: 5.15.2

- [UBLOX ROS](#ublox-ros)
- [MPU9250 - MPU6500](#mpu9250---mpu6500)
  * [STM32](#stm32)
- [ROS STM32 COMMUNICATION](#ros-stm32-communication)
- [ROS WORKSPACE SETUP GUIDE](#ros-workspace-setup-guide)

# MPU9250 - MPU6500
## STM32
![alt text](https://github.com/David-LeK/CapstoneProject/blob/main/img/Screenshot%202023-03-16%20005402.png?raw=true)
* Reference library: https://github.com/MarkSherstan/MPU-6050-9250-I2C-CompFilter
* main.cpp (only complied using C++)
```
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "MPUXX50.h"
/* USER CODE END Includes */
```

```
/* USER CODE BEGIN PD */
#define TRUE  1
#define FALSE 0
/* USER CODE END PD */
```

```
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t serialBuf[100];
Attitude attitude;
/* USER CODE END PV */
```

```
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MPUXX50 imu(&hi2c2, AD0_LOW);
/* USER CODE END 0 */
```

```
  /* USER CODE BEGIN 2 */
  // Configure IMU
  imu.setGyroFullScaleRange(GFSR_500DPS);
  imu.setAccFullScaleRange(AFSR_4G);
  imu.setDeltaTime(0.004);
  imu.setTau(0.98);

  // Check if IMU configured properly and block if it didn't
  while (imu.begin() != TRUE){HAL_Delay(500);}

  // Calibrate the IMU
  sprintf((char *)serialBuf, "CALIBRATING...\r\n");
  HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);
  imu.calibrateGyro(1500);

  // Start timer and put processor into an efficient low power mode
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_PWR_EnableSleepOnExit();
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  /* USER CODE END 2 */
```

* Can comment out while loop
```
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//
//  }
```

```
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Callback, timer has rolled over
  if (htim == &htim11)
  {
    HAL_ResumeTick();

    attitude = imu.calcAttitude();

    sprintf((char *)serialBuf, "%.1f,%.1f,%.1f\r\n", attitude.r, attitude.p, attitude.y);
    HAL_UART_Transmit(&huart2, serialBuf, strlen((char *)serialBuf), HAL_MAX_DELAY);

    HAL_SuspendTick();
  }
}
/* USER CODE END 4 */
```

* Timer configuration
```
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1600-1; //Should configure using clock*100
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 40-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}
```

The HAL_TIM_PeriodElapsedCallback function is called when the timer overflows. The frequency of the timer overflow is determined by the timer’s prescaler, period, and clock frequency. In your case, the timer’s prescaler is set to 1600-1, the period is set to 40-1, and the clock division is set to TIM_CLOCKDIVISION_DIV1. Therefore, the timer frequency is 16 MHz / 1600 / 40 = 25 kHz. The HAL_TIM_PeriodElapsedCallback function will be called at a frequency of 25 kHz / 40 = 625 Hz. 625Hz is equivalent to 0.0016 seconds or 1.6 milliseconds.

# ROS STM32 COMMUNICATION
## Reading data from STM32
### MPU NODE
* Make serial port available to read for every user
```
sudo chmod 666 /dev/ttyUSB0
```

* Prerequisites
```
roscore
rosrun rviz rviz
#rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map my_frame 10
```

```
rostopic echo /roll
```

* Create read_serial.py and rviz_visualization.py in scripts directory
```python
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
```

```python
import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
import math

roll = 0.0
pitch = 0.0
yaw = 0.0

def callback_roll(data):
    global roll
    roll = data.data

def callback_pitch(data):
    global pitch
    pitch = data.data

def callback_yaw(data):
    global yaw
    yaw = data.data

def quaternion_from_euler(roll, pitch, yaw):
    roll = roll * math.pi/180;
    pitch = pitch * math.pi/180;
    yaw = yaw * math.pi/180;
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr
    return [x, y, z, w]

def listener():
    
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.Subscriber("roll", Float64, callback_roll)
    rospy.Subscriber("pitch", Float64, callback_pitch)
    rospy.Subscriber("yaw", Float64, callback_yaw)
    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = marker.CUBE
    #marker.type = marker.MESH_RESOURCE
    #marker.mesh_resource = "file:///home/tien/Downloads/v1-01/model.dae";
    marker.action = marker.ADD
    marker.scale.x = 9.0
    marker.scale.y = 3.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        print("RPY: ")
        print(roll, pitch, yaw)
        print("Quaternion: ")
        print(quaternion)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = -quaternion[1]
        marker.pose.orientation.z = -quaternion[2]
        marker.pose.orientation.w = quaternion[3]
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

```

# ROS WORKSPACE SETUP GUIDE
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
#catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
roscd <package_name>
mkdir scripts
cd scripts
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun {package_name} {script_name}
```

Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter.
```
catkin_install_python(PROGRAMS scripts/{file1}.py scripts/{file2}.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

