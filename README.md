HOW TO DOWNLOAD:
```
sudo apt install git -y
git clone git@github.com:David-LeK/CapstoneProject.git
```

PLEASE UPDATE BASHRC TO ALWAYS SOURCE PROJECT

RANDOM SCRIPTS

```
sudo nano /etc/udev/rules.d/50-myusb.rules


KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyACM[0-9]*",MODE="0666"

sudo udevadm control --reload

sudo chmod 666 /dev/ttyUSB0
sudo adduser user_name dialout

BEGIN:10.7729,10.773,10.7729,10.7728,10.7728,10.773;106.66,106.66,106.66,106.66,106.66,106.66:END

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

export ROS_MASTER_URI=http://192.168.1.100:11311

nmcli d wifi connect <WiFiSSID> password <WiFiPassword> iface <WifiInterface>

nmcli connection delete $(nmcli connection show | grep -i wifi | awk '{print $1}')

https://github.com/oblique/create_ap

sudo ln -sfn /usr/bin/python3.8 /usr/bin/python

cd /usr/bin/ && sudo rm python3 && sudo ln -s python3.8 python3
```

Qt sample GUI
```
rosrun rqt_topic rqt_topic
rosrun rqt_graph rqt_graph
rosrun rqt_plot rqt_plot
```
Qt version: 5.15.2

- [UBLOX ROS](#ublox-ros)
- [MPU9250 - MPU6500](#mpu9250---mpu6500)
  * [STM32](#stm32)
- [ROS STM32 COMMUNICATION](#ros-stm32-communication)
- [ROS WORKSPACE SETUP GUIDE](#ros-workspace-setup-guide)

# MPU9250 - MPU6500
## STM32
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
    attitude = imu.calcAttitude();
  }
}
/* USER CODE END 4 */
```

* Timer interrupt frequency
```
Frequency = Timer clock frequency / (2 * Counter period * (Prescaler + 1))
```

* Timer PWM frequency
```
Frequency = Timer clock frequency / ((Counter period + 1) * (Prescaler + 1))
```

# ROS STM32 COMMUNICATION
See https://github.com/xav-jann1/rosserial_stm32f4.git for more details.

```
cd ~/catkin_ws/src
git clone https://github.com/xav-jann1/rosserial_stm32f4.git
cd ..
catkin_make
source devel/setup.bash
cd path/to/your/stm32/project/Core
rosrun rosserial_stm32f4 make_libraries.py .
```
See example at https://github.com/xav-jann1/rosserial_stm32f4/tree/aa9dd1169d02928b82984afd19366f66296c6a30/example

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

