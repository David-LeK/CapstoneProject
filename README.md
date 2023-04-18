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

