cmake_minimum_required(VERSION 3.9)
project(TwoAxisMachine C)

add_definitions(
  -DUSE_HAL_DRIVER
  -DSTM32F401xE
  -DARM_MATH_CM4
)

add_subdirectory(Drivers/CMSIS)
add_subdirectory(Drivers/STM32F4xx_HAL_Driver)
add_subdirectory(MicrosteppingMotor)
