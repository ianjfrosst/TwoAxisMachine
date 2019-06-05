cmake_minimum_required(VERSION 3.9)
project(CMSIS_Driver C)

add_library(${PROJECT_NAME} STATIC

)

target_include_directories(${PROJECT_NAME} PUBLIC
  Include
  Device/ST/STM32F4xx/Include
)
