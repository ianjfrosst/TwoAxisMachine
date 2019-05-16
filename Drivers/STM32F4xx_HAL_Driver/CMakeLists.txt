cmake_minimum_required(VERSION 3.9)
project(STM32F4_HAL C)

file(GLOB HAL_SOURCES "Src/*.c")

add_library(${PROJECT_NAME} STATIC
  ${HAL_SOURCES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  Inc
)
