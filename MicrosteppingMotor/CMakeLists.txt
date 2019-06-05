cmake_minimum_required(VERSION 3.9)
project(MicrosteppingMotor C ASM)

add_library(${PROJECT_NAME} STATIC
  Src/NUCLEO-F401RE/startup_stm32f401xe.s
  Src/NUCLEO-F401RE/stm32f4xx_it.c
  Src/NUCLEO-F401RE/stm32f4xx_hal_msp.c
  Src/example.c
  Src/example_usart.c
  Src/params.c
  Src/main.c
)

target_include_directories(${PROJECT_NAME} PUBLIC
  Src
  Src/NUCLEO-F401RE
)
