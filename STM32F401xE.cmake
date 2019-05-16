set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR armv7e-m)
set(CMAKE_SYSTEM_VERSION 1)

find_program(CMAKE_C_COMPILER arm-none-eabi-gcc)
find_program(CMAKE_CXX_COMPILER arm-none-eabi-g++)

if(NOT CMAKE_C_COMPILER)
  message(FATAL_ERROR "Could not find ARM GCC")
endif()

if(NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR "Could not find ARM G++")
endif()

set(CMAKE_C_COMPILER
  "${CMAKE_C_COMPILER}"
  CACHE PATH
  "ARM GCC"
)

set(CMAKE_CXX_COMPILER
  "${CMAKE_CXX_COMPILER}"
  CACHE PATH
  "ARM G++"
)

set(MCU_CPU cortex-m4)
set(MCU_FPU fpv4-sp-d16)
set(MCU_FLOAT_ABI hard)

set(MCU_FLAGS "-mcpu=${MCU_CPU} -mthumb -mfloat-abi=${MCU_FLOAT_ABI}")
if(MCU_FLOAT_ABI STREQUAL hard)
    set(MCU_FLAGS "${MCU_FLAGS} -mfpu=${MCU_FPU}")
endif()




set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
