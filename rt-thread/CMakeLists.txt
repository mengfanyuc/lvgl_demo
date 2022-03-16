SET(RTT_NAME rt-thread)
MESSAGE(STATUS "CMake: ${RTT_NAME}")
SET(RTT_ROOT_DIR .)
SET(RTT_SRC_DIR ${RTT_ROOT_DIR}/src)
FILE(GLOB RTT_SOURCES ${RTT_SRC_DIR}/*.c)

SET(RTT_COM_LIB_SRC_DIR
     ${RTT_ROOT_DIR}/components/libc/compilers/common/delay.c
     ${RTT_ROOT_DIR}/components/libc/compilers/common/stdlib.c
     ${RTT_ROOT_DIR}/components/libc/compilers/common/time.c
     ${RTT_ROOT_DIR}/components/libc/compilers/common/unistd.c
     ${RTT_ROOT_DIR}/components/libc/compilers/gcc/newlib/libc.c
     ${RTT_ROOT_DIR}/components/libc/compilers/gcc/newlib/stdio.c
     ${RTT_ROOT_DIR}/components/libc/compilers/gcc/newlib/syscalls.c
) 

SET(RTT_COM_DRIVER_SRC_DIR
     ${RTT_ROOT_DIR}/components/drivers/misc/pin.c
     ${RTT_ROOT_DIR}/components/drivers/i2c/i2c_core.c
     ${RTT_ROOT_DIR}/components/drivers/i2c/i2c_dev.c
     ${RTT_ROOT_DIR}/components/drivers/i2c/i2c-bit-ops.c
     ${RTT_ROOT_DIR}/components/drivers/touch/touch.c
) 

SET(RTT_CPU_SRC_DIR
     ${RTT_ROOT_DIR}/libcpu/arm/common/div0.c
     ${RTT_ROOT_DIR}/libcpu/arm/common/showmem.c
     ${RTT_ROOT_DIR}/libcpu/arm/common/backtrace.c
     ${RTT_ROOT_DIR}/libcpu/arm/cortex-m4/cpuport.c
     ${RTT_ROOT_DIR}/libcpu/arm/cortex-m4/context_gcc.S
)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
SET(RTT_DEBUG_SRC_DIR
     ${RTT_ROOT_DIR}/components/drivers/serial/serial.c
     ${RTT_ROOT_DIR}/components/finsh/shell.c
     ${RTT_ROOT_DIR}/components/finsh/msh.c
     ${RTT_ROOT_DIR}/components/finsh/cmd.c
     ${RTT_ROOT_DIR}/components/drivers/src/dataqueue.c
     ${RTT_ROOT_DIR}/components/drivers/src/ringbuffer.c
     ${RTT_ROOT_DIR}/components/drivers/src/completion.c
     ${RTT_ROOT_DIR}/components/drivers/src/pipe.c
     ${RTT_ROOT_DIR}/components/drivers/src/waitqueue.c
     ${RTT_ROOT_DIR}/components/drivers/src/workqueue.c
     ${RTT_ROOT_DIR}/components/drivers/src/ringblk_buf.c
)
ENDIF()

ADD_LIBRARY(
     ${RTT_NAME} OBJECT  
          ${RTT_SOURCES}
          ${RTT_COM_LIB_SRC_DIR}
          ${RTT_COM_DRIVER_SRC_DIR}
          ${RTT_CPU_SRC_DIR}
          ${RTT_DEBUG_SRC_DIR}
)

TARGET_INCLUDE_DIRECTORIES(
     ${RTT_NAME} SYSTEM PUBLIC 
          ${RTT_ROOT_DIR}
          ${RTT_ROOT_DIR}/include
          ${RTT_ROOT_DIR}/components/drivers/include
          ${RTT_ROOT_DIR}/components/finsh
          ${RTT_ROOT_DIR}/components/libc/compilers/common
          ${RTT_ROOT_DIR}/components/libc/compilers/gcc/newlib
          ${RTT_ROOT_DIR}/components/drivers/include/drivers
          ${RTT_ROOT_DIR}/components/drivers/touch
)                                                   
 
SET_TARGET_PROPERTIES(
     ${RTT_NAME} PROPERTIES
          PUBLIC_HEADER "${LVGL_ROOT_DIR}/rtconfig.h"
)
