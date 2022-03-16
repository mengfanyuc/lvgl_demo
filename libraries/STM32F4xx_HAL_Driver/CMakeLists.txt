SET(HAL_NAME hal)
MESSAGE(STATUS "CMake: ${HAL_NAME}")
SET(HAL_ROOT_DIR .)
SET(HAL_SRC_DIR ${HAL_ROOT_DIR}/Src)

SET(HAL_DRIVER_SRC_DIR
    ${HAL_SRC_DIR}/stm32f4xx_hal.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_cortex.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_rcc.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_rcc_ex.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_gpio.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_pwr.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_pwr_ex.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_sdram.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_ltdc.c
    ${HAL_SRC_DIR}/stm32f4xx_ll_fmc.c
    ${HAL_SRC_DIR}/stm32f4xx_hal_dma.c
)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
SET(HAL_DRIVER_DEBUG_SRC_DIR
    ${HAL_SRC_DIR}/stm32f4xx_hal_uart.c
)
ENDIF()

ADD_LIBRARY(${HAL_NAME} OBJECT
    ${HAL_DRIVER_SRC_DIR}
    ${HAL_DRIVER_DEBUG_SRC_DIR}
)

TARGET_COMPILE_DEFINITIONS(
    ${HAL_NAME} PUBLIC 
        USE_HAL_DRIVER
        STM32F429xx
)

TARGET_INCLUDE_DIRECTORIES(
    ${HAL_NAME} SYSTEM PUBLIC 
        ${HAL_ROOT_DIR}/Inc
)

SET(LVGL_PUBLIC_HEADERS "${LVGL_ROOT_DIR}/lv_conf.h"
    "${LVGL_ROOT_DIR}/lvgl.h")
     
SET_TARGET_PROPERTIES(
    ${HAL_NAME} PROPERTIES
        PUBLIC_HEADER "${LVGL_PUBLIC_HEADERS}"
)
