CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-m4)
SET(CMAKE_VERBOSE_MAKEFILE ON)

PROJECT(Lvgl C ASM)

SET(VERSION_MAJOR 1 CACHE STRING "Project major version number.")
SET(VERSION_MINOR 0 CACHE STRING "Project minor version number.")
SET(VERSION_PATCH 0 CACHE STRING "Project patch version number.")

IF(CMAKE_BUILD_TYPE MATCHES Debug)
    MESSAGE(STATUS "Build type: Debug")
    SET(DBG_FLAGS "-g3 -gdwarf-2 -O0 -Wall")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    MESSAGE(STATUS "Build type: Release")
    SET(DBG_FLAGS "-Os -Wall")
ENDIF()

SET(CROSS_COMPILE_PREFIX arm-none-eabi-)

SET(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
SET(SECTIONS_FLAGS "-ffunction-sections -fdata-sections")
SET(LINK_MAP_FLAGS "-Map=${PROJECT_NAME}.map")
SET(LINK_FLAGS "${MCU_FLAGS} ${SECTIONS_FLAGS} --specs=nano.specs -Wl,--gc-sections,${LINK_MAP_FLAGS},-cref")
SET(LINK_SCRIPTS_FLAGS "-T ${CMAKE_SOURCE_DIR}/linkscripts/STM32F429IG/link.lds")

SET(CMAKE_C_COMPILER ${CROSS_COMPILE_PREFIX}gcc)
SET(CMAKE_ASM_COMPILER ${CROSS_COMPILE_PREFIX}gcc)
SET(CMAKE_CXX_COMPILER ${CROSS_COMPILE_PREFIX}g++)
SET(CMAKE_OBJCOPY ${CROSS_COMPILE_PREFIX}objcopy)
SET(CMAKE_SIZE ${CROSS_COMPILE_PREFIX}size)
SET(CMAKE_C_COMPILER_WORKS TRUE)
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_C_FLAGS_DEBUG " ${MCU_FLAGS} ${SECTIONS_FLAGS} ${DBG_FLAGS}")
SET(CMAKE_C_FLAGS_RELEASE " ${MCU_FLAGS} ${SECTIONS_FLAGS} ${DBG_FLAGS}")
SET(CMAKE_CXX_FLAGS_DEBUG " ${MCU_FLAGS} ${SECTIONS_FLAGS} ${DBG_FLAGS}")
SET(CMAKE_CXX_FLAGS_RELEASE " ${MCU_FLAGS} ${SECTIONS_FLAGS} ${DBG_FLAGS}")
SET(CMAKE_ASM_FLAGS_DEBUG " -c ${MCU_FLAGS} ${SECTIONS_FLAGS} -x assembler-with-cpp -Wa,-mimplicit-it=thumb ${DBG_FLAGS}")
SET(CMAKE_ASM_FLAGS_RELEASE " -c ${MCU_FLAGS} ${SECTIONS_FLAGS} -x assembler-with-cpp -Wa,-mimplicit-it=thumb ${DBG_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG " ${LINK_FLAGS},-u,Reset_Handler ${LINK_SCRIPTS_FLAGS} ${DBG_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE " ${LINK_FLAGS},-u,Reset_Handler ${LINK_SCRIPTS_FLAGS} ${DBG_FLAGS}")

SET(CMAKE_CXX_STANDARD 14)

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/ports
    ${CMAKE_SOURCE_DIR}/ports/lvgl
    #${CMAKE_SOURCE_DIR}/applications/music
    #${CMAKE_SOURCE_DIR}/applications/music/assets
    ${CMAKE_SOURCE_DIR}/applications/benchmark/
    ${CMAKE_SOURCE_DIR}/drivers/include
    ${CMAKE_SOURCE_DIR}/drivers/include/config
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Include
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F4xx/Include
)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
ADD_DEFINITIONS(
    -DCMAKE_DEBUG
    )
ENDIF()

#FILE(GLOB ASSETS_SOURCES ${CMAKE_SOURCE_DIR}/applications/music/assets/*.c)
FILE(GLOB ASSETS_SOURCES ${CMAKE_SOURCE_DIR}/applications/benchmark/assets/*.c)

SET(PROJECT_SOURCES
    ${ASSETS_SOURCES}
    ${CMAKE_SOURCE_DIR}/applications/main.c
    ${CMAKE_SOURCE_DIR}/applications/benchmark/lv_demo_benchmark.c
    #${CMAKE_SOURCE_DIR}/applications/music/lv_demo_music.c
    #${CMAKE_SOURCE_DIR}/applications/music/lv_demo_music_list.c
    #${CMAKE_SOURCE_DIR}/applications/music/lv_demo_music_main.c
    ${CMAKE_SOURCE_DIR}/drivers/board.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_clk.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_common.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_gpio.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_sdram.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_lcd.c
    ${CMAKE_SOURCE_DIR}/drivers/drv_soft_i2c.c
    ${CMAKE_SOURCE_DIR}/ports/lvgl/lv_port_disp.c
    ${CMAKE_SOURCE_DIR}/ports/lvgl/lv_port_indev.c
    ${CMAKE_SOURCE_DIR}/ports/lvgl/lv_music.c
    ${CMAKE_SOURCE_DIR}/ports/touch_port.c
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/libraries/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f429xx.s
)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
SET(PROJECT_DEBUG_SOURCES
    ${CMAKE_SOURCE_DIR}/drivers/drv_usart.c
)
ENDIF()

ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/libraries/STM32F4xx_HAL_Driver)
ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/rt-thread)
ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/packages/lvgl)
ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${PROJECT_SOURCES} ${PROJECT_DEBUG_SOURCES})
TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf PUBLIC hal)
TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf PUBLIC rt-thread)
TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf PUBLIC lvgl)
ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf ${CMAKE_PROJECT_NAME}.bin COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf)
