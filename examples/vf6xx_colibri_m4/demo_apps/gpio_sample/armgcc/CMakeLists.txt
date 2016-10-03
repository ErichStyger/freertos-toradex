INCLUDE(CMakeForceCompiler)

# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)


# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

# DEBUG LINK FILE
#set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${ProjDirPath}/../../../../../platform/devices/VF6XX/linker/gcc/vf6xx_tcm.ld  -static")

# RELEASE LINK FILE
#set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/../../../../../platform/devices/VF6XX/linker/gcc/vf6xx_tcm.ld  -static")

# DEBUG LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -T${ProjDirPath}/../../../../../platform/devices/VF6XX/linker/gcc/vf6xx_sysram-sram.ld  -static")

# RELEASE LINK FILE
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -T${ProjDirPath}/../../../../../platform/devices/VF6XX/linker/gcc/vf6xx_sysram-sram.ld  -static")

# DEBUG ASM FLAGS
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# DEBUG C FLAGS
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x400  -Xlinker --defsym=__heap_size__=0x200")

# RELEASE ASM FLAGS
SET(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE C FLAGS
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mapcs  -std=gnu99")

# RELEASE LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x400  -Xlinker --defsym=__heap_size__=0x200")

# ASM MACRO
SET(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS_DEBUG}  -D__DEBUG")

# C MACRO
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -D__DEBUG")
SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}  -DCPU_VF6XX_M4")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -D__NDEBUG")
SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}  -DCPU_VF6XX_M4")

# CXX MACRO

# INCLUDE_DIRECTORIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices/VF6XX/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices/VF6XX/startup)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/drivers/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/CMSIS/Include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices/VF6XX/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/devices/VF6XX/startup)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/drivers/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../../../../platform/utilities/inc)
    INCLUDE_DIRECTORIES(${ProjDirPath}/../../..)
ENDIF()

# ADD_EXECUTABLE
ADD_EXECUTABLE(gpio_sample
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F/portmacro.h"
   "${ProjDirPath}/../../../../../platform/devices/VF6XX/startup/gcc/startup_VF6XX_M4.S"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/MemMang/heap_2.c"
    "${ProjDirPath}/../../../FreeRTOSConfig.h"
    "${ProjDirPath}/../main.c"
    "${ProjDirPath}/../../../../../platform/drivers/src/lpuart_vf6xx.c"
    "${ProjDirPath}/../../../../../platform/drivers/inc/lpuart_vf6xx.h"
    "${ProjDirPath}/../../../../../platform/drivers/src/ccm_vf6xx.c"
    "${ProjDirPath}/../../../../../platform/drivers/inc/ccm_vf6xx.h"
    "${ProjDirPath}/../../../../../platform/drivers/src/gpio_vf6xx.c"
    "${ProjDirPath}/../../../../../platform/drivers/inc/gpio_vf6xx.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/croutine.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/event_groups.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/FreeRTOS.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/list.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/mpu_wrappers.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/portable.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/projdefs.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/queue.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/semphr.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/StackMacros.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/task.h"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include/timers.h"
     "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/croutine.c"
     "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/event_groups.c"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/list.c"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/queue.c"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/tasks.c"
    "${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/timers.c"
    "${ProjDirPath}/../../../../../platform/utilities/src/debug_console.c"
    "${ProjDirPath}/../../../../../platform/utilities/inc/debug_console.h"
    "${ProjDirPath}/../../../../../platform/utilities/src/debug_console_vf6xx.c"
    "${ProjDirPath}/../../../../../platform/utilities/inc/debug_console_vf6xx.h"
    "${ProjDirPath}/../../../../../platform/utilities/src/print_scan.c"
    "${ProjDirPath}/../../../../../platform/utilities/src/print_scan.h"
    "${ProjDirPath}/../../../../../platform/devices/VF6XX/startup/system_VF6XX_M4.c"
    "${ProjDirPath}/../../../../../platform/devices/VF6XX/startup/system_VF6XX_M4.h"
    "${ProjDirPath}/../../../pin_mux.c"
    "${ProjDirPath}/../../../pin_mux.h"
)
SET_TARGET_PROPERTIES(gpio_sample PROPERTIES OUTPUT_NAME "gpio_sample.elf")

TARGET_LINK_LIBRARIES(gpio_sample -Wl,--start-group)
# LIBRARIES
IF(CMAKE_BUILD_TYPE MATCHES Debug)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
ENDIF()

# SYSTEM LIBRARIES
TARGET_LINK_LIBRARIES(gpio_sample m)
TARGET_LINK_LIBRARIES(gpio_sample c)
TARGET_LINK_LIBRARIES(gpio_sample gcc)
TARGET_LINK_LIBRARIES(gpio_sample nosys)
TARGET_LINK_LIBRARIES(gpio_sample -Wl,--end-group)

# MAP FILE
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG}  -Xlinker -Map=debug/gpio_sample.map")
SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE}  -Xlinker -Map=release/gpio_sample.map")

# BIN AND HEX
ADD_CUSTOM_COMMAND(TARGET gpio_sample POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Oihex ${EXECUTABLE_OUTPUT_PATH}/gpio_sample.elf ${EXECUTABLE_OUTPUT_PATH}/gpio_sample.hex)
ADD_CUSTOM_COMMAND(TARGET gpio_sample POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary ${EXECUTABLE_OUTPUT_PATH}/gpio_sample.elf ${EXECUTABLE_OUTPUT_PATH}/gpio_sample.bin)
