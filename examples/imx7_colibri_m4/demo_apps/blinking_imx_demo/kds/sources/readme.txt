readme.txt
----------
Project to build the 'hello world' application for the Toradex i.MX7 Colibri board using Kinetis Design Studio.

SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g  -O0  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  -mthumb  -MMD  -MP  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -mapcs  -std=gnu99")

# DEBUG LD FLAGS
SET(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g  -mcpu=cortex-m4  -mfloat-abi=hard  -mfpu=fpv4-sp-d16  --specs=nano.specs  -lm  -Wall  -fno-common  -ffunction-sections  -fdata-sections  -ffreestanding  -fno-builtin  -Os  -mthumb  -mapcs  -Xlinker --gc-sections  -Xlinker -static  -Xlinker -z  -Xlinker muldefs")


Settings needed if starting from an 'empty' project:

Compiler:
- cortex-m4
- FP instructions (hard)
- fpv4-sp-d16

Preprocessor Defines (-D):
CPU_MCIMX7D_M4
__DEBUG
or:
__NDEBUG

Include Path (-I):
${ProjDirPath}/../../../../../examples/imx7_colibri_m4/demo_apps/blinking_imx_demo
${ProjDirPath}/../../../../../examples/imx7_colibri_m4
${ProjDirPath}/../../../../../platform/devices
${ProjDirPath}/../../../../../platform/devices/MCIMX7D/startup
${ProjDirPath}/../../../../../platform/CMSIS/Include
${ProjDirPath}/../../../../../platform/devices/MCIMX7D/include
${ProjDirPath}/../../../../../platform/drivers/inc
${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/include
${ProjDirPath}/../../../../../rtos/FreeRTOS/Source/portable/GCC/ARM_CM4F
${ProjDirPath}/../../../../../platform/utilities/inc

Linker:
-L "${ProjDirPath}/../../../../../platform/devices/MCIMX7D/linker/gcc"
-T "MCIMX7D_M4_tcm.ld"
-specs=nano.specs -specs=nosys.specs
-ffunction-sections -fdata-sections -ffreestanding -fno-builtin

Build output:
Flash image: binary