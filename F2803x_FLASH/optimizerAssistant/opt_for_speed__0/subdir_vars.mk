################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd 

CMD_UPPER_SRCS += \
../F28035_FLASH_HVBLDC_Sensorless.CMD 

LIB_SRCS += \
D:/ti/controlSUITE/libs/math/IQmath/v15c/lib/IQmath.lib 

ASM_SRCS += \
../DLOG4CHC.asm \
D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm \
../DSP2803x_usDelay.asm 

C_SRCS += \
../DSP2803x_CpuTimers.c \
../DSP2803x_GlobalVariableDefs.c \
../HVBLDC_Sensorless-DevInit_F2803x.c \
../HVBLDC_Sensorless.c 

OBJS += \
./DLOG4CHC.obj \
./DSP2803x_CodeStartBranch.obj \
./DSP2803x_CpuTimers.obj \
./DSP2803x_GlobalVariableDefs.obj \
./DSP2803x_usDelay.obj \
./HVBLDC_Sensorless-DevInit_F2803x.obj \
./HVBLDC_Sensorless.obj 

ASM_DEPS += \
./DLOG4CHC.d \
./DSP2803x_CodeStartBranch.d \
./DSP2803x_usDelay.d 

C_DEPS += \
./DSP2803x_CpuTimers.d \
./DSP2803x_GlobalVariableDefs.d \
./HVBLDC_Sensorless-DevInit_F2803x.d \
./HVBLDC_Sensorless.d 

C_DEPS__QUOTED += \
"DSP2803x_CpuTimers.d" \
"DSP2803x_GlobalVariableDefs.d" \
"HVBLDC_Sensorless-DevInit_F2803x.d" \
"HVBLDC_Sensorless.d" 

OBJS__QUOTED += \
"DLOG4CHC.obj" \
"DSP2803x_CodeStartBranch.obj" \
"DSP2803x_CpuTimers.obj" \
"DSP2803x_GlobalVariableDefs.obj" \
"DSP2803x_usDelay.obj" \
"HVBLDC_Sensorless-DevInit_F2803x.obj" \
"HVBLDC_Sensorless.obj" 

ASM_DEPS__QUOTED += \
"DLOG4CHC.d" \
"DSP2803x_CodeStartBranch.d" \
"DSP2803x_usDelay.d" 

ASM_SRCS__QUOTED += \
"../DLOG4CHC.asm" \
"D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm" \
"../DSP2803x_usDelay.asm" 

C_SRCS__QUOTED += \
"../DSP2803x_CpuTimers.c" \
"../DSP2803x_GlobalVariableDefs.c" \
"../HVBLDC_Sensorless-DevInit_F2803x.c" \
"../HVBLDC_Sensorless.c" 


