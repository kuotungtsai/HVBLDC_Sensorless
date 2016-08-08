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
D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm \
../DSP2803x_usDelay.asm 

C_SRCS += \
../DSP2803x_CpuTimers.c \
../DSP2803x_GlobalVariableDefs.c \
../DSP2803x_I2C.c \
../HVBLDC_Sensorless-DevInit_F2803x.c \
../HVBLDC_Sensorless.c \
../i2c_gy80.c 

OBJS += \
./DSP2803x_CodeStartBranch.obj \
./DSP2803x_CpuTimers.obj \
./DSP2803x_GlobalVariableDefs.obj \
./DSP2803x_I2C.obj \
./DSP2803x_usDelay.obj \
./HVBLDC_Sensorless-DevInit_F2803x.obj \
./HVBLDC_Sensorless.obj \
./i2c_gy80.obj 

ASM_DEPS += \
./DSP2803x_CodeStartBranch.d \
./DSP2803x_usDelay.d 

C_DEPS += \
./DSP2803x_CpuTimers.d \
./DSP2803x_GlobalVariableDefs.d \
./DSP2803x_I2C.d \
./HVBLDC_Sensorless-DevInit_F2803x.d \
./HVBLDC_Sensorless.d \
./i2c_gy80.d 

C_DEPS__QUOTED += \
"DSP2803x_CpuTimers.d" \
"DSP2803x_GlobalVariableDefs.d" \
"DSP2803x_I2C.d" \
"HVBLDC_Sensorless-DevInit_F2803x.d" \
"HVBLDC_Sensorless.d" \
"i2c_gy80.d" 

OBJS__QUOTED += \
"DSP2803x_CodeStartBranch.obj" \
"DSP2803x_CpuTimers.obj" \
"DSP2803x_GlobalVariableDefs.obj" \
"DSP2803x_I2C.obj" \
"DSP2803x_usDelay.obj" \
"HVBLDC_Sensorless-DevInit_F2803x.obj" \
"HVBLDC_Sensorless.obj" \
"i2c_gy80.obj" 

ASM_DEPS__QUOTED += \
"DSP2803x_CodeStartBranch.d" \
"DSP2803x_usDelay.d" 

ASM_SRCS__QUOTED += \
"D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm" \
"../DSP2803x_usDelay.asm" 

C_SRCS__QUOTED += \
"../DSP2803x_CpuTimers.c" \
"../DSP2803x_GlobalVariableDefs.c" \
"../DSP2803x_I2C.c" \
"../HVBLDC_Sensorless-DevInit_F2803x.c" \
"../HVBLDC_Sensorless.c" \
"../i2c_gy80.c" 


