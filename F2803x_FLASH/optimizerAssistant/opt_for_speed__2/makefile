################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_headers/cmd/DSP2803x_Headers_nonBIOS.cmd" \
"./DLOG4CHC.obj" \
"./DSP2803x_CodeStartBranch.obj" \
"./DSP2803x_CpuTimers.obj" \
"./DSP2803x_GlobalVariableDefs.obj" \
"./DSP2803x_usDelay.obj" \
"./HVBLDC_Sensorless-DevInit_F2803x.obj" \
"./HVBLDC_Sensorless.obj" \
"../F28035_FLASH_HVBLDC_Sensorless.CMD" \
"D:/ti/controlSUITE/libs/math/IQmath/v15c/lib/IQmath.lib" \
$(GEN_CMDS__FLAG) \
-lrts2800_ml.lib \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
HVBLDC_Sensorless.out \

EXE_OUTPUTS__QUOTED += \
"HVBLDC_Sensorless.out" \


# All Target
all: HVBLDC_Sensorless.out

# Tool invocations
HVBLDC_Sensorless.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=2 -z -m"HVBLDC_Sensorless.map" --stack_size=0x380 --warn_sections -i"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/lib" -i"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" -i"D:/ti/controlSUITE/development_kits/HVMotorCtrl+PfcKit_v2.0/HVBLDC_Sensorless" --reread_libs --xml_link_info="HVBLDC_Sensorless_linkInfo.xml" --rom_model -o "HVBLDC_Sensorless.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "DSP2803x_CpuTimers.d" "DSP2803x_GlobalVariableDefs.d" "HVBLDC_Sensorless-DevInit_F2803x.d" "HVBLDC_Sensorless.d" 
	-$(RM) "DLOG4CHC.obj" "DSP2803x_CodeStartBranch.obj" "DSP2803x_CpuTimers.obj" "DSP2803x_GlobalVariableDefs.obj" "DSP2803x_usDelay.obj" "HVBLDC_Sensorless-DevInit_F2803x.obj" "HVBLDC_Sensorless.obj" 
	-$(RM) "DLOG4CHC.d" "DSP2803x_CodeStartBranch.d" "DSP2803x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

