################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
DLOG4CHC.obj: ../DLOG4CHC.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="DLOG4CHC.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_CodeStartBranch.obj: D:/ti/controlSUITE/device_support/f2803x/v121/DSP2803x_common/source/DSP2803x_CodeStartBranch.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="DSP2803x_CodeStartBranch.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_CpuTimers.obj: ../DSP2803x_CpuTimers.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="DSP2803x_CpuTimers.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_GlobalVariableDefs.obj: ../DSP2803x_GlobalVariableDefs.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="DSP2803x_GlobalVariableDefs.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

DSP2803x_usDelay.obj: ../DSP2803x_usDelay.asm $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="DSP2803x_usDelay.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

HVBLDC_Sensorless-DevInit_F2803x.obj: ../HVBLDC_Sensorless-DevInit_F2803x.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="HVBLDC_Sensorless-DevInit_F2803x.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

HVBLDC_Sensorless.obj: ../HVBLDC_Sensorless.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/bin/cl2000" -v28 -ml -g --include_path="D:/ti/ccsv6/tools/compiler/ti-cgt-c2000_15.12.1.LTS/include" --include_path="D:/ti/controlSUITE/development_kits/~SupportFiles/F2803x_headers" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/drivers/f2803x_v2.0" --include_path="D:/ti/controlSUITE/libs/app_libs/motor_control/math_blocks/v4.0" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_headers/include" --include_path="D:/ti/controlSUITE/device_support/f2803x/v125/DSP2803x_common/include" --include_path="D:/ti/controlSUITE/libs/math/IQmath/v15c/include" --define="_DEBUG" --define="LARGE_MODEL" --define="FLASH" --quiet --diag_warning=225 --opt_for_speed=1 --preproc_with_compile --preproc_dependency="HVBLDC_Sensorless.d" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


