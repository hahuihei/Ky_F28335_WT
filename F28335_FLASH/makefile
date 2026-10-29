################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./DSP2833x_ADC_cal.obj" \
"./DSP2833x_CodeStartBranch.obj" \
"./DSP2833x_GlobalVariableDefs.obj" \
"./DSP2833x_usDelay.obj" \
"./KY-Config.obj" \
"./KY-DevInit_F28335.obj" \
"./KY-Main.obj" \
"./KY_CAN_Config.obj" \
"./PWM_2ch_UpDwnCnt_Cnf.obj" \
"./PWM_UpDwnCnt_Cnf.obj" \
"../DSP2833x_Headers_nonBIOS.cmd" \
"../F28335_FLASH.cmd" \
"../kySolar/v1.2/float/lib/kySolar_Lib_Float.lib" \
"../lib/C28x_SGEN_Lib_fpu32.lib" \
"../lib/IQmath_fpu32.lib" \
"../lib/SFO_TI_Build_V6b.lib" \
"../lib/rts2800_fpu32_fast_supplement.lib" \
$(GEN_CMDS__FLAG) \
-lrts2800_fpu32_fast_supplement.lib \
-lrts2800_fpu32.lib \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include kySolar/v1.2/float/lib/subdir_vars.mk
-include lib/subdir_vars.mk
-include subdir_rules.mk
-include kySolar/v1.2/float/lib/subdir_rules.mk
-include lib/subdir_rules.mk
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
Ky_F28335_WT.out \

EXE_OUTPUTS__QUOTED += \
"Ky_F28335_WT.out" \

BIN_OUTPUTS += \
Ky_F28335_WT.hex \

BIN_OUTPUTS__QUOTED += \
"Ky_F28335_WT.hex" \


# All Target
all: Ky_F28335_WT.out

# Tool invocations
Ky_F28335_WT.out: $(OBJS) $(CMD_SRCS) $(CMD_UPPER_SRCS) $(LIB_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 -O1 --advice:performance=all --define=_DEBUG --define=FLASH --define=LARGE_MODEL -g --optimize_with_debug=on --diag_warning=225 --abi=coffabi -ms -z -m"Ky_F28335_WT.map" --stack_size=0x380 --warn_sections -i"C:/ti/ti-cgt-c2000_22.6.1.LTS/lib" -i"D:/wenjian/OneDrive/workspace_v6_1_3/Ky_F28335_WT/lib" -i"D:/wenjian/OneDrive/workspace_v6_1_3/Ky_F28335_WT/kySolar/v1.2/float/lib" -i"C:/ti/ti-cgt-c2000_22.6.1.LTS/include" -i"D:/wenjian/OneDrive/workspace_v6_1_3/Ky_F28335_WT/kySolar/v1.2/float/include" --reread_libs --xml_link_info="Resolver_33x_flash_linkInfo.xml" --rom_model -o "Ky_F28335_WT.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

Ky_F28335_WT.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C2000 Hex Utility'
	"C:/ti/ti-cgt-c2000_22.6.1.LTS/bin/hex2000"  -o "Ky_F28335_WT.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "DSP2833x_GlobalVariableDefs.d" "KY-Config.d" "KY-DevInit_F28335.d" "KY-Main.d" "KY_CAN_Config.d" "PWM_2ch_UpDwnCnt_Cnf.d" "PWM_UpDwnCnt_Cnf.d" 
	-$(RM) "DSP2833x_ADC_cal.obj" "DSP2833x_CodeStartBranch.obj" "DSP2833x_GlobalVariableDefs.obj" "DSP2833x_usDelay.obj" "KY-Config.obj" "KY-DevInit_F28335.obj" "KY-Main.obj" "KY_CAN_Config.obj" "PWM_2ch_UpDwnCnt_Cnf.obj" "PWM_UpDwnCnt_Cnf.obj" 
	-$(RM) "DSP2833x_ADC_cal.d" "DSP2833x_CodeStartBranch.d" "DSP2833x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

