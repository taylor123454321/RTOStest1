################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./ADC_interface.obj" \
"./PWM_module.obj" \
"./button_data.obj" \
"./data_process.obj" \
"./display.obj" \
"./init.obj" \
"./int_module.obj" \
"./list.obj" \
"./main.obj" \
"./queue.obj" \
"./rit128x96x4.obj" \
"./speed.obj" \
"./startup_ccs.obj" \
"./tasks.obj" \
"./demo_code/basic_io.obj" \
"./portable/heap_2.obj" \
"./portable/port.obj" \
"./portable/portasm.obj" \
"../lm3s1968.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/StellarisWare/driverlib/ccs-cm3/Debug/driverlib-cm3.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include demo_code/subdir_vars.mk
-include portable/subdir_vars.mk
-include subdir_rules.mk
-include demo_code/subdir_rules.mk
-include portable/subdir_rules.mk
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
RTOStest1.out \

EXE_OUTPUTS__QUOTED += \
"RTOStest1.out" \

BIN_OUTPUTS += \
RTOStest1.hex \

BIN_OUTPUTS__QUOTED += \
"RTOStest1.hex" \


# All Target
all: RTOStest1.out

# Tool invocations
RTOStest1.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/bin/armcl" -mv7M3 --code_state=16 -me --define="ccs" --define=PART_LM3S1968 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi -z -m"RTOStest1.map" --heap_size=0xFFF --stack_size=0xFFF -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="RTOStest1_linkInfo.xml" --rom_model -o "RTOStest1.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

RTOStest1.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/bin/armhex"  -o "RTOStest1.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "ADC_interface.d" "PWM_module.d" "button_data.d" "data_process.d" "display.d" "init.d" "int_module.d" "list.d" "main.d" "queue.d" "rit128x96x4.d" "speed.d" "startup_ccs.d" "tasks.d" "demo_code\basic_io.d" "portable\heap_2.d" "portable\port.d" 
	-$(RM) "ADC_interface.obj" "PWM_module.obj" "button_data.obj" "data_process.obj" "display.obj" "init.obj" "int_module.obj" "list.obj" "main.obj" "queue.obj" "rit128x96x4.obj" "speed.obj" "startup_ccs.obj" "tasks.obj" "demo_code\basic_io.obj" "portable\heap_2.obj" "portable\port.obj" "portable\portasm.obj" 
	-$(RM) "portable\portasm.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

