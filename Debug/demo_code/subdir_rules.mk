################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
demo_code/basic_io.obj: ../demo_code/basic_io.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/bin/armcl" -mv7M3 --code_state=16 -me --include_path="C:/ti/ccsv6/tools/compiler/ti-cgt-arm_16.6.0.STS/include" --include_path="C:/ti/StellarisWare" --include_path="C:/ti/StellarisWare/boards/ek-lm3s1968" --include_path="C:/ti/StellarisWare/driverlib/ccs-cm3/Debug" --define="ccs" --define=PART_LM3S1968 -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="demo_code/basic_io.d" --obj_directory="demo_code" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


