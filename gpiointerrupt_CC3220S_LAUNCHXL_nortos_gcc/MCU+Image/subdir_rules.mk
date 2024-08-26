################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=soft -DDeviceFamily_CC3220 -DNORTOS_SUPPORT -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc" -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/MCU+Image" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/source" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos/posix" -I"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include/newlib-nano" -I"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include" -O3 -ffunction-sections -fdata-sections -g -gstrict-dwarf -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/MCU+Image/syscfg" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1109335953: ../gpiointerrupt.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1220/ccs/utils/sysconfig_1.15.0/sysconfig_cli.bat" -s "C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" -s "C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" --script "C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/gpiointerrupt.syscfg" -o "syscfg" --compiler gcc
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_config.c: build-1109335953 ../gpiointerrupt.syscfg
syscfg/ti_drivers_config.h: build-1109335953
syscfg/ti_utils_build_linker.cmd.genlibs: build-1109335953
syscfg/syscfg_c.rov.xs: build-1109335953
syscfg/: build-1109335953

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: GNU Compiler'
	"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc-9.2.1.exe" -c -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=soft -DDeviceFamily_CC3220 -DNORTOS_SUPPORT -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc" -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/MCU+Image" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/source" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos" -I"C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/kernel/nortos/posix" -I"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include/newlib-nano" -I"C:/ti/ccs1220/ccs/tools/compiler/gcc-arm-none-eabi-9-2019-q4-major/arm-none-eabi/include" -O3 -ffunction-sections -fdata-sections -g -gstrict-dwarf -Wall -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/MCU+Image/syscfg" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1438997727: ../image.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/ccs1220/ccs/utils/sysconfig_1.15.0/sysconfig_cli.bat" -s "C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" -s "C:/ti/ccs1270/simplelink_cc32xx_sdk_7_10_00_13/.metadata/product.json" --script "C:/Users/Reece Filteau/workspace_v12/gpiointerrupt_CC3220S_LAUNCHXL_nortos_gcc/image.syscfg" -o "syscfg" --compiler gcc
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_drivers_net_wifi_config.json: build-1438997727 ../image.syscfg
syscfg/: build-1438997727


