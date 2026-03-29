################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := A137_STM32F103C8T6_Project_RSA
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
A137_STM32F103C8T6_Project_RSA.elf \

MAP_FILES += \
A137_STM32F103C8T6_Project_RSA.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
A137_STM32F103C8T6_Project_RSA.list \

OBJCOPY_HEX += \
A137_STM32F103C8T6_Project_RSA.hex \

OBJCOPY_BIN += \
A137_STM32F103C8T6_Project_RSA.bin \


# All Target
all: main-build

# Main-build Target
main-build: A137_STM32F103C8T6_Project_RSA.elf secondary-outputs

# Tool invocations
A137_STM32F103C8T6_Project_RSA.elf A137_STM32F103C8T6_Project_RSA.map: $(OBJS) $(USER_OBJS) C:\Users\Acer\STM32CubeIDE\workspace_1.18.0\A137_STM32F103C8T6_Project_RSA\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "A137_STM32F103C8T6_Project_RSA.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Acer\STM32CubeIDE\workspace_1.18.0\A137_STM32F103C8T6_Project_RSA\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="A137_STM32F103C8T6_Project_RSA.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

A137_STM32F103C8T6_Project_RSA.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "A137_STM32F103C8T6_Project_RSA.list"
	@echo 'Finished building: $@'
	@echo ' '

A137_STM32F103C8T6_Project_RSA.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "A137_STM32F103C8T6_Project_RSA.hex"
	@echo 'Finished building: $@'
	@echo ' '

A137_STM32F103C8T6_Project_RSA.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "A137_STM32F103C8T6_Project_RSA.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) A137_STM32F103C8T6_Project_RSA.bin A137_STM32F103C8T6_Project_RSA.elf A137_STM32F103C8T6_Project_RSA.hex A137_STM32F103C8T6_Project_RSA.list A137_STM32F103C8T6_Project_RSA.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
