################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
HSImeaserement.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
HSImeaserement.list \

OBJCOPY_BIN += \
HSImeaserement.bin \


# All Target
all: HSImeaserement.elf secondary-outputs

# Tool invocations
HSImeaserement.elf: $(OBJS) $(USER_OBJS) C:\Users\hqnqb\STM32CubeIDE\workspace_1.3.0\HSImeaserement\STM32F303RETX_FLASH.ld
	arm-none-eabi-gcc -o "HSImeaserement.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\hqnqb\STM32CubeIDE\workspace_1.3.0\HSImeaserement\STM32F303RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="HSImeaserement.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

HSImeaserement.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "HSImeaserement.list"
	@echo 'Finished building: $@'
	@echo ' '

HSImeaserement.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "HSImeaserement.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
