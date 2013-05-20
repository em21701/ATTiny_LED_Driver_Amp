################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
Heartbeat_LED.lss \

FLASH_IMAGE += \
Heartbeat_LED.hex \

EEPROM_IMAGE += \
Heartbeat_LED.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Heartbeat_LED.elf secondary-outputs

# Tool invocations
Heartbeat_LED.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Heartbeat_LED.map -mmcu=attiny5 -o "Heartbeat_LED.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Heartbeat_LED.lss: Heartbeat_LED.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Heartbeat_LED.elf  >"Heartbeat_LED.lss"
	@echo 'Finished building: $@'
	@echo ' '

Heartbeat_LED.hex: Heartbeat_LED.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex Heartbeat_LED.elf  "Heartbeat_LED.hex"
	@echo 'Finished building: $@'
	@echo ' '

Heartbeat_LED.eep: Heartbeat_LED.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Heartbeat_LED.elf  "Heartbeat_LED.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Heartbeat_LED.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny5 Heartbeat_LED.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) Heartbeat_LED.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
