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
Fade_LED.lss \

FLASH_IMAGE += \
Fade_LED.hex \

EEPROM_IMAGE += \
Fade_LED.eep \

SIZEDUMMY += \
sizedummy \


# All Target
all: Fade_LED.elf secondary-outputs

# Tool invocations
Fade_LED.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Fade_LED.map -mmcu=attiny5 -o "Fade_LED.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Fade_LED.lss: Fade_LED.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S Fade_LED.elf  >"Fade_LED.lss"
	@echo 'Finished building: $@'
	@echo ' '

Fade_LED.hex: Fade_LED.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex Fade_LED.elf  "Fade_LED.hex"
	@echo 'Finished building: $@'
	@echo ' '

Fade_LED.eep: Fade_LED.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Fade_LED.elf  "Fade_LED.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Fade_LED.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny5 Fade_LED.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(EEPROM_IMAGE)$(FLASH_IMAGE)$(ELFS)$(LSS)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS) Fade_LED.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
