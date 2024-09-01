################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/App_RTC_LCD.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/App_RTC_LCD.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/App_RTC_LCD.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Src" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/App_RTC_LCD.cyclo ./Src/App_RTC_LCD.d ./Src/App_RTC_LCD.o ./Src/App_RTC_LCD.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

