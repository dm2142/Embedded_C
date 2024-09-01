################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BSP/Src/ds1307.c \
../BSP/Src/lcd.c 

OBJS += \
./BSP/Src/ds1307.o \
./BSP/Src/lcd.o 

C_DEPS += \
./BSP/Src/ds1307.d \
./BSP/Src/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
BSP/Src/%.o BSP/Src/%.su BSP/Src/%.cyclo: ../BSP/Src/%.c BSP/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Src" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-BSP-2f-Src

clean-BSP-2f-Src:
	-$(RM) ./BSP/Src/ds1307.cyclo ./BSP/Src/ds1307.d ./BSP/Src/ds1307.o ./BSP/Src/ds1307.su ./BSP/Src/lcd.cyclo ./BSP/Src/lcd.d ./BSP/Src/lcd.o ./BSP/Src/lcd.su

.PHONY: clean-BSP-2f-Src

