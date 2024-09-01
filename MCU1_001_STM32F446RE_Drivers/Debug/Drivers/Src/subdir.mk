################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/smt32f446re_rcc_driver.c \
../Drivers/Src/stm32f446re_gpio_driver.c \
../Drivers/Src/stm32f446re_i2c_driver.c \
../Drivers/Src/stm32f446re_spi_driver.c \
../Drivers/Src/stm32f446re_usart_driver.c 

OBJS += \
./Drivers/Src/smt32f446re_rcc_driver.o \
./Drivers/Src/stm32f446re_gpio_driver.o \
./Drivers/Src/stm32f446re_i2c_driver.o \
./Drivers/Src/stm32f446re_spi_driver.o \
./Drivers/Src/stm32f446re_usart_driver.o 

C_DEPS += \
./Drivers/Src/smt32f446re_rcc_driver.d \
./Drivers/Src/stm32f446re_gpio_driver.d \
./Drivers/Src/stm32f446re_i2c_driver.d \
./Drivers/Src/stm32f446re_spi_driver.d \
./Drivers/Src/stm32f446re_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su Drivers/Src/%.cyclo: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F446RETx -c -I../Inc -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/BSP/Src" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Inc" -I"C:/Users/DM/STM32CubeIDE/workspace_embedded_c/MCU1_001_STM32F446RE_Drivers/Drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/smt32f446re_rcc_driver.cyclo ./Drivers/Src/smt32f446re_rcc_driver.d ./Drivers/Src/smt32f446re_rcc_driver.o ./Drivers/Src/smt32f446re_rcc_driver.su ./Drivers/Src/stm32f446re_gpio_driver.cyclo ./Drivers/Src/stm32f446re_gpio_driver.d ./Drivers/Src/stm32f446re_gpio_driver.o ./Drivers/Src/stm32f446re_gpio_driver.su ./Drivers/Src/stm32f446re_i2c_driver.cyclo ./Drivers/Src/stm32f446re_i2c_driver.d ./Drivers/Src/stm32f446re_i2c_driver.o ./Drivers/Src/stm32f446re_i2c_driver.su ./Drivers/Src/stm32f446re_spi_driver.cyclo ./Drivers/Src/stm32f446re_spi_driver.d ./Drivers/Src/stm32f446re_spi_driver.o ./Drivers/Src/stm32f446re_spi_driver.su ./Drivers/Src/stm32f446re_usart_driver.cyclo ./Drivers/Src/stm32f446re_usart_driver.d ./Drivers/Src/stm32f446re_usart_driver.o ./Drivers/Src/stm32f446re_usart_driver.su

.PHONY: clean-Drivers-2f-Src

