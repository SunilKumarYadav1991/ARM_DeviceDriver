################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f411_gpio_driver.c 

OBJS += \
./Drivers/Src/stm32f411_gpio_driver.o 

C_DEPS += \
./Drivers/Src/stm32f411_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/%.o Drivers/Src/%.su: ../Drivers/Src/%.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -c -I../Inc -I"D:/Win7_Vostro/G_Drive/VectorCAST_Copy/SKY/11.udemy/3.ARM CortexM/Workspace/STM32F411/DD.3.STM32F411_Driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-Src

clean-Drivers-2f-Src:
	-$(RM) ./Drivers/Src/stm32f411_gpio_driver.d ./Drivers/Src/stm32f411_gpio_driver.o ./Drivers/Src/stm32f411_gpio_driver.su

.PHONY: clean-Drivers-2f-Src

