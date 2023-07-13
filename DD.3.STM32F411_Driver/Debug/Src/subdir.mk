################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/3.ButtonInterrupt.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/3.ButtonInterrupt.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/3.ButtonInterrupt.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F411VETx -DSTM32 -DSTM32F4 -c -I../Inc -I"D:/Win7_Vostro/G_Drive/VectorCAST_Copy/SKY/11.udemy/3.ARM CortexM/Workspace/STM32F411/DD.3.STM32F411_Driver/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/3.ButtonInterrupt.d ./Src/3.ButtonInterrupt.o ./Src/3.ButtonInterrupt.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

