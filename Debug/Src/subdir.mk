################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SPI_STM32&ARDUINO_SendData.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/SPI_STM32&ARDUINO_SendData.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/SPI_STM32&ARDUINO_SendData.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411CEUx -c -I../Inc -I"/home/aditya/Documents/Programming/ERS/Driver Development/WS1/STM32F411CEU6_Drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/SPI_STM32&ARDUINO_SendData.cyclo ./Src/SPI_STM32&ARDUINO_SendData.d ./Src/SPI_STM32&ARDUINO_SendData.o ./Src/SPI_STM32&ARDUINO_SendData.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

