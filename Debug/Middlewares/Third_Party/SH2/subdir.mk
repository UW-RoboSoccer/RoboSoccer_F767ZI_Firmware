################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SH2/euler.c \
../Middlewares/Third_Party/SH2/sh2.c \
../Middlewares/Third_Party/SH2/sh2_SensorValue.c \
../Middlewares/Third_Party/SH2/sh2_util.c \
../Middlewares/Third_Party/SH2/shtp.c 

OBJS += \
./Middlewares/Third_Party/SH2/euler.o \
./Middlewares/Third_Party/SH2/sh2.o \
./Middlewares/Third_Party/SH2/sh2_SensorValue.o \
./Middlewares/Third_Party/SH2/sh2_util.o \
./Middlewares/Third_Party/SH2/shtp.o 

C_DEPS += \
./Middlewares/Third_Party/SH2/euler.d \
./Middlewares/Third_Party/SH2/sh2.d \
./Middlewares/Third_Party/SH2/sh2_SensorValue.d \
./Middlewares/Third_Party/SH2/sh2_util.d \
./Middlewares/Third_Party/SH2/shtp.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SH2/%.o Middlewares/Third_Party/SH2/%.su Middlewares/Third_Party/SH2/%.cyclo: ../Middlewares/Third_Party/SH2/%.c Middlewares/Third_Party/SH2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/SH2 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -pedantic -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-SH2

clean-Middlewares-2f-Third_Party-2f-SH2:
	-$(RM) ./Middlewares/Third_Party/SH2/euler.cyclo ./Middlewares/Third_Party/SH2/euler.d ./Middlewares/Third_Party/SH2/euler.o ./Middlewares/Third_Party/SH2/euler.su ./Middlewares/Third_Party/SH2/sh2.cyclo ./Middlewares/Third_Party/SH2/sh2.d ./Middlewares/Third_Party/SH2/sh2.o ./Middlewares/Third_Party/SH2/sh2.su ./Middlewares/Third_Party/SH2/sh2_SensorValue.cyclo ./Middlewares/Third_Party/SH2/sh2_SensorValue.d ./Middlewares/Third_Party/SH2/sh2_SensorValue.o ./Middlewares/Third_Party/SH2/sh2_SensorValue.su ./Middlewares/Third_Party/SH2/sh2_util.cyclo ./Middlewares/Third_Party/SH2/sh2_util.d ./Middlewares/Third_Party/SH2/sh2_util.o ./Middlewares/Third_Party/SH2/sh2_util.su ./Middlewares/Third_Party/SH2/shtp.cyclo ./Middlewares/Third_Party/SH2/shtp.d ./Middlewares/Third_Party/SH2/shtp.o ./Middlewares/Third_Party/SH2/shtp.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-SH2

