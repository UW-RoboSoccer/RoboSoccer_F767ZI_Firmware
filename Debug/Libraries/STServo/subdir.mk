################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Libraries/STServo/STServo.c \
../Libraries/STServo/STServo_Protocol.c 

OBJS += \
./Libraries/STServo/STServo.o \
./Libraries/STServo/STServo_Protocol.o 

C_DEPS += \
./Libraries/STServo/STServo.d \
./Libraries/STServo/STServo_Protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Libraries/STServo/%.o Libraries/STServo/%.su Libraries/STServo/%.cyclo: ../Libraries/STServo/%.c Libraries/STServo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1 -I../Middlewares/Third_Party/SH2 -I../Libraries/STServo -O0 -ffunction-sections -fdata-sections -Wall -pedantic -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Libraries-2f-STServo

clean-Libraries-2f-STServo:
	-$(RM) ./Libraries/STServo/STServo.cyclo ./Libraries/STServo/STServo.d ./Libraries/STServo/STServo.o ./Libraries/STServo/STServo.su ./Libraries/STServo/STServo_Protocol.cyclo ./Libraries/STServo/STServo_Protocol.d ./Libraries/STServo/STServo_Protocol.o ./Libraries/STServo/STServo_Protocol.su

.PHONY: clean-Libraries-2f-STServo

