################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.c 

OBJS += \
./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.o 

C_DEPS += \
./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.d 


# Each subdirectory must supply rules for building sources it contributes
STM32_RTOS_PeripheralDriver/GPIO/%.o STM32_RTOS_PeripheralDriver/GPIO/%.su STM32_RTOS_PeripheralDriver/GPIO/%.cyclo: ../STM32_RTOS_PeripheralDriver/GPIO/%.c STM32_RTOS_PeripheralDriver/GPIO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/STM32-Hardware-Driver" -I../Core/ThreadSafe -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/UtilitySmallFunction" -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/CRC" -I../CAN_Handle -I../STM32_RTOS_PeripheralDriver -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32_RTOS_PeripheralDriver-2f-GPIO

clean-STM32_RTOS_PeripheralDriver-2f-GPIO:
	-$(RM) ./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.cyclo ./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.d ./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.o ./STM32_RTOS_PeripheralDriver/GPIO/GPIOHandler.su

.PHONY: clean-STM32_RTOS_PeripheralDriver-2f-GPIO

