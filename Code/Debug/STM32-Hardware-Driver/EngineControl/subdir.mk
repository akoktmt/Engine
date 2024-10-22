################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32-Hardware-Driver/EngineControl/EngineControl.c 

OBJS += \
./STM32-Hardware-Driver/EngineControl/EngineControl.o 

C_DEPS += \
./STM32-Hardware-Driver/EngineControl/EngineControl.d 


# Each subdirectory must supply rules for building sources it contributes
STM32-Hardware-Driver/EngineControl/%.o STM32-Hardware-Driver/EngineControl/%.su STM32-Hardware-Driver/EngineControl/%.cyclo: ../STM32-Hardware-Driver/EngineControl/%.c STM32-Hardware-Driver/EngineControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/STM32-Hardware-Driver" -I../Core/ThreadSafe -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/UtilitySmallFunction" -I"C:/Users/win 10/OneDrive/Desktop/EngineNode/golfelectric/enginenode/Code/CRC" -I../CAN_Handle -I../STM32_RTOS_PeripheralDriver -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32-2d-Hardware-2d-Driver-2f-EngineControl

clean-STM32-2d-Hardware-2d-Driver-2f-EngineControl:
	-$(RM) ./STM32-Hardware-Driver/EngineControl/EngineControl.cyclo ./STM32-Hardware-Driver/EngineControl/EngineControl.d ./STM32-Hardware-Driver/EngineControl/EngineControl.o ./STM32-Hardware-Driver/EngineControl/EngineControl.su

.PHONY: clean-STM32-2d-Hardware-2d-Driver-2f-EngineControl

