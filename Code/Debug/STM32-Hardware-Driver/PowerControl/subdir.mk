################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32-Hardware-Driver/PowerControl/PowerControl.c 

OBJS += \
./STM32-Hardware-Driver/PowerControl/PowerControl.o 

C_DEPS += \
./STM32-Hardware-Driver/PowerControl/PowerControl.d 


# Each subdirectory must supply rules for building sources it contributes
STM32-Hardware-Driver/PowerControl/%.o STM32-Hardware-Driver/PowerControl/%.su STM32-Hardware-Driver/PowerControl/%.cyclo: ../STM32-Hardware-Driver/PowerControl/%.c STM32-Hardware-Driver/PowerControl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Project/GolfCartV2/EngineNode/Code/STM32_RTOS_PeripheralDriver" -I"D:/Project/GolfCartV2/EngineNode/Code/STM32-Hardware-Driver" -I"D:/Project/GolfCartV2/EngineNode/Code/Periodics" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32-2d-Hardware-2d-Driver-2f-PowerControl

clean-STM32-2d-Hardware-2d-Driver-2f-PowerControl:
	-$(RM) ./STM32-Hardware-Driver/PowerControl/PowerControl.cyclo ./STM32-Hardware-Driver/PowerControl/PowerControl.d ./STM32-Hardware-Driver/PowerControl/PowerControl.o ./STM32-Hardware-Driver/PowerControl/PowerControl.su

.PHONY: clean-STM32-2d-Hardware-2d-Driver-2f-PowerControl
