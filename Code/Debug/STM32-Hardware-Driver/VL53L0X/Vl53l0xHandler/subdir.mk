################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.c 

OBJS += \
./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.o 

C_DEPS += \
./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.d 


# Each subdirectory must supply rules for building sources it contributes
STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/%.o STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/%.su STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/%.cyclo: ../STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/%.c STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DSTM32_THREAD_SAFE_STRATEGY=4 -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Project/GolfCartV2/EngineNode/Code/STM32_RTOS_PeripheralDriver" -I"D:/Project/GolfCartV2/EngineNode/Code/STM32-Hardware-Driver" -I"D:/Project/GolfCartV2/EngineNode/Code/Periodics" -I../Core/ThreadSafe -I"D:/Project/GolfCartV2/EngineNode/Code/UtilitySmallFunction" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32-2d-Hardware-2d-Driver-2f-VL53L0X-2f-Vl53l0xHandler

clean-STM32-2d-Hardware-2d-Driver-2f-VL53L0X-2f-Vl53l0xHandler:
	-$(RM) ./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.cyclo ./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.d ./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.o ./STM32-Hardware-Driver/VL53L0X/Vl53l0xHandler/Vl53l0x.su

.PHONY: clean-STM32-2d-Hardware-2d-Driver-2f-VL53L0X-2f-Vl53l0xHandler

