################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utils/CRC/CRC.c 

OBJS += \
./Utils/CRC/CRC.o 

C_DEPS += \
./Utils/CRC/CRC.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/CRC/%.o Utils/CRC/%.su Utils/CRC/%.cyclo: ../Utils/CRC/%.c Utils/CRC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Project/GolfCartV2/EngineNode/Code/STM32_RTOS_PeripheralDriver" -I"D:/Project/GolfCartV2/EngineNode/Code/STM32-Hardware-Driver" -I"D:/Project/GolfCartV2/EngineNode/Code/Periodics" -I"D:/Project/GolfCartV2/EngineNode/Code/Utils" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utils-2f-CRC

clean-Utils-2f-CRC:
	-$(RM) ./Utils/CRC/CRC.cyclo ./Utils/CRC/CRC.d ./Utils/CRC/CRC.o ./Utils/CRC/CRC.su

.PHONY: clean-Utils-2f-CRC

