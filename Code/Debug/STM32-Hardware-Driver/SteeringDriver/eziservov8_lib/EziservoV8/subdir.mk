################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.c \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.c \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.c \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.c \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.c \
../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.c 

OBJS += \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.o \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.o \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.o \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.o \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.o \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.o 

C_DEPS += \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.d \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.d \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.d \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.d \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.d \
./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.d 


# Each subdirectory must supply rules for building sources it contributes
STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/%.o STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/%.su STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/%.cyclo: ../STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/%.c STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"D:/Project/GolfCartV2/EngineNode/Code/STM32_RTOS_PeripheralDriver" -I"D:/Project/GolfCartV2/EngineNode/Code/STM32-Hardware-Driver" -I"D:/Project/GolfCartV2/EngineNode/Code/Periodics" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-STM32-2d-Hardware-2d-Driver-2f-SteeringDriver-2f-eziservov8_lib-2f-EziservoV8

clean-STM32-2d-Hardware-2d-Driver-2f-SteeringDriver-2f-eziservov8_lib-2f-EziservoV8:
	-$(RM) ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_IOControl.su ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Motion.su ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Port.su ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_ServoControl.su ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_StatusPos.su ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.cyclo ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.d ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.o ./STM32-Hardware-Driver/SteeringDriver/eziservov8_lib/EziservoV8/FAS_Util.su

.PHONY: clean-STM32-2d-Hardware-2d-Driver-2f-SteeringDriver-2f-eziservov8_lib-2f-EziservoV8

