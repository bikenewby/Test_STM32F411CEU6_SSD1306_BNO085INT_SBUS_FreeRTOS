################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/sh2/euler.c \
../Core/Src/sh2/sh2.c \
../Core/Src/sh2/sh2_SensorValue.c \
../Core/Src/sh2/sh2_platform_impl.c \
../Core/Src/sh2/sh2_util.c \
../Core/Src/sh2/shtp.c 

OBJS += \
./Core/Src/sh2/euler.o \
./Core/Src/sh2/sh2.o \
./Core/Src/sh2/sh2_SensorValue.o \
./Core/Src/sh2/sh2_platform_impl.o \
./Core/Src/sh2/sh2_util.o \
./Core/Src/sh2/shtp.o 

C_DEPS += \
./Core/Src/sh2/euler.d \
./Core/Src/sh2/sh2.d \
./Core/Src/sh2/sh2_SensorValue.d \
./Core/Src/sh2/sh2_platform_impl.d \
./Core/Src/sh2/sh2_util.d \
./Core/Src/sh2/shtp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/sh2/euler.o: ../Core/Src/sh2/euler.c Core/Src/sh2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Krit/STM32CubeIDE/workspace/Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS/Core/Inc/sh2" -I"C:/Users/Krit/STM32CubeIDE/workspace/Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS/Core/Src/sh2" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Core/Src/sh2/%.o Core/Src/sh2/%.su Core/Src/sh2/%.cyclo: ../Core/Src/sh2/%.c Core/Src/sh2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Krit/STM32CubeIDE/workspace/Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS/Core/Inc/sh2" -I"C:/Users/Krit/STM32CubeIDE/workspace/Test_STM32F411CEU6_SSD1306_BNO085INT_SBUS_FreeRTOS/Core/Src/sh2" -I../I-CUBE-EE -I../Middlewares/Third_Party/NimaLTD_Driver/EE -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-sh2

clean-Core-2f-Src-2f-sh2:
	-$(RM) ./Core/Src/sh2/euler.cyclo ./Core/Src/sh2/euler.d ./Core/Src/sh2/euler.o ./Core/Src/sh2/euler.su ./Core/Src/sh2/sh2.cyclo ./Core/Src/sh2/sh2.d ./Core/Src/sh2/sh2.o ./Core/Src/sh2/sh2.su ./Core/Src/sh2/sh2_SensorValue.cyclo ./Core/Src/sh2/sh2_SensorValue.d ./Core/Src/sh2/sh2_SensorValue.o ./Core/Src/sh2/sh2_SensorValue.su ./Core/Src/sh2/sh2_platform_impl.cyclo ./Core/Src/sh2/sh2_platform_impl.d ./Core/Src/sh2/sh2_platform_impl.o ./Core/Src/sh2/sh2_platform_impl.su ./Core/Src/sh2/sh2_util.cyclo ./Core/Src/sh2/sh2_util.d ./Core/Src/sh2/sh2_util.o ./Core/Src/sh2/sh2_util.su ./Core/Src/sh2/shtp.cyclo ./Core/Src/sh2/shtp.d ./Core/Src/sh2/shtp.o ./Core/Src/sh2/shtp.su

.PHONY: clean-Core-2f-Src-2f-sh2

