################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/tests/test_v327_mixer.c 

OBJS += \
./Core/tests/test_v327_mixer.o 

C_DEPS += \
./Core/tests/test_v327_mixer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/tests/%.o Core/tests/%.su Core/tests/%.cyclo: ../Core/tests/%.c Core/tests/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-tests

clean-Core-2f-tests:
	-$(RM) ./Core/tests/test_v327_mixer.cyclo ./Core/tests/test_v327_mixer.d ./Core/tests/test_v327_mixer.o ./Core/tests/test_v327_mixer.su

.PHONY: clean-Core-2f-tests

