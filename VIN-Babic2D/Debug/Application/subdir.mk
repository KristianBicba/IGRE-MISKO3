################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/game.c \
../Application/graphics.c \
../Application/math_utils.c \
../Application/objects.c 

C_DEPS += \
./Application/game.d \
./Application/graphics.d \
./Application/math_utils.d \
./Application/objects.d 

OBJS += \
./Application/game.o \
./Application/graphics.o \
./Application/math_utils.o \
./Application/objects.o 


# Each subdirectory must supply rules for building sources it contributes
Application/%.o Application/%.su: ../Application/%.c Application/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Kristian/Desktop/VIN/VIN-Babic2D/System/Include" -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I"C:/Users/Kristian/Desktop/VIN/VIN-Babic2D/Application/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application

clean-Application:
	-$(RM) ./Application/game.d ./Application/game.o ./Application/game.su ./Application/graphics.d ./Application/graphics.o ./Application/graphics.su ./Application/math_utils.d ./Application/math_utils.o ./Application/math_utils.su ./Application/objects.d ./Application/objects.o ./Application/objects.su

.PHONY: clean-Application

