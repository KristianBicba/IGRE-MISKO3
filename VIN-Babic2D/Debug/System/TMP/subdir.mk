################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/TMP/XPT2046_touch.c \
../System/TMP/lcd.c \
../System/TMP/lcd_ili9341.c \
../System/TMP/ugui.c 

C_DEPS += \
./System/TMP/XPT2046_touch.d \
./System/TMP/lcd.d \
./System/TMP/lcd_ili9341.d \
./System/TMP/ugui.d 

OBJS += \
./System/TMP/XPT2046_touch.o \
./System/TMP/lcd.o \
./System/TMP/lcd_ili9341.o \
./System/TMP/ugui.o 


# Each subdirectory must supply rules for building sources it contributes
System/TMP/%.o: ../System/TMP/%.c System/TMP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32G474xx -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"H:/ome/Rok_Plesko/Workspace/Vaja10/System/Include" -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-TMP

clean-System-2f-TMP:
	-$(RM) ./System/TMP/XPT2046_touch.d ./System/TMP/XPT2046_touch.o ./System/TMP/lcd.d ./System/TMP/lcd.o ./System/TMP/lcd_ili9341.d ./System/TMP/lcd_ili9341.o ./System/TMP/ugui.d ./System/TMP/ugui.o

.PHONY: clean-System-2f-TMP

