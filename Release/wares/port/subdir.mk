################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../wares/port/rtklib_mem.c \
../wares/port/rtklib_port.c \
../wares/port/rtklib_serial.c \
../wares/port/rtklib_time.c 

OBJS += \
./wares/port/rtklib_mem.o \
./wares/port/rtklib_port.o \
./wares/port/rtklib_serial.o \
./wares/port/rtklib_time.o 

C_DEPS += \
./wares/port/rtklib_mem.d \
./wares/port/rtklib_port.d \
./wares/port/rtklib_serial.d \
./wares/port/rtklib_time.d 


# Each subdirectory must supply rules for building sources it contributes
wares/port/%.o wares/port/%.su wares/port/%.cyclo: ../wares/port/%.c wares/port/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DDONT_USE_MALLOC -DSTM32_PLATFORM -DRTKLIB_EMBEDDED -DNO_SYSTEMTIME -DENACMP -DSTM32L476xx -DRTK_DEBUG_PORT_UART1=0 -c -I../Core/Inc -I"D:/ST/UBLOX/U/wares/inc" -I"D:/ST/UBLOX/U/wares/port" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-wares-2f-port

clean-wares-2f-port:
	-$(RM) ./wares/port/rtklib_mem.cyclo ./wares/port/rtklib_mem.d ./wares/port/rtklib_mem.o ./wares/port/rtklib_mem.su ./wares/port/rtklib_port.cyclo ./wares/port/rtklib_port.d ./wares/port/rtklib_port.o ./wares/port/rtklib_port.su ./wares/port/rtklib_serial.cyclo ./wares/port/rtklib_serial.d ./wares/port/rtklib_serial.o ./wares/port/rtklib_serial.su ./wares/port/rtklib_time.cyclo ./wares/port/rtklib_time.d ./wares/port/rtklib_time.o ./wares/port/rtklib_time.su

.PHONY: clean-wares-2f-port

