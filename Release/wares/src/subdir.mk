################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../wares/src/datum.c \
../wares/src/ephemeris.c \
../wares/src/lambda.c \
../wares/src/options.c \
../wares/src/pntpos.c \
../wares/src/rcvraw.c \
../wares/src/rtcm.c \
../wares/src/rtcm2.c \
../wares/src/rtcm3.c \
../wares/src/rtkcmn.c \
../wares/src/rtkpos.c \
../wares/src/solution.c \
../wares/src/ublox.c 

OBJS += \
./wares/src/datum.o \
./wares/src/ephemeris.o \
./wares/src/lambda.o \
./wares/src/options.o \
./wares/src/pntpos.o \
./wares/src/rcvraw.o \
./wares/src/rtcm.o \
./wares/src/rtcm2.o \
./wares/src/rtcm3.o \
./wares/src/rtkcmn.o \
./wares/src/rtkpos.o \
./wares/src/solution.o \
./wares/src/ublox.o 

C_DEPS += \
./wares/src/datum.d \
./wares/src/ephemeris.d \
./wares/src/lambda.d \
./wares/src/options.d \
./wares/src/pntpos.d \
./wares/src/rcvraw.d \
./wares/src/rtcm.d \
./wares/src/rtcm2.d \
./wares/src/rtcm3.d \
./wares/src/rtkcmn.d \
./wares/src/rtkpos.d \
./wares/src/solution.d \
./wares/src/ublox.d 


# Each subdirectory must supply rules for building sources it contributes
wares/src/%.o wares/src/%.su wares/src/%.cyclo: ../wares/src/%.c wares/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DDONT_USE_MALLOC -DSTM32_PLATFORM -DRTKLIB_EMBEDDED -DNO_SYSTEMTIME -DSTM32L476xx -DRTK_DEBUG_PORT_UART1=0 -DMCU_SOLVE_RTK=0 -c -I../Core/Inc -I"D:/ST/UBLOX/U/wares/inc" -I"D:/ST/UBLOX/U/wares/port" -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O2 -ffunction-sections -fdata-sections -Wall -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-wares-2f-src

clean-wares-2f-src:
	-$(RM) ./wares/src/datum.cyclo ./wares/src/datum.d ./wares/src/datum.o ./wares/src/datum.su ./wares/src/ephemeris.cyclo ./wares/src/ephemeris.d ./wares/src/ephemeris.o ./wares/src/ephemeris.su ./wares/src/lambda.cyclo ./wares/src/lambda.d ./wares/src/lambda.o ./wares/src/lambda.su ./wares/src/options.cyclo ./wares/src/options.d ./wares/src/options.o ./wares/src/options.su ./wares/src/pntpos.cyclo ./wares/src/pntpos.d ./wares/src/pntpos.o ./wares/src/pntpos.su ./wares/src/rcvraw.cyclo ./wares/src/rcvraw.d ./wares/src/rcvraw.o ./wares/src/rcvraw.su ./wares/src/rtcm.cyclo ./wares/src/rtcm.d ./wares/src/rtcm.o ./wares/src/rtcm.su ./wares/src/rtcm2.cyclo ./wares/src/rtcm2.d ./wares/src/rtcm2.o ./wares/src/rtcm2.su ./wares/src/rtcm3.cyclo ./wares/src/rtcm3.d ./wares/src/rtcm3.o ./wares/src/rtcm3.su ./wares/src/rtkcmn.cyclo ./wares/src/rtkcmn.d ./wares/src/rtkcmn.o ./wares/src/rtkcmn.su ./wares/src/rtkpos.cyclo ./wares/src/rtkpos.d ./wares/src/rtkpos.o ./wares/src/rtkpos.su ./wares/src/solution.cyclo ./wares/src/solution.d ./wares/src/solution.o ./wares/src/solution.su ./wares/src/ublox.cyclo ./wares/src/ublox.d ./wares/src/ublox.o ./wares/src/ublox.su

.PHONY: clean-wares-2f-src

