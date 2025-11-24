################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../external/startup.s 

C_SRCS += \
../external/sleep.c \
../external/stdio_uart.c 

OBJS += \
./external/sleep.o \
./external/startup.o \
./external/stdio_uart.o 

S_DEPS += \
./external/startup.d 

C_DEPS += \
./external/sleep.d \
./external/stdio_uart.d 


# Each subdirectory must supply rules for building sources it contributes
external/%.o external/%.su external/%.cyclo: ../external/%.c external/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411xE -DSTM32F411RETx -DENABLE_SERIAL=1 -c -I../ -I../external -I../external/CMSIS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
external/%.o: ../external/%.s external/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-external

clean-external:
	-$(RM) ./external/sleep.cyclo ./external/sleep.d ./external/sleep.o ./external/sleep.su ./external/startup.d ./external/startup.o ./external/stdio_uart.cyclo ./external/stdio_uart.d ./external/stdio_uart.o ./external/stdio_uart.su

.PHONY: clean-external

