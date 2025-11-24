################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../external/CMSIS/system_stm32f4xx.c 

OBJS += \
./external/CMSIS/system_stm32f4xx.o 

C_DEPS += \
./external/CMSIS/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
external/CMSIS/%.o external/CMSIS/%.su external/CMSIS/%.cyclo: ../external/CMSIS/%.c external/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F411xE -DSTM32F411RETx -DENABLE_SERIAL=1 -c -I../ -I../external -I../external/CMSIS -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-external-2f-CMSIS

clean-external-2f-CMSIS:
	-$(RM) ./external/CMSIS/system_stm32f4xx.cyclo ./external/CMSIS/system_stm32f4xx.d ./external/CMSIS/system_stm32f4xx.o ./external/CMSIS/system_stm32f4xx.su

.PHONY: clean-external-2f-CMSIS

