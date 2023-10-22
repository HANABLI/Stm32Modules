################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/007main.c \
../Drivers/Src/stm32f303xx_gpio_drivers.c \
../Drivers/Src/stm32f303xx_i2c_drivers.c \
../Drivers/Src/stm32f303xx_rcc_driver.c 

OBJS += \
./Drivers/Src/007main.o \
./Drivers/Src/stm32f303xx_gpio_drivers.o \
./Drivers/Src/stm32f303xx_i2c_drivers.o \
./Drivers/Src/stm32f303xx_rcc_driver.o 

C_DEPS += \
./Drivers/Src/007main.d \
./Drivers/Src/stm32f303xx_gpio_drivers.d \
./Drivers/Src/stm32f303xx_i2c_drivers.d \
./Drivers/Src/stm32f303xx_rcc_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/007main.o: ../Drivers/Src/007main.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -c -I../Inc -I"C:/Users/hqnqb/STM32CubeIDE/workspace_1.3.0/stm32f303xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/007main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f303xx_gpio_drivers.o: ../Drivers/Src/stm32f303xx_gpio_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -c -I../Inc -I"C:/Users/hqnqb/STM32CubeIDE/workspace_1.3.0/stm32f303xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f303xx_gpio_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f303xx_i2c_drivers.o: ../Drivers/Src/stm32f303xx_i2c_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -c -I../Inc -I"C:/Users/hqnqb/STM32CubeIDE/workspace_1.3.0/stm32f303xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f303xx_i2c_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f303xx_rcc_driver.o: ../Drivers/Src/stm32f303xx_rcc_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F3 -DDEBUG -DSTM32F303RETx -c -I../Inc -I"C:/Users/hqnqb/STM32CubeIDE/workspace_1.3.0/stm32f303xx_drivers/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f303xx_rcc_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

