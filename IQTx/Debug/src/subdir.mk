################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/AlsaCapture.cpp \
../src/PRUloader.cpp \
../src/rfmd2080.cpp 

OBJS += \
./src/AlsaCapture.o \
./src/PRUloader.o \
./src/rfmd2080.o 

CPP_DEPS += \
./src/AlsaCapture.d \
./src/PRUloader.d \
./src/rfmd2080.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabi-g++ -I/usr/arm-linux-gnueabi/include -I/usr/arm-linux-gnueabi/include/c++/4.7.3 -I../inc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


