################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/gsvFitcher.cpp \
../src/line_detector.cpp \
../src/test_main.cpp \
../src/tracker.cpp 

OBJS += \
./src/gsvFitcher.o \
./src/line_detector.o \
./src/test_main.o \
./src/tracker.o 

CPP_DEPS += \
./src/gsvFitcher.d \
./src/line_detector.d \
./src/test_main.d \
./src/tracker.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/local/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


