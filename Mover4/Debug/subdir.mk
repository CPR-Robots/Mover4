################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../cpr_Matrix4.cpp \
../cpr_InputKeyboard.cpp \
../cpr_KinematicMover.cpp \
../cpr_RS232CAN.cpp \
../cpr_PCAN.cpp \
../cpr_mover4.cpp 

OBJS += \
./cpr_Matrix4.o \
./cpr_InputKeyboard.o \
./cpr_KinematicMover.o \
./cpr_RS232CAN.o \
./cpr_PCAN.o \
./cpr_mover4.o 

CPP_DEPS += \
./cpr_Matrix4.d \
./cpr_InputKeyboard.d \
./cpr_KinematicMover.d \
./cpr_RS232CAN.d \
./cpr_PCAN.d \
./cpr_mover4.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/usr/include/boost -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


