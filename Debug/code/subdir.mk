################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/\ control.c \
../code/adc.c \
../code/breakroad.c \
../code/cross.c \
../code/elements.c \
../code/fuse.c \
../code/gyro.c \
../code/image.c \
../code/list.c \
../code/obstacle.c \
../code/pid.c \
../code/ramp.c \
../code/road.c \
../code/zebra.c 

OBJS += \
./code/\ control.o \
./code/adc.o \
./code/breakroad.o \
./code/cross.o \
./code/elements.o \
./code/fuse.o \
./code/gyro.o \
./code/image.o \
./code/list.o \
./code/obstacle.o \
./code/pid.o \
./code/ramp.o \
./code/road.o \
./code/zebra.o 

COMPILED_SRCS += \
./code/\ control.src \
./code/adc.src \
./code/breakroad.src \
./code/cross.src \
./code/elements.src \
./code/fuse.src \
./code/gyro.src \
./code/image.src \
./code/list.src \
./code/obstacle.src \
./code/pid.src \
./code/ramp.src \
./code/road.src \
./code/zebra.src 

C_DEPS += \
./code/\ control.d \
./code/adc.d \
./code/breakroad.d \
./code/cross.d \
./code/elements.d \
./code/fuse.d \
./code/gyro.d \
./code/image.d \
./code/list.d \
./code/obstacle.d \
./code/pid.d \
./code/ramp.d \
./code/road.d \
./code/zebra.d 


# Each subdirectory must supply rules for building sources it contributes
code/\ control.src: ../code/\ control.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/BenBen/BEN_ben ver1.01/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/\ control.o: ./code/\ control.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '

code/%.src: ../code/%.c code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb "-fD:/BenBen/BEN_ben ver1.01/Debug/TASKING_C_C___Compiler-Include_paths.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file="$(@:.src=.d)" --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

code/%.o: ./code/%.src code/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


