# example disassemble
# arm-none-eabi-objdump -dS stm32f4-rom-emulator.elf >asm.out

# Put your stlink folder here so make burn will work.
STLINK=~/stlink.git

#SRCS=main.c system_stm32f4xx.c stm32f4xx_it.c
#SRCS=main.c system_stm32f4xx.c working-stm32f4xx_it.c

#
#
SRCS=ff.c ffunicode.c sdmm.c main.c system_stm32f4xx.c 

# Library modules
SRCS += stm32f4xx_syscfg.c misc.c stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_exti.c stm32f4xx_pwr.c stm32f4xx_adc.c
#SRCS += stm32f4xx_tim.c 
#SRCS += stm32f4_discovery.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=electron-rom-emulator

#######################################################################################

#STM_COMMON=../../..
# You need libs somewhere!
STM_COMMON=../STM32F4-Discovery_FW_V1.1.0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

#CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
#CFLAGS += --specs=nosys.specs
CFLAGS += -specs=nano.specs -specs=rdimon.specs -lc -lrdimon
## Even though we specify these registers in main.c as global, still need to specify here to stop FATFS and anything else using though (though we luck out with any other libs linked in)
CFLAGS += -ffixed-s0
CFLAGS += -ffixed-s1
CFLAGS += -ffixed-s2
CFLAGS += -ffixed-s3
CFLAGS += -ffixed-s4
CFLAGS += -ffixed-s5
CFLAGS += -ffixed-s6
CFLAGS += -ffixed-s7
CFLAGS += -ffixed-s8
CFLAGS += -ffixed-s9
CFLAGS += -ffixed-s30


# NB: You have to have DEBUG_OUPUT_ON_GPIOA set in order to see any output on PA0
#CFLAGS += -DDEBUG_OUTPUT_ON_GPIOA
#CFLAGS += -DDEBUG_EXTI0_START -DDEBUG_EXTI0_END
#CFLAGS += -DDEBUG_EXTI0_ROM_ACCESS
#CFLAGS += -DDEBUG_EXTI0_SWRAM_WRITE
#CFLAGS += -DDEBUG_FE05
#CFLAGS += -DDEBUG_FC71
#CFLAGS += -DDEBUG_FC72
# If you board has an SD card slot with pullups, then set DISABLE_PULLUPS_FOR_SDCARD
#CFLAGS += -DDISABLE_PULLUPS_FOR_SDCARD
#CFLAGS += -DDEBUG_ADC
#CFLAGS += -DENABLE_SEMIHOSTING
#CFLAGS += -DENABLE_EXTRA_READ_DELAY

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include 
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc


# add startup file to build
SRCS += startup_stm32f4xx.s 
# You need to end asm files in capital S to get them to see preprocessor directives
SRCS += interrupt.S

OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src $(STM_COMMON)/Utilities/STM32F4-Discovery

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin


# Flash the STM32F4
#burn: proj
#	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000
