#directory definitions
ROOT = ..
SCANDAL = $(ROOT)/scandal
ARCH = $(SCANDAL)/src/arch/lpc11c14
UTILS = ../utilities
LINKER = $(ARCH)/linker
CMSIS = $(ARCH)/cmsis
DRIVERS = $(ARCH)/drivers
STARTUP = $(ARCH)/startup
BUILD=./build# never . or clean will delete everything
SRC = ./src

#define utilities
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy
START_PROGRAMMER = $(UTILS)/dfu-util
FLASH_CHIP = crt_emu_lpc11_13_nxp
CHECKSUM = checksum

#architecture
MCU = cortex-m0
UC = LPC11C14/301

#LINKSCRIPT = lpc1114_flash_crp.ld
LINKSCRIPT=lpc1114_flash_can_crp.ld
#Should LINKSCRIPT=lpc1114_flash_can_crp.ld, It is slightly different in terms of memory management
#Changed on 23 Aug - Charith, saw something in the user manual to do with some black of memory at the
#top being required by the CAN controller
STARTUPSCRIPT = cr_startup_lpc11

OBJECTS  = main.o scandal_obligations.o
OBJECTS += core_cm0.o system_LPC11xx.o leds.o
OBJECTS += gpio.o timer32.o uart.o can.o i2c.o flash.o
OBJECTS += engine.o error.o manager.o maths.o message.o utils.o
OBJECTS += $(STARTUPSCRIPT).o

#compiler flags
OPTIMISATION = s # we need this to fit our code in the flash at the moment :-(
DEBUG = 

CFLAGS  = -mcpu=$(MCU) -DLPC11C14 -D__NEWLIB__ -D__USE_CMSIS -D__CODE_RED -mthumb -O$(OPTIMISATION) $(DEBUG)
CFLAGS += -I$(SCANDAL)/include # for scandal includes
CFLAGS += -I$(ARCH)/include # for arch drivers
CFLAGS += -I$(ARCH) # for cmsis
CFLAGS += -I./include # for project includes
CFLAGS += -lm #Added lm for math.h

LSCRIPT = $(LINKER)/$(LINKSCRIPT)
LFLAGS  = -nostdlib  -Xlinker -Map=$(BUILD)/main.map -Xlinker --gc-sections -L $(LINKER) -T "$(LSCRIPT)"

.PHONY: all clean program

RED="\\033[91m"
NORMAL="\\033[0m"

all: main.axf main.bin

main.axf: $(OBJECTS)
	@echo "[LINK] main.axf"
	@$(CC) $(CFLAGS) $(LFLAGS) $(addprefix $(BUILD)/,$(OBJECTS)) -o $(BUILD)/main.axf 

###compile object files individually
main.o: src/main.c
	@if [ ! -e "cscope.out" ]; then \
		echo -e "[HINT] $(RED)You should learn to use cscope! Type make cscope$(NORMAL)"; \
	fi

	@mkdir -p $(BUILD)
	@echo "[CC] main.c                      this is where your main loop is!"
	@if [ "$(BUILD)" == "." ]; then echo -e "\033[91mDon't use . as BUILD\033[0m"; exit 1; fi
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

scandal_obligations.o: src/scandal_obligations.c
	@echo "[CC] scandal_obligations.c       this is where some local scandal implementations are"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

leds.o: src/leds.c
	@echo "[CC] leds.c                      this is your LED routines"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

core_cm0.o: $(CMSIS)/core_cm0.c
	@echo "[CC] core_cm0.c"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

engine.o: $(SCANDAL)/src/engine.c
	@echo "[CC] scandal/engine.c            this is some core scandal stuff"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

error.o: $(SCANDAL)/src/error.c
	@echo "[CC] scandal/error.c             this is some scandal error handling code"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

manager.o: $(SCANDAL)/src/manager.c
	@echo "[CC] scandal/manager.c           this some of scandal helper code"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

message.o: $(SCANDAL)/src/message.c
	@echo "[CC] scandal/message.c           this is where scandal builds CAN messages"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

utils.o: $(SCANDAL)/src/utils.c
	@echo "[CC] scandal/utils.c             this may have some useful helper functions"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

maths.o: $(SCANDAL)/src/maths.c
	@echo "[CC] scandal/maths.c             this may have some useful math functions"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

system_LPC11xx.o: $(CMSIS)/system_LPC11xx.c
	@echo "[CC] cmsis/system_LPC11xx.c"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

gpio.o: $(DRIVERS)/gpio.c
	@echo "[CC] cmsis/gpio.c                this is the LPC GPIO driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

timer32.o: $(DRIVERS)/timer32.c
	@echo "[CC] cmsis/timer32.c             this is the LPC timer driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

uart.o: $(DRIVERS)/uart.c
	@echo "[CC] cmsis/uart.c                this is the LPC UART driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

can.o: $(DRIVERS)/can.c
	@echo "[CC] cmsis/can.c                 this is the LPC CAN driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

flash.o: $(DRIVERS)/flash.c
	@echo "[CC] cmsis/flash.c               this is the LPC flash driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

i2c.o: $(DRIVERS)/i2c.c
	@echo "[CC] cmsis/i2c.c                 this is the LPC I2C driver"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<

$(STARTUPSCRIPT).o: $(STARTUP)/$(STARTUPSCRIPT).c
	@echo "[CC] startup.c"
	@$(CC) $(CFLAGS) -c -o $(BUILD)/$@ $<


program: main.axf
	@echo "[PROGRAM] main.axf               this is what gets flashed to the target"

	@echo "[PROGRAM]  checking for programmer...";
	@-lsusb | grep -q "df55"; \
	if [ $$? -eq 0 ]; then \
		echo "[PROGRAM]   detected LPC Expresso"; \
		if [ -x "$(START_PROGRAMMER)" ]; then \
			sudo $(START_PROGRAMMER) -d 0x471:0xdf55 -c 0 -t 2048 -R -D $(UTILS)/LPCXpressoWIN.enc > /dev/null 2>&1; \
			if [ $$? -ne 0 ]; then \
				echo -e "\033[91m[PROGRAM]    dfu-util failed to change programmer mode!\033[0m"; \
			else \
				echo -e "\033[91m[PROGRAM]    Successfully changed LPCXpresso into correct mode.\033[0m"; \
				echo -e "\e[1;33m[PROGRAM]    If in VMware Fusion, you may need to reattach the CodeRed device...\033[0m"; \
			fi; \
		else \
			echo -e "\033[91m[PROGRAM]    dfu-util doesn't exist in path!\033[0m"; \
		fi; \
		sleep 1; \
	else \
		lsusb | grep -q "1fc9:0009"; \
		if [ $$? -ne 0 ]; then \
			echo -e "\033[91m[PROGRAM]    No programmer detected!\033[0m"; \
		fi; \
	fi

	@lsusb | grep -q "1fc9:0009"; \
	if [ $$? -eq 0 ]; then \
		echo -e "\033[91m[PROGRAM]   detected LPC Expresso in correct mode, attempting program\033[0m"; \
		$(FLASH_CHIP) -wire=winusb -p$(UC) -flash-load-exec=$(BUILD)/main.axf; \
	fi

#program using .bin, doesn't seem to work atm
#sudo $(FLASH_CHIP) -wire=winusb -p$(UC) -flash-load=$(BUILD)/main.bin -load-base=0x0000

clean:
	@echo "[CLEAN] $(BUILD)"
	@if [ "$(BUILD)" == "." ]; then echo -e "\033[91mDon't use . as BUILD\033[0m"; exit 1; fi
	@rm -Rf $(BUILD)

cscope:
	@echo "[GEN] cscope"
	@if [ -d "$(SCANDAL)" ]; then \
		find $(SCANDAL) -name "*.[chSs]" >> cscope.files; \
	fi
	@find . -name "*.[chSs]">> cscope.files
	@cscope -b -f cscope.out

main.bin: main.axf
	@echo "[SIZE] main.axf"
	@$(SIZE) $(BUILD)/main.axf; 
	@echo "[GEN] main.bin"
	@$(OBJCOPY) -O binary $(BUILD)/main.axf $(BUILD)/main.bin;
	@echo "[CHECKSUM] main.bin"
	@echo -ne "   "
	@$(CHECKSUM) -p $(UC) -d $(BUILD)/main.bin;
	@echo
