####################################################################
#                              Note                                #
#                                                                  #
# This Makefile is intended for inclusion by the user Makefile.    #
# Do not run make on this file.                                    #
#                                                                  #
####################################################################


####################################################################
# Hardware                                                         #
####################################################################

DEVICE        = EFM32G280F128
DEVICE_FAMILY = EFM32G
CPU           = cortex-m3


####################################################################
# You might need to do changes to match your system setup          #
####################################################################

# Sourcery CodeBench tools from
# https://sourcery.mentor.com/GNUToolchain/release2322
CMSISDIR  = $(SYSTEMDIR)/libs

PATH_TEST_PRG = arm-none-eabi-gcc

ifdef SystemRoot
	FULL_PRG = $(shell where $(PATH_TEST_PRG))
	TOOLDIR  = $(patsubst %\bin\$(PATH_TEST_PRG).exe,%,$(FULL_PRG))
	FLASH    = eACommander.exe
	RM       = "$(TOOLDIR)/bin/cs-rm" -rf
else
	FULL_PRG = $(shell which $(PATH_TEST_PRG))
	TOOLDIR  = $(patsubst %/bin/$(PATH_TEST_PRG),%,$(FULL_PRG))
	FLASH    = eACommander.sh
	RM       = rm -rf
endif

OBJ_LST_DIR = tmp
EXE_DIR     = bin

MAINFILE = main

CC      = "$(TOOLDIR)/bin/arm-none-eabi-gcc"
CXX_CC  = "$(TOOLDIR)/bin/arm-none-eabi-g++"
LD      = "$(TOOLDIR)/bin/arm-none-eabi-ld"
AR      = "$(TOOLDIR)/bin/arm-none-eabi-ar"
OBJCOPY = "$(TOOLDIR)/bin/arm-none-eabi-objcopy"
DUMP    = "$(TOOLDIR)/bin/arm-none-eabi-objdump"
SIZE    = "$(TOOLDIR)/bin/arm-none-eabi-size"

GCCVERSION = $(shell $(CC) -dumpversion)

.SUFFIXES:


####################################################################
# Flags                                                            #
####################################################################

LIBS += \
	-lc           \
	-lcs3         \
	-lcs3unhosted \

# -MMD : Don't generate dependencies on system header files.
# -MP  : Add phony targets, useful when a h-file is removed from a project.
# -MF  : Specify a file to write the dependencies to.

CPPFLAGS += \
	-D$(DEVICE)                             \
	-MMD                                    \
	-MP                                     \
	-MF                                     \
	$(@:.o=.d)                              \
	-mcpu=$(CPU)                            \
	-Wa,-ahlms=$(OBJ_LST_DIR)/$(@F:.o=.lst) \
	-Os                                     \
	-mthumb                                 \
	-ffunction-sections                     \

ASFLAGS += -Ttext 0x0

LDFLAGS += \
	-Xlinker                                                               \
	-Map=$(OBJ_LST_DIR)/main.map                                           \
	-mcpu=$(CPU)                                                           \
	-mthumb                                                                \
	-L"$(TOOLDIR)/lib/gcc/arm-none-eabi/$(GCCVERSION)/thumb2"              \
	-T$(CMSISDIR)/Device/EnergyMicro/$(DEVICE_FAMILY)/Source/G++/efm32g.ld \
	-Wl,--gc-sections                                                      \
	-feliminate-unused-debug-symbols                                       \


####################################################################
# Source files                                                     #
####################################################################

SYSTEM_C_SRC += \
	$(CMSISDIR)/Device/EnergyMicro/EFM32G/Source/system_efm32g.c \
	$(CMSISDIR)/emlib/src/em_assert.c                            \
	$(CMSISDIR)/emlib/src/em_system.c                            \
	$(CMSISDIR)/emlib/src/em_usart.c                             \
	$(CMSISDIR)/emlib/src/em_cmu.c                               \
	$(CMSISDIR)/emlib/src/em_emu.c                               \
	$(CMSISDIR)/emlib/src/em_gpio.c                              \
	$(CMSISDIR)/emlib/src/em_adc.c                               \
	$(CMSISDIR)/emlib/src/em_i2c.c                               \
	$(SYSTEMDIR)/core/syscalls.c                                 \

SYSTEM_CXX_SRC += \
	$(SYSTEMDIR)/drv/time.cpp             \
	$(SYSTEMDIR)/drv/RTC_DS3234.cpp       \
	$(SYSTEMDIR)/drv/DebugInterface.cpp   \
	$(SYSTEMDIR)/drv/board/sentio_em.cpp  \
	$(SYSTEMDIR)/drv/AnalogInput.cpp      \
	$(SYSTEMDIR)/drv/XBEE_Radio.cpp       \
	$(SYSTEMDIR)/drv/CC1101_Radio.cpp     \
	$(SYSTEMDIR)/drv/SHT1X_Sensirion.cpp  \
	$(SYSTEMDIR)/drv/LTC2990.cpp          \
	$(SYSTEMDIR)/drv/extern/ConfEH.cpp    \
	$(SYSTEMDIR)/core/fsm.cpp             \
	$(SYSTEMDIR)/core/DriverInterface.cpp \
	$(SYSTEMDIR)/core/main.cpp            \

SYSTEM_ASM += \
	$(CMSISDIR)/Device/EnergyMicro/$(DEVICE_FAMILY)/Source/G++/startup_efm32g.s

SYSTEMINCLUDEPATHS += \
	$(CMSISDIR)/emlib/inc                         \
	$(CMSISDIR)/CMSIS/Include                     \
	$(CMSISDIR)/Device/EnergyMicro/EFM32G/Include \
	$(SYSTEMDIR)/core                             \
	$(SYSTEMDIR)/drv                              \
	$(SYSTEMDIR)/drv/board                        \
	$(SYSTEMDIR)/drv/extern                       \

INCLUDES += \
	$(foreach includedir,$(SYSTEMINCLUDEPATHS),-I$(includedir)) \
	$(foreach includedir,$(USERINCLUDEPATHS),-I$(includedir))   \
	-I.                                                         \

C_SRC   = $(SYSTEM_C_SRC)   $(USER_C_SRC)
CXX_SRC = $(SYSTEM_CXX_SRC) $(USER_CXX_SRC)
S_SRC   = $(SYSTEM_ASM)     $(USER_ASM_SRC)


####################################################################
# Rules                                                            #
####################################################################

C_FILES   = $(notdir $(C_SRC))
CXX_FILES = $(notdir $(CXX_SRC))
S_FILES   = $(notdir $(S_SRC))

C_PATHS   = $(sort $(dir $(C_SRC) ))
CXX_PATHS = $(sort $(dir $(CXX_SRC)))
S_PATHS   = $(sort $(dir $(S_SRC)))

C_OBJS   = $(addprefix $(OBJ_LST_DIR)/, $(C_FILES:.c=.o))
CXX_OBJS = $(addprefix $(OBJ_LST_DIR)/, $(CXX_FILES:.cpp=.o))
S_OBJS   = $(addprefix $(OBJ_LST_DIR)/, $(S_FILES:.s=.o))

vpath %.c   $(C_PATHS)
vpath %.cpp $(CXX_PATHS)
vpath %.s   $(S_PATHS)

.PHONY: all
all: $(EXE_DIR)/$(MAINFILE).bin

.PHONY: doc
doc:
	doxygen $(DOC_DIR)/Doxyfile

$(OBJ_LST_DIR):
	- mkdir $(OBJ_LST_DIR)
	@- echo "Created build directory."

$(EXE_DIR):
	- mkdir $(EXE_DIR)
	@- echo "Created executable directory."

$(OBJ_LST_DIR)/%.o: %.cpp
	@- echo "Building file: $<"
	$(CXX_CC) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -c -o $@ $<
	$(SIZE) $@

$(OBJ_LST_DIR)/%.o: %.c
	@- echo "Building file: $<"
	$(CC) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -c -o $@ $<
	$(SIZE) $@

$(OBJ_LST_DIR)/%.o: %.s
	@- echo "Assembling file: $<"
	$(CC) $(ASFLAGS) $(INCLUDES) -c -o $@ $<
	$(SIZE) $@

$(EXE_DIR)/$(MAINFILE).out: $(EXE_DIR) $(OBJ_LST_DIR) $(C_OBJS) $(S_OBJS) $(CXX_OBJS)
	@- echo "Linking target: $@"
	$(CXX_CC) $(LDFLAGS) $(C_OBJS) $(S_OBJS) $(CXX_OBJS) $(LIBS) -o $(EXE_DIR)/$(MAINFILE).out

$(EXE_DIR)/$(MAINFILE).bin: $(EXE_DIR)/$(MAINFILE).out
	@- echo "Creating binary file"
	$(OBJCOPY) -O binary $(EXE_DIR)/$(MAINFILE).out $(EXE_DIR)/$(MAINFILE).bin
	$(DUMP) -S $(EXE_DIR)/$(MAINFILE).out > $(EXE_DIR)/$(MAINFILE).list
	$(SIZE) --target=elf32-littlearm $(OBJ_LST_DIR)/$(MAINFILE).o
	$(SIZE) --target=elf32-littlearm $(EXE_DIR)/$(MAINFILE).out

.PHONY: clean
clean:
	$(RM) $(OBJ_LST_DIR) $(EXE_DIR) $(DOC_DIR)/html $(DOC_DIR)/latex
	
.PHONY: flash
flash:
	$(FLASH) \
		--verify                           \
		--mode out                         \
		--flash $(EXE_DIR)/$(MAINFILE).bin \
		--reset || true
