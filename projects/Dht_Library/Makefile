SHELL          = bash
ARDUINO_ROOT  ?= /usr/share/arduino
COREPATH       = $(ARDUINO_ROOT)/hardware/arduino/cores/arduino
LIBSPATH       = $(ARDUINO_ROOT)/libraries
ARDUINO_VER    = 105
SERIAL        ?= /dev/ttyACM0
BAUD          ?= 115200

# The supported boards
BOARDS         = uno mega

# Various tools
CC             = avr-gcc
AR             = avr-ar
OBJCOPY        = avr-objcopy
CXX            = avr-g++
AVRDUDE        = $(ARDUINO_ROOT)/hardware/tools/avrdude

# Target-specific MCU definitions
MCU_uno        = atmega328p
MCU_mega       = atmega2560

# Target-specific mcu frequency definitions
MCU_FREQ_uno   = 16000000L
MCU_FREQ_mega  = 16000000L

# Target-specific code variant definitions
VARIANT_uno    = standard
VARIANT_mega   = mega

# Target-specific arvdude programmers
PROG_uno       = arduino
PROG_mega      = wiring

# Target-specific linker options
LDFLAGS_uno    = -Wl,--gc-sections
LDFLAGS_mega   = -Wl,--gc-sections,--relax

# C compiler options
CFLAGS         =
CFLAGS        += -g
CFLAGS        += -Os
CFLAGS        += -Wall
CFLAGS        += -Wextra
CFLAGS        += -ffunction-sections
CFLAGS        += -fdata-sections
CFLAGS        += -mmcu=$(MCU_$(T))
CFLAGS        += -DF_CPU=$(MCU_FREQ_$(T))
CFLAGS        += -MMD
CFLAGS        += -DUSB_VID=null
CFLAGS        += -DUSB_PID=null
CFLAGS        += -DARDUINO=$(ARDUINO_VER)

# C++ compiler options
CXXFLAGS       =
CXXFLAGS      += $(CFLAGS)
CXXFLAGS      += -fno-exceptions

# Directories in which to look for header files
INCLUDES       =
INCLUDES      += -I$(COREPATH)
INCLUDES      += -I$(ARDUINO_ROOT)/hardware/arduino/variants/$(VARIANT_$(T))

# Programmer options
AVRDUDE_FLAGS  =
AVRDUDE_FLAGS += -C$(ARDUINO_ROOT)/hardware/tools/avrdude.conf
AVRDUDE_FLAGS += -v -v -v -v
AVRDUDE_FLAGS += -p$(MCU_$(T))
AVRDUDE_FLAGS += -c$(PROG_$(T))
AVRDUDE_FLAGS += -P$(SERIAL)
AVRDUDE_FLAGS += -b$(BAUD)
AVRDUDE_FLAGS += -D
AVRDUDE_FLAGS += -Uflash:w:$(HEXFILE):i


# All source files that will be compiled as part of this project.  This includes
# all C/C++ source files in the current directory, as well as all C/C++ source
# files in the $(COREPATH) directory
SOURCES        =
SOURCES       += $(wildcard *.cpp)
SOURCES       += $(wildcard *.c)
SOURCES       += $(wildcard $(COREPATH)/*.cpp)
SOURCES       += $(wildcard $(COREPATH)/*.c)

OBJ_DIR        = objs
CORE           = $(OBJ_DIR)/core.a
HEXFILE        = $(OBJ_DIR)/project.hex
ELFFILE        = $(OBJ_DIR)/project.elf
EEPFILE        = $(OBJ_DIR)/project.eep

#
# create_board_rule
#
# Create a board rule so that I can type 'make <target>,<board>' to build
# target <target> for board <uno> (e.g, make all,uno).
#
# Call using:
#     $(eval $(call create_board_rule,<YOUR_BOARD>))
#
# Parameters:
#     $(1) The board name
#
define create_board_rule
%,$(1): T=$(1)
%,$(1): %
	@true
endef

#
# add_library_dir
#
# Adds the given library directory to the include path, and adds all C/C++
# source files in that directory to the list of files that will be compiled
# as part of this project.
#
# Call using:
#     $(eval $(call add_library_dir,YOUR_DIRECTORY))
#
# Parameters:
#     $(1) The library directory to add
#
define add_library_dir
INCLUDES  += -I$(1)

SOURCES   += $$(wildcard $(1)/*.c)
SOURCES   += $$(wildcard $(1)/*.cpp)
endef

#
# define_compile_rules
#
# Defines compile rules for C/C++ source files in the given directory.
#
# Call using:
#     $(eval $(call define_compile_rules,YOUR_DIRECTORY))
#
# Parameters:
#     $(1) The name of the directory containing source files that need compile
#          rules generated.
#
define define_compile_rules
$(OBJ_DIR)/%.o: $(1)%.c
	@echo '(CC)      ' $$<
	@mkdir -p $$(@D)
	@$(CC) -c $$(CFLAGS) $$(INCLUDES) -o $$@ $$<

$(OBJ_DIR)/%.o: $(1)%.cpp
	@echo '(CXX)     ' $$<
	@mkdir -p $$(@D)
	@$(CXX) -c $$(CXXFLAGS) $$(INCLUDES) -o $$@ $$<
endef

# Default target
all: check_target check_objdir $(HEXFILE)

check_target:
	@if [[ "$(T)" == "" ]]; then echo "Error: Target not specified"; exit 1; fi

# We put a breadcrumb in the OBJ_DIR directory to keep track of which target
# it was compiled against.  If the OBJ_DIR directory exists, but the
# corresponding breadcrumb doens't, blow away the build directory -- it was
# for a different target
check_objdir:
	@if [[ -d "$(OBJ_DIR)" && ! -e "$(OBJ_DIR)/.$(T)" ]]; then "Cleaning old build"; rm -rf $(OBJ_DIR); fi

# Include project.mk if it exists
-include project.mk

# Create a all,<board> and install,<board> for every board in BOARDS
$(foreach board,$(BOARDS),$(eval $(call create_board_rule,$(board))))

# Create a compile rule for every directory containing source files
$(foreach directory,$(sort $(dir $(SOURCES))),$(eval $(call define_compile_rules,$(directory))))

# Object files that are part of this project.  This does some magic on the
# content of $(SOURCES) to produce a list of object files in $(OBJ_DIR)
# corresponding to the source files in $(SOURCES)
OBJS := $(sort $(patsubst %.cpp,$(OBJ_DIR)/%.o,$(patsubst %.c,$(OBJ_DIR)/%.o,$(notdir $(SOURCES)))))

$(CORE): $(OBJS)
	@for i in $(OBJS); do \
		echo "(AR)       $$i"; \
		$(AR) rcs $(CORE) $$i; \
	done

$(ELFFILE): $(CORE)
	@echo "(CC)       $(CORE)"
	@$(CC) -Os $(LDFLAGS_$(T)) -mmcu=$(MCU_$(T)) -o $(ELFFILE) $(CORE) -lm

$(EEPFILE): $(ELFFILE)
	@echo "(OBJCOPY)  $(ELFFILE)"
	@$(OBJCOPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(ELFFILE) $(EEPFILE)

$(HEXFILE): $(EEPFILE)
	@echo "(OBJCOPY)  $(EEPFILE)"
	@$(OBJCOPY) -O ihex -R .eeprom $(ELFFILE) $(HEXFILE)
	@touch $(OBJ_DIR)/.$(T)

install: $(HEXFILE)
	@echo "(AVRDUDE)  $(HEXFILE)"
	@$(AVRDUDE) $(AVRDUDE_FLAGS)

clean:
	@echo -n "Cleaning up object output directory..."
	@rm -rf $(OBJ_DIR)
	@echo " Done."
