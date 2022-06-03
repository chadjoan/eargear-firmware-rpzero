##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer \
    -march=armv6k -mfpu=vfp -mfloat-abi=soft \
    -Wall
# NOTE: The "-march=armv6k -mfpu=vfp -mfloat-abi=soft" options are because
#   I had to disable hardware floating point because ChibiOS would not boot
#   with it AND floating-point-chprintf enabled at the same time. And as of
#   the most recent build, floating-point-chprintf was important as
#   a convenient way to print the output of the MS8607's (mostly unmodified)
#   driver. It might not be too hard to remove all floating point calculations
#   from the driver and thus render floating point calculations 100%
#   unnecessary again, and I probably will (but just to avoid the class
#   of floating-point errors and failure modes), but disabling floating-point
#   chprintf is starting to sound like a poor idea. Then again, the failure
#   mode for that isn't very bad... it's the failure mode for
#   hard-float-with-chprintf-float-simultaneously that's _really_ bad.
#   -- Chad Joan, 2022-01-02
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = no
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

ifneq ($(EXTENDED_SHELL),no)
  EXTENDED_SHELL = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = kernel

# Imported source files and paths
CHIBIOS = depends/ChibiOS-RPi
#include $(CHIBIOS)/boards/RASPBERRYPI_MODB/board.mk
include $(CHIBIOS)/boards/RP_ZERO/board.mk
include $(CHIBIOS)/os/hal/platforms/BCM2835/platform.mk
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/ports/GCC/ARM/BCM2835/port.mk
include $(CHIBIOS)/os/kernel/kernel.mk
include $(CHIBIOS)/test/test.mk

# Define linker script file here
LDSCRIPT= $(PORTLD)/BCM2835.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(PORTSRC) \
       $(KERNSRC) \
       $(TESTSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       ${CHIBIOS}/os/various/shell.c \
       ${CHIBIOS}/os/various/chprintf.c \
       depends/drivers/MS5840/ms5840.c \
       depends/drivers/MS8607/ms8607.c \
       src/main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(PORTASM)

INCDIR = src \
         $(PORTINC) $(KERNINC) $(TESTINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         depends/drivers/MS5840 \
         depends/drivers/MS8607

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = arm1176jz-s

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
DDEFS =

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List extra objdump defines here.
ODDEFS = -D

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
ifeq ($(EXTENDED_SHELL), yes)
UDEFS = -DEXTENDED_SHELL
else
UDEFS =
endif


# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user defines
##############################################################################

##############################################################################
# The 'all' target and related things are in this include
#

include $(CHIBIOS)/os/ports/GCC/ARM/rules.mk

#
# End of ALL
##############################################################################

##############################################################################
# Start of (Raspberry Pi Zero)-specific build targets
#

MAKE_ALL_RULE_HOOK: sdcard-final-contents/$(PROJECT).img

sdcard-final-contents: $(BUILDDIR)/$(PROJECT).bin $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	mkdir -p $(BUILDDIR)/sdcard-final-contents
	cp -a sdcard-boilerplate/* $(BUILDDIR)/sdcard-final-contents
else
	@echo "Creating $(BUILDDIR)/sdcard-final-contents (directory)"
	@mkdir -p $(BUILDDIR)/sdcard-final-contents
	@cp -a sdcard-boilerplate/* $(BUILDDIR)/sdcard-final-contents
endif


sdcard-final-contents/$(PROJECT).img: sdcard-final-contents $(BUILDDIR)/$(PROJECT).bin $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	cp $(BUILDDIR)/$(PROJECT).bin $(BUILDDIR)/sdcard-final-contents/$(PROJECT).img
else
	@echo Copying $(BUILDDIR)/$(PROJECT).bin to $(BUILDDIR)/sdcard-final-contents/$(PROJECT).img
	@cp $(BUILDDIR)/$(PROJECT).bin $(BUILDDIR)/sdcard-final-contents/$(PROJECT).img
endif

#
# End of (Raspberry Pi Zero)-specific build targets
##############################################################################

