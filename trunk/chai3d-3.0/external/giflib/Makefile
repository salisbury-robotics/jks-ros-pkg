#  Copyright (C) 2003-2011 by CHAI 3D.
#  All Rights Reserved.
#
#  $Author: seb $
#  $Date: 2011-10-27 02:43:57 -0700 (Thu, 27 Oct 2011) $
#  $Rev: 657 $


# top level directory
TOP_DIR = .

# determine build configuration
ifneq ($(CFG),debug)
CFG = release
endif

# determine architecture
ifeq ($(ARCH), )
ARCH  ?= $(shell echo `uname -m`)
MARCH ?= native
else
MARCH ?= $(ARCH)
endif

# determine platform
SYSTEM ?= $(shell echo `uname -s`)
ifeq ($(SYSTEM), Linux)
OS      = lin
endif
ifeq ($(SYSTEM), Darwin)
OS      = mac
endif
ifeq ($(SYSTEM), QNX)
OS      = qnx
endif

# paths
LIB_DIR = $(TOP_DIR)/lib/$(OS)-$(ARCH)$(GCC_VER)/$(CFG)
INC_DIR = $(TOP_DIR)/include
SRC_DIR = $(TOP_DIR)/src
OBJ_DIR = $(TOP_DIR)/obj/$(OS)-$(ARCH)$(GCC_VER)/$(CFG)

# compiler flags
CC       = gcc$(GCC_VER)
CCFLAGS  = -Wall -I$(INC_DIR) -I$(SRC_DIR) -DHAVE_CONFIG_H

# librarian flags
ARFLAGS  = rs

# build configuration specific flags
ifneq ($(CFG),debug)
CCFLAGS  += -O3 -ffast-math
else
CCFLAGS  += -O0 -g
endif

# set target architecture
ifeq ($(OS), lin)
ifeq ($(ARCH), x86_64)
CCFLAGS += -m64
ifneq ($(MARCH), $(ARCH))
CCFLAGS += -march=$(MARCH)
endif
endif
ifeq ($(ARCH), i386)
CCFLAGS += -m32 -march=$(MARCH)
endif
ifeq ($(ARCH), i486)
CCFLAGS += -m32 -march=$(MARCH)
endif
ifeq ($(ARCH), i586)
CCFLAGS += -m32 -march=$(MARCH)
endif
ifeq ($(ARCH), i686)
CCFLAGS += -m32 -march=$(MARCH)
endif
endif
ifeq ($(OS), mac)
CCFLAGS += -arch $(ARCH)
endif

# target
LIB_TARGET = $(LIB_DIR)/libgif.a

# QNX-specific gcc tweaks
ifeq ($(OS),qnx)
CC = qcc
# upgrade to gcc-3.3.5 for QNX 6.3.x
QNX_VERSION ?= $(shell echo `uname -r | awk 'BEGIN { FS = "." } { print $$1 "." $$2  }'`)
ifeq ($(QNX_VERSION), 6.3)
CCFLAGS += -V3.3.5,gcc_ntox86
endif
endif

# source files
EXCLUDE  = $(wildcard $(SRC_DIR)/*_core.c)
SOURCES  = $(filter-out $(EXCLUDE), $(wildcard $(SRC_DIR)/*.c))
OBJECTS  = $(patsubst %.c, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
INCLUDES = $(wildcard $(INC_DIR)/*.h)


# target rules

all: lib

lib: $(OBJ_DIR) $(LIB_DIR) $(LIB_TARGET)

$(LIB_TARGET): $(OBJECTS)
	$(AR) $(ARFLAGS) $(LIB_TARGET) $?

$(OBJECTS): $(INCLUDES)

$(LIB_DIR) $(OBJ_DIR):
	mkdir -p $@

$(OBJ_DIR)/%.o : $(SRC_DIR)/%.c
	$(CC) $(CCFLAGS) -c -o $@ $<

tags:
	find ../.. -name \*.cpp -o -name \*h | xargs etags -o TAGS

clean:
	-rm -f $(OBJECTS) $(LIB_TARGET) *~ TAGS core *.bak #*#
	-rm -rf $(OBJ_DIR)

