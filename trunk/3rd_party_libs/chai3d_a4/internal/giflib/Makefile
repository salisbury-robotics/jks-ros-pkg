#  Software License Agreement (BSD License)
#  Copyright (c) 2003-2012, CHAI3D.
#  (www.chai3d.org)
#
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#  notice, this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above
#  copyright notice, this list of conditions and the following
#  disclaimer in the documentation and/or other materials provided
#  with the distribution.
#
#  * Neither the name of CHAI3D nor the names of its contributors may
#  be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  $Author: seb $
#  $Date: 2012-05-07 16:50:14 -0700 (Mon, 07 May 2012) $
#  $Rev: 836 $


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

# paths
LIB_DIR = $(TOP_DIR)/lib/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)
INC_DIR = $(TOP_DIR)/include
SRC_DIR = $(TOP_DIR)/src
OBJ_DIR = $(TOP_DIR)/obj/$(CFG)/$(OS)-$(ARCH)$(GCC_VER)

# compiler flags
CC       = gcc$(GCC_VER)
CCFLAGS  = -Wall -I$(INC_DIR) -I$(SRC_DIR) -DHAVE_CONFIG_H

# librarian flags
ARFLAGS  = rs

# build configuration specific flags
ifneq ($(CFG),debug)
CCFLAGS += -O3 -ffast-math -Wno-unused-parameter
else
CCFLAGS += -O0 -g -DDEBUG -Wall -Wshadow -Wpointer-arith -Wcast-qual -Wstrict-overflow=5 -Wextra
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
	-rm -f $(OBJECTS) $(LIB_TARGET) *~
	-rm -rf $(OBJ_DIR)
